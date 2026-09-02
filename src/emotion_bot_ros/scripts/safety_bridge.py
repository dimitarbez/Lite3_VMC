#!/usr/bin/env python3
"""Clamp, watchdog, arbitrate, and bridge safe commands to Lite3's Joy input."""

import json
import math
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

from emotion_bot_ros.contract import loads_state
from emotion_bot_ros.mapping import TwistValue
from emotion_bot_ros.safety import Limits, SafetyController


class SafetyBridge:
    def __init__(self):
        self.lock = threading.Lock()
        self.backend = "unknown"
        self.require_sim_ready = bool(rospy.get_param("/emotion_bot/safety/require_sim_ready", False))
        self.sim_ready = not self.require_sim_ready
        self.monitor_sim_health = bool(
            rospy.get_param("/emotion_bot/safety/monitor_sim_health", self.require_sim_ready)
        )
        self.minimum_body_height = float(rospy.get_param("/emotion_bot/safety/minimum_body_height", 0.18))
        self.maximum_body_height = float(rospy.get_param("/emotion_bot/safety/maximum_body_height", 0.40))
        self.maximum_tilt = float(rospy.get_param("/emotion_bot/safety/maximum_tilt", 0.60))
        self.maximum_planar_displacement = float(
            rospy.get_param("/emotion_bot/safety/maximum_planar_displacement", 0.040)
        )
        self.gazebo_stomp_impulse_enabled = bool(
            rospy.get_param("/emotion_bot/safety/gazebo_stomp_impulse_enabled", False)
        )
        self.gazebo_stomp_force = float(
            rospy.get_param("/emotion_bot/safety/gazebo_stomp_force", 160.0)
        )
        self.gazebo_stomp_duration = float(
            rospy.get_param("/emotion_bot/safety/gazebo_stomp_duration", 0.08)
        )
        self.gazebo_stomp_delay = float(
            rospy.get_param("/emotion_bot/safety/gazebo_stomp_delay", 0.12)
        )
        self.gazebo_wrench = (
            rospy.ServiceProxy("/gazebo/apply_body_wrench", ApplyBodyWrench)
            if self.gazebo_stomp_impulse_enabled else None
        )
        self.health_timeout = float(rospy.get_param("/emotion_bot/safety/health_timeout", 0.50))
        self.model_health_ok = not self.monitor_sim_health
        self.joint_health_ok = not self.monitor_sim_health
        self.health_ok = not self.monitor_sim_health
        self.health_fault = "waiting_for_sim_health" if self.monitor_sim_health else ""
        self.last_model_health_wall = 0.0
        self.last_joint_health_wall = 0.0
        self.motion_anchor = None
        self.prepare_stance_on_ready = bool(
            rospy.get_param("/emotion_bot/safety/prepare_stance_on_ready", False)
        )
        self.stance_settle_time = float(rospy.get_param("/emotion_bot/safety/stance_settle_time", 4.0))
        self.stance_ready_at = None
        self.stance_prepared = not self.prepare_stance_on_ready
        requested_initial_motion = bool(
            rospy.get_param("/emotion_bot/safety/motion_enabled", False)
        )
        # In the integrated simulator, a requested initial enable must wait for
        # the controller-ready, model-health, and stable-stance interlocks.  It
        # is consumed exactly once: a later fault or manual disable remains
        # latched off until the service is deliberately called again.
        self.auto_enable_pending = requested_initial_motion and self.prepare_stance_on_ready
        limits = Limits(
            x=float(rospy.get_param("/emotion_bot/safety/limits/linear_x", 0.10)),
            y=float(rospy.get_param("/emotion_bot/safety/limits/linear_y", 0.05)),
            yaw=float(rospy.get_param("/emotion_bot/safety/limits/angular_z", 0.10)),
            z=float(rospy.get_param("/emotion_bot/safety/limits/body_height", 0.070)),
            roll=float(rospy.get_param("/emotion_bot/safety/limits/roll", 0.50)),
            pitch=float(rospy.get_param("/emotion_bot/safety/limits/pitch", 0.50)),
        )
        self.controller = SafetyController(
            limits=limits,
            expression_timeout=float(rospy.get_param("/emotion_bot/safety/expression_timeout", 0.5)),
            manual_timeout=float(rospy.get_param("/emotion_bot/safety/manual_timeout", 0.5)),
            manual_priority_hold=float(rospy.get_param("/emotion_bot/safety/manual_priority_hold", 0.75)),
            mode_transition_delay=float(rospy.get_param("/emotion_bot/safety/mode_transition_delay", 0.35)),
            minimum_locomotion_time=float(rospy.get_param("/emotion_bot/safety/minimum_locomotion_time", 2.0)),
            manual_locomotion_timeout=float(rospy.get_param("/emotion_bot/safety/manual_locomotion_timeout", 4.0)),
            locomotion_command_delay=float(rospy.get_param("/emotion_bot/safety/locomotion_command_delay", 1.1)),
            max_linear_rate=float(rospy.get_param("/emotion_bot/safety/limits/linear_rate", 0.18)),
            max_yaw_rate=float(rospy.get_param("/emotion_bot/safety/limits/yaw_rate", 0.30)),
            max_height_rate=float(rospy.get_param("/emotion_bot/safety/limits/body_height_rate", 0.25)),
            max_attitude_rate=float(rospy.get_param("/emotion_bot/safety/limits/attitude_rate", 1.50)),
            allow_locomotion=bool(rospy.get_param("/emotion_bot/safety/allow_locomotion", False)),
            allow_dynamic_actions=bool(
                rospy.get_param("/emotion_bot/safety/allow_dynamic_actions", True)
            ),
        )
        self.controller.set_enabled(
            requested_initial_motion and not self.auto_enable_pending,
            rospy.Time.now().to_sec(),
        )
        topics = rospy.get_param("/emotion_bot/topics")
        self.joy_pub = rospy.Publisher(topics["joy_output"], Joy, queue_size=10)
        self.safe_pub = rospy.Publisher(topics["safe_cmd"], Twist, queue_size=10)
        self.status_pub = rospy.Publisher(topics["status"], String, queue_size=10, latch=True)
        self.expression_sub = rospy.Subscriber(topics["expression_cmd"], Twist, self.on_expression, queue_size=10)
        self.expression_action_sub = rospy.Subscriber(
            topics["expression_action"], String, self.on_expression_action, queue_size=10
        )
        self.manual_sub = rospy.Subscriber(topics["manual_joy"], Joy, self.on_manual, queue_size=10)
        self.state_sub = rospy.Subscriber(topics["state"], String, self.on_state, queue_size=10)
        self.ready_sub = rospy.Subscriber(topics["sim_ready"], Bool, self.on_sim_ready, queue_size=1)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_model_states, queue_size=1)
        self.joint_sub = rospy.Subscriber(
            "/lite3_gazebo/joint_states", JointState, self.on_joint_states, queue_size=1
        )
        self.service = rospy.Service(
            rospy.get_param("/emotion_bot/services/set_motion_enabled", "/emotion_bot/set_motion_enabled"),
            SetBool,
            self.on_enable,
        )
        rate = float(rospy.get_param("/emotion_bot/safety/publish_rate", 20.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.on_timer)
        rospy.on_shutdown(self.shutdown)

    def on_expression(self, message):
        with self.lock:
            self.controller.update_expression(
                TwistValue(
                    x=message.linear.x,
                    y=message.linear.y,
                    yaw=message.angular.z,
                    z=message.linear.z,
                    roll=message.angular.x,
                    pitch=message.angular.y,
                ),
                rospy.Time.now().to_sec(),
            )

    def on_expression_action(self, message):
        with self.lock:
            self.controller.update_expression_action(message.data, rospy.Time.now().to_sec())

    def on_manual(self, message):
        with self.lock:
            self.controller.update_manual(list(message.axes), list(message.buttons), rospy.Time.now().to_sec())

    def on_state(self, message):
        try:
            self.backend = loads_state(message.data)["backend"]
        except Exception:
            self.backend = "invalid"

    def on_sim_ready(self, message):
        with self.lock:
            self.sim_ready = bool(message.data)
            if not self.sim_ready:
                self.controller.set_enabled(False, rospy.Time.now().to_sec())
                self.stance_ready_at = None
                self.stance_prepared = not self.prepare_stance_on_ready
            elif self.prepare_stance_on_ready and not self.stance_prepared:
                now = rospy.Time.now().to_sec()
                self.controller.request_stance_preparation(now)
                self.stance_ready_at = now + self.stance_settle_time

    def _update_health_locked(self, fault=""):
        was_healthy = self.health_ok
        self.health_ok = self.model_health_ok and self.joint_health_ok
        self.health_fault = "" if self.health_ok else (fault or self.health_fault or "sim_health_unavailable")
        if not self.health_ok:
            self.controller.set_enabled(False, rospy.Time.now().to_sec())
            # A displacement fault is measured from the enable-time anchor.
            # Once motion is disabled, discard that anchor so healthy stationary
            # model updates can recover and require a deliberate re-enable.
            self.motion_anchor = None
            if self.prepare_stance_on_ready:
                self.stance_prepared = False
                self.stance_ready_at = None
        elif not was_healthy and self.prepare_stance_on_ready and self.sim_ready:
            now = rospy.Time.now().to_sec()
            self.controller.request_stance_preparation(now)
            self.stance_ready_at = now + self.stance_settle_time

    def on_model_states(self, message):
        if not self.monitor_sim_health:
            return
        now_wall = time.monotonic()
        if now_wall - self.last_model_health_wall < 0.05:
            return
        self.last_model_health_wall = now_wall
        try:
            index = message.name.index("lite3_gazebo")
            pose = message.pose[index]
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("nonfinite_model_pose")
            q = pose.orientation
            roll = math.atan2(
                2.0 * (q.w * q.x + q.y * q.z),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y),
            )
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            if not self.minimum_body_height <= pose.position.z <= self.maximum_body_height:
                raise ValueError("body_height_out_of_range")
            if max(abs(roll), abs(pitch)) > self.maximum_tilt:
                raise ValueError("body_tilt_out_of_range")
            if self.motion_anchor is not None:
                planar_displacement = math.hypot(
                    pose.position.x - self.motion_anchor[0],
                    pose.position.y - self.motion_anchor[1],
                )
                if planar_displacement > self.maximum_planar_displacement:
                    raise ValueError("planar_displacement_out_of_range")
        except (ValueError, IndexError) as exc:
            with self.lock:
                self.model_health_ok = False
                self._update_health_locked(str(exc))
            return
        with self.lock:
            self.model_health_ok = True
            if self.controller.motion_enabled and self.motion_anchor is None:
                self.motion_anchor = (pose.position.x, pose.position.y)
            self._update_health_locked()

    def on_joint_states(self, message):
        if not self.monitor_sim_health:
            return
        now_wall = time.monotonic()
        if now_wall - self.last_joint_health_wall < 0.05:
            return
        self.last_joint_health_wall = now_wall
        values = list(message.position) + list(message.velocity) + list(message.effort)
        healthy = len(message.name) >= 12 and all(math.isfinite(value) for value in values)
        with self.lock:
            self.joint_health_ok = healthy
            self._update_health_locked("invalid_joint_state" if not healthy else "")

    def on_enable(self, request):
        with self.lock:
            if request.data and self.require_sim_ready and not self.sim_ready:
                return SetBoolResponse(success=False, message="Simulation controller is not ready; motion remains disabled")
            if request.data and self.monitor_sim_health and not self.health_ok:
                return SetBoolResponse(
                    success=False,
                    message="Simulation health check failed (%s); motion remains disabled" % self.health_fault,
                )
            if request.data and self.prepare_stance_on_ready and not self.stance_prepared:
                return SetBoolResponse(
                    success=False,
                    message="Stable expression stance is still settling; motion remains disabled",
                )
            self.auto_enable_pending = False
            self.controller.set_enabled(request.data, rospy.Time.now().to_sec())
            self.motion_anchor = None
        action = "enabled" if request.data else "disabled and zeroed"
        return SetBoolResponse(success=True, message="Simulation motion %s" % action)

    @staticmethod
    def joy_message(value):
        message = Joy()
        message.header.stamp = rospy.Time.now()
        message.axes = value.axes
        message.buttons = value.buttons
        return message

    @staticmethod
    def twist_message(value):
        message = Twist()
        message.linear.x = value.x
        message.linear.y = value.y
        message.linear.z = value.z
        message.angular.x = value.roll
        message.angular.y = value.pitch
        message.angular.z = value.yaw
        return message

    def apply_gazebo_stomp_impulse(self):
        """Add a calibrated simulation-only flight impulse to a stomp."""
        request = ApplyBodyWrenchRequest()
        request.body_name = "lite3_gazebo::TORSO"
        request.reference_frame = "world"
        request.wrench.force.z = self.gazebo_stomp_force
        request.start_time = rospy.Time.now() + rospy.Duration(self.gazebo_stomp_delay)
        request.duration = rospy.Duration(self.gazebo_stomp_duration)
        try:
            response = self.gazebo_wrench(request)
            if not response.success:
                rospy.logwarn("Gazebo stomp impulse rejected: %s", response.status_message)
        except rospy.ServiceException as exc:
            rospy.logwarn("Gazebo stomp impulse failed: %s", exc)

    def on_timer(self, _event):
        with self.lock:
            if self.monitor_sim_health:
                now_wall = time.monotonic()
                fault = ""
                if now_wall - self.last_model_health_wall > self.health_timeout:
                    self.model_health_ok = False
                    fault = "model_state_timeout"
                if now_wall - self.last_joint_health_wall > self.health_timeout:
                    self.joint_health_ok = False
                    fault = "joint_state_timeout"
                self._update_health_locked(fault)
            if (
                self.prepare_stance_on_ready
                and not self.stance_prepared
                and self.stance_ready_at is not None
                and rospy.Time.now().to_sec() >= self.stance_ready_at
                and self.health_ok
            ):
                self.stance_prepared = True
            if (
                self.auto_enable_pending
                and self.sim_ready
                and self.health_ok
                and self.stance_prepared
            ):
                self.controller.set_enabled(True, rospy.Time.now().to_sec())
                self.motion_anchor = None
                self.auto_enable_pending = False
                rospy.loginfo("Simulation motion enabled after readiness and stance checks")
            decision = self.controller.step(rospy.Time.now().to_sec())
            enabled = self.controller.motion_enabled
        self.joy_pub.publish(self.joy_message(decision.joy))
        if self.gazebo_stomp_impulse_enabled and decision.action == "bounded_emotion_stomp":
            self.apply_gazebo_stomp_impulse()
        self.safe_pub.publish(self.twist_message(decision.twist))
        status = {
            "stamp": rospy.Time.now().to_sec(),
            "motion_enabled": enabled,
            "stale": decision.stale,
            "backend": self.backend,
            "selected_source": decision.selected_source,
            "last_safety_action": decision.action,
            "sim_ready": self.sim_ready,
            "health_ok": self.health_ok,
            "health_fault": self.health_fault,
            "locomotion_allowed": self.controller.allow_locomotion,
            "dynamic_actions_allowed": self.controller.allow_dynamic_actions,
            "stance_prepared": self.stance_prepared,
            "auto_enable_pending": self.auto_enable_pending,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def shutdown(self):
        zero_joy = Joy()
        zero_joy.axes = [0.0] * 8
        zero_joy.buttons = [0] * 11
        zero_joy.buttons[1] = 1
        self.safe_pub.publish(Twist())
        self.joy_pub.publish(zero_joy)


def main():
    rospy.init_node("safety_bridge")
    SafetyBridge()
    rospy.loginfo("Emotion safety bridge ready; motion permission follows the selected launch profile")
    rospy.spin()


if __name__ == "__main__":
    main()
