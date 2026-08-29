#!/usr/bin/env python3
"""Clamp, watchdog, arbitrate, and bridge safe commands to Lite3's Joy input."""

import json
import threading

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
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
        limits = Limits(
            x=float(rospy.get_param("/emotion_bot/safety/limits/linear_x", 0.10)),
            y=float(rospy.get_param("/emotion_bot/safety/limits/linear_y", 0.05)),
            yaw=float(rospy.get_param("/emotion_bot/safety/limits/angular_z", 0.10)),
        )
        self.controller = SafetyController(
            limits=limits,
            expression_timeout=float(rospy.get_param("/emotion_bot/safety/expression_timeout", 0.5)),
            manual_timeout=float(rospy.get_param("/emotion_bot/safety/manual_timeout", 0.5)),
            manual_priority_hold=float(rospy.get_param("/emotion_bot/safety/manual_priority_hold", 0.75)),
            mode_transition_delay=float(rospy.get_param("/emotion_bot/safety/mode_transition_delay", 0.35)),
        )
        self.controller.set_enabled(
            bool(rospy.get_param("/emotion_bot/safety/motion_enabled", False)),
            rospy.Time.now().to_sec(),
        )
        topics = rospy.get_param("/emotion_bot/topics")
        self.joy_pub = rospy.Publisher(topics["joy_output"], Joy, queue_size=10)
        self.safe_pub = rospy.Publisher(topics["safe_cmd"], Twist, queue_size=10)
        self.status_pub = rospy.Publisher(topics["status"], String, queue_size=10, latch=True)
        self.expression_sub = rospy.Subscriber(topics["expression_cmd"], Twist, self.on_expression, queue_size=10)
        self.manual_sub = rospy.Subscriber(topics["manual_joy"], Joy, self.on_manual, queue_size=10)
        self.state_sub = rospy.Subscriber(topics["state"], String, self.on_state, queue_size=10)
        self.ready_sub = rospy.Subscriber(topics["sim_ready"], Bool, self.on_sim_ready, queue_size=1)
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
                TwistValue(message.linear.x, message.linear.y, message.angular.z),
                rospy.Time.now().to_sec(),
            )

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

    def on_enable(self, request):
        with self.lock:
            if request.data and self.require_sim_ready and not self.sim_ready:
                return SetBoolResponse(success=False, message="Simulation controller is not ready; motion remains disabled")
            self.controller.set_enabled(request.data, rospy.Time.now().to_sec())
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
        message.angular.z = value.yaw
        return message

    def on_timer(self, _event):
        with self.lock:
            decision = self.controller.step(rospy.Time.now().to_sec())
            enabled = self.controller.motion_enabled
        self.joy_pub.publish(self.joy_message(decision.joy))
        self.safe_pub.publish(self.twist_message(decision.twist))
        status = {
            "stamp": rospy.Time.now().to_sec(),
            "motion_enabled": enabled,
            "stale": decision.stale,
            "backend": self.backend,
            "selected_source": decision.selected_source,
            "last_safety_action": decision.action,
            "sim_ready": self.sim_ready,
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
    rospy.loginfo("Emotion safety bridge ready; motion is disabled by default")
    rospy.spin()


if __name__ == "__main__":
    main()
