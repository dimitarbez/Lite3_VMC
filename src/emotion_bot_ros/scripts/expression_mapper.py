#!/usr/bin/env python3
"""Convert validated emotion states into smooth, looping expression patterns."""

import json
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from emotion_bot_ros.contract import ContractError, loads_state
from emotion_bot_ros.mapping import FluidExpressionController, TwistValue, load_patterns


def to_message(value):
    message = Twist()
    message.linear.x = value.x
    message.linear.y = value.y
    message.linear.z = value.z
    message.angular.x = value.roll
    message.angular.y = value.pitch
    message.angular.z = value.yaw
    return message


class ExpressionMapper:
    def __init__(self):
        self.lock = threading.Lock()
        self.patterns = load_patterns(rospy.get_param("/emotion_bot/mappings"))
        fluid = rospy.get_param("/emotion_bot/mapper/fluid")
        self.controller = FluidExpressionController(
            self.patterns,
            valence_tau=float(fluid["valence_tau"]),
            arousal_tau=float(fluid["arousal_tau"]),
            blend_time=float(fluid["blend_time"]),
            neutral_return_time=float(fluid["neutral_return_time"]),
            neutral_hold_time=float(fluid.get("neutral_hold_time", 0.0)),
            min_dwell=float(fluid["min_dwell"]),
            hysteresis=float(fluid["hysteresis"]),
            min_intensity=float(fluid["min_intensity"]),
            amplitude_scale=float(fluid.get("amplitude_scale", 1.0)),
            idle_amplitude_scale=float(fluid.get("idle_amplitude_scale", 1.0)),
            idle_time_scale=float(fluid.get("idle_time_scale", 1.0)),
            max_linear_rate=float(fluid["max_linear_rate"]),
            max_yaw_rate=float(fluid["max_yaw_rate"]),
        )
        self.last_state_at = None
        self.last_sequence = -1
        self.state_timeout = float(rospy.get_param("/emotion_bot/mapper/state_timeout", 1.0))
        state_topic = rospy.get_param("/emotion_bot/topics/state", "/emotion_bot/state")
        command_topic = rospy.get_param("/emotion_bot/topics/expression_cmd", "/emotion_bot/expression_cmd")
        action_topic = rospy.get_param("/emotion_bot/topics/expression_action", "/emotion_bot/expression_action")
        self.publisher = rospy.Publisher(command_topic, Twist, queue_size=10)
        self.action_publisher = rospy.Publisher(action_topic, String, queue_size=10)
        self.subscriber = rospy.Subscriber(state_topic, String, self.on_state, queue_size=10)
        rate = float(rospy.get_param("/emotion_bot/mapper/publish_rate", 20.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self.on_timer)
        rospy.on_shutdown(self.stop)

    def on_state(self, message):
        try:
            state = loads_state(message.data)
        except ContractError as exc:
            rospy.logerr("Malformed emotion state rejected: %s", exc)
            with self.lock:
                self.last_state_at = None
                self.controller.force_neutral(rospy.Time.now().to_sec())
            return
        with self.lock:
            now = rospy.Time.now().to_sec()
            if state["sequence"] > self.last_sequence:
                self.controller.update(
                    state["emotion"], state["valence"], state["arousal"], now
                )
                self.last_sequence = state["sequence"]
            self.last_state_at = now

    def on_timer(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            stale = self.last_state_at is None or now - self.last_state_at > self.state_timeout
            value = self.controller.command(now, stale=stale)
            action = self.controller.consume_action(now, stale=stale)
        self.publisher.publish(to_message(value))
        if action is not None:
            self.action_publisher.publish(
                String(data=json.dumps(action, sort_keys=True, separators=(",", ":")))
            )

    def stop(self):
        action = {
            "schema_version": "1.0",
            "kind": "cancel",
            "generation": self.controller.generation + 1,
        }
        self.action_publisher.publish(
            String(data=json.dumps(action, sort_keys=True, separators=(",", ":")))
        )
        self.publisher.publish(Twist())


def main():
    rospy.init_node("expression_mapper")
    ExpressionMapper()
    rospy.loginfo("Emotion expression mapper ready")
    rospy.spin()


if __name__ == "__main__":
    main()
