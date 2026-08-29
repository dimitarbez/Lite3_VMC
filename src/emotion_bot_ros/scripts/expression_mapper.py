#!/usr/bin/env python3
"""Convert validated emotion states into finite-duration Twist patterns."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from emotion_bot_ros.contract import ContractError, loads_state
from emotion_bot_ros.mapping import PatternPlayer, TwistValue, load_patterns


def to_message(value):
    message = Twist()
    message.linear.x = value.x
    message.linear.y = value.y
    message.angular.z = value.yaw
    return message


class ExpressionMapper:
    def __init__(self):
        self.lock = threading.Lock()
        self.patterns = load_patterns(rospy.get_param("/emotion_bot/mappings"))
        self.player = PatternPlayer(self.patterns)
        self.last_state_at = None
        self.last_sequence = -1
        self.state_timeout = float(rospy.get_param("/emotion_bot/mapper/state_timeout", 1.0))
        state_topic = rospy.get_param("/emotion_bot/topics/state", "/emotion_bot/state")
        command_topic = rospy.get_param("/emotion_bot/topics/expression_cmd", "/emotion_bot/expression_cmd")
        self.publisher = rospy.Publisher(command_topic, Twist, queue_size=10)
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
            self.publisher.publish(Twist())
            return
        with self.lock:
            now = rospy.Time.now().to_sec()
            if state["sequence"] > self.last_sequence:
                self.player.start(state["emotion"], now)
                self.last_sequence = state["sequence"]
            self.last_state_at = now

    def on_timer(self, _event):
        now = rospy.Time.now().to_sec()
        with self.lock:
            if self.last_state_at is None or now - self.last_state_at > self.state_timeout:
                value = TwistValue()
            else:
                value = self.player.command(now)
        self.publisher.publish(to_message(value))

    def stop(self):
        self.publisher.publish(Twist())


def main():
    rospy.init_node("expression_mapper")
    ExpressionMapper()
    rospy.loginfo("Emotion expression mapper ready")
    rospy.spin()


if __name__ == "__main__":
    main()
