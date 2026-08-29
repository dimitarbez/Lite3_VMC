#!/usr/bin/env python3
"""Deterministic ROS demo that always disables motion before exiting."""

import argparse
import json
import threading

import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool


class Demo:
    def __init__(self):
        self.condition = threading.Condition()
        self.state = None
        self.response = None
        self.publisher = rospy.Publisher("/emotion_bot/input", String, queue_size=10)
        self.state_sub = rospy.Subscriber("/emotion_bot/state", String, self.on_state, queue_size=10)
        self.response_sub = rospy.Subscriber("/emotion_bot/response", String, self.on_response, queue_size=10)

    def on_state(self, message):
        with self.condition:
            self.state = json.loads(message.data)
            self.condition.notify_all()

    def on_response(self, message):
        with self.condition:
            self.response = message.data
            self.condition.notify_all()

    def send(self, text):
        with self.condition:
            old_sequence = self.state["sequence"] if self.state else -1
            self.response = None
            self.publisher.publish(String(data=text))
            deadline = rospy.get_time() + 10.0
            while not rospy.is_shutdown() and rospy.get_time() < deadline:
                if self.state and self.state["sequence"] > old_sequence and self.response is not None:
                    return self.state, self.response
                self.condition.wait(0.2)
        raise RuntimeError("timed out waiting for %s" % text)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--hold", type=float, default=2.0, help="wall seconds between events")
    args, _unknown = parser.parse_known_args()
    rospy.init_node("emotion_demo", anonymous=True)
    demo = Demo()
    rospy.wait_for_service("/emotion_bot/set_motion_enabled", timeout=10.0)
    set_motion = rospy.ServiceProxy("/emotion_bot/set_motion_enabled", SetBool)
    rospy.sleep(0.5)
    try:
        if args.enable_motion:
            print(set_motion(True).message)
        for event in ("event:joy", "event:anger", "event:curiosity"):
            state, response = demo.send(event)
            print(
                "%s -> emotion=%s valence=%+.3f arousal=%.3f\n  response=%s"
                % (event, state["emotion"], state["valence"], state["arousal"], response)
            )
            rospy.sleep(args.hold)
        state, response = demo.send("event:neutral")
        print("event:neutral -> emotion=%s response=%s" % (state["emotion"], response))
    finally:
        print(set_motion(False).message)
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
