#!/usr/bin/env python3
"""Deterministic ROS demo that always disables motion before exiting."""

import argparse
import json
import threading
import time

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
    parser.add_argument(
        "--animation-review",
        action="store_true",
        help="show every entrance and at least two complete idle cycles",
    )
    args, _unknown = parser.parse_known_args()
    rospy.init_node("emotion_demo", anonymous=True)
    demo = Demo()
    rospy.wait_for_service("/emotion_bot/set_motion_enabled", timeout=10.0)
    set_motion = rospy.ServiceProxy("/emotion_bot/set_motion_enabled", SetBool)
    rospy.sleep(0.5)
    try:
        if args.enable_motion:
            print(set_motion(True).message)
        if args.animation_review:
            sequence = (
                ("neutral", 8.5),
                ("joy", 8.0),
                ("sadness", 11.6),
                ("anger", 29.5),
                ("fear", 2.6),
                ("surprise", 7.8),
                ("disgust", 9.0),
                ("curiosity", 7.8),
                ("affection", 11.1),
            )
        else:
            sequence = (("joy", args.hold), ("anger", args.hold), ("curiosity", args.hold))
        for emotion, hold in sequence:
            event = "event:%s" % emotion
            state, response = demo.send(event)
            print(
                "%s -> emotion=%s valence=%+.3f arousal=%.3f hold=%.1fs\n  response=%s"
                % (
                    event,
                    state["emotion"],
                    state["valence"],
                    state["arousal"],
                    hold,
                    response,
                )
            )
            # Animation phases use ROS time: slow rendering must not truncate
            # the promised idle cycles. Keep the original demo's wall holds.
            if args.animation_review:
                rospy.sleep(hold)
            else:
                deadline = time.monotonic() + hold
                while not rospy.is_shutdown() and time.monotonic() < deadline:
                    time.sleep(max(0.0, min(0.1, deadline - time.monotonic())))
        if not args.animation_review:
            state, response = demo.send("event:neutral")
            print("event:neutral -> emotion=%s response=%s" % (state["emotion"], response))
    finally:
        print(set_motion(False).message)
        rospy.sleep(0.5)


if __name__ == "__main__":
    main()
