#!/usr/bin/env python3
"""Interactive ROS client for EmotionBot."""

import json
import threading

import rospy
from std_msgs.msg import String


class ChatClient:
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

    def ask(self, text):
        with self.condition:
            previous = self.state["sequence"] if self.state else -1
            self.response = None
            self.publisher.publish(String(data=text))
            deadline = rospy.get_time() + 8.0
            while not rospy.is_shutdown():
                if self.state and self.state["sequence"] > previous and self.response is not None:
                    return self.response, self.state
                remaining = deadline - rospy.get_time()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for EmotionBot")
                self.condition.wait(min(0.2, remaining))


def main():
    rospy.init_node("emotion_chat", anonymous=True, disable_signals=True)
    client = ChatClient()
    rospy.sleep(0.5)
    print("EmotionBot ROS chat. Use 'event:<emotion>' for a deterministic event; :quit exits.")
    try:
        while not rospy.is_shutdown():
            text = input("you> ").strip()
            if text.lower() in (":quit", ":q", "exit"):
                break
            if not text:
                continue
            response, state = client.ask(text)
            print("bot> %s" % response)
            print(
                "state> %s valence=%+.3f arousal=%.3f backend=%s"
                % (state["emotion"], state["valence"], state["arousal"], state["backend"])
            )
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
