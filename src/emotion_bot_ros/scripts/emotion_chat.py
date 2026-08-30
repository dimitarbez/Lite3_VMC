#!/usr/bin/env python3
"""Interactive streaming ROS chat client for EmotionBot."""

import json
import threading
import time

import rospy
from std_msgs.msg import String


class ChatClient:
    def __init__(self):
        self.condition = threading.Condition()
        self.state = None
        self.response = None
        self.response_backend = None
        self.error = None
        self.turn_id = None
        self.publisher = rospy.Publisher("/emotion_bot/chat/input", String, queue_size=10)
        self.state_sub = rospy.Subscriber("/emotion_bot/state", String, self.on_state, queue_size=10)
        self.events_sub = rospy.Subscriber("/emotion_bot/chat/events", String, self.on_event, queue_size=100)

    def on_state(self, message):
        with self.condition:
            self.state = json.loads(message.data)
            self.condition.notify_all()

    def on_event(self, message):
        event = json.loads(message.data)
        with self.condition:
            if event["type"] == "accepted" and self.turn_id is None:
                self.turn_id = event["turn_id"]
            if event.get("turn_id") != self.turn_id:
                return
            if event["type"] == "delta":
                print(event["text"], end="", flush=True)
            elif event["type"] == "completed":
                self.response = event["text"]
                self.response_backend = event.get("backend", "unknown")
                print()
            elif event["type"] == "offline_fallback":
                print("\n[warning] OpenAI unavailable; this turn is using the offline fallback.")
            elif event["type"] == "error":
                self.error = event.get("message", "chat failed")
                print("[error] %s" % self.error)
            self.condition.notify_all()

    def ask(self, text):
        with self.condition:
            previous = self.state["sequence"] if self.state else -1
            self.response = None
            self.response_backend = None
            self.error = None
            self.turn_id = None
            self.publisher.publish(String(data=text))
            deadline = time.monotonic() + 30.0
            while not rospy.is_shutdown():
                if self.error is not None:
                    raise RuntimeError(self.error)
                matching_state = (
                    self.state
                    and self.state["sequence"] > previous
                    and self.state.get("turn_id") == self.turn_id
                    and self.state.get("source") == "assistant"
                )
                if matching_state and self.response is not None:
                    return self.response, self.state, self.response_backend
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("timed out waiting for EmotionBot")
                self.condition.wait(min(0.2, remaining))


def main():
    rospy.init_node("emotion_chat", anonymous=True, disable_signals=True)
    client = ChatClient()
    rospy.sleep(0.5)
    print("EmotionBot ROS streaming chat. :quit exits; a new turn cancels an unfinished one.")
    try:
        while not rospy.is_shutdown():
            text = input("you> ").strip()
            if text.lower() in (":quit", ":q", "exit"):
                break
            if not text:
                continue
            print("bot> ", end="", flush=True)
            try:
                _response, state, chat_backend = client.ask(text)
            except RuntimeError as exc:
                print("status> %s" % exc)
                continue
            print(
                "state> %s valence=%+.3f arousal=%.3f chat_backend=%s emotion_backend=%s"
                % (
                    state["emotion"], state["valence"], state["arousal"],
                    chat_backend, state["backend"],
                )
            )
    except (EOFError, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
