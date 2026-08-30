#!/usr/bin/env python3
"""Asynchronous streaming chat boundary for EmotionBot ROS."""

import json

import rospy
from std_msgs.msg import String

from emotion_bot_ros.conversation import (
    ConversationCoordinator,
    ConversationError,
    DeterministicChatBackend,
    OpenAIResponsesBackend,
)


class ChatAdapter:
    def __init__(self):
        topics = rospy.get_param("/emotion_bot/topics")
        cfg = rospy.get_param("/emotion_bot/chat")
        backend_name = str(cfg.get("backend", "deterministic"))
        deterministic = DeterministicChatBackend()
        if backend_name == "openai":
            backend = OpenAIResponsesBackend(
                model=str(cfg.get("model", "gpt-5-mini")),
                timeout=float(cfg.get("request_timeout", 20.0)),
                max_output_tokens=int(cfg.get("max_output_tokens", 180)),
                endpoint=str(cfg.get("bridge_endpoint", "http://127.0.0.1:8765/v1/stream")),
            )
            fallback = deterministic if bool(cfg.get("offline_fallback", True)) else None
        elif backend_name == "deterministic":
            backend = deterministic
            fallback = None
        else:
            raise RuntimeError("unsupported chat backend")
        self.backend_name = backend_name
        self.events_pub = rospy.Publisher(topics["chat_events"], String, queue_size=100)
        self.response_pub = rospy.Publisher(topics["chat_response"], String, queue_size=10)
        self.conversation_pub = rospy.Publisher(topics["conversation_events"], String, queue_size=20)
        self.turn_index_param = "/emotion_bot/chat/last_turn_index"
        self.coordinator = ConversationCoordinator(
            backend=backend,
            fallback_backend=fallback,
            emit=self.on_event,
            max_turns=int(cfg.get("max_context_turns", 6)),
            max_context_chars=int(cfg.get("max_context_chars", 6000)),
            max_input_chars=int(cfg.get("max_input_chars", 2000)),
            max_response_chars=int(cfg.get("max_response_chars", 6000)),
            max_retries=int(cfg.get("max_retries", 1)),
            retry_delay=float(cfg.get("retry_delay", 0.25)),
            emotion_sync_timeout=float(cfg.get("emotion_sync_timeout", 0.5)),
            initial_turn_index=int(rospy.get_param(self.turn_index_param, 0)),
        )
        self.input_sub = rospy.Subscriber(topics["chat_input"], String, self.on_input, queue_size=10)
        self.cancel_sub = rospy.Subscriber(topics["chat_cancel"], String, self.on_cancel, queue_size=10)
        self.state_sub = rospy.Subscriber(topics["state"], String, self.on_state, queue_size=10)
        rospy.on_shutdown(self.coordinator.shutdown)

    @staticmethod
    def _dump(event):
        return json.dumps(event, sort_keys=True, separators=(",", ":"))

    def on_event(self, event):
        if event["type"] == "accepted":
            rospy.set_param(self.turn_index_param, int(event["turn_index"]))
        payload = self._dump(event)
        self.events_pub.publish(String(data=payload))
        if event["type"] in ("accepted", "completed", "cancelled", "error"):
            self.conversation_pub.publish(String(data=payload))
        if event["type"] == "completed":
            self.response_pub.publish(String(data=payload))

    def on_input(self, message):
        text = message.data
        turn_id = None
        try:
            decoded = json.loads(message.data)
            if isinstance(decoded, dict) and "text" in decoded:
                text = decoded["text"]
                turn_id = decoded.get("turn_id")
        except (TypeError, ValueError):
            pass
        try:
            self.coordinator.submit(text, turn_id=turn_id)
        except ConversationError as exc:
            rospy.logwarn("Chat input rejected safely: %s", exc)

    def on_cancel(self, message):
        self.coordinator.cancel(message.data.strip() or None)

    def on_state(self, message):
        try:
            state = json.loads(message.data)
            self.coordinator.set_emotion(
                state.get("emotion", "neutral"),
                turn_id=state.get("turn_id"),
            )
        except (TypeError, ValueError):
            pass


def main():
    rospy.init_node("chat_adapter")
    adapter = ChatAdapter()
    rospy.loginfo("Chat adapter ready (backend=%s)", adapter.backend_name)
    rospy.spin()


if __name__ == "__main__":
    main()
