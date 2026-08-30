#!/usr/bin/env python3
"""ROS adapter for a reusable headless EmotionBot session."""

import json
import os
import sys
import threading

import rospy
from std_msgs.msg import String

from emotion_bot_ros.contract import build_state, dumps_state
from emotion_bot_ros.conversation import ConversationError, TurnGate, loads_conversation_event


class EmotionAdapter:
    def __init__(self):
        emotion_bot_path = rospy.get_param("/emotion_bot/runtime/emotion_bot_path", "/workspaces/emotion-bot")
        if not os.path.isdir(os.path.join(emotion_bot_path, "emotional_core")):
            raise RuntimeError("emotion-bot checkout is unavailable at %s" % emotion_bot_path)
        if emotion_bot_path not in sys.path:
            sys.path.insert(0, emotion_bot_path)

        from emotional_core.engine import EmotionEngine

        self.backend = rospy.get_param("/emotion_bot/runtime/backend", "deterministic")
        self.engine = EmotionEngine(
            personality_type=rospy.get_param("/emotion_bot/runtime/personality", "balanced"),
            backend=self.backend,
            seed=int(rospy.get_param("/emotion_bot/runtime/seed", 7)),
            randomness_enabled=bool(rospy.get_param("/emotion_bot/runtime/randomness_enabled", False)),
        )
        self.sequence = 0
        self.turn_id = "system"
        self.turn_gate = TurnGate()
        self.lock = threading.Lock()
        state_topic = rospy.get_param("/emotion_bot/topics/state", "/emotion_bot/state")
        response_topic = rospy.get_param("/emotion_bot/topics/response", "/emotion_bot/response")
        input_topic = rospy.get_param("/emotion_bot/topics/input", "/emotion_bot/input")
        conversation_topic = rospy.get_param(
            "/emotion_bot/topics/conversation_events", "/emotion_bot/conversation/events"
        )
        self.state_pub = rospy.Publisher(state_topic, String, queue_size=10, latch=True)
        self.response_pub = rospy.Publisher(response_topic, String, queue_size=10)
        self.input_sub = rospy.Subscriber(input_topic, String, self.on_input, queue_size=10)
        self.conversation_sub = rospy.Subscriber(
            conversation_topic, String, self.on_conversation_event, queue_size=20
        )
        self.publish_snapshot(source="startup")
        heartbeat_rate = float(rospy.get_param("/emotion_bot/runtime/heartbeat_rate", 2.0))
        self.heartbeat = rospy.Timer(rospy.Duration(1.0 / heartbeat_rate), self.on_heartbeat)

    def publish_snapshot(self, source=None):
        snapshot = self.engine.snapshot()
        state = build_state(
            rospy.Time.now(),
            self.sequence,
            snapshot.emotion,
            snapshot.valence,
            snapshot.arousal,
            self.backend,
            source or snapshot.source,
            self.turn_id,
        )
        self.state_pub.publish(String(data=dumps_state(state)))

    def on_heartbeat(self, _event):
        with self.lock:
            self.publish_snapshot(source="heartbeat")

    def on_input(self, message):
        with self.lock:
            try:
                self.turn_id = "legacy-%06d" % (self.sequence + 1)
                result = self.engine.process(message.data, now=rospy.Time.now().to_sec())
                self.sequence += 1
                state = build_state(
                    rospy.Time.now(),
                    self.sequence,
                    result.emotion,
                    result.valence,
                    result.arousal,
                    self.backend,
                    result.source,
                    self.turn_id,
                )
                self.state_pub.publish(String(data=dumps_state(state)))
                self.response_pub.publish(String(data=result.response))
            except Exception as exc:
                rospy.logerr("Emotion input rejected safely: %s", exc)
                self.response_pub.publish(String(data="Input rejected: %s" % exc))

    def on_conversation_event(self, message):
        try:
            event = loads_conversation_event(message.data)
        except ConversationError as exc:
            rospy.logwarn("Conversation event rejected safely: %s", exc)
            return
        with self.lock:
            if not self.turn_gate.accepts(event):
                return
            try:
                result = self.engine.process(event["text"], now=rospy.Time.now().to_sec())
                self.sequence += 1
                self.turn_id = event["turn_id"]
                source = "user" if event["type"] == "accepted" else "assistant"
                state = build_state(
                    rospy.Time.now(),
                    self.sequence,
                    result.emotion,
                    result.valence,
                    result.arousal,
                    self.backend,
                    source,
                    self.turn_id,
                )
                self.state_pub.publish(String(data=dumps_state(state)))
            except Exception as exc:
                rospy.logerr("Conversation appraisal rejected safely: %s", exc)


def main():
    rospy.init_node("adapter")
    EmotionAdapter()
    rospy.loginfo("EmotionBot adapter ready (backend=%s)", rospy.get_param("/emotion_bot/runtime/backend", "deterministic"))
    rospy.spin()


if __name__ == "__main__":
    main()
