#!/usr/bin/env python3
"""Offline process-boundary check for the Python 3.12 streaming sidecar."""

import os
import time

from emotion_bot_ros.conversation import ConversationCoordinator, OpenAIResponsesBackend


def test_loopback_sidecar_streams_and_completes_current_turn():
    events = []
    backend = OpenAIResponsesBackend(
        model="test-model",
        timeout=3.0,
        max_output_tokens=32,
        endpoint=os.environ.get("OPENAI_BRIDGE_ENDPOINT", "http://127.0.0.1:8765/v1/stream"),
    )
    coordinator = ConversationCoordinator(backend, events.append, max_retries=0)
    turn_id = coordinator.submit("test input")
    deadline = time.monotonic() + 5.0
    while not any(item["type"] in ("completed", "error") for item in events) and time.monotonic() < deadline:
        time.sleep(0.01)
    completed = [item for item in events if item["type"] == "completed"]
    assert len(completed) == 1
    assert completed[0]["turn_id"] == turn_id
    assert completed[0]["text"] == "offline bridge test"
    assert len([item for item in events if item["type"] == "delta"]) == 2
    coordinator.shutdown()
