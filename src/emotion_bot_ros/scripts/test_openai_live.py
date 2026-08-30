#!/usr/bin/env python3
"""Minimal live Responses API streaming smoke test with metadata-only output."""

import argparse
import json
import sys
import time

from emotion_bot_ros.conversation import ConversationCoordinator, OpenAIResponsesBackend


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="gpt-5-mini")
    parser.add_argument("--endpoint", default="http://127.0.0.1:8765/v1/stream")
    args = parser.parse_args()
    events = []
    coordinator = ConversationCoordinator(
        OpenAIResponsesBackend(args.model, 20.0, 64, args.endpoint),
        events.append,
        max_retries=0,
    )
    started = time.monotonic()
    turn_id = coordinator.submit("Say a brief, friendly hello in one sentence.", turn_id="live-smoke")
    deadline = time.monotonic() + 30.0
    while not any(item["type"] in ("completed", "error") for item in events) and time.monotonic() < deadline:
        time.sleep(0.02)
    coordinator.shutdown()
    completed = [item for item in events if item["type"] == "completed"]
    deltas = [item for item in events if item["type"] == "delta"]
    if len(completed) != 1 or not deltas or completed[0].get("backend") != "openai":
        print("OPENAI_LIVE_SMOKE_FAIL", file=sys.stderr)
        return 1
    print(
        "OPENAI_LIVE_SMOKE_PASS "
        + json.dumps(
            {
                "turn_id": turn_id,
                "model": args.model,
                "stream_events": len(deltas),
                "response_chars": len(completed[0]["text"]),
                "elapsed_seconds": round(time.monotonic() - started, 3),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
