"""Strict NDJSON transport for forwarding emotion state to a robot host."""

from __future__ import annotations

import json
import uuid
from typing import Any, Dict

from emotion_bot_ros.contract import validate_state

TRANSPORT_SCHEMA_VERSION = "1.0"
MAX_FRAME_BYTES = 2048


class TransportError(ValueError):
    pass


def new_session_id() -> str:
    return str(uuid.uuid4())


def build_envelope(session_id: str, sequence: int, state: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(session_id, str) or not session_id:
        raise TransportError("session_id must be non-empty")
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
        raise TransportError("transport sequence must be positive")
    validate_state(state)
    return {
        "schema_version": TRANSPORT_SCHEMA_VERSION,
        "session_id": session_id,
        "sequence": sequence,
        "payload": state,
    }


def encode_envelope(envelope: Dict[str, Any]) -> bytes:
    validate_envelope(envelope)
    encoded = (json.dumps(envelope, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    if len(encoded) > MAX_FRAME_BYTES:
        raise TransportError("frame exceeds %d bytes" % MAX_FRAME_BYTES)
    return encoded


def decode_envelope(frame: bytes) -> Dict[str, Any]:
    if not isinstance(frame, bytes) or not frame or len(frame) > MAX_FRAME_BYTES:
        raise TransportError("invalid frame length")
    if not frame.endswith(b"\n") or b"\n" in frame[:-1]:
        raise TransportError("frame must be one newline-terminated JSON object")
    try:
        envelope = json.loads(frame[:-1].decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise TransportError("malformed NDJSON frame") from exc
    return validate_envelope(envelope)


def validate_envelope(envelope: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(envelope, dict):
        raise TransportError("envelope must be an object")
    if set(envelope) != {"schema_version", "session_id", "sequence", "payload"}:
        raise TransportError("unexpected envelope fields")
    if envelope.get("schema_version") != TRANSPORT_SCHEMA_VERSION:
        raise TransportError("unsupported transport schema")
    if not isinstance(envelope.get("session_id"), str) or not envelope["session_id"]:
        raise TransportError("session_id must be non-empty")
    sequence = envelope.get("sequence")
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1:
        raise TransportError("transport sequence must be positive")
    validate_state(envelope.get("payload"))
    return envelope


class SequenceGate:
    """Accept a new session, then only strictly increasing transport sequences."""

    def __init__(self):
        self.session_id = None
        self.sequence = 0

    def accept(self, envelope: Dict[str, Any]) -> bool:
        validate_envelope(envelope)
        session_id = envelope["session_id"]
        sequence = envelope["sequence"]
        if self.session_id != session_id:
            self.session_id = session_id
            self.sequence = sequence
            return True
        if sequence <= self.sequence:
            return False
        self.sequence = sequence
        return True
