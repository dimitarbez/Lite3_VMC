"""Versioned JSON transport contract for emotion state."""

from __future__ import annotations

import json
import math
from typing import Any, Dict

SCHEMA_VERSION = "1.0"
EMOTIONS = (
    "neutral",
    "joy",
    "sadness",
    "anger",
    "fear",
    "surprise",
    "disgust",
    "curiosity",
    "affection",
)


class ContractError(ValueError):
    pass


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError("%s must be numeric" % field)
    value = float(value)
    if not math.isfinite(value):
        raise ContractError("%s must be finite" % field)
    return value


def build_state(
    ros_time: Any,
    sequence: int,
    emotion: str,
    valence: float,
    arousal: float,
    backend: str,
    source: str,
) -> Dict[str, Any]:
    state = {
        "schema_version": SCHEMA_VERSION,
        "stamp": {"secs": int(ros_time.secs), "nsecs": int(ros_time.nsecs)},
        "sequence": int(sequence),
        "emotion": emotion,
        "valence": float(valence),
        "arousal": float(arousal),
        "backend": str(backend),
        "source": str(source),
    }
    validate_state(state)
    return state


def validate_state(state: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(state, dict):
        raise ContractError("state must be a JSON object")
    if state.get("schema_version") != SCHEMA_VERSION:
        raise ContractError("unsupported schema_version")
    stamp = state.get("stamp")
    if not isinstance(stamp, dict):
        raise ContractError("stamp must be an object")
    secs = stamp.get("secs")
    nsecs = stamp.get("nsecs")
    if not isinstance(secs, int) or isinstance(secs, bool) or secs < 0:
        raise ContractError("stamp.secs must be a non-negative integer")
    if not isinstance(nsecs, int) or isinstance(nsecs, bool) or not 0 <= nsecs < 1000000000:
        raise ContractError("stamp.nsecs is outside ROS bounds")
    sequence = state.get("sequence")
    if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
        raise ContractError("sequence must be a non-negative integer")
    if state.get("emotion") not in EMOTIONS:
        raise ContractError("unknown emotion")
    valence = _finite_number(state.get("valence"), "valence")
    arousal = _finite_number(state.get("arousal"), "arousal")
    if not -1.0 <= valence <= 1.0:
        raise ContractError("valence outside [-1, 1]")
    if not 0.0 <= arousal <= 1.0:
        raise ContractError("arousal outside [0, 1]")
    for field in ("backend", "source"):
        if not isinstance(state.get(field), str) or not state[field]:
            raise ContractError("%s must be a non-empty string" % field)
    return state


def dumps_state(state: Dict[str, Any]) -> str:
    validate_state(state)
    return json.dumps(state, sort_keys=True, separators=(",", ":"))


def loads_state(payload: str) -> Dict[str, Any]:
    try:
        state = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise ContractError("malformed state JSON") from exc
    return validate_state(state)
