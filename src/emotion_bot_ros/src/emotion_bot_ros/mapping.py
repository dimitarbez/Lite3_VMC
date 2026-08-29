"""Finite-duration emotion expression patterns."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional

from .contract import EMOTIONS


@dataclass(frozen=True)
class TwistValue:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def is_zero(self, epsilon: float = 1e-9) -> bool:
        return abs(self.x) <= epsilon and abs(self.y) <= epsilon and abs(self.yaw) <= epsilon


@dataclass(frozen=True)
class Segment:
    duration: float
    twist: TwistValue


class MappingError(ValueError):
    pass


def load_patterns(raw: Dict[str, Any]) -> Dict[str, List[Segment]]:
    if not isinstance(raw, dict):
        raise MappingError("mappings must be a dictionary")
    if set(raw) != set(EMOTIONS):
        missing = sorted(set(EMOTIONS) - set(raw))
        extra = sorted(set(raw) - set(EMOTIONS))
        raise MappingError("mapping keys mismatch; missing=%s extra=%s" % (missing, extra))

    patterns = {}
    for emotion in EMOTIONS:
        segments_raw = raw[emotion].get("segments") if isinstance(raw[emotion], dict) else None
        if not isinstance(segments_raw, list) or not segments_raw:
            raise MappingError("%s requires at least one segment" % emotion)
        segments = []
        for item in segments_raw:
            if not isinstance(item, dict):
                raise MappingError("%s segment must be a dictionary" % emotion)
            duration = float(item.get("duration", 0.0))
            if not math.isfinite(duration) or duration <= 0.0 or duration > 10.0:
                raise MappingError("%s duration must be in (0, 10]" % emotion)
            values = []
            for field in ("x", "y", "yaw"):
                value = float(item.get(field, 0.0))
                if not math.isfinite(value):
                    raise MappingError("%s %s must be finite" % (emotion, field))
                values.append(value)
            segments.append(Segment(duration, TwistValue(*values)))
        patterns[emotion] = segments
    return patterns


class PatternPlayer:
    def __init__(self, patterns: Dict[str, List[Segment]]):
        self.patterns = patterns
        self.emotion = "neutral"
        self.started_at = 0.0

    def start(self, emotion: str, now: float) -> None:
        if emotion not in self.patterns:
            raise MappingError("unknown emotion: %s" % emotion)
        self.emotion = emotion
        self.started_at = float(now)

    def command(self, now: float) -> TwistValue:
        elapsed = max(0.0, float(now) - self.started_at)
        for segment in self.patterns[self.emotion]:
            if elapsed < segment.duration:
                return segment.twist
            elapsed -= segment.duration
        return TwistValue()
