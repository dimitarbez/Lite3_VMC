"""Finite-duration emotion expression patterns."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional

from .contract import EMOTIONS


EXPRESSION_ACTIONS = frozenset(("none", "hop", "stomp"))


@dataclass(frozen=True)
class TwistValue:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0

    def is_zero(self, epsilon: float = 1e-9) -> bool:
        return all(
            abs(value) <= epsilon
            for value in (self.x, self.y, self.yaw, self.z, self.roll, self.pitch)
        )

    def has_locomotion(self, epsilon: float = 1e-9) -> bool:
        return any(abs(value) > epsilon for value in (self.x, self.y, self.yaw))

    def posture_only(self, epsilon: float = 1e-9) -> bool:
        return not self.has_locomotion(epsilon) and any(
            abs(value) > epsilon for value in (self.z, self.roll, self.pitch)
        )

    def without_locomotion(self) -> "TwistValue":
        return TwistValue(z=self.z, roll=self.roll, pitch=self.pitch)


@dataclass(frozen=True)
class Segment:
    duration: float
    twist: TwistValue
    action: str = "none"


@dataclass(frozen=True)
class EmotionProfile:
    """A theatrical entrance followed by a repeatable standing idle loop.

    All fields are deliberately declarative so a designer can tune one emotion
    without changing the safety or ROS transport layers.
    """

    transition: List[Segment]
    idle: List[Segment]
    intensity: float = 1.0
    duration: float = 1.0
    speed: float = 1.0
    acceleration: float = 1.0
    cooldown: float = 0.0
    variation: float = 0.0
    transition_blend: float = 0.18


class MappingError(ValueError):
    pass


def load_patterns(raw: Dict[str, Any]) -> Dict[str, EmotionProfile]:
    if not isinstance(raw, dict):
        raise MappingError("mappings must be a dictionary")
    if set(raw) != set(EMOTIONS):
        missing = sorted(set(EMOTIONS) - set(raw))
        extra = sorted(set(raw) - set(EMOTIONS))
        raise MappingError("mapping keys mismatch; missing=%s extra=%s" % (missing, extra))

    profiles = {}
    for emotion in EMOTIONS:
        profile = raw[emotion] if isinstance(raw[emotion], dict) else None
        segments_raw = profile.get("segments") if profile else None
        if not isinstance(segments_raw, list) or not segments_raw:
            raise MappingError("%s requires at least one segment" % emotion)
        idle_raw = profile.get("idle_segments", segments_raw)
        if not isinstance(idle_raw, list) or not idle_raw:
            raise MappingError("%s requires at least one idle segment" % emotion)

        def parse_segments(items, phase):
            segments = []
            for item in items:
                if not isinstance(item, dict):
                    raise MappingError("%s %s segment must be a dictionary" % (emotion, phase))
                duration = float(item.get("duration", 0.0))
                if not math.isfinite(duration) or duration <= 0.0 or duration > 10.0:
                    raise MappingError("%s duration must be in (0, 10]" % emotion)
                values = {}
                for field in ("x", "y", "yaw", "z", "roll", "pitch"):
                    value = float(item.get(field, 0.0))
                    if not math.isfinite(value):
                        raise MappingError("%s %s must be finite" % (emotion, field))
                    values[field] = value
                action = str(item.get("action", "none"))
                if action not in EXPRESSION_ACTIONS:
                    raise MappingError(
                        "%s action must be one of %s" % (emotion, sorted(EXPRESSION_ACTIONS))
                    )
                segments.append(Segment(duration, TwistValue(**values), action))
            return segments

        transition = parse_segments(segments_raw, "transition")
        idle = parse_segments(idle_raw, "idle")
        values = {}
        for field, default, low, high in (
            ("intensity", 1.0, 0.0, 1.0),
            ("duration", 1.0, 0.10, 5.0),
            ("speed", 1.0, 0.10, 3.0),
            ("acceleration", 1.0, 0.10, 3.0),
            ("cooldown", 0.0, 0.0, 5.0),
            ("variation", 0.0, 0.0, 0.50),
            ("transition_blend", 0.18, 0.01, 2.0),
        ):
            value = float(profile.get(field, default))
            if not math.isfinite(value) or not low <= value <= high:
                raise MappingError("%s %s must be in [%.2f, %.2f]" % (emotion, field, low, high))
            values[field] = value
        profiles[emotion] = EmotionProfile(transition=transition, idle=idle, **values)
    return profiles


class PatternPlayer:
    def __init__(self, patterns: Dict[str, EmotionProfile]):
        self.patterns = patterns
        self.emotion = "neutral"
        self.started_at = 0.0

    def start(self, emotion: str, now: float) -> None:
        if emotion not in self.patterns:
            raise MappingError("unknown emotion: %s" % emotion)
        self.emotion = emotion
        self.started_at = float(now)

    def command(self, now: float) -> TwistValue:
        segment, _index = self.segment(now)
        return segment.twist if segment is not None else TwistValue()

    def segment(self, now: float):
        """Return (segment, index), looping the idle phase forever."""
        profile = self.patterns[self.emotion]
        # A profile's duration scales its entrance while speed controls its
        # cadence.  This keeps individual timing parameters meaningful.
        transition_scale = profile.duration / profile.speed
        elapsed = max(0.0, float(now) - self.started_at)
        for index, segment in enumerate(profile.transition):
            scaled = segment.duration * transition_scale
            if elapsed < scaled:
                return segment, index
            elapsed -= scaled
        idle_total = sum(segment.duration for segment in profile.idle) / profile.speed
        if idle_total <= 0.0:
            return None, -1
        elapsed = math.fmod(elapsed, idle_total)
        for index, segment in enumerate(profile.idle):
            scaled = segment.duration / profile.speed
            if elapsed < scaled:
                return segment, len(profile.transition) + index
            elapsed -= scaled
        return profile.idle[-1], len(profile.transition) + len(profile.idle) - 1


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _smoothstep(value: float) -> float:
    value = _clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def _mix(first: TwistValue, second: TwistValue, amount: float) -> TwistValue:
    return TwistValue(
        first.x + (second.x - first.x) * amount,
        first.y + (second.y - first.y) * amount,
        first.yaw + (second.yaw - first.yaw) * amount,
        first.z + (second.z - first.z) * amount,
        first.roll + (second.roll - first.roll) * amount,
        first.pitch + (second.pitch - first.pitch) * amount,
    )


def _slew(current: TwistValue, target: TwistValue, dt: float, linear_rate: float, yaw_rate: float) -> TwistValue:
    def approach(value: float, desired: float, maximum_delta: float) -> float:
        return value + _clamp(desired - value, -maximum_delta, maximum_delta)

    return TwistValue(
        approach(current.x, target.x, linear_rate * dt),
        approach(current.y, target.y, linear_rate * dt),
        approach(current.yaw, target.yaw, yaw_rate * dt),
        approach(current.z, target.z, linear_rate * dt),
        approach(current.roll, target.roll, yaw_rate * dt),
        approach(current.pitch, target.pitch, yaw_rate * dt),
    )


class FluidExpressionController:
    """Filter affect and cross-fade finite categorical gestures at a fixed rate."""

    def __init__(
        self,
        patterns: Dict[str, EmotionProfile],
        valence_tau: float = 0.45,
        arousal_tau: float = 0.35,
        blend_time: float = 0.18,
        neutral_return_time: float = 0.35,
        min_dwell: float = 0.30,
        hysteresis: float = 0.06,
        min_intensity: float = 0.55,
        max_linear_rate: float = 0.35,
        max_yaw_rate: float = 1.50,
    ):
        self.player = PatternPlayer(patterns)
        self.valence_tau = max(0.001, float(valence_tau))
        self.arousal_tau = max(0.001, float(arousal_tau))
        self.blend_time = max(0.01, float(blend_time))
        self.neutral_return_time = max(0.01, float(neutral_return_time))
        self.min_dwell = max(0.0, float(min_dwell))
        self.hysteresis = max(0.0, float(hysteresis))
        self.min_intensity = _clamp(min_intensity, 0.0, 1.0)
        self.max_linear_rate = max(0.001, float(max_linear_rate))
        self.max_yaw_rate = max(0.001, float(max_yaw_rate))
        self.filtered_valence = 0.0
        self.filtered_arousal = 0.2
        self.target_valence = 0.0
        self.target_arousal = 0.2
        self.selected_emotion = "neutral"
        self.selected_since = 0.0
        self.pending_emotion: Optional[str] = None
        self.pending_since = 0.0
        self.pending_valence = 0.0
        self.pending_arousal = 0.2
        self.last_at: Optional[float] = None
        self.output = TwistValue()
        self.blend_from = TwistValue()
        self.blend_started = 0.0
        self.last_action_marker = None

    def update(self, emotion: str, valence: float, arousal: float, now: float) -> None:
        if emotion not in self.player.patterns:
            raise MappingError("unknown emotion: %s" % emotion)
        valence = _clamp(valence, -1.0, 1.0)
        arousal = _clamp(arousal, 0.0, 1.0)
        # Dynamic expressions are short, one-shot reactions.  Do not let a
        # low simulated-time dwell be overtaken by a fast streamed assistant
        # reply before the robot gets a chance to react to the user.
        if (
            emotion != self.selected_emotion
            and self.player.patterns[emotion].transition[0].action != "none"
        ):
            self._select(emotion, valence, arousal, float(now))
            self.pending_emotion = None
            return
        if emotion == self.selected_emotion:
            affect_change = max(
                abs(valence - self.target_valence),
                abs(arousal - self.target_arousal),
            )
            self.target_valence = valence
            self.target_arousal = arousal
            if emotion != "neutral":
                cooldown = max(self.min_dwell, self.player.patterns[emotion].cooldown)
                # Every non-heartbeat state is a deliberate emotional event.
                # Repeating an emotion after its cooldown therefore gives a
                # readable replay, while rapid repeats blend into the active
                # gesture rather than hammering the simulator.
                if now - self.selected_since >= cooldown:
                    self._select(emotion, valence, arousal, float(now))
                    self.pending_emotion = None
                elif affect_change >= self.hysteresis:
                    self.pending_emotion = emotion
                    self.pending_since = float(now)
                    self.pending_valence = valence
                    self.pending_arousal = arousal
            return
        # Emotion changes interrupt immediately. The output cross-fade retains
        # continuity, while each newly selected profile gets its readable
        # entrance reaction rather than being swallowed by a dwell timer.
        self._select(emotion, valence, arousal, float(now))
        self.pending_emotion = None

    def force_neutral(self, now: float) -> None:
        if self.selected_emotion != "neutral" or self.pending_emotion is not None:
            self._select("neutral", 0.0, 0.2, float(now))
        self.pending_emotion = None

    def _select(self, emotion: str, valence: float, arousal: float, now: float) -> None:
        self.blend_from = self.output
        self.blend_started = now
        self.selected_emotion = emotion
        self.selected_since = now
        self.target_valence = valence
        self.target_arousal = arousal
        self.player.start(emotion, now)
        self.last_action_marker = None

    def _maybe_select_pending(self, now: float) -> None:
        if self.pending_emotion is None:
            return
        cooldown = max(self.min_dwell, self.player.patterns[self.pending_emotion].cooldown)
        if now - self.pending_since < cooldown:
            return
        if self.selected_since and now - self.selected_since < cooldown:
            return
        self._select(
            self.pending_emotion,
            self.pending_valence,
            self.pending_arousal,
            now,
        )
        self.pending_emotion = None

    def command(self, now: float, stale: bool = False) -> TwistValue:
        now = float(now)
        if self.last_at is None:
            self.last_at = now
        dt = _clamp(now - self.last_at, 0.0, 0.25)
        self.last_at = now
        if stale:
            self.force_neutral(now)
        else:
            self._maybe_select_pending(now)

        valence_alpha = 1.0 - math.exp(-dt / self.valence_tau)
        arousal_alpha = 1.0 - math.exp(-dt / self.arousal_tau)
        self.filtered_valence += (self.target_valence - self.filtered_valence) * valence_alpha
        self.filtered_arousal += (self.target_arousal - self.filtered_arousal) * arousal_alpha

        profile = self.player.patterns[self.selected_emotion]
        raw = self.player.command(now)
        intensity = _clamp(
            profile.intensity * (self.min_intensity + (1.0 - self.min_intensity) * self.filtered_arousal),
            0.0,
            1.0,
        )
        # Deterministic, phase-continuous variation prevents a held emotion
        # from looking like a frozen pose without adding random motion.
        variation = 1.0 + profile.variation * math.sin((now - self.player.started_at) * 2.0 * math.pi * profile.speed)
        target = TwistValue(
            x=raw.x * intensity,
            y=raw.y * intensity,
            yaw=raw.yaw * intensity,
            z=raw.z * intensity * variation,
            roll=raw.roll * intensity * variation,
            pitch=raw.pitch * intensity * variation,
        )
        base_blend = self.neutral_return_time if self.selected_emotion == "neutral" else profile.transition_blend
        duration = base_blend / (0.75 + 0.50 * intensity)
        blend = _smoothstep((now - self.blend_started) / max(duration, 0.01))
        eased = _mix(self.blend_from, target, blend)
        self.output = _slew(
            self.output,
            eased,
            dt,
            self.max_linear_rate * profile.acceleration,
            self.max_yaw_rate * profile.acceleration,
        )
        if target.is_zero(1e-7) and self.output.is_zero(1e-4):
            self.output = TwistValue()
        return self.output

    def consume_action(self, now: float, stale: bool = False) -> str:
        """Return a one-shot in-place action for the active pattern segment.

        This is intentionally separate from the Twist posture intention: a hop
        or stomp is a finite simulator action, never x/y/yaw locomotion.
        """
        if stale:
            return "none"
        segment, index = self.player.segment(now)
        if segment is None or segment.action == "none":
            return "none"
        marker = (self.player.started_at, index, segment.action)
        if marker == self.last_action_marker:
            return "none"
        self.last_action_marker = marker
        return segment.action
