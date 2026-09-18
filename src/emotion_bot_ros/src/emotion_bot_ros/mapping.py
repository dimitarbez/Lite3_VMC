"""Smooth entrance and looping idle patterns for emotion expressions."""

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
    def __init__(
        self,
        patterns: Dict[str, EmotionProfile],
        idle_amplitude_scale: float = 1.0,
        idle_time_scale: float = 1.0,
    ):
        self.patterns = patterns
        self.idle_amplitude_scale = _clamp(idle_amplitude_scale, 0.1, 1.0)
        self.idle_time_scale = _clamp(idle_time_scale, 1.0, 3.0)
        self.emotion = "neutral"
        self.started_at = 0.0

    def start(self, emotion: str, now: float) -> None:
        if emotion not in self.patterns:
            raise MappingError("unknown emotion: %s" % emotion)
        self.emotion = emotion
        self.started_at = float(now)

    def command(self, now: float) -> TwistValue:
        value, _segment, _index, _phase, _cycle = self.sample(now)
        return value

    def segment(self, now: float):
        """Return the active segment and flattened index."""
        _value, segment, index, _phase, _cycle = self.sample(now)
        return segment, index

    def sample(self, now: float):
        """Return an eased pose and cycle-aware segment occurrence.

        Segment twists are keyframe endpoints. A segment duration is the
        travel time from the preceding endpoint. Quintic interpolation keeps
        velocity and acceleration continuous at every keyframe.
        """
        profile = self.patterns[self.emotion]
        transition_scale = profile.duration / profile.speed
        elapsed = max(0.0, float(now) - self.started_at)
        for index, segment in enumerate(profile.transition):
            scaled = segment.duration * transition_scale
            if elapsed < scaled:
                previous = profile.transition[index - 1].twist if index else TwistValue()
                amount = _smootherstep(elapsed / max(scaled, 1e-9))
                return _mix(previous, segment.twist, amount), segment, index, "transition", 0
            elapsed -= scaled
        idle_total = (
            sum(segment.duration for segment in profile.idle)
            * self.idle_time_scale
            / profile.speed
        )
        if idle_total <= 0.0:
            return TwistValue(), None, -1, "idle", 0
        cycle = int(math.floor(elapsed / idle_total))
        elapsed = math.fmod(elapsed, idle_total)
        for index, segment in enumerate(profile.idle):
            scaled = segment.duration * self.idle_time_scale / profile.speed
            if elapsed < scaled:
                target = _scale_twist(segment.twist, self.idle_amplitude_scale)
                if index:
                    previous = _scale_twist(
                        profile.idle[index - 1].twist, self.idle_amplitude_scale
                    )
                elif cycle:
                    previous = _scale_twist(
                        profile.idle[-1].twist, self.idle_amplitude_scale
                    )
                else:
                    previous = profile.transition[-1].twist
                amount = _smootherstep(elapsed / max(scaled, 1e-9))
                return (
                    _mix(previous, target, amount),
                    segment,
                    len(profile.transition) + index,
                    "idle",
                    cycle,
                )
            elapsed -= scaled
        segment = profile.idle[-1]
        return (
            _scale_twist(segment.twist, self.idle_amplitude_scale),
            segment,
            len(profile.transition) + len(profile.idle) - 1,
            "idle",
            cycle,
        )


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _smootherstep(value: float) -> float:
    value = _clamp(value, 0.0, 1.0)
    return value * value * value * (value * (value * 6.0 - 15.0) + 10.0)


def _mix(first: TwistValue, second: TwistValue, amount: float) -> TwistValue:
    return TwistValue(
        first.x + (second.x - first.x) * amount,
        first.y + (second.y - first.y) * amount,
        first.yaw + (second.yaw - first.yaw) * amount,
        first.z + (second.z - first.z) * amount,
        first.roll + (second.roll - first.roll) * amount,
        first.pitch + (second.pitch - first.pitch) * amount,
    )


def _scale_twist(value: TwistValue, amount: float) -> TwistValue:
    return TwistValue(
        x=value.x * amount,
        y=value.y * amount,
        yaw=value.yaw * amount,
        z=value.z * amount,
        roll=value.roll * amount,
        pitch=value.pitch * amount,
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
    """Filter affect and cross-fade continuous categorical gestures."""

    def __init__(
        self,
        patterns: Dict[str, EmotionProfile],
        valence_tau: float = 0.45,
        arousal_tau: float = 0.35,
        blend_time: float = 0.18,
        neutral_return_time: float = 0.35,
        neutral_hold_time: float = 0.0,
        min_dwell: float = 0.30,
        hysteresis: float = 0.06,
        min_intensity: float = 0.55,
        amplitude_scale: float = 1.0,
        idle_amplitude_scale: float = 1.0,
        idle_time_scale: float = 1.0,
        max_linear_rate: float = 0.35,
        max_yaw_rate: float = 1.50,
    ):
        self.player = PatternPlayer(
            patterns,
            idle_amplitude_scale=idle_amplitude_scale,
            idle_time_scale=idle_time_scale,
        )
        self.valence_tau = max(0.001, float(valence_tau))
        self.arousal_tau = max(0.001, float(arousal_tau))
        self.blend_time = max(0.01, float(blend_time))
        self.neutral_return_time = max(0.01, float(neutral_return_time))
        self.neutral_hold_time = max(0.0, float(neutral_hold_time))
        self.min_dwell = max(0.0, float(min_dwell))
        self.hysteresis = max(0.0, float(hysteresis))
        self.min_intensity = _clamp(min_intensity, 0.0, 1.0)
        self.amplitude_scale = _clamp(amplitude_scale, 0.1, 2.0)
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
        self.generation = 0
        self.cancel_pending = False
        self.inactive = False
        self.inactive_from = TwistValue()
        self.transition_emotion: Optional[str] = None
        self.transition_valence = 0.0
        self.transition_arousal = 0.2
        self.transition_started = 0.0
        self.transition_from = TwistValue()
        self.neutral_reached_at: Optional[float] = None

    def update(self, emotion: str, valence: float, arousal: float, now: float) -> None:
        if emotion not in self.player.patterns:
            raise MappingError("unknown emotion: %s" % emotion)
        valence = _clamp(valence, -1.0, 1.0)
        arousal = _clamp(arousal, 0.0, 1.0)
        now = float(now)
        if self.inactive:
            self._select(emotion, valence, arousal, now)
            self.pending_emotion = None
            return
        if self.transition_emotion is not None:
            if emotion == "neutral":
                self._select("neutral", valence, arousal, now)
            else:
                self._begin_neutral_transition(emotion, valence, arousal, now)
            self.pending_emotion = None
            return
        if emotion == self.selected_emotion:
            # Conversation turns commonly publish a user appraisal followed by
            # an assistant appraisal.  If both resolve to the same category,
            # update its intensity without restarting/cancelling a loop (and,
            # for anger, without firing a second entrance stomp).
            self.target_valence = valence
            self.target_arousal = arousal
            self.pending_emotion = None
            return
        # Reset through a short, explicit neutral pose before changing from one
        # non-neutral choreography to another.  Besides making the transition
        # readable, this gives a cancelled hop/stomp time to regain four-foot
        # support before a new discrete action is eligible.
        if self.selected_emotion != "neutral" and emotion != "neutral":
            self._begin_neutral_transition(emotion, valence, arousal, now)
        else:
            self._select(emotion, valence, arousal, now)
        self.pending_emotion = None

    def force_neutral(self, now: float) -> None:
        if self.selected_emotion != "neutral" or self.pending_emotion is not None:
            self._select("neutral", 0.0, 0.2, float(now))
        self.pending_emotion = None

    def _set_inactive(self, now: float) -> None:
        if self.inactive:
            return
        self.inactive = True
        self.inactive_from = self.output
        self.blend_started = float(now)
        self.pending_emotion = None
        self.transition_emotion = None
        self.neutral_reached_at = None
        self.generation += 1
        self.cancel_pending = True
        self.last_action_marker = None

    def _select(self, emotion: str, valence: float, arousal: float, now: float) -> None:
        self.blend_from = self.output
        self.blend_started = now
        self.selected_emotion = emotion
        self.selected_since = now
        self.target_valence = valence
        self.target_arousal = arousal
        self.player.start(emotion, now)
        self.last_action_marker = None
        self.generation += 1
        self.cancel_pending = True
        self.inactive = False
        self.transition_emotion = None
        self.neutral_reached_at = None

    def _begin_neutral_transition(
        self, emotion: str, valence: float, arousal: float, now: float
    ) -> None:
        """Cancel the old gesture and move to exact neutral before ``emotion``."""
        if (
            self.transition_emotion == emotion
            and valence == self.transition_valence
            and arousal == self.transition_arousal
        ):
            return
        self.transition_emotion = emotion
        self.transition_valence = valence
        self.transition_arousal = arousal
        self.transition_started = now
        self.transition_from = self.output
        self.neutral_reached_at = None
        self.pending_emotion = None
        self.generation += 1
        self.cancel_pending = True
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
            self._set_inactive(now)
            amount = _smootherstep(
                (now - self.blend_started) / max(self.neutral_return_time, 0.01)
            )
            target = _mix(self.inactive_from, TwistValue(), amount)
            self.output = _slew(
                self.output,
                target,
                dt,
                self.max_linear_rate,
                self.max_yaw_rate,
            )
            if amount >= 1.0 or self.output.is_zero(1e-4):
                self.output = TwistValue()
            return self.output

        if self.transition_emotion is not None:
            amount = _smootherstep(
                (now - self.transition_started) / max(self.neutral_return_time, 0.01)
            )
            target = _mix(self.transition_from, TwistValue(), amount)
            self.output = _slew(
                self.output,
                target,
                dt,
                self.max_linear_rate,
                self.max_yaw_rate,
            )
            if amount < 1.0 or not self.output.is_zero(1e-4):
                return self.output
            self.output = TwistValue()
            if self.neutral_reached_at is None:
                self.neutral_reached_at = now
            if now - self.neutral_reached_at < self.neutral_hold_time:
                return self.output
            emotion = self.transition_emotion
            valence = self.transition_valence
            arousal = self.transition_arousal
            self._select(emotion, valence, arousal, now)

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
        target = TwistValue(
            x=raw.x * intensity * self.amplitude_scale,
            y=raw.y * intensity * self.amplitude_scale,
            yaw=raw.yaw * intensity * self.amplitude_scale,
            z=raw.z * intensity * self.amplitude_scale,
            roll=raw.roll * intensity * self.amplitude_scale,
            pitch=raw.pitch * intensity * self.amplitude_scale,
        )
        duration = profile.transition_blend
        blend = _smootherstep((now - self.blend_started) / max(duration, 0.01))
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

    def consume_action(self, now: float, stale: bool = False):
        """Return one generation-aware action command, or ``None``."""
        if stale:
            self._set_inactive(float(now))
        if self.cancel_pending:
            self.cancel_pending = False
            return {
                "schema_version": "1.0",
                "kind": "cancel",
                "generation": self.generation,
            }
        if stale or self.inactive:
            return None
        if self.transition_emotion is not None:
            return None
        _value, segment, index, phase, cycle = self.player.sample(now)
        if segment is None or segment.action == "none":
            return None
        marker = (self.generation, phase, cycle, index, segment.action)
        if marker == self.last_action_marker:
            return None
        self.last_action_marker = marker
        return {
            "schema_version": "1.0",
            "kind": "start",
            "action": segment.action,
            "emotion": self.selected_emotion,
            "generation": self.generation,
            "occurrence_id": "%s:%d:%d" % (phase, cycle, index),
        }
