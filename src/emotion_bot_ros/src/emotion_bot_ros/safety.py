"""ROS-independent command clamping, watchdog, and arbitration logic."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional

from .mapping import TwistValue


@dataclass(frozen=True)
class Limits:
    x: float = 0.10
    y: float = 0.05
    yaw: float = 0.10


@dataclass
class JoyValue:
    axes: List[float] = field(default_factory=lambda: [0.0] * 8)
    buttons: List[int] = field(default_factory=lambda: [0] * 11)


@dataclass
class SafetyDecision:
    twist: TwistValue
    joy: JoyValue
    selected_source: str
    stale: bool
    action: str


def clamp(value: float, limit: float) -> float:
    if not math.isfinite(value):
        return 0.0
    return max(-limit, min(limit, value))


def clamp_twist(command: TwistValue, limits: Limits) -> TwistValue:
    return TwistValue(clamp(command.x, limits.x), clamp(command.y, limits.y), clamp(command.yaw, limits.yaw))


class SafetyController:
    def __init__(
        self,
        limits: Limits,
        expression_timeout: float = 0.5,
        manual_timeout: float = 0.5,
        manual_priority_hold: float = 0.75,
        mode_transition_delay: float = 0.35,
    ):
        self.limits = limits
        self.expression_timeout = expression_timeout
        self.manual_timeout = manual_timeout
        self.manual_priority_hold = manual_priority_hold
        self.mode_transition_delay = mode_transition_delay
        self.motion_enabled = False
        self.expression = TwistValue()
        self.expression_at: Optional[float] = None
        self.manual = JoyValue()
        self.manual_at: Optional[float] = None
        self.manual_active_until = 0.0
        self.locomotion_ready = False
        self.mode_stage = 0
        self.mode_due = 0.0
        self.stop_pulse_pending = False
        self.last_action = "startup_zero"

    def set_enabled(self, enabled: bool, now: float) -> None:
        self.motion_enabled = bool(enabled)
        if enabled:
            self.last_action = "motion_enabled_waiting_for_command"
        else:
            self.stop_pulse_pending = self.locomotion_ready or self.mode_stage != 0
            self.locomotion_ready = False
            self.mode_stage = 0
            self.last_action = "motion_disabled_zero"

    def update_expression(self, command: TwistValue, now: float) -> None:
        self.expression = clamp_twist(command, self.limits)
        self.expression_at = float(now)

    def update_manual(self, axes: List[float], buttons: List[int], now: float) -> None:
        safe_axes = [0.0] * 8
        for index in (0, 3, 4):
            if index < len(axes):
                safe_axes[index] = clamp(float(axes[index]), 1.0)
        safe_buttons = [0] * 11
        for index in (1, 2, 3, 5):
            if index < len(buttons):
                safe_buttons[index] = 1 if buttons[index] else 0
        self.manual = JoyValue(safe_axes, safe_buttons)
        self.manual_at = float(now)
        if any(abs(value) > 1e-4 for value in safe_axes) or any(safe_buttons):
            self.manual_active_until = float(now) + self.manual_priority_hold
        if safe_buttons[1]:
            self.locomotion_ready = False
            self.mode_stage = 0
        if safe_buttons[2]:
            self.locomotion_ready = True
            self.mode_stage = 0

    def _twist_to_joy(self, twist: TwistValue) -> JoyValue:
        joy = JoyValue()
        joy.axes[4] = clamp(twist.x / 0.2, 1.0)
        joy.axes[3] = clamp(twist.y / 0.1, 1.0)
        joy.axes[0] = clamp(twist.yaw / 0.2, 1.0)
        return joy

    def _manual_twist(self, joy: JoyValue) -> TwistValue:
        return TwistValue(
            clamp(joy.axes[4] * 0.2, 0.2),
            clamp(joy.axes[3] * 0.1, 0.1),
            clamp(joy.axes[0] * 0.2, 0.2),
        )

    def _start_locomotion_if_needed(self, now: float) -> Optional[JoyValue]:
        if self.locomotion_ready:
            return None
        joy = JoyValue()
        if self.mode_stage == 0:
            joy.buttons[1] = 1
            self.mode_stage = 1
            self.mode_due = now + self.mode_transition_delay
            self.last_action = "enter_stand_before_locomotion"
            return joy
        if self.mode_stage == 1 and now >= self.mode_due:
            joy.buttons[2] = 1
            self.mode_stage = 0
            self.locomotion_ready = True
            self.last_action = "enter_locomotion"
            return joy
        return JoyValue()

    def step(self, now: float) -> SafetyDecision:
        now = float(now)
        expression_stale = self.expression_at is None or now - self.expression_at > self.expression_timeout
        manual_fresh = self.manual_at is not None and now - self.manual_at <= self.manual_timeout
        manual_active = manual_fresh and now <= self.manual_active_until

        if not self.motion_enabled:
            joy = JoyValue()
            if self.stop_pulse_pending:
                joy.buttons[1] = 1
                self.stop_pulse_pending = False
            self.last_action = "motion_disabled_zero"
            return SafetyDecision(TwistValue(), joy, "none", True, self.last_action)

        source = "none"
        twist = TwistValue()
        joy = JoyValue()
        if manual_active:
            source = "manual"
            joy = self.manual
            twist = self._manual_twist(joy)
        elif not expression_stale and not self.expression.is_zero():
            source = "emotion"
            twist = self.expression
            joy = self._twist_to_joy(twist)

        if source != "none":
            if source == "manual" and any(joy.buttons):
                self.last_action = "manual_priority"
                return SafetyDecision(twist, joy, source, expression_stale, self.last_action)
            mode_command = self._start_locomotion_if_needed(now)
            if mode_command is not None:
                return SafetyDecision(TwistValue(), mode_command, source, expression_stale, self.last_action)
            self.last_action = "manual_priority" if source == "manual" else "bounded_emotion_command"
            return SafetyDecision(twist, joy, source, expression_stale, self.last_action)

        if self.locomotion_ready or self.mode_stage:
            joy.buttons[1] = 1
            self.locomotion_ready = False
            self.mode_stage = 0
            self.last_action = "watchdog_stand_zero" if expression_stale else "expression_complete_stand_zero"
        else:
            self.last_action = "watchdog_zero" if expression_stale else "neutral_zero"
        return SafetyDecision(TwistValue(), joy, "none", expression_stale, self.last_action)
