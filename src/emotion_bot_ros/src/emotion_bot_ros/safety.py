"""ROS-independent command clamping, watchdog, and arbitration logic."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import List, Optional

from .mapping import EXPRESSION_ACTIONS, TwistValue


@dataclass(frozen=True)
class Limits:
    x: float = 0.10
    y: float = 0.05
    yaw: float = 0.10
    z: float = 0.070
    roll: float = 0.50
    pitch: float = 0.50


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
    return TwistValue(
        x=clamp(command.x, limits.x),
        y=clamp(command.y, limits.y),
        yaw=clamp(command.yaw, limits.yaw),
        z=clamp(command.z, limits.z),
        roll=clamp(command.roll, limits.roll),
        pitch=clamp(command.pitch, limits.pitch),
    )


class SafetyController:
    def __init__(
        self,
        limits: Limits,
        expression_timeout: float = 0.5,
        manual_timeout: float = 0.5,
        manual_priority_hold: float = 0.75,
        mode_transition_delay: float = 0.35,
        minimum_locomotion_time: float = 2.0,
        manual_locomotion_timeout: float = 4.0,
        locomotion_command_delay: float = 1.1,
        max_linear_rate: float = 0.18,
        max_yaw_rate: float = 0.30,
        max_height_rate: float = 0.25,
        max_attitude_rate: float = 1.50,
        allow_locomotion: bool = False,
        allow_dynamic_actions: bool = True,
    ):
        self.limits = limits
        self.expression_timeout = expression_timeout
        self.manual_timeout = manual_timeout
        self.manual_priority_hold = manual_priority_hold
        self.mode_transition_delay = mode_transition_delay
        self.minimum_locomotion_time = max(0.0, float(minimum_locomotion_time))
        self.manual_locomotion_timeout = max(self.minimum_locomotion_time, float(manual_locomotion_timeout))
        self.locomotion_command_delay = max(0.0, float(locomotion_command_delay))
        self.max_linear_rate = max(0.001, float(max_linear_rate))
        self.max_yaw_rate = max(0.001, float(max_yaw_rate))
        self.max_height_rate = max(0.001, float(max_height_rate))
        self.max_attitude_rate = max(0.001, float(max_attitude_rate))
        self.allow_locomotion = bool(allow_locomotion)
        self.allow_dynamic_actions = bool(allow_dynamic_actions)
        self.motion_enabled = False
        self.expression = TwistValue()
        self.expression_at: Optional[float] = None
        self.expression_action = "none"
        self.expression_action_at: Optional[float] = None
        self.expression_action_consumed = True
        self.manual = JoyValue()
        self.manual_at: Optional[float] = None
        self.manual_active_until = 0.0
        self.locomotion_ready = False
        self.stand_ready = False
        self.locomotion_started_at: Optional[float] = None
        self.command_ready_at: Optional[float] = None
        self.manual_mode_latched = False
        self.mode_stage = 0
        self.mode_due = 0.0
        self.stop_pulse_pending = False
        self.output = TwistValue()
        self.output_at: Optional[float] = None
        self.last_action = "startup_zero"

    def set_enabled(self, enabled: bool, now: float) -> None:
        self.motion_enabled = bool(enabled)
        if enabled:
            self.last_action = "motion_enabled_waiting_for_command"
        else:
            self.stop_pulse_pending = self.locomotion_ready or self.mode_stage != 0
            self.locomotion_ready = False
            self.stand_ready = False
            self.locomotion_started_at = None
            self.command_ready_at = None
            self.manual_mode_latched = False
            self.mode_stage = 0
            self.output = TwistValue()
            self.output_at = float(now)
            self.expression_action = "none"
            self.expression_action_at = None
            self.expression_action_consumed = True
            self.last_action = "motion_disabled_zero"

    def update_expression(self, command: TwistValue, now: float) -> None:
        self.expression = clamp_twist(command, self.limits)
        self.expression_at = float(now)

    def update_expression_action(self, action: str, now: float) -> None:
        """Queue one simulator-only vertical action behind the normal watchdog."""
        action = str(action)
        if action not in EXPRESSION_ACTIONS or action == "none":
            return
        self.expression_action = action
        self.expression_action_at = float(now)
        self.expression_action_consumed = False

    def request_stance_preparation(self, now: float) -> None:
        """Request the stable four-contact stance while permission stays off."""
        self.stop_pulse_pending = True
        self.locomotion_ready = False
        self.stand_ready = True
        self.locomotion_started_at = None
        self.command_ready_at = None
        self.manual_mode_latched = False
        self.mode_stage = 0
        self._force_output_zero(now)
        self.last_action = "preparing_expression_stance"

    def update_manual(self, axes: List[float], buttons: List[int], now: float) -> None:
        safe_axes = [0.0] * 8
        for index in (0, 2, 3, 4, 6, 7):
            if index < len(axes):
                safe_axes[index] = clamp(float(axes[index]), 1.0)
        safe_buttons = [0] * 11
        for index in (1, 2, 3):
            if index < len(buttons):
                safe_buttons[index] = 1 if buttons[index] else 0
        self.manual = JoyValue(safe_axes, safe_buttons)
        self.manual_at = float(now)
        if any(abs(value) > 1e-4 for value in safe_axes) or any(safe_buttons):
            self.manual_active_until = float(now) + self.manual_priority_hold
        if safe_buttons[1]:
            self.locomotion_ready = False
            self.stand_ready = True
            self.locomotion_started_at = None
            self.command_ready_at = None
            self.manual_mode_latched = False
            self.mode_stage = 0
            self.output = TwistValue()
            self.output_at = float(now)
        if safe_buttons[2] and self.allow_locomotion:
            self.locomotion_ready = True
            self.stand_ready = False
            self.locomotion_started_at = float(now)
            self.command_ready_at = float(now) + self.locomotion_command_delay
            self.manual_mode_latched = True
            self.mode_stage = 0
        elif self.manual_mode_latched and self.locomotion_ready and any(abs(value) > 1e-4 for value in safe_axes):
            self.locomotion_started_at = float(now)

    def _twist_to_joy(self, twist: TwistValue) -> JoyValue:
        joy = JoyValue()
        joy.axes[4] = clamp(twist.x / 0.2, 1.0)
        joy.axes[3] = clamp(twist.y / 0.1, 1.0)
        joy.axes[0] = clamp(twist.yaw / 0.2, 1.0)
        joy.axes[2] = clamp(twist.z / self.limits.z, 1.0)
        joy.axes[6] = clamp(twist.roll / self.limits.roll, 1.0)
        joy.axes[7] = clamp(twist.pitch / self.limits.pitch, 1.0)
        return joy

    def _manual_twist(self, joy: JoyValue) -> TwistValue:
        return clamp_twist(
            TwistValue(
                x=clamp(joy.axes[4] * 0.2, 0.2),
                y=clamp(joy.axes[3] * 0.1, 0.1),
                yaw=clamp(joy.axes[0] * 0.2, 0.2),
                z=clamp(joy.axes[2] * self.limits.z, self.limits.z),
                roll=clamp(joy.axes[6] * self.limits.roll, self.limits.roll),
                pitch=clamp(joy.axes[7] * self.limits.pitch, self.limits.pitch),
            ),
            self.limits,
        )

    def _slew_output(self, target: TwistValue, now: float) -> TwistValue:
        if self.output_at is None:
            self.output_at = float(now)
            return self.output
        dt = max(0.0, min(0.25, float(now) - self.output_at))
        self.output_at = float(now)

        def approach(value: float, desired: float, maximum_delta: float) -> float:
            return value + clamp(desired - value, maximum_delta)

        self.output = TwistValue(
            approach(self.output.x, target.x, self.max_linear_rate * dt),
            approach(self.output.y, target.y, self.max_linear_rate * dt),
            approach(self.output.yaw, target.yaw, self.max_yaw_rate * dt),
            approach(self.output.z, target.z, self.max_height_rate * dt),
            approach(self.output.roll, target.roll, self.max_attitude_rate * dt),
            approach(self.output.pitch, target.pitch, self.max_attitude_rate * dt),
        )
        if target.is_zero() and self.output.is_zero(1e-6):
            self.output = TwistValue()
        return self.output

    def _force_output_zero(self, now: float) -> None:
        self.output = TwistValue()
        self.output_at = float(now)

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
            self.stand_ready = False
            self.locomotion_started_at = now
            self.command_ready_at = now + self.locomotion_command_delay
            self.last_action = "enter_locomotion"
            return joy
        return JoyValue()

    def _start_stance_if_needed(self, now: float) -> Optional[JoyValue]:
        if self.stand_ready and not self.locomotion_ready:
            return None
        joy = JoyValue()
        if self.mode_stage == 0:
            joy.buttons[1] = 1
            self.mode_stage = 2
            self.mode_due = now + self.mode_transition_delay
            self.locomotion_ready = False
            self.last_action = "enter_expression_stance"
            return joy
        if self.mode_stage == 2 and now >= self.mode_due:
            self.mode_stage = 0
            self.stand_ready = True
            self.last_action = "expression_stance_ready"
            return None
        return JoyValue()

    def _pending_dynamic_action(self, now: float, source: str, expression_stale: bool) -> str:
        if (
            not self.allow_dynamic_actions
            or source == "manual"
            or expression_stale
            or self.expression_action_consumed
            or self.expression_action_at is None
            or now - self.expression_action_at > self.expression_timeout
        ):
            return "none"
        return self.expression_action

    def _consume_dynamic_action(self) -> JoyValue:
        joy = JoyValue()
        if self.expression_action == "hop":
            joy.buttons[4] = 1
        elif self.expression_action == "stomp":
            joy.buttons[5] = 1
        else:
            return joy
        self.expression_action_consumed = True
        return joy

    def step(self, now: float) -> SafetyDecision:
        now = float(now)
        expression_stale = self.expression_at is None or now - self.expression_at > self.expression_timeout
        manual_fresh = self.manual_at is not None and now - self.manual_at <= self.manual_timeout
        manual_active = manual_fresh and now <= self.manual_active_until

        if not self.motion_enabled:
            self._force_output_zero(now)
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
            joy = JoyValue(list(self.manual.axes), list(self.manual.buttons))
            # Keyboard mode buttons are edge-like commands.  Replaying a held
            # button at the control rate can toggle Lite3 gaits repeatedly.
            self.manual.buttons = [0] * 11
            twist = self._manual_twist(joy)
            source = "manual" if (any(joy.buttons) or not twist.is_zero()) else "manual_hold"
        elif not expression_stale and not self.expression.is_zero():
            source = "emotion"
            twist = self.expression if self.allow_locomotion else self.expression.without_locomotion()
            joy = self._twist_to_joy(twist)

        if source != "none":
            if source == "manual_hold":
                self.last_action = "manual_priority_zero"
                limited = self._slew_output(TwistValue(), now)
                return SafetyDecision(limited, self._twist_to_joy(limited), source, expression_stale, self.last_action)
            if source == "manual" and any(joy.buttons):
                if not self.allow_locomotion:
                    joy.buttons[2] = 0
                self._force_output_zero(now)
                self.last_action = "manual_priority"
                return SafetyDecision(TwistValue(), joy, source, expression_stale, self.last_action)
            if not self.allow_locomotion and twist.has_locomotion():
                twist = twist.without_locomotion()
                if twist.is_zero():
                    self._force_output_zero(now)
                    self.last_action = "locomotion_blocked_zero"
                    return SafetyDecision(TwistValue(), JoyValue(), source, expression_stale, self.last_action)
            mode_command = (
                self._start_stance_if_needed(now)
                if twist.posture_only()
                else self._start_locomotion_if_needed(now)
            )
            if mode_command is not None:
                self._force_output_zero(now)
                return SafetyDecision(TwistValue(), mode_command, source, expression_stale, self.last_action)
            if self.command_ready_at is not None and now < self.command_ready_at:
                self._force_output_zero(now)
                self.last_action = "locomotion_settling"
                return SafetyDecision(TwistValue(), JoyValue(), source, expression_stale, self.last_action)
            limited = self._slew_output(twist, now)
            joy = self._twist_to_joy(limited)
            dynamic_action = self._pending_dynamic_action(now, source, expression_stale)
            if dynamic_action != "none":
                action_joy = self._consume_dynamic_action()
                joy.buttons = action_joy.buttons
                self.last_action = "bounded_emotion_%s" % dynamic_action
            else:
                self.last_action = "manual_priority" if source == "manual" else "bounded_emotion_command"
            return SafetyDecision(limited, joy, source, expression_stale, self.last_action)

        # Mapper Twist and String publications travel on independent ROS
        # connections.  The one-shot action can therefore arrive one control
        # tick before its blended posture becomes non-zero.  Keep it behind the
        # same enabled/fresh watchdog, but do not discard that legitimate pulse.
        dynamic_action = self._pending_dynamic_action(now, source, expression_stale)
        if dynamic_action != "none":
            mode_command = self._start_stance_if_needed(now)
            if mode_command is not None:
                self._force_output_zero(now)
                return SafetyDecision(TwistValue(), mode_command, "emotion_action", expression_stale, self.last_action)
            action_joy = self._consume_dynamic_action()
            self._force_output_zero(now)
            self.last_action = "bounded_emotion_%s" % dynamic_action
            return SafetyDecision(TwistValue(), action_joy, "emotion_action", expression_stale, self.last_action)

        limited = self._slew_output(TwistValue(), now)
        if not limited.is_zero():
            self.last_action = "watchdog_ramp_zero" if expression_stale else "expression_complete_ramp_zero"
            return SafetyDecision(limited, self._twist_to_joy(limited), "none", expression_stale, self.last_action)

        if self.mode_stage:
            joy.buttons[1] = 1
            self.locomotion_ready = False
            self.stand_ready = True
            self.locomotion_started_at = None
            self.command_ready_at = None
            self.mode_stage = 0
            self.last_action = "watchdog_stand_zero" if expression_stale else "expression_complete_stand_zero"
        elif self.locomotion_ready:
            if self.manual_mode_latched:
                started_at = self.locomotion_started_at if self.locomotion_started_at is not None else now
                if now - started_at < self.manual_locomotion_timeout:
                    self.last_action = "manual_mode_zero"
                    return SafetyDecision(TwistValue(), JoyValue(), "none", expression_stale, self.last_action)
                joy.buttons[1] = 1
                self.locomotion_ready = False
                self.stand_ready = True
                self.locomotion_started_at = None
                self.command_ready_at = None
                self.manual_mode_latched = False
                self.last_action = "manual_idle_stand_zero"
                return SafetyDecision(TwistValue(), joy, "none", expression_stale, self.last_action)
            # Zero axes immediately, but do not interrupt the upstream gait
            # transition with B. Once its minimum settling interval has passed,
            # request torque stance exactly once.
            started_at = self.locomotion_started_at if self.locomotion_started_at is not None else now
            if now - started_at >= self.minimum_locomotion_time:
                joy.buttons[1] = 1
                self.locomotion_ready = False
                self.stand_ready = True
                self.locomotion_started_at = None
                self.command_ready_at = None
                self.last_action = "watchdog_stand_zero" if expression_stale else "expression_complete_stand_zero"
            else:
                self.last_action = "watchdog_locomotion_zero" if expression_stale else "expression_complete_locomotion_zero"
        else:
            self.last_action = "watchdog_zero" if expression_stale else "neutral_zero"
        return SafetyDecision(TwistValue(), joy, "none", expression_stale, self.last_action)
