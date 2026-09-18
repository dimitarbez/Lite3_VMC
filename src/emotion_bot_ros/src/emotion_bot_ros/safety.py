"""ROS-independent command clamping, watchdog, and arbitration logic."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import math
from typing import List, Optional

from .mapping import EXPRESSION_ACTIONS, TwistValue


@dataclass(frozen=True)
class Limits:
    x: float = 0.10
    y: float = 0.05
    yaw: float = 0.10
    z: float = 0.100
    roll: float = 0.625
    pitch: float = 0.625


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


def home_return_target(
    anchor_x: float,
    anchor_y: float,
    current_x: float,
    current_y: float,
    maximum_step: float,
):
    """Move toward an anchor by at most one bounded planar step."""
    dx = float(anchor_x) - float(current_x)
    dy = float(anchor_y) - float(current_y)
    distance = math.hypot(dx, dy)
    step = max(0.0, float(maximum_step))
    if distance <= step or distance <= 1e-9:
        return float(anchor_x), float(anchor_y)
    scale = step / distance
    return float(current_x) + dx * scale, float(current_y) + dy * scale


class SafetyController:
    def __init__(
        self,
        limits: Limits,
        expression_timeout: float = 0.5,
        manual_timeout: float = 0.5,
        manual_priority_hold: float = 0.75,
        mode_transition_delay: float = 0.35,
        gait_transition_guard: float = 0.60,
        minimum_locomotion_time: float = 2.0,
        manual_locomotion_timeout: float = 4.0,
        locomotion_command_delay: float = 1.1,
        max_linear_rate: float = 0.18,
        max_yaw_rate: float = 0.30,
        max_height_rate: float = 0.25,
        max_attitude_rate: float = 1.50,
        allow_locomotion: bool = False,
        allow_dynamic_actions: bool = True,
        action_queue_timeout: float = 2.0,
        locomotion_zero_hold: float = 0.25,
    ):
        self.limits = limits
        self.expression_timeout = expression_timeout
        self.manual_timeout = manual_timeout
        self.manual_priority_hold = manual_priority_hold
        self.mode_transition_delay = mode_transition_delay
        self.gait_transition_guard = max(0.0, float(gait_transition_guard))
        self.minimum_locomotion_time = max(0.0, float(minimum_locomotion_time))
        self.manual_locomotion_timeout = max(self.minimum_locomotion_time, float(manual_locomotion_timeout))
        self.locomotion_command_delay = max(0.0, float(locomotion_command_delay))
        self.max_linear_rate = max(0.001, float(max_linear_rate))
        self.max_yaw_rate = max(0.001, float(max_yaw_rate))
        self.max_height_rate = max(0.001, float(max_height_rate))
        self.max_attitude_rate = max(0.001, float(max_attitude_rate))
        self.allow_locomotion = bool(allow_locomotion)
        self.allow_dynamic_actions = bool(allow_dynamic_actions)
        self.action_queue_timeout = max(float(expression_timeout), float(action_queue_timeout))
        self.locomotion_zero_hold = max(0.0, float(locomotion_zero_hold))
        self.motion_enabled = False
        self.expression = TwistValue()
        self.expression_at: Optional[float] = None
        self.expression_actions = deque()
        self.action_generation = -1
        self.seen_action_occurrences = set()
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
        self.cancel_pulse_pending = False
        self.stance_zero_since: Optional[float] = None
        self.output = TwistValue()
        self.output_at: Optional[float] = None
        self.last_action = "startup_zero"

    def set_enabled(self, enabled: bool, now: float) -> None:
        self.motion_enabled = bool(enabled)
        if enabled:
            self.last_action = "motion_enabled_waiting_for_command"
        else:
            self.stop_pulse_pending = self.locomotion_ready or self.mode_stage != 0
            self.cancel_pulse_pending = True
            self.locomotion_ready = False
            self.stand_ready = False
            self.locomotion_started_at = None
            self.command_ready_at = None
            self.manual_mode_latched = False
            self.mode_stage = 0
            self.output = TwistValue()
            self.output_at = float(now)
            self.expression_actions.clear()
            self.seen_action_occurrences.clear()
            self.stance_zero_since = None
            self.last_action = "motion_disabled_zero"

    def update_expression(self, command: TwistValue, now: float) -> None:
        self.expression = clamp_twist(command, self.limits)
        self.expression_at = float(now)

    def update_expression_action(
        self,
        action: str,
        now: float,
        kind: str = "start",
        generation: Optional[int] = None,
        occurrence: str = "legacy",
    ) -> None:
        """Queue a generation-ordered simulator action or cancellation."""
        kind = str(kind)
        generation = self.action_generation + 1 if generation is None else int(generation)
        if generation < self.action_generation:
            return
        if generation > self.action_generation:
            self.action_generation = generation
            self.expression_actions.clear()
            self.seen_action_occurrences.clear()
        if kind == "cancel":
            self.expression_actions.clear()
            self.expression_actions.append(("cancel", "none", float(now), generation, str(occurrence)))
            return
        action = str(action)
        if kind != "start" or action not in EXPRESSION_ACTIONS or action == "none":
            return
        occurrence_key = (generation, str(occurrence))
        if occurrence_key in self.seen_action_occurrences:
            return
        self.seen_action_occurrences.add(occurrence_key)
        marker = ("start", action, float(now), generation, str(occurrence))
        self.expression_actions.append(marker)

    def request_stance_preparation(self, now: float) -> None:
        """Request the stable four-contact stance while permission stays off."""
        self.stop_pulse_pending = True
        self.locomotion_ready = False
        self.stand_ready = True
        self.locomotion_started_at = None
        self.command_ready_at = None
        self.manual_mode_latched = False
        self.mode_stage = 0
        self.stance_zero_since = None
        self._force_output_zero(now)
        self.last_action = "preparing_expression_stance"

    def begin_recenter_transition(self, now: float) -> None:
        """Stop an active gait, then make the next locomotion request restart cleanly.

        The caller sends the one-shot stand button on the same tick.  Keeping
        the transition state here prevents the return velocity from merely
        reversing an already committed gait, which can overshoot badly in the
        Lite3 simulator.
        """
        self._force_output_zero(now)
        self.locomotion_ready = False
        self.stand_ready = False
        self.locomotion_started_at = None
        self.command_ready_at = None
        self.manual_mode_latched = False
        # Stage 1 waits for the stand transition and then emits the internal
        # advanced-trot selector.  The usual command delay applies afterward.
        self.mode_stage = 1
        self.mode_due = float(now) + self.mode_transition_delay
        self.stance_zero_since = None
        self.last_action = "recenter_stand_transition"

    def hold_stance_for_recenter(self, now: float) -> None:
        """Stop locomotion and hold the normal four-contact expression stance."""
        self._force_output_zero(now)
        self.locomotion_ready = False
        self.stand_ready = True
        self.locomotion_started_at = None
        self.command_ready_at = None
        self.manual_mode_latched = False
        self.mode_stage = 0
        self.stance_zero_since = None
        self.last_action = "recenter_stance_hold"

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
        # Fixed wire units must match qrDesiredStateCommand. Configured safety
        # limits only clamp: lowering a limit must never amplify a command.
        joy.axes[2] = clamp(twist.z / 0.100, 1.0)
        joy.axes[6] = clamp(twist.roll / 0.625, 1.0)
        joy.axes[7] = clamp(twist.pitch / 0.625, 1.0)
        return joy

    def _manual_twist(self, joy: JoyValue) -> TwistValue:
        return clamp_twist(
            TwistValue(
                x=clamp(joy.axes[4] * 0.2, 0.2),
                y=clamp(joy.axes[3] * 0.1, 0.1),
                yaw=clamp(joy.axes[0] * 0.2, 0.2),
                z=joy.axes[2] * 0.100,
                roll=joy.axes[6] * 0.625,
                pitch=joy.axes[7] * 0.625,
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
        if self.mode_stage == 2:
            if now < self.mode_due:
                self.last_action = "gait_to_stance_settling"
                return joy
            self.mode_stage = 0
            self.stand_ready = True
        if self.mode_stage == 0:
            joy.buttons[1] = 1
            self.mode_stage = 1
            self.mode_due = now + self.mode_transition_delay
            self.last_action = "enter_stand_before_locomotion"
            return joy
        if self.mode_stage == 1 and now >= self.mode_due:
            # Button 7 is internal to emotion_bot_ros and selects the tested,
            # MPC-backed advanced trot. Manual button 2 keeps gait cycling.
            joy.buttons[7] = 1
            self.mode_stage = 0
            self.locomotion_ready = True
            self.stand_ready = False
            self.locomotion_started_at = now
            self.command_ready_at = now + self.locomotion_command_delay
            self.last_action = "enter_locomotion"
            return joy
        return JoyValue()

    def _advance_stance_transition(
        self, now: float, source: str, expression_stale: bool
    ) -> Optional[SafetyDecision]:
        """Ramp a gait to zero, hold, request stand once, then settle."""
        if self.stand_ready and not self.locomotion_ready and self.mode_stage == 0:
            return None

        if self.locomotion_ready:
            limited = self._slew_output(TwistValue(), now)
            if not limited.is_zero():
                self.stance_zero_since = None
                self.last_action = "gait_to_stance_ramp_zero"
                return SafetyDecision(
                    limited,
                    self._twist_to_joy(limited),
                    source,
                    expression_stale,
                    self.last_action,
                )
            if self.stance_zero_since is None:
                self.stance_zero_since = now
            if now - self.stance_zero_since < self.locomotion_zero_hold:
                self.last_action = "gait_to_stance_zero_hold"
                return SafetyDecision(
                    TwistValue(), JoyValue(), source, expression_stale, self.last_action
                )
            # Never request stand while the upstream gait-to-gait transition
            # is still running. Interrupting it can install a new gait after
            # the stop request and leave controller/gait state inconsistent.
            started_at = (
                self.locomotion_started_at
                if self.locomotion_started_at is not None
                else now
            )
            if now - started_at < self.gait_transition_guard:
                self.last_action = "gait_to_stance_minimum_hold"
                return SafetyDecision(
                    TwistValue(), JoyValue(), source, expression_stale, self.last_action
                )

        if self.mode_stage != 2:
            joy = JoyValue()
            joy.buttons[1] = 1
            self.mode_stage = 2
            self.mode_due = now + self.mode_transition_delay
            self.locomotion_ready = False
            self.stand_ready = False
            self.locomotion_started_at = None
            self.command_ready_at = None
            self.manual_mode_latched = False
            self.stance_zero_since = None
            self.last_action = "gait_to_stance_request"
            return SafetyDecision(
                TwistValue(), joy, source, expression_stale, self.last_action
            )

        if now < self.mode_due:
            self.last_action = "gait_to_stance_settling"
            return SafetyDecision(
                TwistValue(), JoyValue(), source, expression_stale, self.last_action
            )

        self.mode_stage = 0
        self.stand_ready = True
        self.last_action = "expression_stance_ready"
        return None

    def _pending_dynamic_action(self, now: float, source: str) -> str:
        while self.expression_actions:
            kind, action, queued_at, _generation, _occurrence = self.expression_actions[0]
            if kind == "cancel":
                return "cancel"
            if now - queued_at > self.action_queue_timeout:
                self.expression_actions.popleft()
                continue
            if not self.allow_dynamic_actions or source == "manual":
                return "none"
            return action
        return "none"

    def _consume_dynamic_action(self) -> JoyValue:
        joy = JoyValue()
        if not self.expression_actions:
            return joy
        kind, action, _queued_at, _generation, _occurrence = self.expression_actions.popleft()
        if kind == "cancel":
            joy.buttons[6] = 1
        elif action == "hop":
            joy.buttons[4] = 1
        elif action == "stomp":
            joy.buttons[5] = 1
        return joy

    def step(self, now: float) -> SafetyDecision:
        now = float(now)
        expression_stale = self.expression_at is None or now - self.expression_at > self.expression_timeout
        manual_fresh = self.manual_at is not None and now - self.manual_at <= self.manual_timeout
        manual_active = manual_fresh and now <= self.manual_active_until

        if not self.motion_enabled:
            self._force_output_zero(now)
            joy = JoyValue()
            if self.cancel_pulse_pending:
                joy.buttons[6] = 1
                self.cancel_pulse_pending = False
            if self.stop_pulse_pending:
                joy.buttons[1] = 1
                self.stop_pulse_pending = False
            self.last_action = "motion_disabled_zero"
            return SafetyDecision(TwistValue(), joy, "none", True, self.last_action)

        source = "none"
        twist = TwistValue()
        joy = JoyValue()

        if self.expression_actions and self.expression_actions[0][0] == "cancel":
            action_joy = self._consume_dynamic_action()
            action_joy.axes = self._twist_to_joy(self.output).axes
            self.last_action = "bounded_emotion_cancel"
            return SafetyDecision(
                self.output, action_joy, "emotion_action", expression_stale, self.last_action
            )

        # A discrete hop/stomp begins from the four-contact Lite3 stance.  It
        # must never share a tick with gait velocity or a
        # theatrical roll/pitch command: that combination can leave the
        # support polygon while the controller changes modes.  Give the action
        # priority, bring the robot to torque stance with zero axes, then send
        # its one-shot button pulse on a later tick.
        dynamic_action = self._pending_dynamic_action(
            now, "manual" if manual_active else source
        )
        if dynamic_action != "none":
            transition = self._advance_stance_transition(
                now, "emotion_action", expression_stale
            )
            if transition is not None:
                return transition
            action_joy = self._consume_dynamic_action()
            self._force_output_zero(now)
            self.last_action = "bounded_emotion_%s" % dynamic_action
            return SafetyDecision(
                TwistValue(), action_joy, "emotion_action", expression_stale, self.last_action
            )

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
            if twist.posture_only():
                transition = self._advance_stance_transition(now, source, expression_stale)
                if transition is not None:
                    return transition
            else:
                mode_command = self._start_locomotion_if_needed(now)
                if mode_command is not None:
                    self._force_output_zero(now)
                    return SafetyDecision(
                        TwistValue(), mode_command, source, expression_stale, self.last_action
                    )
            if self.command_ready_at is not None and now < self.command_ready_at:
                self._force_output_zero(now)
                self.last_action = "locomotion_settling"
                return SafetyDecision(TwistValue(), JoyValue(), source, expression_stale, self.last_action)
            limited = self._slew_output(twist, now)
            joy = self._twist_to_joy(limited)
            self.last_action = "manual_priority" if source == "manual" else "bounded_emotion_command"
            return SafetyDecision(limited, joy, source, expression_stale, self.last_action)

        # Mapper Twist and String publications travel on independent ROS
        # connections.  The one-shot action can therefore arrive one control
        # tick before its blended posture becomes non-zero.  Keep it behind the
        # same enabled/fresh watchdog, but do not discard that legitimate pulse.
        limited = self._slew_output(TwistValue(), now)
        if not limited.is_zero():
            self.last_action = "watchdog_ramp_zero" if expression_stale else "expression_complete_ramp_zero"
            return SafetyDecision(limited, self._twist_to_joy(limited), "none", expression_stale, self.last_action)

        if self.mode_stage == 2:
            if now >= self.mode_due:
                self.mode_stage = 0
                self.stand_ready = True
                self.last_action = "expression_stance_ready"
            else:
                self.last_action = "gait_to_stance_settling"
        elif self.mode_stage:
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
