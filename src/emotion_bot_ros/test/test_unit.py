#!/usr/bin/env python3
import json
import os
import unittest

import yaml

from emotion_bot_ros.contract import ContractError, EMOTIONS, build_state, dumps_state, loads_state
from emotion_bot_ros.mapping import PatternPlayer, TwistValue, load_patterns
from emotion_bot_ros.safety import JoyValue, Limits, SafetyController, clamp_twist


class FakeTime:
    secs = 12
    nsecs = 345


class ContractTests(unittest.TestCase):
    def test_all_emotions_round_trip_and_bounds(self):
        for sequence, emotion in enumerate(EMOTIONS):
            state = build_state(FakeTime(), sequence, emotion, -1.0, 1.0, "deterministic", "test")
            self.assertEqual(loads_state(dumps_state(state)), state)

    def test_malformed_and_out_of_bounds_rejected(self):
        with self.assertRaises(ContractError):
            loads_state("not json")
        state = build_state(FakeTime(), 0, "neutral", 0.0, 0.2, "deterministic", "test")
        state["valence"] = 1.1
        with self.assertRaises(ContractError):
            loads_state(json.dumps(state))


class MappingAndSafetyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = os.path.join(os.path.dirname(__file__), "..", "config", "default.yaml")
        with open(config_path, "r", encoding="utf-8") as stream:
            cls.patterns = load_patterns(yaml.safe_load(stream)["mappings"])

    def test_all_nine_mappings_are_finite(self):
        self.assertEqual(set(self.patterns), set(EMOTIONS))
        player = PatternPlayer(self.patterns)
        for emotion in EMOTIONS:
            player.start(emotion, 10.0)
            self.assertIsInstance(player.command(10.0), TwistValue)
            self.assertEqual(player.command(30.0), TwistValue())

    def test_clamps_every_expression_axis(self):
        result = clamp_twist(TwistValue(5.0, -6.0, float("nan")), Limits())
        self.assertEqual(result, TwistValue(0.10, -0.05, 0.0))

    def test_disabled_stale_disable_and_manual_priority(self):
        controller = SafetyController(Limits(), expression_timeout=0.5)
        controller.update_expression(TwistValue(0.5, 0.5, 0.5), 0.0)
        disabled = controller.step(0.1)
        self.assertEqual(disabled.twist, TwistValue())

        controller.set_enabled(True, 0.1)
        controller.update_expression(TwistValue(0.5, 0.5, 0.5), 0.1)
        controller.step(0.1)
        controller.step(0.5)
        bounded = controller.step(0.6)
        self.assertEqual(bounded.twist, TwistValue(0.10, 0.05, 0.10))

        axes = [0.0] * 8
        axes[4] = -0.4
        controller.update_manual(axes, [0] * 11, 0.61)
        manual = controller.step(0.62)
        self.assertEqual(manual.selected_source, "manual")

        stale = controller.step(1.3)
        self.assertEqual(stale.twist, TwistValue())
        self.assertTrue(stale.stale)

        controller.set_enabled(False, 1.4)
        self.assertEqual(controller.step(1.4).twist, TwistValue())

    def test_seeded_decisions_repeat(self):
        def decisions():
            controller = SafetyController(Limits())
            controller.set_enabled(True, 0.0)
            controller.update_expression(TwistValue(0.04, 0.0, 0.02), 0.0)
            return [controller.step(moment) for moment in (0.0, 0.4, 0.45)]
        self.assertEqual(decisions(), decisions())


if __name__ == "__main__":
    unittest.main()
