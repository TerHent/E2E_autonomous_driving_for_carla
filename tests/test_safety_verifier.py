"""Unit tests for the deterministic Safety Verifier veto gate."""

import unittest

from src.safety_verifier import VALID_ACTIONS, verify_decision
from src.scenarios import build_all_scenarios


class TestSafetyVerifier(unittest.TestCase):
    def test_all_scenarios_allow_expected_and_veto_forbidden(self) -> None:
        for case in build_all_scenarios():
            sid = case.scene.scenario_id
            for action in case.expectation.allowed_actions:
                verdict = verify_decision(case.scene, action)
                self.assertTrue(
                    verdict.allowed,
                    f"[{sid}] expected {action!r} to be allowed, got: {verdict.reason}",
                )
                self.assertEqual(verdict.final_action, action)

            for action in case.expectation.forbidden_actions:
                verdict = verify_decision(case.scene, action)
                self.assertFalse(
                    verdict.allowed,
                    f"[{sid}] expected {action!r} to be vetoed",
                )
                self.assertIn(verdict.final_action, VALID_ACTIONS)

    def test_unknown_action_is_vetoed_to_decelerate(self) -> None:
        scene = build_all_scenarios()[0].scene
        verdict = verify_decision(scene, "swerve_wildly")
        self.assertFalse(verdict.allowed)
        self.assertEqual(verdict.final_action, "decelerate")

    def test_speed_limit_blocks_accelerate(self) -> None:
        scene = build_all_scenarios()[3].scene  # open road ahead, green light
        scene.ego.speed_mps = scene.ego.speed_limit_mps
        verdict = verify_decision(scene, "accelerate")
        self.assertFalse(verdict.allowed)
        self.assertEqual(verdict.final_action, "idle")


if __name__ == "__main__":
    unittest.main()
