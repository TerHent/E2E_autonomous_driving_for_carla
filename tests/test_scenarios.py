"""Validation tests for offline tactical driving scenarios."""

import json
import tempfile
import unittest
from pathlib import Path

from src.scenarios import build_all_scenarios, export_scenarios
from src.schemas import ScenarioCase


class TestScenarios(unittest.TestCase):
    def test_scenario_suite_is_valid_and_unique(self) -> None:
        cases = build_all_scenarios()
        self.assertEqual(len(cases), 5)

        ids = [c.scene.scenario_id for c in cases]
        self.assertEqual(len(ids), len(set(ids)))

        for case in cases:
            self.assertGreater(len(case.expectation.allowed_actions), 0)
            overlap = set(case.expectation.allowed_actions) & set(
                case.expectation.forbidden_actions
            )
            self.assertEqual(overlap, set(), f"Overlap in {case.scene.scenario_id}")
            self.assertGreaterEqual(case.scene.ego.speed_mps, 0.0)

    def test_roundtrip_json_export(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            paths = export_scenarios(Path(tmpdir))
            self.assertEqual(len(paths), 5)
            for path in paths:
                raw = json.loads(path.read_text(encoding="utf-8"))
                loaded = ScenarioCase.from_dict(raw)
                self.assertEqual(loaded.to_dict(), raw)


if __name__ == "__main__":
    unittest.main()
