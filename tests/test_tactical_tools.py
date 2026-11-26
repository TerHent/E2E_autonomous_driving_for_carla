"""Unit tests for deterministic tactical scene-query tools."""

import math
import unittest

from src.scenarios import build_all_scenarios
from src.tactical_tools import (
    check_traffic_light,
    compute_ttc,
    is_adjacent_lane_free,
    summarize_scene_tools,
)


class TestTacticalTools(unittest.TestCase):
    def setUp(self) -> None:
        self.by_id = {c.scene.scenario_id: c.scene for c in build_all_scenarios()}

    def test_compute_ttc_on_closing_scenarios(self) -> None:
        red = self.by_id["red_light_stop"]
        ttc_red = compute_ttc(red, lane_offset=0)
        self.assertEqual(ttc_red.lead_actor_id, "veh_lead")
        self.assertAlmostEqual(ttc_red.ttc_s, 2.4, places=2)

        ped = self.by_id["emergency_pedestrian_crossing"]
        ttc_ped = compute_ttc(ped, lane_offset=0)
        self.assertEqual(ttc_ped.lead_actor_id, "ped_crossing")
        self.assertAlmostEqual(ttc_ped.ttc_s, 1.63, places=2)

    def test_compute_ttc_empty_lane_is_inf(self) -> None:
        merge = self.by_id["upcoming_right_turn_merge"]
        ttc = compute_ttc(merge, lane_offset=0)
        self.assertIsNone(ttc.lead_actor_id)
        self.assertTrue(math.isinf(ttc.ttc_s))

    def test_is_adjacent_lane_free_detects_clear_and_blocked(self) -> None:
        free_scene = self.by_id["slow_lead_overtake_left_free"]
        self.assertTrue(is_adjacent_lane_free(free_scene, "left").is_free)
        self.assertFalse(is_adjacent_lane_free(free_scene, "right").is_free)

        blocked_scene = self.by_id["slow_lead_left_blocked"]
        res = is_adjacent_lane_free(blocked_scene, "left")
        self.assertFalse(res.is_free)
        self.assertEqual(res.blocking_actor_id, "veh_left_fast")

        merge_scene = self.by_id["upcoming_right_turn_merge"]
        self.assertTrue(is_adjacent_lane_free(merge_scene, "right").is_free)

    def test_check_traffic_light(self) -> None:
        red = check_traffic_light(self.by_id["red_light_stop"])
        self.assertTrue(red.must_stop)

        green = check_traffic_light(self.by_id["upcoming_right_turn_merge"])
        self.assertFalse(green.must_stop)

        none_tl = check_traffic_light(self.by_id["slow_lead_overtake_left_free"])
        self.assertFalse(none_tl.must_stop)

    def test_summarize_scene_tools_shape(self) -> None:
        summary = summarize_scene_tools(self.by_id["red_light_stop"])
        self.assertEqual(
            set(summary.keys()),
            {"ttc_ego_lane", "left_lane", "right_lane", "traffic_light"},
        )


if __name__ == "__main__":
    unittest.main()
