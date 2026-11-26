"""Deterministic tactical scene-query tools for the LLM decision layer."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional

from src.schemas import ActorState, SceneState


@dataclass(frozen=True)
class TTCResult:
    lane_offset: int
    lead_actor_id: Optional[str]
    gap_m: Optional[float]
    closing_speed_mps: float
    ttc_s: float


@dataclass(frozen=True)
class LaneClearanceResult:
    direction: str
    is_free: bool
    reason: str
    blocking_actor_id: Optional[str] = None


@dataclass(frozen=True)
class TrafficLightResult:
    state: str
    distance_m: Optional[float]
    must_stop: bool
    reason: str


def compute_ttc(scene: SceneState, lane_offset: int = 0) -> TTCResult:
    """Compute Time-to-Collision (s) to the closest actor ahead in `lane_offset`."""
    ahead = [a for a in scene.actors if a.lane_offset == lane_offset and a.x_m > 0.0]
    if not ahead:
        return TTCResult(
            lane_offset=lane_offset,
            lead_actor_id=None,
            gap_m=None,
            closing_speed_mps=0.0,
            ttc_s=math.inf,
        )

    closest: ActorState = min(ahead, key=lambda a: a.x_m)
    closing = scene.ego.speed_mps - closest.speed_mps
    ttc = closest.x_m / closing if closing > 1e-3 else math.inf
    return TTCResult(
        lane_offset=lane_offset,
        lead_actor_id=closest.id,
        gap_m=round(closest.x_m, 2),
        closing_speed_mps=round(closing, 2),
        ttc_s=round(ttc, 2) if math.isfinite(ttc) else math.inf,
    )


def is_adjacent_lane_free(
    scene: SceneState,
    direction: str,
    ahead_m: float = 25.0,
    behind_m: float = 15.0,
    min_rear_ttc_s: float = 3.5,
) -> LaneClearanceResult:
    """Check whether changing lane `left` (-1) or `right` (+1) is permitted and clear."""
    if direction not in ("left", "right"):
        raise ValueError(f"direction must be 'left' or 'right', got {direction!r}")

    if direction not in scene.route_window.allowed_lane_changes:
        return LaneClearanceResult(
            direction=direction,
            is_free=False,
            reason=f"Lane change '{direction}' not permitted by route_window.",
        )

    target_offset = -1 if direction == "left" else 1
    for actor in scene.actors:
        if actor.lane_offset != target_offset:
            continue
        if -behind_m <= actor.x_m <= ahead_m:
            return LaneClearanceResult(
                direction=direction,
                is_free=False,
                reason=f"Blocked by {actor.id} at x={actor.x_m:.1f}m.",
                blocking_actor_id=actor.id,
            )
        if actor.x_m < -behind_m:
            closing_from_behind = actor.speed_mps - scene.ego.speed_mps
            if closing_from_behind > 1e-3:
                rear_ttc = abs(actor.x_m) / closing_from_behind
                if rear_ttc < min_rear_ttc_s:
                    return LaneClearanceResult(
                        direction=direction,
                        is_free=False,
                        reason=f"Fast approaching {actor.id} from behind (TTC={rear_ttc:.1f}s).",
                        blocking_actor_id=actor.id,
                    )

    return LaneClearanceResult(
        direction=direction,
        is_free=True,
        reason=f"Adjacent '{direction}' lane is permitted and clear.",
    )


def check_traffic_light(
    scene: SceneState,
    approach_range_m: float = 40.0,
    comfortable_decel_mps2: float = 3.0,
) -> TrafficLightResult:
    """Assess whether the ego vehicle must stop for an upcoming traffic light."""
    state = scene.traffic_light.state.lower()
    dist = scene.traffic_light.distance_m
    if state not in ("red", "yellow") or dist is None:
        return TrafficLightResult(
            state=state,
            distance_m=dist,
            must_stop=False,
            reason="No red/yellow stop signal ahead.",
        )

    stop_dist = (scene.ego.speed_mps ** 2) / (2.0 * comfortable_decel_mps2) + 5.0
    if dist <= max(approach_range_m, stop_dist):
        return TrafficLightResult(
            state=state,
            distance_m=dist,
            must_stop=True,
            reason=f"{state.capitalize()} light at {dist:.1f}m within stopping/approach range.",
        )

    return TrafficLightResult(
        state=state,
        distance_m=dist,
        must_stop=False,
        reason=f"{state.capitalize()} light at {dist:.1f}m is still far ahead.",
    )


def summarize_scene_tools(scene: SceneState) -> Dict[str, Any]:
    """Run all tactical query tools on a SceneState and return a JSON-ready summary."""
    return {
        "ttc_ego_lane": asdict(compute_ttc(scene, lane_offset=0)),
        "left_lane": asdict(is_adjacent_lane_free(scene, "left")),
        "right_lane": asdict(is_adjacent_lane_free(scene, "right")),
        "traffic_light": asdict(check_traffic_light(scene)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect tactical tool outputs on a SceneState JSONL.")
    parser.add_argument("jsonl_path", type=Path, help="Path to a recorded SceneState .jsonl file.")
    parser.add_argument("--frame", type=int, default=0, help="0-based frame index to inspect.")
    args = parser.parse_args()

    lines = args.jsonl_path.read_text(encoding="utf-8").splitlines()
    scene = SceneState.from_dict(json.loads(lines[args.frame]))
    print(json.dumps(summarize_scene_tools(scene), indent=2))


if __name__ == "__main__":
    main()
