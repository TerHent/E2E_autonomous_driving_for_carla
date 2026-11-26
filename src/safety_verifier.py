"""Deterministic safety verifier (veto gate) for proposed tactical meta-actions."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path

from src.schemas import SceneState
from src.tactical_tools import (
    check_traffic_light,
    compute_ttc,
    is_adjacent_lane_free,
)

VALID_ACTIONS = frozenset(
    {"accelerate", "idle", "decelerate", "lane_change_left", "lane_change_right"}
)


@dataclass(frozen=True)
class SafetyVerdict:
    allowed: bool
    proposed_action: str
    final_action: str
    reason: str


def verify_decision(
    scene: SceneState,
    proposed_action: str,
    emergency_ttc_s: float = 2.0,
    emergency_gap_m: float = 10.0,
    caution_ttc_s: float = 3.0,
    caution_gap_m: float = 25.0,
) -> SafetyVerdict:
    """Evaluate a proposed meta-action against deterministic safety rules."""
    if proposed_action not in VALID_ACTIONS:
        return SafetyVerdict(
            allowed=False,
            proposed_action=proposed_action,
            final_action="decelerate",
            reason=f"Unknown action {proposed_action!r}; falling back to 'decelerate'.",
        )

    ttc = compute_ttc(scene, lane_offset=0)
    tl = check_traffic_light(scene)

    # 1. Hard emergency collision gate: only braking is safe.
    if (ttc.ttc_s < emergency_ttc_s) or (
        ttc.gap_m is not None and ttc.gap_m < emergency_gap_m
    ):
        if proposed_action != "decelerate":
            return SafetyVerdict(
                allowed=False,
                proposed_action=proposed_action,
                final_action="decelerate",
                reason=(
                    f"Emergency collision risk with {ttc.lead_actor_id} "
                    f"(gap={ttc.gap_m}m, TTC={ttc.ttc_s}s); must decelerate."
                ),
            )
        return SafetyVerdict(
            allowed=True,
            proposed_action=proposed_action,
            final_action=proposed_action,
            reason="Emergency deceleration approved.",
        )

    # 2. Traffic light compliance gate.
    if tl.must_stop and proposed_action in (
        "accelerate",
        "lane_change_left",
        "lane_change_right",
    ):
        return SafetyVerdict(
            allowed=False,
            proposed_action=proposed_action,
            final_action="decelerate",
            reason=f"{tl.reason} Vetoed {proposed_action!r}.",
        )

    # 3. Adjacent lane clearance gate.
    if proposed_action in ("lane_change_left", "lane_change_right"):
        direction = "left" if proposed_action == "lane_change_left" else "right"
        clearance = is_adjacent_lane_free(scene, direction)
        if not clearance.is_free:
            fallback = "decelerate" if ttc.ttc_s < caution_ttc_s else "idle"
            return SafetyVerdict(
                allowed=False,
                proposed_action=proposed_action,
                final_action=fallback,
                reason=clearance.reason,
            )

    # 4. Forward caution & speed-limit gate for acceleration.
    if proposed_action == "accelerate":
        if (ttc.ttc_s < caution_ttc_s) or (
            ttc.gap_m is not None and ttc.gap_m < caution_gap_m
        ):
            return SafetyVerdict(
                allowed=False,
                proposed_action=proposed_action,
                final_action="decelerate",
                reason=(
                    f"Lead actor {ttc.lead_actor_id} within caution envelope "
                    f"(gap={ttc.gap_m}m, TTC={ttc.ttc_s}s); cannot accelerate."
                ),
            )
        if scene.ego.speed_mps >= scene.ego.speed_limit_mps:
            return SafetyVerdict(
                allowed=False,
                proposed_action=proposed_action,
                final_action="idle",
                reason=(
                    f"At speed limit ({scene.ego.speed_mps:.1f} >= "
                    f"{scene.ego.speed_limit_mps:.1f} m/s); cannot accelerate."
                ),
            )

    return SafetyVerdict(
        allowed=True,
        proposed_action=proposed_action,
        final_action=proposed_action,
        reason="Passed all safety checks.",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify a proposed action on a recorded SceneState frame.")
    parser.add_argument("jsonl_path", type=Path, help="Path to a recorded SceneState .jsonl file.")
    parser.add_argument("action", help="Proposed meta-action to verify.")
    parser.add_argument("--frame", type=int, default=0, help="0-based frame index.")
    args = parser.parse_args()

    lines = args.jsonl_path.read_text(encoding="utf-8").splitlines()
    scene = SceneState.from_dict(json.loads(lines[args.frame]))
    verdict = verify_decision(scene, args.action)
    print(json.dumps(asdict(verdict), indent=2))


if __name__ == "__main__":
    main()
