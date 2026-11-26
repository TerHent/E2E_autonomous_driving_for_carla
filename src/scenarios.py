"""Hand-crafted tactical driving scenarios for offline testing without CARLA."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List

from src.schemas import (
    ActorState,
    EgoState,
    RouteWindow,
    ScenarioCase,
    ScenarioExpectation,
    SceneState,
    TrafficLightState,
    UpcomingEvent,
)


def build_all_scenarios() -> List[ScenarioCase]:
    """Return the core suite of 5 tactical test scenarios."""
    return [
        ScenarioCase(
            scene=SceneState(
                scenario_id="red_light_stop",
                description="Approaching a red traffic light with a slowing lead vehicle.",
                timestamp_s=12.4,
                ego=EgoState(
                    x_m=0.0,
                    y_m=0.0,
                    yaw_deg=0.0,
                    speed_mps=11.0,
                    lane_id=-1,
                    speed_limit_mps=13.9,
                ),
                traffic_light=TrafficLightState(state="red", distance_m=32.0),
                route_window=RouteWindow(
                    target_lane_id=-1,
                    recommended_speed_mps=11.0,
                    allowed_lane_changes=[],
                    upcoming_events=[
                        UpcomingEvent(
                            kind="junction",
                            distance_m=35.0,
                            description="Signalized 4-way intersection ahead.",
                        )
                    ],
                ),
                actors=[
                    ActorState(
                        id="veh_lead",
                        kind="vehicle",
                        x_m=18.0,
                        y_m=0.0,
                        speed_mps=3.5,
                        yaw_deg=0.0,
                        lane_offset=0,
                    )
                ],
            ),
            expectation=ScenarioExpectation(
                allowed_actions=["decelerate", "idle"],
                forbidden_actions=["accelerate", "lane_change_left", "lane_change_right"],
                rationale="Red light at 32m and slowing lead vehicle require braking.",
            ),
        ),
        ScenarioCase(
            scene=SceneState(
                scenario_id="slow_lead_overtake_left_free",
                description="Slow truck ahead in ego lane; left lane is clear for overtaking.",
                timestamp_s=24.0,
                ego=EgoState(
                    x_m=0.0,
                    y_m=0.0,
                    yaw_deg=0.0,
                    speed_mps=13.0,
                    lane_id=-2,
                    speed_limit_mps=13.9,
                ),
                traffic_light=TrafficLightState(state="none"),
                route_window=RouteWindow(
                    target_lane_id=-2,
                    recommended_speed_mps=13.5,
                    allowed_lane_changes=["left"],
                    upcoming_events=[],
                ),
                actors=[
                    ActorState(
                        id="veh_slow",
                        kind="vehicle",
                        x_m=22.0,
                        y_m=0.0,
                        speed_mps=5.0,
                        yaw_deg=0.0,
                        lane_offset=0,
                    ),
                    ActorState(
                        id="veh_left_far",
                        kind="vehicle",
                        x_m=-55.0,
                        y_m=3.5,
                        speed_mps=12.0,
                        yaw_deg=0.0,
                        lane_offset=-1,
                    ),
                ],
            ),
            expectation=ScenarioExpectation(
                allowed_actions=["lane_change_left", "decelerate"],
                forbidden_actions=["accelerate", "lane_change_right"],
                rationale="Left lane is free and permitted; overtaking or slowing are safe.",
            ),
        ),
        ScenarioCase(
            scene=SceneState(
                scenario_id="slow_lead_left_blocked",
                description="Slow car ahead, but fast vehicle is closing in the left lane.",
                timestamp_s=31.2,
                ego=EgoState(
                    x_m=0.0,
                    y_m=0.0,
                    yaw_deg=0.0,
                    speed_mps=12.5,
                    lane_id=-2,
                    speed_limit_mps=13.9,
                ),
                traffic_light=TrafficLightState(state="none"),
                route_window=RouteWindow(
                    target_lane_id=-2,
                    recommended_speed_mps=13.0,
                    allowed_lane_changes=["left"],
                    upcoming_events=[],
                ),
                actors=[
                    ActorState(
                        id="veh_slow",
                        kind="vehicle",
                        x_m=20.0,
                        y_m=0.0,
                        speed_mps=6.0,
                        yaw_deg=0.0,
                        lane_offset=0,
                    ),
                    ActorState(
                        id="veh_left_fast",
                        kind="vehicle",
                        x_m=-8.0,
                        y_m=3.5,
                        speed_mps=18.5,
                        yaw_deg=0.0,
                        lane_offset=-1,
                    ),
                ],
            ),
            expectation=ScenarioExpectation(
                allowed_actions=["decelerate", "idle"],
                forbidden_actions=["lane_change_left", "accelerate"],
                rationale="Fast approaching vehicle in left lane makes lane change unsafe.",
            ),
        ),
        ScenarioCase(
            scene=SceneState(
                scenario_id="upcoming_right_turn_merge",
                description="Route requires turning right in 75m; ego is in left lane with clear right gap.",
                timestamp_s=45.6,
                ego=EgoState(
                    x_m=0.0,
                    y_m=0.0,
                    yaw_deg=0.0,
                    speed_mps=10.5,
                    lane_id=-1,
                    speed_limit_mps=13.9,
                ),
                traffic_light=TrafficLightState(state="green", distance_m=72.0),
                route_window=RouteWindow(
                    target_lane_id=-2,
                    recommended_speed_mps=10.0,
                    allowed_lane_changes=["right"],
                    upcoming_events=[
                        UpcomingEvent(
                            kind="turn_right",
                            distance_m=75.0,
                            description="Right turn onto Oak Ave; move to rightmost lane (-2).",
                        )
                    ],
                ),
                actors=[
                    ActorState(
                        id="veh_right_behind",
                        kind="vehicle",
                        x_m=-30.0,
                        y_m=-3.5,
                        speed_mps=10.0,
                        yaw_deg=0.0,
                        lane_offset=1,
                    )
                ],
            ),
            expectation=ScenarioExpectation(
                allowed_actions=["lane_change_right"],
                forbidden_actions=["lane_change_left"],
                rationale="Route window targets lane -2 for upcoming right turn and gap is open.",
            ),
        ),
        ScenarioCase(
            scene=SceneState(
                scenario_id="emergency_pedestrian_crossing",
                description="Pedestrian steps into crosswalk 14m ahead while ego is moving at 10 m/s.",
                timestamp_s=58.1,
                ego=EgoState(
                    x_m=0.0,
                    y_m=0.0,
                    yaw_deg=0.0,
                    speed_mps=10.0,
                    lane_id=-1,
                    speed_limit_mps=11.1,
                ),
                traffic_light=TrafficLightState(state="green", distance_m=20.0),
                route_window=RouteWindow(
                    target_lane_id=-1,
                    recommended_speed_mps=10.0,
                    allowed_lane_changes=[],
                    upcoming_events=[
                        UpcomingEvent(
                            kind="crosswalk",
                            distance_m=14.0,
                            description="Unsignalized pedestrian crosswalk.",
                        )
                    ],
                ),
                actors=[
                    ActorState(
                        id="ped_crossing",
                        kind="pedestrian",
                        x_m=14.0,
                        y_m=0.8,
                        speed_mps=1.4,
                        yaw_deg=-90.0,
                        lane_offset=0,
                    )
                ],
            ),
            expectation=ScenarioExpectation(
                allowed_actions=["decelerate"],
                forbidden_actions=["accelerate", "idle", "lane_change_left", "lane_change_right"],
                rationale="Immediate collision risk with pedestrian (TTC ~1.4s) demands braking.",
            ),
        ),
    ]


def export_scenarios(output_dir: Path) -> List[Path]:
    """Write each scenario as an indented JSON file for inspection and sharing."""
    output_dir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []
    for case in build_all_scenarios():
        path = output_dir / f"{case.scene.scenario_id}.json"
        path.write_text(json.dumps(case.to_dict(), indent=2) + "\n", encoding="utf-8")
        written.append(path)
    return written


def main() -> None:
    parser = argparse.ArgumentParser(description="List or export tactical driving scenarios.")
    parser.add_argument(
        "--export",
        type=Path,
        default=None,
        help="Optional directory to write scenario JSON files into.",
    )
    args = parser.parse_args()

    cases = build_all_scenarios()
    for case in cases:
        s = case.scene
        print(
            f"- {s.scenario_id}: speed={s.ego.speed_mps:.1f} m/s, "
            f"light={s.traffic_light.state}, actors={len(s.actors)}, "
            f"allowed={case.expectation.allowed_actions}"
        )

    if args.export is not None:
        paths = export_scenarios(args.export)
        print(f"Exported {len(paths)} scenarios to {args.export}")


if __name__ == "__main__":
    main()
