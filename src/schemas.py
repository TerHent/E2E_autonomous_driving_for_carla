"""Minimal data contracts for local scene states, route windows, and test scenarios."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ActorState:
    """Surrounding traffic participant in ego-centric road coordinates.

    x_m: longitudinal distance ahead of ego (meters, negative = behind).
    y_m: lateral offset from ego center (meters, positive = left, negative = right).
    lane_offset: relative lane index (-1 = left, 0 = ego lane, +1 = right).
    """

    id: str
    kind: str  # "vehicle" | "pedestrian" | "cyclist"
    x_m: float
    y_m: float
    speed_mps: float
    yaw_deg: float
    lane_offset: int


@dataclass
class TrafficLightState:
    """Traffic light affecting the ego vehicle's current approach."""

    state: str  # "red" | "yellow" | "green" | "none"
    distance_m: Optional[float] = None


@dataclass
class EgoState:
    """Current kinematic and lane state of the ego vehicle."""

    x_m: float
    y_m: float
    yaw_deg: float
    speed_mps: float
    lane_id: int
    speed_limit_mps: float


@dataclass
class UpcomingEvent:
    """Upcoming road/navigation event published by long-term planning."""

    kind: str  # "junction" | "turn_left" | "turn_right" | "crosswalk" | "roadblock"
    distance_m: float
    description: str


@dataclass
class RouteWindow:
    """Rolling route window produced by long-term planning (Matthew's layer)."""

    target_lane_id: int
    recommended_speed_mps: float
    allowed_lane_changes: List[str] = field(default_factory=list)  # "left", "right"
    upcoming_events: List[UpcomingEvent] = field(default_factory=list)


@dataclass
class SceneState:
    """Compact local scene snapshot consumed by the tactical layer (Timur's layer)."""

    scenario_id: str
    description: str
    timestamp_s: float
    ego: EgoState
    traffic_light: TrafficLightState
    route_window: RouteWindow
    actors: List[ActorState] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> SceneState:
        rw_data = data["route_window"]
        return cls(
            scenario_id=data["scenario_id"],
            description=data["description"],
            timestamp_s=data["timestamp_s"],
            ego=EgoState(**data["ego"]),
            traffic_light=TrafficLightState(**data["traffic_light"]),
            route_window=RouteWindow(
                target_lane_id=rw_data["target_lane_id"],
                recommended_speed_mps=rw_data["recommended_speed_mps"],
                allowed_lane_changes=list(rw_data.get("allowed_lane_changes", [])),
                upcoming_events=[
                    UpcomingEvent(**ev) for ev in rw_data.get("upcoming_events", [])
                ],
            ),
            actors=[ActorState(**a) for a in data.get("actors", [])],
        )


@dataclass
class ScenarioExpectation:
    """Expected tactical decision bounds for testing planners and safety vetoes."""

    allowed_actions: List[str]
    forbidden_actions: List[str]
    rationale: str


@dataclass
class ScenarioCase:
    """A self-contained test scenario pairing a SceneState with expected decisions."""

    scene: SceneState
    expectation: ScenarioExpectation

    def to_dict(self) -> Dict[str, Any]:
        return {
            "scene": self.scene.to_dict(),
            "expectation": asdict(self.expectation),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> ScenarioCase:
        return cls(
            scene=SceneState.from_dict(data["scene"]),
            expectation=ScenarioExpectation(**data["expectation"]),
        )
