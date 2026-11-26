#!/usr/bin/env python3
"""Run lightweight driving scenarios on a CARLA 0.9.16 server and log SceneState JSONL.

Designed to run on the CARLA Linux host (e.g., sim5) with `carla==0.9.16` installed.
Supports `--list` locally on Mac without requiring `carla`.
"""

from __future__ import annotations

import argparse
import json
import math
import queue
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from src.schemas import (  # noqa: E402
    ActorState,
    EgoState,
    RouteWindow,
    SceneState,
    TrafficLightState,
    UpcomingEvent,
)

TICK_RATE = 20.0
EGO_BLUEPRINT = "vehicle.tesla.model3"
NPC_BLUEPRINT = "vehicle.audi.a2"


@dataclass(frozen=True)
class CarlaScenarioSpec:
    name: str
    description: str
    setup_fn: str  # method name on ScenarioRunner


SCENARIOS: Dict[str, CarlaScenarioSpec] = {
    "autopilot_cruise": CarlaScenarioSpec(
        name="autopilot_cruise",
        description="Ego drives with Traffic Manager alongside 8 NPC vehicles.",
        setup_fn="_setup_autopilot_cruise",
    ),
    "lead_vehicle": CarlaScenarioSpec(
        name="lead_vehicle",
        description="Ego approaches a slower lead vehicle spawned 25m ahead in the same lane.",
        setup_fn="_setup_lead_vehicle",
    ),
    "adjacent_traffic": CarlaScenarioSpec(
        name="adjacent_traffic",
        description="Slow lead vehicle 25m ahead plus a vehicle in the adjacent left lane.",
        setup_fn="_setup_adjacent_traffic",
    ),
    "junction_approach": CarlaScenarioSpec(
        name="junction_approach",
        description="Ego spawns ~45m before a junction and approaches under Traffic Manager.",
        setup_fn="_setup_junction_approach",
    ),
}


def _speed_mps(actor: Any) -> float:
    v = actor.get_velocity()
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _world_to_ego(ego_tf: Any, target_tf: Any) -> tuple[float, float, float]:
    """Convert target world pose to ego-centric (x_ahead, y_left, rel_yaw_deg)."""
    dx = target_tf.location.x - ego_tf.location.x
    dy = target_tf.location.y - ego_tf.location.y
    yaw_rad = math.radians(ego_tf.rotation.yaw)
    cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
    # CARLA is left-handed (x forward, y right); negate lateral for y_left > 0.
    x_ahead = dx * cos_y + dy * sin_y
    y_left = -(-dx * sin_y + dy * cos_y)
    rel_yaw = (target_tf.rotation.yaw - ego_tf.rotation.yaw + 180.0) % 360.0 - 180.0
    return x_ahead, y_left, rel_yaw


def _lane_offset(ego_wp: Any, actor_wp: Any, y_left: float) -> int:
    if ego_wp.road_id == actor_wp.road_id:
        diff = actor_wp.lane_id - ego_wp.lane_id
        if diff != 0:
            return -1 if y_left > 0 else 1
        return 0
    if abs(y_left) < 1.8:
        return 0
    return -1 if y_left > 0 else 1


def _find_straight_spawn(carla_map: Any, min_clear_ahead_m: float = 50.0) -> Any:
    """Pick a spawn waypoint that has at least `min_clear_ahead_m` non-junction road ahead."""
    for tf in carla_map.get_spawn_points():
        wp = carla_map.get_waypoint(tf.location)
        if wp is None or wp.is_junction:
            continue
        ahead = wp.next(min_clear_ahead_m)
        if ahead and not ahead[0].is_junction:
            return wp
    raise RuntimeError("No suitable straight non-junction spawn point found on map")


def _find_junction_approach_spawn(carla_map: Any, target_dist_m: float = 45.0) -> Any:
    """Pick a spawn waypoint roughly `target_dist_m` before a junction."""
    for tf in carla_map.get_spawn_points():
        wp = carla_map.get_waypoint(tf.location)
        if wp is None or wp.is_junction:
            continue
        step_wps = wp.next(target_dist_m)
        if step_wps and step_wps[0].is_junction:
            return wp
    # Fallback: any valid spawn point
    return carla_map.get_waypoint(carla_map.get_spawn_points()[0].location)


class ScenarioRunner:
    def __init__(self, carla_mod: Any, client: Any, tm_port: int) -> None:
        self.carla = carla_mod
        self.client = client
        self.world = client.get_world()
        self.carla_map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()
        self.tm_port = tm_port
        self.spawned: List[Any] = []
        self.camera: Any = None

    def _spawn_vehicle(self, blueprint_id: str, tf: Any) -> Optional[Any]:
        bp = self.bp_lib.find(blueprint_id)
        for _ in range(3):
            actor = self.world.try_spawn_actor(bp, tf)
            if actor is not None:
                self.spawned.append(actor)
                return actor
            self.world.tick()
        return None

    def _lift_tf(self, wp: Any) -> Any:
        tf = wp.transform
        tf.location.z += 0.3
        return tf

    def _setup_autopilot_cruise(self, tm: Any) -> Any:
        spawns = self.carla_map.get_spawn_points()
        ego = None
        for tf in spawns[:10]:
            ego = self._spawn_vehicle(EGO_BLUEPRINT, tf)
            if ego is not None:
                break
        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle")
        ego.set_autopilot(True, self.tm_port)

        for tf in spawns[10:25]:
            npc = self._spawn_vehicle(NPC_BLUEPRINT, tf)
            if npc is not None:
                npc.set_autopilot(True, self.tm_port)
            if len(self.spawned) >= 9:
                break
        return ego

    def _setup_lead_vehicle(self, tm: Any) -> Any:
        ego_wp = _find_straight_spawn(self.carla_map, min_clear_ahead_m=45.0)
        ego = self._spawn_vehicle(EGO_BLUEPRINT, self._lift_tf(ego_wp))
        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle")

        lead_wps = ego_wp.next(25.0)
        if lead_wps:
            lead = self._spawn_vehicle(NPC_BLUEPRINT, self._lift_tf(lead_wps[0]))
            if lead is not None:
                lead.set_autopilot(True, self.tm_port)
                tm.vehicle_percentage_speed_difference(lead, 50.0)

        ego.set_autopilot(True, self.tm_port)
        return ego

    def _setup_adjacent_traffic(self, tm: Any) -> Any:
        ego_wp = _find_straight_spawn(self.carla_map, min_clear_ahead_m=45.0)
        ego = self._spawn_vehicle(EGO_BLUEPRINT, self._lift_tf(ego_wp))
        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle")

        lead_wps = ego_wp.next(25.0)
        if lead_wps:
            lead = self._spawn_vehicle(NPC_BLUEPRINT, self._lift_tf(lead_wps[0]))
            if lead is not None:
                lead.set_autopilot(True, self.tm_port)
                tm.vehicle_percentage_speed_difference(lead, 50.0)

        left_wp = ego_wp.get_left_lane()
        if left_wp is not None and str(left_wp.lane_type) == "Driving":
            adj = self._spawn_vehicle(NPC_BLUEPRINT, self._lift_tf(left_wp))
            if adj is not None:
                adj.set_autopilot(True, self.tm_port)

        ego.set_autopilot(True, self.tm_port)
        return ego

    def _setup_junction_approach(self, tm: Any) -> Any:
        ego_wp = _find_junction_approach_spawn(self.carla_map, target_dist_m=45.0)
        ego = self._spawn_vehicle(EGO_BLUEPRINT, self._lift_tf(ego_wp))
        if ego is None:
            raise RuntimeError("Failed to spawn ego vehicle")
        ego.set_autopilot(True, self.tm_port)
        return ego

    def _capture_scene(self, scenario_name: str, description: str, ego: Any, tick_idx: int) -> SceneState:
        ego_tf = ego.get_transform()
        ego_wp = self.carla_map.get_waypoint(ego_tf.location)
        speed = _speed_mps(ego)

        tl_state_str = "none"
        tl_dist: Optional[float] = None
        if ego.is_at_traffic_light():
            tl = ego.get_traffic_light()
            if tl is not None:
                tl_state_str = str(tl.get_state()).lower()
                tl_dist = round(ego_tf.location.distance(tl.get_transform().location), 2)

        actors: List[ActorState] = []
        for other in self.world.get_actors().filter("vehicle.*"):
            if other.id == ego.id:
                continue
            other_tf = other.get_transform()
            if ego_tf.location.distance(other_tf.location) > 60.0:
                continue
            x_ahead, y_left, rel_yaw = _world_to_ego(ego_tf, other_tf)
            other_wp = self.carla_map.get_waypoint(other_tf.location)
            actors.append(
                ActorState(
                    id=f"veh_{other.id}",
                    kind="vehicle",
                    x_m=round(x_ahead, 2),
                    y_m=round(y_left, 2),
                    speed_mps=round(_speed_mps(other), 2),
                    yaw_deg=round(rel_yaw, 1),
                    lane_offset=_lane_offset(ego_wp, other_wp, y_left),
                )
            )

        upcoming: List[UpcomingEvent] = []
        for dist in (20.0, 40.0, 60.0):
            nxt = ego_wp.next(dist)
            if nxt and nxt[0].is_junction:
                upcoming.append(
                    UpcomingEvent(
                        kind="junction",
                        distance_m=dist,
                        description="Approaching junction ahead.",
                    )
                )
                break

        allowed_lc: List[str] = []
        left_wp = ego_wp.get_left_lane()
        right_wp = ego_wp.get_right_lane()
        if left_wp is not None and str(left_wp.lane_type) == "Driving":
            allowed_lc.append("left")
        if right_wp is not None and str(right_wp.lane_type) == "Driving":
            allowed_lc.append("right")

        speed_limit_mps = round(ego.get_speed_limit() / 3.6, 2)
        return SceneState(
            scenario_id=scenario_name,
            description=description,
            timestamp_s=round(tick_idx / TICK_RATE, 2),
            ego=EgoState(
                x_m=round(ego_tf.location.x, 2),
                y_m=round(ego_tf.location.y, 2),
                yaw_deg=round(ego_tf.rotation.yaw, 1),
                speed_mps=round(speed, 2),
                lane_id=ego_wp.lane_id,
                speed_limit_mps=speed_limit_mps,
            ),
            traffic_light=TrafficLightState(state=tl_state_str, distance_m=tl_dist),
            route_window=RouteWindow(
                target_lane_id=ego_wp.lane_id,
                recommended_speed_mps=speed_limit_mps,
                allowed_lane_changes=allowed_lc,
                upcoming_events=upcoming,
            ),
            actors=actors,
        )

    def run_scenario(
        self,
        spec: CarlaScenarioSpec,
        ticks: int,
        out_dir: Path,
        record_video: bool,
    ) -> Path:
        original_settings = self.world.get_settings()
        tm = None
        images: List[Any] = []
        cam_queue: queue.Queue[Any] = queue.Queue()
        out_dir.mkdir(parents=True, exist_ok=True)
        jsonl_path = out_dir / f"{spec.name}.jsonl"

        try:
            # Recover from any frozen synchronous state first.
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            self.world.apply_settings(settings)
            for actor in self.world.get_actors().filter("vehicle.*"):
                actor.destroy()

            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / TICK_RATE
            self.world.apply_settings(settings)

            tm = self.client.get_trafficmanager(self.tm_port)
            tm.set_synchronous_mode(True)

            setup: Callable[[Any], Any] = getattr(self, spec.setup_fn)
            ego = setup(tm)

            # Tick once so actor transforms settle.
            for _ in range(3):
                self.world.tick()

            if record_video:
                cam_bp = self.bp_lib.find("sensor.camera.rgb")
                cam_bp.set_attribute("image_size_x", "960")
                cam_bp.set_attribute("image_size_y", "540")
                cam_bp.set_attribute("fov", "90")
                cam_tf = self.carla.Transform(
                    self.carla.Location(x=-8.5, z=3.5),
                    self.carla.Rotation(pitch=-20.0),
                )
                self.camera = self.world.spawn_actor(
                    cam_bp,
                    cam_tf,
                    attach_to=ego,
                    attachment_type=self.carla.AttachmentType.Rigid,
                )
                self.camera.listen(cam_queue.put)

            with jsonl_path.open("w", encoding="utf-8") as f:
                for tick_idx in range(ticks):
                    self.world.tick()
                    while not cam_queue.empty():
                        images.append(cam_queue.get())
                    scene = self._capture_scene(spec.name, spec.description, ego, tick_idx)
                    f.write(json.dumps(scene.to_dict()) + "\n")

            if record_video and images:
                _write_mp4(images, out_dir / f"{spec.name}.mp4")

            print(f"[{spec.name}] Logged {ticks} ticks -> {jsonl_path}")
            return jsonl_path
        finally:
            if self.camera is not None:
                time.sleep(0.1)
                self.camera.stop()
                self.camera.destroy()
                self.camera = None
            for actor in self.spawned:
                if actor.is_alive:
                    actor.destroy()
            self.spawned.clear()
            if tm is not None:
                tm.set_synchronous_mode(False)
            self.world.apply_settings(original_settings)


def _write_mp4(images: List[Any], out_path: Path) -> None:
    import imageio.v2 as imageio
    import numpy as np

    h, w = images[0].height, images[0].width
    writer = imageio.get_writer(str(out_path), fps=int(TICK_RATE))
    for img in images:
        pixels = np.frombuffer(img.raw_data, dtype=np.uint8).reshape(h, w, 4)
        writer.append_data(pixels[:, :, :3][:, :, ::-1])
    writer.close()
    print(f"Saved video -> {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run CARLA scenarios and log SceneState JSONL.")
    parser.add_argument("--list", action="store_true", help="List available scenarios and exit.")
    parser.add_argument(
        "--scenario",
        default="autopilot_cruise",
        choices=[*SCENARIOS.keys(), "all"],
        help="Scenario to run (default: autopilot_cruise).",
    )
    parser.add_argument("--host", default="localhost", help="CARLA host (default: localhost).")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port (default: 2000).")
    parser.add_argument("--tm-port", type=int, default=8000, help="Traffic Manager port.")
    parser.add_argument("--ticks", type=int, default=120, help="Ticks per scenario at 20 Hz.")
    parser.add_argument("--out-dir", type=Path, default=Path("recordings"), help="Output directory.")
    parser.add_argument("--record", action="store_true", help="Also save chase camera MP4.")
    args = parser.parse_args()

    if args.list:
        for spec in SCENARIOS.values():
            print(f"- {spec.name}: {spec.description}")
        return 0

    try:
        import carla  # type: ignore
    except ImportError:
        print(
            "Error: `carla` package is not installed here. Run this script on the CARLA Linux server.",
            file=sys.stderr,
        )
        return 1

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    print(f"Connected to CARLA {args.host}:{args.port} (server {client.get_server_version()})")

    runner = ScenarioRunner(carla, client, tm_port=args.tm_port)
    targets = list(SCENARIOS.values()) if args.scenario == "all" else [SCENARIOS[args.scenario]]
    for spec in targets:
        runner.run_scenario(spec, ticks=args.ticks, out_dir=args.out_dir, record_video=args.record)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
