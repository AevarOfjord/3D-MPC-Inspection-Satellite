"""
Shared unified-mission loading pipeline for CLI and dashboard entry points.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.mission.mission_types import Obstacle
from src.satellite_control.mission.unified_compiler import compile_unified_mission_path
from src.satellite_control.mission.unified_mission import MissionDefinition


@dataclass
class UnifiedMissionRuntime:
    """Compiled mission artifacts required by runtime entry points."""

    simulation_config: SimulationConfig
    path: list[Tuple[float, float, float]]
    path_length: float
    path_speed: float
    start_pos: Tuple[float, float, float]
    end_pos: Tuple[float, float, float]


def parse_unified_mission_payload(payload: Mapping[str, Any]) -> MissionDefinition:
    """
    Parse a unified mission payload.

    Raises:
        ValueError: if the payload does not match the unified mission v2 contract.
    """
    if not isinstance(payload, Mapping):
        raise ValueError("Mission payload must be an object.")
    if "segments" not in payload or "start_pose" not in payload:
        raise ValueError(
            "Unsupported legacy mission format. Expected unified mission v2."
        )
    try:
        return MissionDefinition.from_dict(dict(payload))
    except Exception as exc:
        raise ValueError(f"Invalid unified mission: {exc}") from exc


def compile_unified_mission_runtime(
    mission: MissionDefinition,
    *,
    simulation_config: Optional[SimulationConfig] = None,
    output_frame: Optional[str] = None,
) -> UnifiedMissionRuntime:
    """
    Compile a unified mission into a simulation-ready configuration.
    """
    sim_config = simulation_config or SimulationConfig.create_default()

    path, path_length, path_speed = compile_unified_mission_path(
        mission=mission,
        sim_config=sim_config,
        output_frame=output_frame,
    )

    sim_config.app_config.mpc.path_speed = float(path_speed)
    mission_state = sim_config.mission_state
    mission_state.obstacles = _to_runtime_obstacles(mission.obstacles)
    mission_state.obstacles_enabled = bool(mission_state.obstacles)
    mission_state.path_waypoints = path
    mission_state.path_length = float(path_length)
    mission_state.path_speed = float(path_speed)

    start_pos = tuple(path[0]) if path else tuple(mission.start_pose.position)
    end_pos = tuple(path[-1]) if path else start_pos

    return UnifiedMissionRuntime(
        simulation_config=sim_config,
        path=path,
        path_length=float(path_length),
        path_speed=float(path_speed),
        start_pos=start_pos,
        end_pos=end_pos,
    )


def _to_runtime_obstacles(
    obstacles: Sequence[Any],
) -> list[Obstacle]:
    runtime_obstacles: list[Obstacle] = []
    for obstacle in obstacles:
        runtime_obstacles.append(
            Obstacle(
                position=np.array(obstacle.position, dtype=float),
                radius=float(obstacle.radius),
            )
        )
    return runtime_obstacles
