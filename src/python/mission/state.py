"""
Mission State Management for Satellite Control System

Runtime mission state tracking for waypoint navigation and shape following.
Maintains mutable state variables for mission execution and phase transitions.

Mission type supported:
1. Path Following: MPCC path tracking with 3D waypoints

State tracking is split into component dataclasses for better organization.

Key features:
- Clean separation of mutable state from immutable config
- Type-safe dataclasses with default values
- Thread-safe for concurrent access
- Reset functionality for mission restart
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from config.timing import CONTROL_DT, DEFAULT_PATH_SPEED

DEFAULT_PATH_HOLD_END_S = 10.0


@dataclass
class PathFollowingState:
    """State for MPCC path-following missions."""

    active: bool = False
    waypoints: list[tuple[float, float, float]] = field(default_factory=list)
    path_speed: float = DEFAULT_PATH_SPEED
    path_length: float = 0.0


@dataclass
class MissionState:
    """
    Mission state tracking for runtime execution.

    Composes path-following runtime state.
    """

    path: PathFollowingState = field(default_factory=PathFollowingState)
    path_hold_end: float = DEFAULT_PATH_HOLD_END_S
    # Optional per-waypoint hold schedule: list of {"path_index": int, "duration_s": float}.
    path_hold_schedule: list[dict[str, float]] = field(default_factory=list)
    path_hold_active_index: int | None = None
    path_hold_started_at_s: float | None = None
    path_hold_completed: set[int] = field(default_factory=set)
    # Path tracking runtime fields.
    path_tracking_center: tuple[float, float, float] | None = None
    path_tracking_base_shape: list[tuple[float, float, float]] = field(
        default_factory=list
    )
    path_tracking_phase: str = "POSITIONING"
    path_tracking_closest_point_index: int = 0
    path_tracking_estimated_duration: float = 0.0
    path_tracking_mission_start_time: float | None = None
    path_tracking_tracking_start_time: float | None = None
    path_tracking_positioning_start_time: float | None = None
    path_tracking_stabilization_start_time: float | None = None
    path_tracking_current_target_position: tuple[float, float, float] | None = None
    path_tracking_final_position: tuple[float, float, float] | None = None
    path_tracking_target_start_distance: float = 0.0
    path_tracking_has_return: bool = False
    path_tracking_return_position: tuple[float, float, float] | None = None
    path_tracking_return_angle: tuple[float, float, float] = (0.0, 0.0, 0.0)
    path_tracking_trajectory: Any | None = None
    path_tracking_trajectory_dt: float = CONTROL_DT
    # Origin of the simulation frame in ECI coordinates [x, y, z]
    frame_origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Path frame used for compiled mission waypoints ("ECI" or "LVLH").
    path_frame: str = "LVLH"
    # Optional scan-attitude context for MPC (+Z alignment and object-facing side).
    scan_attitude_center: tuple[float, float, float] | None = None
    scan_attitude_axis: tuple[float, float, float] | None = None
    scan_attitude_direction: str = "CW"
    # Runtime segment-wise pointing spans indexed by path arc-length.
    pointing_path_spans: list[dict[str, Any]] = field(default_factory=list)
    # In-memory migration notices populated during mission compile/load.
    scan_axis_migration_notices: list[str] = field(default_factory=list)
    # Optional viewer metadata for playback reference object rendering.
    visualization_scan_object: dict[str, Any] | None = None

    # --- Path Following ---

    @property
    def path_following_active(self) -> bool:
        return self.path.active

    @path_following_active.setter
    def path_following_active(self, value: bool):
        self.path.active = value

    @property
    def path_waypoints(self) -> list[tuple[float, float, float]]:
        return self.path.waypoints

    @path_waypoints.setter
    def path_waypoints(self, value: list[tuple[float, float, float]]):
        self.path.waypoints = value

    @property
    def path_length(self) -> float:
        return self.path.path_length

    @path_length.setter
    def path_length(self, value: float):
        self.path.path_length = value

    @property
    def path_speed(self) -> float:
        return self.path.path_speed

    @path_speed.setter
    def path_speed(self, value: float):
        self.path.path_speed = value

    def get_current_mission_type(self) -> str:
        """Return the active mission type as a string."""
        if self.path.active:
            return "PATH_FOLLOWING"
        return "IDLE"

    def reset(self) -> None:
        """Reset mission state to initial defaults."""
        self.path.active = False
        self.path_tracking_phase = "POSITIONING"

    def get_resolved_path_waypoints(self) -> list[tuple[float, float, float]]:
        """Return canonical mission path waypoints."""
        return self.path.waypoints

    def get_resolved_path_length(self, compute_if_missing: bool = True) -> float:
        """
        Return path length with optional waypoint fallback.

        Args:
            compute_if_missing: If True, compute polyline length from waypoints
                when explicit length is not set.
        """
        path_length = float(self.path.path_length or 0.0)
        if path_length > 0.0:
            return path_length

        path = self.path.waypoints
        if compute_if_missing and path and len(path) > 1:
            total = 0.0
            for i in range(1, len(path)):
                x0, y0, z0 = path[i - 1]
                x1, y1, z1 = path[i]
                dx = float(x1) - float(x0)
                dy = float(y1) - float(y0)
                dz = float(z1) - float(z0)
                total += math.sqrt(dx * dx + dy * dy + dz * dz)
            return total

        return 0.0


def create_mission_state() -> MissionState:
    """
    Create a new mission state with default values.

    Returns:
        MissionState initialized to defaults
    """
    return MissionState()
