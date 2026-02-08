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

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any

import math

from . import timing


@dataclass
class PathFollowingState:
    """State for MPCC path-following missions."""

    active: bool = False
    waypoints: List[Tuple[float, float, float]] = field(default_factory=list)
    path_speed: float = timing.DEFAULT_PATH_SPEED
    path_length: float = 0.0


@dataclass
class ObstacleState:
    """State for obstacle avoidance."""

    enabled: bool = False
    obstacles: List[Any] = field(default_factory=list)


@dataclass
class MissionState:
    """
    Mission state tracking for runtime execution.

    Composes path-following and obstacle runtime state.
    Maintains a small set of compatibility aliases for path fields.
    """

    path: PathFollowingState = field(default_factory=PathFollowingState)
    obstacle_state: ObstacleState = field(default_factory=ObstacleState)
    path_hold_end: float = 0.0
    # Path tracking runtime fields.
    path_tracking_center: Optional[Tuple[float, float, float]] = None
    path_tracking_base_shape: List[Tuple[float, float, float]] = field(
        default_factory=list
    )
    path_tracking_phase: str = "POSITIONING"
    path_tracking_closest_point_index: int = 0
    path_tracking_estimated_duration: float = 0.0
    path_tracking_mission_start_time: Optional[float] = None
    path_tracking_tracking_start_time: Optional[float] = None
    path_tracking_positioning_start_time: Optional[float] = None
    path_tracking_stabilization_start_time: Optional[float] = None
    path_tracking_current_target_position: Optional[Tuple[float, float, float]] = None
    path_tracking_final_position: Optional[Tuple[float, float, float]] = None
    path_tracking_target_start_distance: float = 0.0
    path_tracking_has_return: bool = False
    path_tracking_return_position: Optional[Tuple[float, float, float]] = None
    path_tracking_return_angle: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    path_tracking_trajectory: Optional[Any] = None
    path_tracking_trajectory_dt: float = timing.CONTROL_DT

    # --- Path Following ---

    @property
    def path_following_active(self) -> bool:
        return self.path.active

    @path_following_active.setter
    def path_following_active(self, value: bool):
        self.path.active = value

    @property
    def path_waypoints(self) -> List[Tuple[float, float, float]]:
        return self.path.waypoints

    @path_waypoints.setter
    def path_waypoints(self, value: List[Tuple[float, float, float]]):
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

    # --- Obstacles ---
    @property
    def obstacles_enabled(self) -> bool:
        return self.obstacle_state.enabled

    @obstacles_enabled.setter
    def obstacles_enabled(self, value: bool):
        self.obstacle_state.enabled = value

    @property
    def obstacles(self) -> List[Any]:
        return self.obstacle_state.obstacles

    @obstacles.setter
    def obstacles(self, value: List[Any]):
        self.obstacle_state.obstacles = value

    # --- Methods ---

    def reset(self) -> None:
        """Reset all mission state to defaults."""
        self.path = PathFollowingState()
        self.obstacle_state = ObstacleState()
        self.path_hold_end = 0.0
        self.path_tracking_center = None
        self.path_tracking_base_shape = []
        self.path_tracking_phase = "POSITIONING"
        self.path_tracking_closest_point_index = 0
        self.path_tracking_estimated_duration = 0.0
        self.path_tracking_mission_start_time = None
        self.path_tracking_tracking_start_time = None
        self.path_tracking_positioning_start_time = None
        self.path_tracking_stabilization_start_time = None
        self.path_tracking_current_target_position = None
        self.path_tracking_final_position = None
        self.path_tracking_target_start_distance = 0.0
        self.path_tracking_has_return = False
        self.path_tracking_return_position = None
        self.path_tracking_return_angle = (0.0, 0.0, 0.0)
        self.path_tracking_trajectory = None
        self.path_tracking_trajectory_dt = timing.CONTROL_DT

    def get_current_mission_type(self) -> str:
        """
        Get the currently active mission type.

        Returns:
            PATH_FOLLOWING or NONE
        """
        if self.path_waypoints:
            return "PATH_FOLLOWING"
        return "NONE"

    def get_resolved_path_waypoints(self) -> List[Tuple[float, float, float]]:
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


def print_mission_state(state: MissionState) -> None:
    """Print current mission state."""
    print("=" * 80)
    print("MISSION STATE")
    print("=" * 80)

    mission_type = state.get_current_mission_type()
    print(f"\nMission: {mission_type}")

    if state.path_waypoints:
        print("\nPath Following:")
        print(f"  Points: {len(state.path_waypoints)}")
        print(f"  Length: {state.path_length:.2f} m")

    print("=" * 80)
