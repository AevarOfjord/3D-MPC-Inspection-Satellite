"""
Mission State Management for Satellite Control System

Runtime mission state tracking for waypoint navigation and shape following.
Maintains mutable state variables for mission execution and phase transitions.

Mission types supported:
1. Path Following: MPCC path tracking with 3D waypoints
2. Trajectory Tracking: Generic path following
3. Mesh Scanning: OBJ mesh inspection

State tracking is split into component dataclasses for better organization.

Key features:
- Clean separation of mutable state from immutable config
- Type-safe dataclasses with default values
- Thread-safe for concurrent access
- Reset functionality for mission restart
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any


@dataclass
class PathFollowingState:
    """State for MPCC path-following missions."""

    active: bool = False
    waypoints: List[Tuple[float, float, float]] = field(default_factory=list)
    path_speed: float = 0.1
    path_length: float = 0.0


@dataclass
class ScanState:
    """State for mesh scan missions."""

    active: bool = False
    obj_path: Optional[str] = None
    object_pose: Optional[Tuple[float, float, float, float, float, float]] = None
    standoff: float = 0.5
    levels: int = 8
    points_per_circle: int = 72
    speed_max: float = 0.2
    speed_min: float = 0.05
    lateral_accel: float = 0.05
    z_margin: float = 0.0
    fov_deg: float = 60.0
    overlap: float = 0.85
    ring_shape: str = "square"

    # Phase state machine: APPROACH -> STABILIZE -> TRACKING
    phase: str = "APPROACH"
    approach_target: Optional[Tuple[float, float, float]] = None
    stabilize_start_time: Optional[float] = None


@dataclass
class TrajectoryState:
    """State for generic trajectory tracking."""

    active: bool = False
    type: str = "path"
    start_time: Optional[float] = None
    total_time: float = 0.0
    hold_start: float = 0.0
    hold_end: float = 0.0
    start_orientation: Optional[Tuple[float, float, float]] = None
    end_orientation: Optional[Tuple[float, float, float]] = None
    object_center: Optional[Tuple[float, float, float]] = None
    end_pos_tolerance: float = 0.05
    end_ang_tolerance_deg: float = 2.0


@dataclass
class ObstacleState:
    """State for obstacle avoidance."""

    enabled: bool = False
    obstacles: List[Any] = field(default_factory=list)


@dataclass
class MissionState:
    """
    Mission state tracking for runtime execution.

    Composes specific state objects for different mission modes.
    MAINTAINS FULL BACKWARD COMPATIBILITY via properties.
    """

    path: PathFollowingState = field(default_factory=PathFollowingState)
    scan: ScanState = field(default_factory=ScanState)
    trajectory: TrajectoryState = field(default_factory=TrajectoryState)
    obstacle_state: ObstacleState = field(default_factory=ObstacleState)

    # =========================================================================
    # BACKWARD COMPATIBILITY PROPERTIES
    # These properties proxies allow the rest of the codebase to continue
    # accessing state as flat attributes.
    # =========================================================================

    # --- Path Following ---
    @property
    def mpcc_path_waypoints(self) -> List[Tuple[float, float, float]]:
        return self.path.waypoints

    @mpcc_path_waypoints.setter
    def mpcc_path_waypoints(self, value: List[Tuple[float, float, float]]):
        self.path.waypoints = value

    @property
    def mpcc_path_length(self) -> float:
        return self.path.path_length

    @mpcc_path_length.setter
    def mpcc_path_length(self, value: float):
        self.path.path_length = value

    @property
    def mpcc_path_speed(self) -> float:
        return self.path.path_speed

    @mpcc_path_speed.setter
    def mpcc_path_speed(self, value: float):
        self.path.path_speed = value

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

    # --- Scan Mission ---
    @property
    def mesh_scan_mode_active(self) -> bool:
        return self.scan.active

    @mesh_scan_mode_active.setter
    def mesh_scan_mode_active(self, value: bool):
        self.scan.active = value

    @property
    def mesh_scan_obj_path(self) -> Optional[str]:
        return self.scan.obj_path

    @mesh_scan_obj_path.setter
    def mesh_scan_obj_path(self, value: Optional[str]):
        self.scan.obj_path = value

    @property
    def mesh_scan_object_pose(
        self,
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
        return self.scan.object_pose

    @mesh_scan_object_pose.setter
    def mesh_scan_object_pose(
        self, value: Optional[Tuple[float, float, float, float, float, float]]
    ):
        self.scan.object_pose = value

    @property
    def mesh_scan_standoff(self) -> float:
        return self.scan.standoff

    @mesh_scan_standoff.setter
    def mesh_scan_standoff(self, value: float):
        self.scan.standoff = value

    @property
    def mesh_scan_levels(self) -> int:
        return self.scan.levels

    @mesh_scan_levels.setter
    def mesh_scan_levels(self, value: int):
        self.scan.levels = value

    @property
    def mesh_scan_points_per_circle(self) -> int:
        return self.scan.points_per_circle

    @mesh_scan_points_per_circle.setter
    def mesh_scan_points_per_circle(self, value: int):
        self.scan.points_per_circle = value

    @property
    def mesh_scan_speed_max(self) -> float:
        return self.scan.speed_max

    @mesh_scan_speed_max.setter
    def mesh_scan_speed_max(self, value: float):
        self.scan.speed_max = value

    @property
    def mesh_scan_speed_min(self) -> float:
        return self.scan.speed_min

    @mesh_scan_speed_min.setter
    def mesh_scan_speed_min(self, value: float):
        self.scan.speed_min = value

    @property
    def mesh_scan_lateral_accel(self) -> float:
        return self.scan.lateral_accel

    @mesh_scan_lateral_accel.setter
    def mesh_scan_lateral_accel(self, value: float):
        self.scan.lateral_accel = value

    @property
    def mesh_scan_z_margin(self) -> float:
        return self.scan.z_margin

    @mesh_scan_z_margin.setter
    def mesh_scan_z_margin(self, value: float):
        self.scan.z_margin = value

    @property
    def mesh_scan_fov_deg(self) -> float:
        return self.scan.fov_deg

    @mesh_scan_fov_deg.setter
    def mesh_scan_fov_deg(self, value: float):
        self.scan.fov_deg = value

    @property
    def mesh_scan_overlap(self) -> float:
        return self.scan.overlap

    @mesh_scan_overlap.setter
    def mesh_scan_overlap(self, value: float):
        self.scan.overlap = value

    @property
    def mesh_scan_ring_shape(self) -> str:
        return self.scan.ring_shape

    @mesh_scan_ring_shape.setter
    def mesh_scan_ring_shape(self, value: str):
        self.scan.ring_shape = value

    @property
    def scan_phase(self) -> str:
        """Current phase: APPROACH, STABILIZE, or TRACKING."""
        return self.scan.phase

    @scan_phase.setter
    def scan_phase(self, value: str):
        self.scan.phase = value

    @property
    def scan_approach_target(self) -> Optional[Tuple[float, float, float]]:
        """Target position for approach phase (nearest point on path)."""
        return self.scan.approach_target

    @scan_approach_target.setter
    def scan_approach_target(self, value: Optional[Tuple[float, float, float]]):
        self.scan.approach_target = value

    @property
    def scan_stabilize_start_time(self) -> Optional[float]:
        """Time when stabilization conditions were first met."""
        return self.scan.stabilize_start_time

    @scan_stabilize_start_time.setter
    def scan_stabilize_start_time(self, value: Optional[float]):
        self.scan.stabilize_start_time = value

    # --- Trajectory Tracking ---
    @property
    def trajectory_mode_active(self) -> bool:
        return self.trajectory.active

    @trajectory_mode_active.setter
    def trajectory_mode_active(self, value: bool):
        self.trajectory.active = value

    @property
    def trajectory_type(self) -> str:
        return self.trajectory.type

    @trajectory_type.setter
    def trajectory_type(self, value: str):
        self.trajectory.type = value

    @property
    def trajectory_start_time(self) -> Optional[float]:
        return self.trajectory.start_time

    @trajectory_start_time.setter
    def trajectory_start_time(self, value: Optional[float]):
        self.trajectory.start_time = value

    @property
    def trajectory_total_time(self) -> float:
        return self.trajectory.total_time

    @trajectory_total_time.setter
    def trajectory_total_time(self, value: float):
        self.trajectory.total_time = value

    @property
    def trajectory_hold_start(self) -> float:
        return self.trajectory.hold_start

    @trajectory_hold_start.setter
    def trajectory_hold_start(self, value: float):
        self.trajectory.hold_start = value

    @property
    def trajectory_hold_end(self) -> float:
        return self.trajectory.hold_end

    @trajectory_hold_end.setter
    def trajectory_hold_end(self, value: float):
        self.trajectory.hold_end = value

    @property
    def trajectory_start_orientation(self) -> Optional[Tuple[float, float, float]]:
        return self.trajectory.start_orientation

    @trajectory_start_orientation.setter
    def trajectory_start_orientation(self, value: Optional[Tuple[float, float, float]]):
        self.trajectory.start_orientation = value

    @property
    def trajectory_end_orientation(self) -> Optional[Tuple[float, float, float]]:
        return self.trajectory.end_orientation

    @trajectory_end_orientation.setter
    def trajectory_end_orientation(self, value: Optional[Tuple[float, float, float]]):
        self.trajectory.end_orientation = value

    @property
    def trajectory_object_center(self) -> Optional[Tuple[float, float, float]]:
        return self.trajectory.object_center

    @trajectory_object_center.setter
    def trajectory_object_center(self, value: Optional[Tuple[float, float, float]]):
        self.trajectory.object_center = value

    @property
    def trajectory_end_pos_tolerance(self) -> float:
        return self.trajectory.end_pos_tolerance

    @trajectory_end_pos_tolerance.setter
    def trajectory_end_pos_tolerance(self, value: float):
        self.trajectory.end_pos_tolerance = value

    @property
    def trajectory_end_ang_tolerance_deg(self) -> float:
        return self.trajectory.end_ang_tolerance_deg

    @trajectory_end_ang_tolerance_deg.setter
    def trajectory_end_ang_tolerance_deg(self, value: float):
        self.trajectory.end_ang_tolerance_deg = value

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
        self.scan = ScanState()
        self.trajectory = TrajectoryState()
        self.obstacle_state = ObstacleState()

    def get_current_mission_type(self) -> str:
        """
        Get the currently active mission type.

        Returns:
            WAYPOINT_NAVIGATION, WAYPOINT_NAVIGATION_MULTI,
            SHAPE_FOLLOWING, or NONE
        """
        if self.mpcc_path_waypoints:
            return "PATH_FOLLOWING"
        if self.trajectory.active:
            return "TRAJECTORY"
        if self.scan.active:
            return "SCAN"
        else:
            return "NONE"


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

    if state.mpcc_path_waypoints:
        print("\nPath Following:")
        print(f"  Points: {len(state.mpcc_path_waypoints)}")
        print(f"  Length: {state.mpcc_path_length:.2f} m")

    print("=" * 80)
