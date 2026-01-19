"""
Mission State Management for Satellite Control System

Runtime mission state tracking for waypoint navigation and shape following.
Maintains mutable state variables for mission execution and phase transitions.

Mission types supported:
1. Waypoint Navigation: Single or multiple sequential waypoints
2. Shape Following: Geometric paths (circle, rectangle, triangle, hexagon, DXF)
3. Trajectory Tracking: Generic path following
4. Mesh Scanning: OBJ mesh inspection

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
class WaypointState:
    """State for waypoint navigation missions."""

    enabled: bool = False
    targets: List[Tuple[float, float, float]] = field(default_factory=list)
    angles: List[Tuple[float, float, float]] = field(default_factory=list)
    current_target_index: int = 0
    start_time: Optional[float] = None  # Stabilization start time
    phase: Optional[str] = None

    # Legacy Multi-point support
    multi_point_phase: Optional[str] = None


@dataclass
class ShapeFollowingState:
    """State for shape following missions."""

    active: bool = False
    center: Optional[Tuple[float, float, float]] = None
    path: List[Tuple[float, float, float]] = field(default_factory=list)
    base_shape: List[Tuple[float, float, float]] = field(default_factory=list)
    target_speed: float = 0.1
    estimated_duration: float = 60.0
    phase: str = "POSITIONING"
    path_length: float = 0.0
    closest_point_index: int = 0
    current_target_position: Optional[Tuple[float, float, float]] = None
    tracking_start_time: Optional[float] = None
    target_start_distance: float = 0.0
    mission_start_time: Optional[float] = None
    stabilization_start_time: Optional[float] = None
    final_position: Optional[Tuple[float, float, float]] = None
    rotation: float = 0.0
    offset_distance: float = 0.5
    has_return: bool = False
    return_position: Optional[Tuple[float, float, float]] = None
    return_angle: Optional[Tuple[float, float, float]] = None
    return_start_time: Optional[float] = None
    trajectory: List[Tuple[float, float, float, float, float, float, float]] = field(
        default_factory=list
    )
    trajectory_dt: float = 0.05


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

    waypoint: WaypointState = field(default_factory=WaypointState)
    shape: ShapeFollowingState = field(default_factory=ShapeFollowingState)
    scan: ScanState = field(default_factory=ScanState)
    trajectory: TrajectoryState = field(default_factory=TrajectoryState)
    obstacle_state: ObstacleState = field(default_factory=ObstacleState)

    # =========================================================================
    # BACKWARD COMPATIBILITY PROPERTIES
    # These properties proxies allow the rest of the codebase to continue
    # accessing state as flat attributes.
    # =========================================================================

    # --- Waypoint Navigation ---
    @property
    def enable_waypoint_mode(self) -> bool:
        return self.waypoint.enabled

    @enable_waypoint_mode.setter
    def enable_waypoint_mode(self, value: bool):
        self.waypoint.enabled = value

    @property
    def waypoint_targets(self) -> List[Tuple[float, float, float]]:
        return self.waypoint.targets

    @waypoint_targets.setter
    def waypoint_targets(self, value: List[Tuple[float, float, float]]):
        self.waypoint.targets = value

    @property
    def waypoint_angles(self) -> List[Tuple[float, float, float]]:
        return self.waypoint.angles

    @waypoint_angles.setter
    def waypoint_angles(self, value: List[Tuple[float, float, float]]):
        self.waypoint.angles = value

    @property
    def current_target_index(self) -> int:
        return self.waypoint.current_target_index

    @current_target_index.setter
    def current_target_index(self, value: int):
        self.waypoint.current_target_index = value

    @property
    def target_stabilization_start_time(self) -> Optional[float]:
        return self.waypoint.start_time

    @target_stabilization_start_time.setter
    def target_stabilization_start_time(self, value: Optional[float]):
        self.waypoint.start_time = value

    @property
    def waypoint_phase(self) -> Optional[str]:
        return self.waypoint.phase

    @waypoint_phase.setter
    def waypoint_phase(self, value: Optional[str]):
        self.waypoint.phase = value

    # --- Multi-Point (Merged) ---
    @property
    def enable_multi_point_mode(self) -> bool:
        return self.waypoint.enabled

    @enable_multi_point_mode.setter
    def enable_multi_point_mode(self, value: bool):
        # Only set if True to likely avoid conflict, though they map to same
        if value:
            self.waypoint.enabled = True

    @property
    def multi_point_targets(self) -> List[Tuple[float, float, float]]:
        return self.waypoint.targets

    @multi_point_targets.setter
    def multi_point_targets(self, value: List[Tuple[float, float, float]]):
        # Avoid clearing if already set via waypoint_targets, unless new value is not empty
        if value or not self.waypoint.targets:
            self.waypoint.targets = value

    @property
    def multi_point_angles(self) -> List[Tuple[float, float, float]]:
        return self.waypoint.angles

    @multi_point_angles.setter
    def multi_point_angles(self, value: List[Tuple[float, float, float]]):
        if value or not self.waypoint.angles:
            self.waypoint.angles = value

    @property
    def multi_point_phase(self) -> Optional[str]:
        return self.waypoint.multi_point_phase

    @multi_point_phase.setter
    def multi_point_phase(self, value: Optional[str]):
        self.waypoint.multi_point_phase = value

    # --- Shape Following ---
    @property
    def dxf_shape_mode_active(self) -> bool:
        return self.shape.active

    @dxf_shape_mode_active.setter
    def dxf_shape_mode_active(self, value: bool):
        self.shape.active = value

    @property
    def dxf_shape_center(self) -> Optional[Tuple[float, float, float]]:
        return self.shape.center

    @dxf_shape_center.setter
    def dxf_shape_center(self, value: Optional[Tuple[float, float, float]]):
        self.shape.center = value

    @property
    def dxf_shape_path(self) -> List[Tuple[float, float, float]]:
        return self.shape.path

    @dxf_shape_path.setter
    def dxf_shape_path(self, value: List[Tuple[float, float, float]]):
        self.shape.path = value

    @property
    def dxf_base_shape(self) -> List[Tuple[float, float, float]]:
        return self.shape.base_shape

    @dxf_base_shape.setter
    def dxf_base_shape(self, value: List[Tuple[float, float, float]]):
        self.shape.base_shape = value

    @property
    def dxf_target_speed(self) -> float:
        return self.shape.target_speed

    @dxf_target_speed.setter
    def dxf_target_speed(self, value: float):
        self.shape.target_speed = value

    @property
    def dxf_estimated_duration(self) -> float:
        return self.shape.estimated_duration

    @dxf_estimated_duration.setter
    def dxf_estimated_duration(self, value: float):
        self.shape.estimated_duration = value

    @property
    def dxf_shape_phase(self) -> str:
        return self.shape.phase

    @dxf_shape_phase.setter
    def dxf_shape_phase(self, value: str):
        self.shape.phase = value

    @property
    def dxf_path_length(self) -> float:
        return self.shape.path_length

    @dxf_path_length.setter
    def dxf_path_length(self, value: float):
        self.shape.path_length = value

    @property
    def dxf_closest_point_index(self) -> int:
        return self.shape.closest_point_index

    @dxf_closest_point_index.setter
    def dxf_closest_point_index(self, value: int):
        self.shape.closest_point_index = value

    @property
    def dxf_current_target_position(self) -> Optional[Tuple[float, float, float]]:
        return self.shape.current_target_position

    @dxf_current_target_position.setter
    def dxf_current_target_position(self, value: Optional[Tuple[float, float, float]]):
        self.shape.current_target_position = value

    @property
    def dxf_tracking_start_time(self) -> Optional[float]:
        return self.shape.tracking_start_time

    @dxf_tracking_start_time.setter
    def dxf_tracking_start_time(self, value: Optional[float]):
        self.shape.tracking_start_time = value

    @property
    def dxf_target_start_distance(self) -> float:
        return self.shape.target_start_distance

    @dxf_target_start_distance.setter
    def dxf_target_start_distance(self, value: float):
        self.shape.target_start_distance = value

    @property
    def dxf_mission_start_time(self) -> Optional[float]:
        return self.shape.mission_start_time

    @dxf_mission_start_time.setter
    def dxf_mission_start_time(self, value: Optional[float]):
        self.shape.mission_start_time = value

    @property
    def dxf_stabilization_start_time(self) -> Optional[float]:
        return self.shape.stabilization_start_time

    @dxf_stabilization_start_time.setter
    def dxf_stabilization_start_time(self, value: Optional[float]):
        self.shape.stabilization_start_time = value

    @property
    def dxf_final_position(self) -> Optional[Tuple[float, float, float]]:
        return self.shape.final_position

    @dxf_final_position.setter
    def dxf_final_position(self, value: Optional[Tuple[float, float, float]]):
        self.shape.final_position = value

    @property
    def dxf_shape_rotation(self) -> float:
        return self.shape.rotation

    @dxf_shape_rotation.setter
    def dxf_shape_rotation(self, value: float):
        self.shape.rotation = value

    @property
    def dxf_offset_distance(self) -> float:
        return self.shape.offset_distance

    @dxf_offset_distance.setter
    def dxf_offset_distance(self, value: float):
        self.shape.offset_distance = value

    @property
    def dxf_has_return(self) -> bool:
        return self.shape.has_return

    @dxf_has_return.setter
    def dxf_has_return(self, value: bool):
        self.shape.has_return = value

    @property
    def dxf_return_position(self) -> Optional[Tuple[float, float, float]]:
        return self.shape.return_position

    @dxf_return_position.setter
    def dxf_return_position(self, value: Optional[Tuple[float, float, float]]):
        self.shape.return_position = value

    @property
    def dxf_return_angle(self) -> Optional[Tuple[float, float, float]]:
        return self.shape.return_angle

    @dxf_return_angle.setter
    def dxf_return_angle(self, value: Optional[Tuple[float, float, float]]):
        self.shape.return_angle = value

    @property
    def dxf_return_start_time(self) -> Optional[float]:
        return self.shape.return_start_time

    @dxf_return_start_time.setter
    def dxf_return_start_time(self, value: Optional[float]):
        self.shape.return_start_time = value

    @property
    def dxf_trajectory(
        self,
    ) -> List[Tuple[float, float, float, float, float, float, float]]:
        return self.shape.trajectory

    @dxf_trajectory.setter
    def dxf_trajectory(
        self, value: List[Tuple[float, float, float, float, float, float, float]]
    ):
        self.shape.trajectory = value

    @property
    def dxf_trajectory_dt(self) -> float:
        return self.shape.trajectory_dt

    @dxf_trajectory_dt.setter
    def dxf_trajectory_dt(self, value: float):
        self.shape.trajectory_dt = value

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
        self.waypoint = WaypointState()
        self.shape = ShapeFollowingState()
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
        if self.trajectory.active:
            return "TRAJECTORY"
        if self.scan.active:
            return "SCAN"
        if self.shape.active:
            return "SHAPE_FOLLOWING"
        elif self.waypoint.enabled:
            num_targets = len(self.waypoint.targets)
            return (
                "WAYPOINT_NAVIGATION_MULTI"
                if num_targets > 1
                else "WAYPOINT_NAVIGATION"
            )
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

    if state.waypoint.enabled:
        print("\nWaypoint Navigation:")
        print(f"  Targets: {len(state.waypoint.targets)}")
        print(f"  Current: {state.waypoint.current_target_index}")
        print(f"  Phase: {state.waypoint.phase}")

    if state.shape.active:
        print("\nShape Following:")
        print(f"  Center: {state.shape.center}")
        print(f"  Points: {len(state.shape.path)}")
        print(f"  Phase: {state.shape.phase}")
        print(f"  Length: {state.shape.path_length:.2f} m")

    print("=" * 80)
