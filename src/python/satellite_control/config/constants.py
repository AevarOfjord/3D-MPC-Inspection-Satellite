"""
System Constants for Satellite Control System

Read-only system-wide constants organized by domain.
These values remain unchanged during execution.

Usage:
    from satellite_control.config.constants import Constants  # backward-compatible
    from satellite_control.config.constants import MPCDefaults, ToleranceConstants
"""

from __future__ import annotations

import os
import platform
from dataclasses import dataclass

import numpy as np

# ============================================================================
# UI / Visualization
# ============================================================================


@dataclass(frozen=True)
class _UIConstants:
    """Read-only UI/visualization constants."""

    WINDOW_WIDTH: int = 700
    WINDOW_HEIGHT: int = 600
    SUBPLOT_CONFIG: int = 111
    OVERLAY_HEIGHT: int = 32
    ARROW_X_OFFSET: float = 0.08
    ARROW_Y_OFFSET: float = 0.08
    ARROW_WIDTH: float = 0.05
    SLEEP_CONTROL_DT: float = 0.9


UIConstants = _UIConstants()


# ============================================================================
# Headless mode (runtime-configurable via env var)
# ============================================================================

HEADLESS_MODE: bool = os.environ.get("SATELLITE_HEADLESS", "1").lower() in (
    "1",
    "true",
    "yes",
)
"""Headless mode (no GUI windows). Set SATELLITE_HEADLESS=0 to enable GUI."""


# ============================================================================
# Unit Conversions
# ============================================================================


@dataclass(frozen=True)
class _ConversionConstants:
    """Read-only unit conversion constants."""

    DEG_PER_CIRCLE: int = 360
    RAD_TO_DEG: float = 180.0 / np.pi
    DEG_TO_RAD: float = np.pi / 180.0


ConversionConstants = _ConversionConstants()


# ============================================================================
# Data Management
# ============================================================================


@dataclass(frozen=True)
class _DataConstants:
    """Read-only data path constants."""

    DATA_DIR: str = "Data"
    LINEARIZED_DATA_DIR: str = os.path.join("Data", "Linearized")
    THRUSTER_DATA_DIR: str = os.path.join("Data", "Thruster_Data")
    CSV_TIMESTAMP_FORMAT: str = "%d-%m-%Y_%H-%M-%S"


DataConstants = _DataConstants()


# ============================================================================
# FFmpeg Paths (platform-specific)
# ============================================================================


def _detect_ffmpeg_path() -> str:
    """Auto-detect FFmpeg path for the current platform."""
    sys_platform = platform.system()
    if sys_platform == "Windows":
        return r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    elif sys_platform == "Darwin":
        return "/opt/homebrew/bin/ffmpeg"
    else:
        return "ffmpeg"


@dataclass(frozen=True)
class _FFmpegConstants:
    """Read-only FFmpeg path constants."""

    FFMPEG_PATH_WINDOWS: str = r"C:\Program Files\ffmpeg\bin\ffmpeg.exe"
    FFMPEG_PATH_MACOS: str = "/opt/homebrew/bin/ffmpeg"
    FFMPEG_PATH_LINUX: str = "ffmpeg"
    FFMPEG_PATH: str = _detect_ffmpeg_path()


FFmpegConstants = _FFmpegConstants()


# ============================================================================
# Mission Defaults
# ============================================================================


@dataclass(frozen=True)
class _MissionDefaults:
    """Read-only mission default constants."""

    DEFAULT_START_POS: tuple[float, float, float] = (-1.0, -1.0, 0.0)
    DEFAULT_END_POS: tuple[float, float, float] = (0.0, 0.0, 0.0)
    DEFAULT_START_ANGLE: tuple[float, float, float] = (
        0.0,
        0.0,
        float(np.deg2rad(90)),
    )
    DEFAULT_END_ANGLE: tuple[float, float, float] = (
        0.0,
        0.0,
        float(np.deg2rad(90)),
    )


MissionDefaults = _MissionDefaults()


# ============================================================================
# Tolerances
# ============================================================================


@dataclass(frozen=True)
class _ToleranceConstants:
    """Read-only tolerance constants."""

    POSITION_TOLERANCE: float = 0.1
    ANGLE_TOLERANCE: float = float(np.deg2rad(2))
    VELOCITY_TOLERANCE: float = 0.005
    ANGULAR_VELOCITY_TOLERANCE: float = float(np.deg2rad(0.5))


ToleranceConstants = _ToleranceConstants()


# ============================================================================
# Obstacle Avoidance
# ============================================================================


@dataclass(frozen=True)
class _ObstacleConstants:
    """Read-only obstacle avoidance constants."""

    DEFAULT_OBSTACLE_RADIUS: float = 0.5
    OBSTACLE_SAFETY_MARGIN: float = 0.1
    MIN_OBSTACLE_DISTANCE: float = 0.5
    OBSTACLE_PATH_RESOLUTION: float = 0.1
    OBSTACLE_WAYPOINT_STABILIZATION_TIME: float = 0.5
    OBSTACLE_FLYTHROUGH_TOLERANCE: float = 0.15
    OBSTACLE_AVOIDANCE_SAFETY_MARGIN: float = 0.25
    OBSTACLE_TURNING_MARGIN: float = 0.2


ObstacleConstants = _ObstacleConstants()


# ============================================================================
# MPC Defaults
# ============================================================================


@dataclass(frozen=True)
class _MPCDefaults:
    """Read-only MPC solver defaults."""

    MPC_PREDICTION_HORIZON: int = 50
    MPC_CONTROL_HORIZON: int = 40
    MPC_SOLVER_TIME_LIMIT: float = 0.025
    MPC_SOLVER_TYPE: str = "OSQP"
    Q_CONTOUR: float = 2000.0
    Q_PROGRESS: float = 80.0
    PROGRESS_REWARD: float = 0.0
    Q_LAG_DEFAULT: float = 4000.0
    Q_VELOCITY_ALIGN: float = 120.0
    Q_S_ANCHOR: float = 500.0
    PATH_SPEED_MIN: float = 0.0
    PATH_SPEED_MAX: float = 0.08
    Q_SMOOTH: float = 15.0
    Q_ATTITUDE: float = 5000.0
    Q_AXIS_ALIGN: float = 5000.0
    Q_ANGULAR_VELOCITY: float = 1200.0
    R_THRUST: float = 0.02
    R_RW_TORQUE: float = 0.003
    THRUST_L1_WEIGHT: float = 0.0
    THRUST_PAIR_WEIGHT: float = 0.5
    COAST_POS_TOLERANCE: float = 0.0
    COAST_VEL_TOLERANCE: float = 0.0
    COAST_MIN_SPEED: float = 0.0
    ENABLE_COLLISION_AVOIDANCE: bool = False
    MAX_LINEAR_VELOCITY: float = 0.0
    MAX_ANGULAR_VELOCITY: float = 0.0
    ENABLE_DELTA_U_COUPLING: bool = False
    ENABLE_GYRO_JACOBIAN: bool = False
    ENABLE_AUTO_STATE_BOUNDS: bool = False
    THRUSTER_TYPE: str = "CON"
    TARGET_MEAN_SOLVE_TIME_MS: float = 3.0
    HARD_MAX_SOLVE_TIME_MS: float = 50.0
    ENFORCE_TIMING_CONTRACT: bool = False


MPCDefaults = _MPCDefaults()


# ============================================================================
# Physics Constants
# ============================================================================


@dataclass(frozen=True)
class _PhysicsConstants:
    """Read-only physics constants."""

    TOTAL_MASS: float = 10.0
    SATELLITE_SIZE: float = 0.30
    GRAVITY_M_S2: float = 9.81
    DEFAULT_SATELLITE_SHAPE: str = "cube"


PhysicsConstants = _PhysicsConstants()


# ============================================================================
# Backward-compatible Constants facade
# ============================================================================


class Constants:
    """
    Backward-compatible facade exposing all constants as class attributes.

    New code should import the domain-specific singletons directly
    (e.g. MPCDefaults, ToleranceConstants).
    """

    # UI
    WINDOW_WIDTH = UIConstants.WINDOW_WIDTH
    WINDOW_HEIGHT = UIConstants.WINDOW_HEIGHT
    SUBPLOT_CONFIG = UIConstants.SUBPLOT_CONFIG
    OVERLAY_HEIGHT = UIConstants.OVERLAY_HEIGHT
    ARROW_X_OFFSET = UIConstants.ARROW_X_OFFSET
    ARROW_Y_OFFSET = UIConstants.ARROW_Y_OFFSET
    ARROW_WIDTH = UIConstants.ARROW_WIDTH
    SLEEP_CONTROL_DT = UIConstants.SLEEP_CONTROL_DT
    HEADLESS_MODE = HEADLESS_MODE

    # Conversions
    DEG_PER_CIRCLE = ConversionConstants.DEG_PER_CIRCLE
    RAD_TO_DEG = ConversionConstants.RAD_TO_DEG
    DEG_TO_RAD = ConversionConstants.DEG_TO_RAD

    # Data
    DATA_DIR = DataConstants.DATA_DIR
    LINEARIZED_DATA_DIR = DataConstants.LINEARIZED_DATA_DIR
    THRUSTER_DATA_DIR = DataConstants.THRUSTER_DATA_DIR
    CSV_TIMESTAMP_FORMAT = DataConstants.CSV_TIMESTAMP_FORMAT

    # FFmpeg
    FFMPEG_PATH_WINDOWS = FFmpegConstants.FFMPEG_PATH_WINDOWS
    FFMPEG_PATH_MACOS = FFmpegConstants.FFMPEG_PATH_MACOS
    FFMPEG_PATH_LINUX = FFmpegConstants.FFMPEG_PATH_LINUX
    FFMPEG_PATH = FFmpegConstants.FFMPEG_PATH
    _platform = platform.system()

    # Mission defaults
    DEFAULT_START_POS = MissionDefaults.DEFAULT_START_POS
    DEFAULT_END_POS = MissionDefaults.DEFAULT_END_POS
    DEFAULT_START_ANGLE = MissionDefaults.DEFAULT_START_ANGLE
    DEFAULT_END_ANGLE = MissionDefaults.DEFAULT_END_ANGLE

    # Tolerances
    POSITION_TOLERANCE = ToleranceConstants.POSITION_TOLERANCE
    ANGLE_TOLERANCE = ToleranceConstants.ANGLE_TOLERANCE
    VELOCITY_TOLERANCE = ToleranceConstants.VELOCITY_TOLERANCE
    ANGULAR_VELOCITY_TOLERANCE = ToleranceConstants.ANGULAR_VELOCITY_TOLERANCE

    # Obstacles
    DEFAULT_OBSTACLE_RADIUS = ObstacleConstants.DEFAULT_OBSTACLE_RADIUS
    OBSTACLE_SAFETY_MARGIN = ObstacleConstants.OBSTACLE_SAFETY_MARGIN
    MIN_OBSTACLE_DISTANCE = ObstacleConstants.MIN_OBSTACLE_DISTANCE
    OBSTACLE_PATH_RESOLUTION = ObstacleConstants.OBSTACLE_PATH_RESOLUTION
    OBSTACLE_WAYPOINT_STABILIZATION_TIME = (
        ObstacleConstants.OBSTACLE_WAYPOINT_STABILIZATION_TIME
    )
    OBSTACLE_FLYTHROUGH_TOLERANCE = ObstacleConstants.OBSTACLE_FLYTHROUGH_TOLERANCE
    OBSTACLE_AVOIDANCE_SAFETY_MARGIN = (
        ObstacleConstants.OBSTACLE_AVOIDANCE_SAFETY_MARGIN
    )
    OBSTACLE_TURNING_MARGIN = ObstacleConstants.OBSTACLE_TURNING_MARGIN

    # MPC
    MPC_PREDICTION_HORIZON = MPCDefaults.MPC_PREDICTION_HORIZON
    MPC_CONTROL_HORIZON = MPCDefaults.MPC_CONTROL_HORIZON
    MPC_SOLVER_TIME_LIMIT = MPCDefaults.MPC_SOLVER_TIME_LIMIT
    MPC_SOLVER_TYPE = MPCDefaults.MPC_SOLVER_TYPE
    Q_CONTOUR = MPCDefaults.Q_CONTOUR
    Q_PROGRESS = MPCDefaults.Q_PROGRESS
    PROGRESS_REWARD = MPCDefaults.PROGRESS_REWARD
    Q_LAG_DEFAULT = MPCDefaults.Q_LAG_DEFAULT
    Q_VELOCITY_ALIGN = MPCDefaults.Q_VELOCITY_ALIGN
    Q_S_ANCHOR = MPCDefaults.Q_S_ANCHOR
    PATH_SPEED_MIN = MPCDefaults.PATH_SPEED_MIN
    PATH_SPEED_MAX = MPCDefaults.PATH_SPEED_MAX
    Q_SMOOTH = MPCDefaults.Q_SMOOTH
    Q_ATTITUDE = MPCDefaults.Q_ATTITUDE
    Q_AXIS_ALIGN = MPCDefaults.Q_AXIS_ALIGN
    Q_ANGULAR_VELOCITY = MPCDefaults.Q_ANGULAR_VELOCITY
    R_THRUST = MPCDefaults.R_THRUST
    R_RW_TORQUE = MPCDefaults.R_RW_TORQUE
    THRUST_L1_WEIGHT = MPCDefaults.THRUST_L1_WEIGHT
    THRUST_PAIR_WEIGHT = MPCDefaults.THRUST_PAIR_WEIGHT
    COAST_POS_TOLERANCE = MPCDefaults.COAST_POS_TOLERANCE
    COAST_VEL_TOLERANCE = MPCDefaults.COAST_VEL_TOLERANCE
    COAST_MIN_SPEED = MPCDefaults.COAST_MIN_SPEED
    ENABLE_COLLISION_AVOIDANCE = MPCDefaults.ENABLE_COLLISION_AVOIDANCE
    MAX_LINEAR_VELOCITY = MPCDefaults.MAX_LINEAR_VELOCITY
    MAX_ANGULAR_VELOCITY = MPCDefaults.MAX_ANGULAR_VELOCITY
    ENABLE_DELTA_U_COUPLING = MPCDefaults.ENABLE_DELTA_U_COUPLING
    ENABLE_GYRO_JACOBIAN = MPCDefaults.ENABLE_GYRO_JACOBIAN
    ENABLE_AUTO_STATE_BOUNDS = MPCDefaults.ENABLE_AUTO_STATE_BOUNDS
    THRUSTER_TYPE = MPCDefaults.THRUSTER_TYPE
    TARGET_MEAN_SOLVE_TIME_MS = MPCDefaults.TARGET_MEAN_SOLVE_TIME_MS
    HARD_MAX_SOLVE_TIME_MS = MPCDefaults.HARD_MAX_SOLVE_TIME_MS
    ENFORCE_TIMING_CONTRACT = MPCDefaults.ENFORCE_TIMING_CONTRACT

    # Physics
    TOTAL_MASS = PhysicsConstants.TOTAL_MASS
    SATELLITE_SIZE = PhysicsConstants.SATELLITE_SIZE
    GRAVITY_M_S2 = PhysicsConstants.GRAVITY_M_S2
    DEFAULT_SATELLITE_SHAPE = PhysicsConstants.DEFAULT_SATELLITE_SHAPE

    @classmethod
    def get_simulation_params(cls) -> dict:
        """Get simulation-specific parameters."""
        return {
            "data_dir": cls.LINEARIZED_DATA_DIR,
            "timestamp_format": cls.CSV_TIMESTAMP_FORMAT,
        }

    @classmethod
    def print_constants(cls) -> None:
        """Print all system constants for debugging."""
        print("=" * 80)
        print("SYSTEM CONSTANTS")
        print("=" * 80)
        print(f"  UI: {cls.WINDOW_WIDTH}x{cls.WINDOW_HEIGHT}")
        print(f"  Data directory: {cls.DATA_DIR}")
        print(f"  FFmpeg path: {cls.FFMPEG_PATH}")
        print("=" * 80)
