"""
System Constants for Satellite Control System

Read-only system-wide constants organized by domain.
These values remain unchanged during execution.

Usage:
    from config.constants import Constants
    from config.constants import MPCDefaults, ToleranceConstants
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
    VELOCITY_TOLERANCE: float = 0.05
    ANGULAR_VELOCITY_TOLERANCE: float = float(np.deg2rad(2))


ToleranceConstants = _ToleranceConstants()


# ============================================================================
# MPC Defaults
# ============================================================================


@dataclass(frozen=True)
class _MPCTimingDefaults:
    """Read-only MPC timing and control horizon defaults."""

    MPC_PREDICTION_HORIZON: int = 50
    MPC_CONTROL_HORIZON: int = 40
    MPC_SOLVER_TIME_LIMIT: float = 0.035
    TARGET_MEAN_SOLVE_TIME_MS: float = 5.0
    HARD_MAX_SOLVE_TIME_MS: float = 35.0
    ENFORCE_TIMING_CONTRACT: bool = False
    DARE_UPDATE_PERIOD_STEPS: int = 8


@dataclass(frozen=True)
class _MPCVueMechanicsDefaults:
    """Read-only MPC core mechanics defaults."""

    MPC_SOLVER_TYPE: str = "OSQP"
    MAX_LINEAR_VELOCITY: float = 0.0
    MAX_ANGULAR_VELOCITY: float = 0.0
    PATH_SPEED_MIN: float = 0.05
    PATH_SPEED_MAX: float = 0.2
    ERROR_PRIORITY_MIN_VS: float = 0.01
    ERROR_PRIORITY_ERROR_SPEED_GAIN: float = 8.0
    PROGRESS_POLICY: str = "speed_tracking"
    ROBUSTNESS_MODE: str = "none"


@dataclass(frozen=True)
class _MPCTuningDefaults:
    """Read-only MPC constraint weights (Q/R matrices)."""

    Q_QUAT_NORM: float = 20.0
    Q_CONTOUR: float = 2400.0
    Q_PROGRESS: float = 70.0
    PROGRESS_REWARD: float = 0.0
    Q_LAG_DEFAULT: float = 4000.0
    Q_VELOCITY_ALIGN: float = 160.0
    Q_TERMINAL_POS: float = 0.0
    Q_TERMINAL_S: float = 0.0
    Q_S_ANCHOR: float = 500.0
    Q_SMOOTH: float = 20.0
    Q_ATTITUDE: float = 3500.0
    Q_AXIS_ALIGN: float = 3000.0
    Q_ANGULAR_VELOCITY: float = 1200.0
    R_THRUST: float = 0.02
    R_RW_TORQUE: float = 0.003
    THRUST_L1_WEIGHT: float = 0.0
    THRUST_PAIR_WEIGHT: float = 0.8


@dataclass(frozen=True)
class _MPCAdaptiveDefaults:
    """Read-only MPC adaptive formulation toggles."""

    THRUSTER_TYPE: str = "CON"
    ENABLE_DELTA_U_COUPLING: bool = False
    ENABLE_GYRO_JACOBIAN: bool = False
    AUTO_ENABLE_GYRO_JACOBIAN: bool = True
    GYRO_ENABLE_THRESHOLD_RADPS: float = 0.1
    ENABLE_AUTO_STATE_BOUNDS: bool = False
    ENABLE_ONLINE_DARE_TERMINAL: bool = True
    TERMINAL_COST_PROFILE: str = "diagonal"
    CONSTRAINT_TIGHTENING_SCALE: float = 0.0
    TUBE_FEEDBACK_GAIN_SCALE: float = 0.15
    TUBE_FEEDBACK_MAX_CORRECTION: float = 0.25
    ENABLE_VARIABLE_SCALING: bool = True
    ENABLE_THRUSTER_HYSTERESIS: bool = True
    THRUSTER_HYSTERESIS_ON: float = 0.015
    THRUSTER_HYSTERESIS_OFF: float = 0.007


@dataclass(frozen=True)
class _MPCV6ContractsDefaults:
    """Read-only V6 Mode/Contract scaling and guardrails."""

    V6_SOLVER_FALLBACK_HOLD_S: float = 0.30
    V6_SOLVER_FALLBACK_DECAY_S: float = 0.70
    V6_SOLVER_FALLBACK_ZERO_AFTER_S: float = 1.00
    V6_RECOVER_ENTER_ERROR_M: float = 0.20
    V6_RECOVER_ENTER_HOLD_S: float = 0.5
    V6_RECOVER_EXIT_ERROR_M: float = 0.10
    V6_RECOVER_EXIT_HOLD_S: float = 1.0
    V6_DURATION_MARGIN_S: float = 30.0
    V6_TERMINAL_BYPASS_BAND_M: float = 0.20
    V6_RECOVER_CONTOUR_SCALE: float = 2.0
    V6_RECOVER_LAG_SCALE: float = 2.0
    V6_RECOVER_PROGRESS_SCALE: float = 0.6
    V6_RECOVER_ATTITUDE_SCALE: float = 0.8
    V6_SETTLE_PROGRESS_SCALE: float = 0.0
    V6_SETTLE_TERMINAL_POS_SCALE: float = 2.0
    V6_SETTLE_TERMINAL_ATTITUDE_SCALE: float = 1.5
    V6_SETTLE_VELOCITY_ALIGN_SCALE: float = 1.5
    V6_SETTLE_ANGULAR_VELOCITY_SCALE: float = 2.0
    V6_HOLD_SMOOTHNESS_SCALE: float = 1.5
    V6_HOLD_THRUSTER_PAIR_SCALE: float = 1.2
    V6_ENABLE_POINTING_CONTRACT: bool = True
    V6_POINTING_SCOPE: str = "all_missions"
    V6_POINTING_GUARDRAILS_ENABLED: bool = True
    V6_POINTING_Z_ERROR_DEG_MAX: float = 4.0
    V6_POINTING_X_ERROR_DEG_MAX: float = 6.0
    V6_POINTING_BREACH_HOLD_S: float = 0.30
    V6_POINTING_CLEAR_HOLD_S: float = 0.80
    V6_SCAN_AXIS_SOURCE: str = "planner"


MPCTimingDefaults = _MPCTimingDefaults()
MPCVueMechanicsDefaults = _MPCVueMechanicsDefaults()
MPCTuningDefaults = _MPCTuningDefaults()
MPCAdaptiveDefaults = _MPCAdaptiveDefaults()
MPCV6ContractsDefaults = _MPCV6ContractsDefaults()

# ============================================================================
# Physics Constants
# ============================================================================


@dataclass(frozen=True)
class _PhysicsConstants:
    """Read-only physics constants."""

    TOTAL_MASS: float = 10.0
    SATELLITE_SIZE: float = 0.30

    DEFAULT_SATELLITE_SHAPE: str = "cube"


PhysicsConstants = _PhysicsConstants()


# ============================================================================
# Constants facade
# ============================================================================


class Constants:
    """
    Constants facade exposing all constants as class attributes.
    """

    # UI
    WINDOW_WIDTH = UIConstants.WINDOW_WIDTH
    WINDOW_HEIGHT = UIConstants.WINDOW_HEIGHT
    HEADLESS_MODE = HEADLESS_MODE

    # FFmpeg
    FFMPEG_PATH = FFmpegConstants.FFMPEG_PATH
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

    # MPC
    MPC_PREDICTION_HORIZON = MPCTimingDefaults.MPC_PREDICTION_HORIZON
    MPC_CONTROL_HORIZON = MPCTimingDefaults.MPC_CONTROL_HORIZON
    MPC_SOLVER_TIME_LIMIT = MPCTimingDefaults.MPC_SOLVER_TIME_LIMIT
    MPC_SOLVER_TYPE = MPCVueMechanicsDefaults.MPC_SOLVER_TYPE
    Q_CONTOUR = MPCTuningDefaults.Q_CONTOUR
    Q_PROGRESS = MPCTuningDefaults.Q_PROGRESS
    PROGRESS_REWARD = MPCTuningDefaults.PROGRESS_REWARD
    Q_LAG_DEFAULT = MPCTuningDefaults.Q_LAG_DEFAULT
    Q_VELOCITY_ALIGN = MPCTuningDefaults.Q_VELOCITY_ALIGN
    Q_TERMINAL_POS = MPCTuningDefaults.Q_TERMINAL_POS
    Q_TERMINAL_S = MPCTuningDefaults.Q_TERMINAL_S
    Q_S_ANCHOR = MPCTuningDefaults.Q_S_ANCHOR
    PATH_SPEED_MIN = MPCVueMechanicsDefaults.PATH_SPEED_MIN
    PATH_SPEED_MAX = MPCVueMechanicsDefaults.PATH_SPEED_MAX
    Q_SMOOTH = MPCTuningDefaults.Q_SMOOTH
    Q_ATTITUDE = MPCTuningDefaults.Q_ATTITUDE
    Q_AXIS_ALIGN = MPCTuningDefaults.Q_AXIS_ALIGN
    Q_QUAT_NORM = MPCTuningDefaults.Q_QUAT_NORM
    Q_ANGULAR_VELOCITY = MPCTuningDefaults.Q_ANGULAR_VELOCITY
    R_THRUST = MPCTuningDefaults.R_THRUST
    R_RW_TORQUE = MPCTuningDefaults.R_RW_TORQUE
    THRUST_L1_WEIGHT = MPCTuningDefaults.THRUST_L1_WEIGHT
    THRUST_PAIR_WEIGHT = MPCTuningDefaults.THRUST_PAIR_WEIGHT
    MAX_LINEAR_VELOCITY = MPCVueMechanicsDefaults.MAX_LINEAR_VELOCITY
    MAX_ANGULAR_VELOCITY = MPCVueMechanicsDefaults.MAX_ANGULAR_VELOCITY
    ENABLE_DELTA_U_COUPLING = MPCAdaptiveDefaults.ENABLE_DELTA_U_COUPLING
    ENABLE_GYRO_JACOBIAN = MPCAdaptiveDefaults.ENABLE_GYRO_JACOBIAN
    AUTO_ENABLE_GYRO_JACOBIAN = MPCAdaptiveDefaults.AUTO_ENABLE_GYRO_JACOBIAN
    GYRO_ENABLE_THRESHOLD_RADPS = MPCAdaptiveDefaults.GYRO_ENABLE_THRESHOLD_RADPS
    ENABLE_AUTO_STATE_BOUNDS = MPCAdaptiveDefaults.ENABLE_AUTO_STATE_BOUNDS
    ENABLE_ONLINE_DARE_TERMINAL = MPCAdaptiveDefaults.ENABLE_ONLINE_DARE_TERMINAL
    DARE_UPDATE_PERIOD_STEPS = MPCTimingDefaults.DARE_UPDATE_PERIOD_STEPS
    TERMINAL_COST_PROFILE = MPCAdaptiveDefaults.TERMINAL_COST_PROFILE
    ROBUSTNESS_MODE = MPCVueMechanicsDefaults.ROBUSTNESS_MODE
    CONSTRAINT_TIGHTENING_SCALE = MPCAdaptiveDefaults.CONSTRAINT_TIGHTENING_SCALE
    TUBE_FEEDBACK_GAIN_SCALE = MPCAdaptiveDefaults.TUBE_FEEDBACK_GAIN_SCALE
    TUBE_FEEDBACK_MAX_CORRECTION = MPCAdaptiveDefaults.TUBE_FEEDBACK_MAX_CORRECTION
    ENABLE_VARIABLE_SCALING = MPCAdaptiveDefaults.ENABLE_VARIABLE_SCALING
    PROGRESS_POLICY = MPCVueMechanicsDefaults.PROGRESS_POLICY
    ERROR_PRIORITY_MIN_VS = MPCVueMechanicsDefaults.ERROR_PRIORITY_MIN_VS
    ERROR_PRIORITY_ERROR_SPEED_GAIN = (
        MPCVueMechanicsDefaults.ERROR_PRIORITY_ERROR_SPEED_GAIN
    )
    ENABLE_THRUSTER_HYSTERESIS = MPCAdaptiveDefaults.ENABLE_THRUSTER_HYSTERESIS
    THRUSTER_HYSTERESIS_ON = MPCAdaptiveDefaults.THRUSTER_HYSTERESIS_ON
    THRUSTER_HYSTERESIS_OFF = MPCAdaptiveDefaults.THRUSTER_HYSTERESIS_OFF
    THRUSTER_TYPE = MPCAdaptiveDefaults.THRUSTER_TYPE
    TARGET_MEAN_SOLVE_TIME_MS = MPCTimingDefaults.TARGET_MEAN_SOLVE_TIME_MS
    HARD_MAX_SOLVE_TIME_MS = MPCTimingDefaults.HARD_MAX_SOLVE_TIME_MS
    ENFORCE_TIMING_CONTRACT = MPCTimingDefaults.ENFORCE_TIMING_CONTRACT
    V6_SOLVER_FALLBACK_HOLD_S = MPCV6ContractsDefaults.V6_SOLVER_FALLBACK_HOLD_S
    V6_SOLVER_FALLBACK_DECAY_S = MPCV6ContractsDefaults.V6_SOLVER_FALLBACK_DECAY_S
    V6_SOLVER_FALLBACK_ZERO_AFTER_S = (
        MPCV6ContractsDefaults.V6_SOLVER_FALLBACK_ZERO_AFTER_S
    )
    V6_RECOVER_ENTER_ERROR_M = MPCV6ContractsDefaults.V6_RECOVER_ENTER_ERROR_M
    V6_RECOVER_ENTER_HOLD_S = MPCV6ContractsDefaults.V6_RECOVER_ENTER_HOLD_S
    V6_RECOVER_EXIT_ERROR_M = MPCV6ContractsDefaults.V6_RECOVER_EXIT_ERROR_M
    V6_RECOVER_EXIT_HOLD_S = MPCV6ContractsDefaults.V6_RECOVER_EXIT_HOLD_S
    V6_DURATION_MARGIN_S = MPCV6ContractsDefaults.V6_DURATION_MARGIN_S
    V6_TERMINAL_BYPASS_BAND_M = MPCV6ContractsDefaults.V6_TERMINAL_BYPASS_BAND_M
    V6_RECOVER_CONTOUR_SCALE = MPCV6ContractsDefaults.V6_RECOVER_CONTOUR_SCALE
    V6_RECOVER_LAG_SCALE = MPCV6ContractsDefaults.V6_RECOVER_LAG_SCALE
    V6_RECOVER_PROGRESS_SCALE = MPCV6ContractsDefaults.V6_RECOVER_PROGRESS_SCALE
    V6_RECOVER_ATTITUDE_SCALE = MPCV6ContractsDefaults.V6_RECOVER_ATTITUDE_SCALE
    V6_SETTLE_PROGRESS_SCALE = MPCV6ContractsDefaults.V6_SETTLE_PROGRESS_SCALE
    V6_SETTLE_TERMINAL_POS_SCALE = MPCV6ContractsDefaults.V6_SETTLE_TERMINAL_POS_SCALE
    V6_SETTLE_TERMINAL_ATTITUDE_SCALE = (
        MPCV6ContractsDefaults.V6_SETTLE_TERMINAL_ATTITUDE_SCALE
    )
    V6_SETTLE_VELOCITY_ALIGN_SCALE = (
        MPCV6ContractsDefaults.V6_SETTLE_VELOCITY_ALIGN_SCALE
    )
    V6_SETTLE_ANGULAR_VELOCITY_SCALE = (
        MPCV6ContractsDefaults.V6_SETTLE_ANGULAR_VELOCITY_SCALE
    )
    V6_HOLD_SMOOTHNESS_SCALE = MPCV6ContractsDefaults.V6_HOLD_SMOOTHNESS_SCALE
    V6_HOLD_THRUSTER_PAIR_SCALE = MPCV6ContractsDefaults.V6_HOLD_THRUSTER_PAIR_SCALE
    V6_ENABLE_POINTING_CONTRACT = MPCV6ContractsDefaults.V6_ENABLE_POINTING_CONTRACT
    V6_POINTING_SCOPE = MPCV6ContractsDefaults.V6_POINTING_SCOPE

    V6_POINTING_GUARDRAILS_ENABLED = (
        MPCV6ContractsDefaults.V6_POINTING_GUARDRAILS_ENABLED
    )
    V6_POINTING_Z_ERROR_DEG_MAX = MPCV6ContractsDefaults.V6_POINTING_Z_ERROR_DEG_MAX
    V6_POINTING_X_ERROR_DEG_MAX = MPCV6ContractsDefaults.V6_POINTING_X_ERROR_DEG_MAX
    V6_POINTING_BREACH_HOLD_S = MPCV6ContractsDefaults.V6_POINTING_BREACH_HOLD_S
    V6_POINTING_CLEAR_HOLD_S = MPCV6ContractsDefaults.V6_POINTING_CLEAR_HOLD_S
    V6_SCAN_AXIS_SOURCE = MPCV6ContractsDefaults.V6_SCAN_AXIS_SOURCE

    # Physics
    TOTAL_MASS = PhysicsConstants.TOTAL_MASS
    SATELLITE_SIZE = PhysicsConstants.SATELLITE_SIZE

    DEFAULT_SATELLITE_SHAPE = PhysicsConstants.DEFAULT_SATELLITE_SHAPE
