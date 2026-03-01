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

    WINDOW_WIDTH: int = 700  # px — initial plot/dashboard window width
    WINDOW_HEIGHT: int = 600  # px — initial plot/dashboard window height


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

    FFMPEG_PATH: str = _detect_ffmpeg_path()  # absolute path or bare "ffmpeg" on Linux


FFmpegConstants = _FFmpegConstants()


# ============================================================================
# Mission Defaults
# ============================================================================


@dataclass(frozen=True)
class _MissionDefaults:
    """Read-only mission default constants."""

    # Default start/end positions in LVLH frame [m].
    # Used when a mission payload does not specify explicit waypoints.
    DEFAULT_START_POS: tuple[float, float, float] = (-1.0, -1.0, 0.0)
    DEFAULT_END_POS: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Duration [s] the satellite must remain within endpoint tolerance
    # before the mission is declared complete.
    PATH_HOLD_END_S: float = 10.0

    # Default start/end Euler angles (XYZ, radians).
    # 90 deg Z-rotation aligns body +X with the along-track direction.
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
    """
    Thresholds used by termination checks and completion contracts.
    All state components must be within tolerance for a mission to be
    declared complete.
    """

    POSITION_TOLERANCE: float = 0.1  # m   — max residual position error at endpoint
    ANGLE_TOLERANCE: float = float(
        np.deg2rad(2)
    )  # rad — max attitude error at endpoint (~2 deg)
    VELOCITY_TOLERANCE: float = 0.05  # m/s — max linear velocity at endpoint
    ANGULAR_VELOCITY_TOLERANCE: float = float(
        np.deg2rad(2)
    )  # rad/s — max body rate at endpoint (~2 deg/s)


ToleranceConstants = _ToleranceConstants()


# ============================================================================
# MPC Defaults
# ============================================================================


@dataclass(frozen=True)
class _MPCTimingDefaults:
    """
    Horizon lengths and timing budget for the RTI-SQP solver.

    Longer horizons improve prediction quality but increase QP size and
    solve time roughly as O(N²).  The control horizon M < N reduces free
    variables while keeping the prediction length.
    """

    MPC_PREDICTION_HORIZON: int = 50  # N — number of stages in the QP.  Higher = looser
    # manoeuvres planned further ahead; diminishing returns above ~60
    MPC_CONTROL_HORIZON: int = 40  # M — free control stages; u_k = u_{M-1} for k ≥ M.
    # Lower values reduce QP variables and solve time
    MPC_SOLVER_TIME_LIMIT: float = (
        0.035  # s — hard OSQP wall-clock cap (capped internally at 0.85*dt)
    )
    TARGET_MEAN_SOLVE_TIME_MS: float = (
        5.0  # ms — performance-monitor soft target (logging only)
    )
    HARD_MAX_SOLVE_TIME_MS: float = (
        35.0  # ms — performance-monitor hard threshold (triggers warning/error)
    )
    ENFORCE_TIMING_CONTRACT: bool = (
        False  # raise SimulationError when HARD_MAX exceeded if True
    )
    DARE_UPDATE_PERIOD_STEPS: int = (
        8  # refresh the DARE terminal-cost approximation every N steps;
    )
    # lower = more current but slightly more CPU per step


@dataclass(frozen=True)
class _MPCVueMechanicsDefaults:
    """
    Core MPC mechanics: solver backend, speed bounds, and progress policy.
    """

    MPC_SOLVER_TYPE: str = "OSQP"  # QP backend — only "OSQP" is supported

    # Linear and angular velocity state bounds [m/s, rad/s].
    # 0.0 disables the bound (uses OSQP_INFTY).  Set non-zero to hard-constrain
    # state trajectories; too tight can cause infeasibility near RECOVER.
    MAX_LINEAR_VELOCITY: float = 0.0
    MAX_ANGULAR_VELOCITY: float = 0.0

    # Virtual path-speed bounds [m/s].
    # v_s is the MPC's "speed" along the arc-length parameter s.
    # PATH_SPEED_MIN prevents the solver from stalling on the path.
    # PATH_SPEED_MAX limits how fast the satellite is driven along the reference.
    PATH_SPEED_MIN: float = 0.05  # m/s — lower bound on v_s; relaxed to 0 near endpoint
    PATH_SPEED_MAX: float = 0.2  # m/s — upper bound on v_s
    REF_TANGENT_LOOKAHEAD_M: float = 0.35  # m — lookahead for smooth heading secant
    REF_TANGENT_LOOKBACK_M: float = 0.10  # m — lookback for smooth heading secant
    REF_QUAT_MAX_RATE_RAD_S: float = 1.57  # rad/s — max reference quaternion slew rate
    REF_QUAT_TERMINAL_RATE_SCALE: float = (
        2.0  # multiplier for ref quaternion slew in SETTLE/HOLD/COMPLETE
    )

    # Progress policy — governs how path speed bounds adapt at runtime.
    #   "speed_tracking" : v_s tracks PATH_SPEED_MAX; pure speed reference
    #   "error_priority"  : v_s_max is reduced when path error is large,
    #                        forcing the solver to close the error before advancing
    PROGRESS_POLICY: str = "speed_tracking"

    # Parameters for "error_priority" mode (ignored in "speed_tracking"):
    ERROR_PRIORITY_MIN_VS: float = 0.01  # m/s — minimum v_s even while recovering
    ERROR_PRIORITY_ERROR_SPEED_GAIN: float = 8.0  # gain k: v_s_max → v_s_max/(1 + k*e²)
    # higher = more aggressive speed reduction


@dataclass(frozen=True)
class _MPCTuningDefaults:
    """
    Cost function weights (Q/R matrices).  These are the primary tuning knobs.

    All weights are quadratic penalties (½ z^T P z form inside OSQP), so
    doubling a weight doubles the sensitivity of that term.  Relative
    magnitudes matter more than absolute values.

    Path-tracking weights (position/progress):
        Increasing Q_CONTOUR tightens lateral path adherence.
        Increasing Q_LAG reduces along-track lead/lag.
        Increasing Q_PROGRESS makes the solver track PATH_SPEED_MAX more closely.
        PROGRESS_REWARD adds a linear incentive for faster progress (no quadratic target).

    Attitude weights:
        Q_ATTITUDE is the primary attitude tracking weight.
        Q_AXIS_ALIGN is added on top for scan-frame axis alignment emphasis.
        Q_QUAT_NORM keeps quaternion components near the unit sphere in the QP.

    Control/effort weights:
        R_THRUST and R_RW_TORQUE penalise actuator usage.  Higher = smoother/cheaper.
        Q_SMOOTH penalises consecutive control increments ||u_k - u_{k-1}||².
        THRUST_PAIR_WEIGHT penalises co-firing of opposing thrusters.
        THRUST_L1_WEIGHT adds an L1 fuel penalty (promotes coasting when > 0).
    """

    # ----- Path geometry -----
    Q_CONTOUR: float = 2400.0  # cross-track (lateral) path error weight
    Q_LAG_DEFAULT: float = 4000.0  # along-track path error weight
    Q_PROGRESS: float = 70.0  # virtual speed tracking (v_s - v_ref)²
    PROGRESS_REWARD: float = 0.0  # linear incentive term −reward·v_s (0 = disabled)

    # ----- Progress anchor and velocity -----
    Q_S_ANCHOR: float = (
        500.0  # anchors arc-length s to current estimate; prevents drift
    )
    Q_VELOCITY_ALIGN: float = (
        160.0  # velocity damping ||v||² (C++ QP); higher = faster deceleration
    )

    # ----- Terminal-stage extra weights (added on top of 10× stage scaling) -----
    Q_TERMINAL_POS: float = (
        8000.0  # additional terminal position weight (0 = use stage scaling only)
    )
    Q_TERMINAL_S: float = 0.0  # additional terminal progress weight
    Q_TERMINAL_VEL: float = 0.0  # additional terminal velocity weight — penalises residual speed at endpoint

    # ----- Attitude -----
    Q_ATTITUDE: float = 3500.0  # attitude tracking ||q - q_ref||²
    Q_AXIS_ALIGN: float = 3000.0  # added to Q_ATTITUDE for scan-axis alignment emphasis
    Q_QUAT_NORM: float = (
        20.0  # soft quaternion normalisation penalty ||q - q_current||²
    )

    # ----- Angular rate -----
    Q_ANGULAR_VELOCITY: float = 1200.0  # body angular rate damping ||ω||²

    # ----- Control effort -----
    Q_SMOOTH: float = 20.0  # smoothness penalty on Δu = u_k − u_{k-1}
    R_THRUST: float = 0.02  # thruster command effort weight
    R_RW_TORQUE: float = 0.003  # reaction-wheel torque command effort weight
    THRUST_PAIR_WEIGHT: float = 0.8  # opposing-thruster co-fire penalty w·(u_i + u_j)²
    THRUST_L1_WEIGHT: float = 0.0  # L1 fuel bias; positive values promote coasting


@dataclass(frozen=True)
class _MPCAdaptiveDefaults:
    """
    Runtime feature toggles and adaptive formulation parameters.
    """

    # ----- Actuator model -----
    THRUSTER_TYPE: str = (
        "CON"  # "CON" (continuous) or "PWM"; selects ThrusterManager mode
    )

    # ----- Thruster hysteresis -----
    # Prevents valve chatter by requiring the command to cross a dead-band
    # before toggling.  Only active when ENABLE_THRUSTER_HYSTERESIS is True.
    ENABLE_THRUSTER_HYSTERESIS: bool = True
    THRUSTER_HYSTERESIS_ON: float = 0.015  # command must exceed this to fire (0→1 edge)
    THRUSTER_HYSTERESIS_OFF: float = (
        0.007  # command must drop below this to stop (1→0 edge)
    )

    # ----- Delta-u coupling -----
    # When True, explicit cross-stage Hessian entries for (u_{k-1}, u_k) are
    # filled; currently pre-allocated but left at zero in practice.
    ENABLE_DELTA_U_COUPLING: bool = False

    # ----- Gyroscopic Jacobian -----
    # Adds the ω×(Iω + h_rw) Jacobian block to the linearization.
    # AUTO mode enables it when ||ω|| > threshold (avoids cost when near-zero).
    ENABLE_GYRO_JACOBIAN: bool = False  # manual override (overridden when AUTO is True)
    AUTO_ENABLE_GYRO_JACOBIAN: bool = True  # enable automatically when spinning
    GYRO_ENABLE_THRESHOLD_RADPS: float = (
        0.1  # rad/s — angular rate threshold for auto-enable
    )

    # ----- State bounds -----
    # When True, state bounds at k=1..N widen to encompass the current measured
    # state (×1.2 + margin) and converge to nominal limits exponentially.
    ENABLE_AUTO_STATE_BOUNDS: bool = False

    # ----- DARE terminal cost -----
    ENABLE_ONLINE_DARE_TERMINAL: bool = (
        True  # add DARE-approximated diagonal to terminal stage
    )
    TERMINAL_COST_PROFILE: str = "diagonal"  # only "diagonal" is implemented

    # ----- Variable scaling (NOT currently implemented in C++) -----
    ENABLE_VARIABLE_SCALING: bool = True  # reserved; has no effect on the QP solver


@dataclass(frozen=True)
class _MPCContractsDefaults:
    """
    Runtime mode contracts: hysteresis thresholds, mode weight scales,
    and pointing guardrail parameters.

    Mode transitions (TRACK → RECOVER → SETTLE → COMPLETE):
        RECOVER is entered when path error exceeds ENTER threshold for ENTER_HOLD_S.
        RECOVER exits when error drops below EXIT threshold for EXIT_HOLD_S.
        SETTLE is entered when position is within TERMINAL_BYPASS_BAND of the endpoint.

    Weight scale factors multiply the corresponding base weight when in that mode.
    A scale of 0.0 disables the term; >1.0 increases emphasis.
    """

    # ----- Solver fallback timing -----
    SOLVER_FALLBACK_HOLD_S: float = (
        0.30  # s — hold last feasible command after solver failure
    )
    SOLVER_FALLBACK_DECAY_S: float = (
        0.70  # s — linearly ramp command to zero after hold period
    )
    SOLVER_FALLBACK_ZERO_AFTER_S: float = (
        1.00  # s — total time before zeroing (= hold + decay)
    )

    # ----- RECOVER mode thresholds -----
    RECOVER_ENTER_ERROR_M: float = 0.20  # m — path error that triggers RECOVER entry
    RECOVER_ENTER_HOLD_S: float = (
        0.5  # s — error must persist this long to enter RECOVER
    )
    RECOVER_EXIT_ERROR_M: float = 0.10  # m — path error below which RECOVER can exit
    RECOVER_EXIT_HOLD_S: float = (
        1.0  # s — must stay below exit threshold this long to leave
    )

    # ----- Mission duration -----
    DURATION_MARGIN_S: float = (
        30.0  # s — extra time buffer added to estimated mission duration
    )

    # ----- SETTLE / terminal zone -----
    TERMINAL_BYPASS_BAND_M: float = (
        0.35  # m — position error within which SETTLE mode activates
    )
    PATH_PROJECTION_LEAD_CAP_M: float = (
        0.25  # m — maximum allowed projected-s lead over controller s
    )
    TERMINAL_POSITION_EXIT_TOLERANCE_M: float = (
        0.12  # m — hold reset only after exceeding this (hysteresis exit)
    )
    TERMINAL_ANGLE_EXIT_TOLERANCE_DEG: float = (
        2.5  # deg — hold reset only after exceeding this (hysteresis exit)
    )
    TERMINAL_VELOCITY_EXIT_TOLERANCE_MPS: float = (
        0.06  # m/s — hold reset only after exceeding this (hysteresis exit)
    )
    TERMINAL_ANGULAR_VELOCITY_EXIT_TOLERANCE_DEGPS: float = (
        2.5  # deg/s — hold reset only after exceeding this (hysteresis exit)
    )

    # ----- RECOVER mode weight scales (multiplied onto base Q values) -----
    RECOVER_CONTOUR_SCALE: float = (
        2.0  # ↑ lateral error emphasis (drives satellite back to path)
    )
    RECOVER_LAG_SCALE: float = 2.0  # ↑ along-track error emphasis
    RECOVER_PROGRESS_SCALE: float = (
        0.6  # ↓ speed pressure (recovery takes priority over progress)
    )
    RECOVER_ATTITUDE_SCALE: float = (
        0.8  # ↓ attitude weight (allow body rotation to recover faster)
    )

    # ----- SETTLE mode weight scales -----
    SETTLE_PROGRESS_SCALE: float = (
        0.0  # 0 = stop advancing (no forward progress in SETTLE)
    )
    SETTLE_TERMINAL_POS_SCALE: float = 3.0  # ↑ terminal position pull to endpoint
    SETTLE_TERMINAL_ATTITUDE_SCALE: float = 1.5  # ↑ terminal attitude tracking
    SETTLE_VELOCITY_ALIGN_SCALE: float = 1.5  # ↑ velocity damping (come to rest)
    SETTLE_ANGULAR_VELOCITY_SCALE: float = 2.0  # ↑ angular rate damping (stop spinning)

    # ----- HOLD mode weight scales -----
    HOLD_SMOOTHNESS_SCALE: float = 1.5  # ↑ smoothness (reduces jitter while holding)
    HOLD_THRUSTER_PAIR_SCALE: float = (
        1.2  # ↑ co-fire penalty (avoids wasted thrust while holding)
    )

    # ----- Pointing contract / guardrails -----
    ENABLE_POINTING_CONTRACT: bool = (
        True  # enforce pointing accuracy throughout mission
    )
    POINTING_SCOPE: str = "all_missions"  # "all_missions" or mission-type filter
    POINTING_GUARDRAILS_ENABLED: bool = True  # monitor attitude error and act on breach

    # Maximum allowed pointing errors before guardrail triggers.
    # Z-axis is the primary scan/instrument axis; X is the forward/velocity axis.
    POINTING_Z_ERROR_DEG_MAX: float = 4.0  # deg — Z-axis pointing error limit
    POINTING_X_ERROR_DEG_MAX: float = 6.0  # deg — X-axis pointing error limit

    # Guardrail hysteresis timing.
    POINTING_BREACH_HOLD_S: float = (
        0.30  # s — error must persist this long before action
    )
    POINTING_CLEAR_HOLD_S: float = (
        0.80  # s — error must clear for this long before reset
    )

    SCAN_AXIS_SOURCE: str = (
        "planner"  # source for scan-axis reference: "planner" or "mission"
    )
    NON_SCAN_ORIENTATION_POLICY: str = "minimal_twist"  # non-scan +Z behavior: minimal_twist, world_up_lock, radial_lock


MPCTimingDefaults = _MPCTimingDefaults()
MPCVueMechanicsDefaults = _MPCVueMechanicsDefaults()
MPCTuningDefaults = _MPCTuningDefaults()
MPCAdaptiveDefaults = _MPCAdaptiveDefaults()
MPCContractsDefaults = _MPCContractsDefaults()

# ============================================================================
# Physics Constants
# ============================================================================


@dataclass(frozen=True)
class _PhysicsConstants:
    """Read-only physics constants."""

    TOTAL_MASS: float = 10.0  # kg — satellite total mass
    SATELLITE_SIZE: float = (
        0.30  # m  — characteristic body dimension (used for 3D rendering)
    )
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
    Q_TERMINAL_VEL = MPCTuningDefaults.Q_TERMINAL_VEL
    Q_S_ANCHOR = MPCTuningDefaults.Q_S_ANCHOR
    PATH_SPEED_MIN = MPCVueMechanicsDefaults.PATH_SPEED_MIN
    PATH_SPEED_MAX = MPCVueMechanicsDefaults.PATH_SPEED_MAX
    REF_TANGENT_LOOKAHEAD_M = MPCVueMechanicsDefaults.REF_TANGENT_LOOKAHEAD_M
    REF_TANGENT_LOOKBACK_M = MPCVueMechanicsDefaults.REF_TANGENT_LOOKBACK_M
    REF_QUAT_MAX_RATE_RAD_S = MPCVueMechanicsDefaults.REF_QUAT_MAX_RATE_RAD_S
    REF_QUAT_TERMINAL_RATE_SCALE = MPCVueMechanicsDefaults.REF_QUAT_TERMINAL_RATE_SCALE
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
    SOLVER_FALLBACK_HOLD_S = MPCContractsDefaults.SOLVER_FALLBACK_HOLD_S
    SOLVER_FALLBACK_DECAY_S = MPCContractsDefaults.SOLVER_FALLBACK_DECAY_S
    SOLVER_FALLBACK_ZERO_AFTER_S = MPCContractsDefaults.SOLVER_FALLBACK_ZERO_AFTER_S
    RECOVER_ENTER_ERROR_M = MPCContractsDefaults.RECOVER_ENTER_ERROR_M
    RECOVER_ENTER_HOLD_S = MPCContractsDefaults.RECOVER_ENTER_HOLD_S
    RECOVER_EXIT_ERROR_M = MPCContractsDefaults.RECOVER_EXIT_ERROR_M
    RECOVER_EXIT_HOLD_S = MPCContractsDefaults.RECOVER_EXIT_HOLD_S
    DURATION_MARGIN_S = MPCContractsDefaults.DURATION_MARGIN_S
    TERMINAL_BYPASS_BAND_M = MPCContractsDefaults.TERMINAL_BYPASS_BAND_M
    PATH_PROJECTION_LEAD_CAP_M = MPCContractsDefaults.PATH_PROJECTION_LEAD_CAP_M
    TERMINAL_POSITION_EXIT_TOLERANCE_M = (
        MPCContractsDefaults.TERMINAL_POSITION_EXIT_TOLERANCE_M
    )
    TERMINAL_ANGLE_EXIT_TOLERANCE_DEG = (
        MPCContractsDefaults.TERMINAL_ANGLE_EXIT_TOLERANCE_DEG
    )
    TERMINAL_VELOCITY_EXIT_TOLERANCE_MPS = (
        MPCContractsDefaults.TERMINAL_VELOCITY_EXIT_TOLERANCE_MPS
    )
    TERMINAL_ANGULAR_VELOCITY_EXIT_TOLERANCE_DEGPS = (
        MPCContractsDefaults.TERMINAL_ANGULAR_VELOCITY_EXIT_TOLERANCE_DEGPS
    )
    RECOVER_CONTOUR_SCALE = MPCContractsDefaults.RECOVER_CONTOUR_SCALE
    RECOVER_LAG_SCALE = MPCContractsDefaults.RECOVER_LAG_SCALE
    RECOVER_PROGRESS_SCALE = MPCContractsDefaults.RECOVER_PROGRESS_SCALE
    RECOVER_ATTITUDE_SCALE = MPCContractsDefaults.RECOVER_ATTITUDE_SCALE
    SETTLE_PROGRESS_SCALE = MPCContractsDefaults.SETTLE_PROGRESS_SCALE
    SETTLE_TERMINAL_POS_SCALE = MPCContractsDefaults.SETTLE_TERMINAL_POS_SCALE
    SETTLE_TERMINAL_ATTITUDE_SCALE = MPCContractsDefaults.SETTLE_TERMINAL_ATTITUDE_SCALE
    SETTLE_VELOCITY_ALIGN_SCALE = MPCContractsDefaults.SETTLE_VELOCITY_ALIGN_SCALE
    SETTLE_ANGULAR_VELOCITY_SCALE = MPCContractsDefaults.SETTLE_ANGULAR_VELOCITY_SCALE
    HOLD_SMOOTHNESS_SCALE = MPCContractsDefaults.HOLD_SMOOTHNESS_SCALE
    HOLD_THRUSTER_PAIR_SCALE = MPCContractsDefaults.HOLD_THRUSTER_PAIR_SCALE
    ENABLE_POINTING_CONTRACT = MPCContractsDefaults.ENABLE_POINTING_CONTRACT
    POINTING_SCOPE = MPCContractsDefaults.POINTING_SCOPE

    POINTING_GUARDRAILS_ENABLED = MPCContractsDefaults.POINTING_GUARDRAILS_ENABLED
    POINTING_Z_ERROR_DEG_MAX = MPCContractsDefaults.POINTING_Z_ERROR_DEG_MAX
    POINTING_X_ERROR_DEG_MAX = MPCContractsDefaults.POINTING_X_ERROR_DEG_MAX
    POINTING_BREACH_HOLD_S = MPCContractsDefaults.POINTING_BREACH_HOLD_S
    POINTING_CLEAR_HOLD_S = MPCContractsDefaults.POINTING_CLEAR_HOLD_S
    SCAN_AXIS_SOURCE = MPCContractsDefaults.SCAN_AXIS_SOURCE
    NON_SCAN_ORIENTATION_POLICY = MPCContractsDefaults.NON_SCAN_ORIENTATION_POLICY

    # Physics
    TOTAL_MASS = PhysicsConstants.TOTAL_MASS
    SATELLITE_SIZE = PhysicsConstants.SATELLITE_SIZE

    DEFAULT_SATELLITE_SHAPE = PhysicsConstants.DEFAULT_SATELLITE_SHAPE
