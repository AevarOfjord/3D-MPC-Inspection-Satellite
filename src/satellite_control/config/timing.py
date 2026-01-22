"""
Timing Configuration for Satellite Control System

Timing parameters for simulation, control loops, and mission phase durations.
Controls timesteps, update intervals, and stabilization requirements.

Timing categories:
- Simulation Timestep: Integration step for physics simulation
- Control Interval: MPC update frequency
- Mission Timeouts: Maximum duration for different mission types
- Stabilization Flags: End-of-path stabilization toggles

Key parameters:
- simulation_dt: Physics integration timestep (typically 0.01s)
- control_dt: MPC control update interval (typically 0.1s)

Features:
- Mode-specific stabilization requirements
- Configurable timeout limits
- Independent control and simulation rates
- Integration with mission_state timing
"""

import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class TimingConfig:
    """
    Timing-related constants for simulation and control loops.

    Attributes:
        simulation_dt: Simulation timestep in seconds
        control_dt: MPC control interval in seconds
        max_simulation_time: Maximum simulation duration in seconds
        use_final_stabilization_in_simulation: Enable final stabilization
        shape_final_stabilization_time: Final hold time for shapes
        shape_positioning_stabilization_time: Positioning phase hold time
    """

    simulation_dt: float
    control_dt: float
    max_simulation_time: float
    use_final_stabilization_in_simulation: bool


# DEFAULT TIMING PARAMETERS
# ============================================================================

# Simulation and control intervals
# SIMULATION_DT is the SINGLE SOURCE OF TRUTH for physics timestep
SIMULATION_DT = 0.001  # 1ms / 1000Hz physics
CONTROL_DT = 0.050  # 50 ms (MPC update rate)
MAX_SIMULATION_TIME = 0.0  # seconds (0 disables time limit)

# Stabilization timers
USE_FINAL_STABILIZATION_IN_SIMULATION = False

DEFAULT_PATH_SPEED = 0.1  # m/s - default speed for path following missions


def get_timing_params() -> TimingConfig:
    """
    Get default timing configuration.

    Returns:
        TimingConfig with default timing parameters
    """
    return TimingConfig(
        simulation_dt=SIMULATION_DT,
        control_dt=CONTROL_DT,
        max_simulation_time=MAX_SIMULATION_TIME,
        use_final_stabilization_in_simulation=(USE_FINAL_STABILIZATION_IN_SIMULATION),
    )


def validate_timing_params(config: TimingConfig) -> bool:
    """
    Validate timing parameters for consistency.

    Args:
        config: TimingConfig to validate

    Returns:
        True if valid, False otherwise
    """
    issues = []

    # Control interval must be larger than simulation timestep
    if config.control_dt <= config.simulation_dt:
        issues.append(
            f"Control interval ({config.control_dt}s) must be > "
            f"simulation timestep ({config.simulation_dt}s)"
        )

    # Ensure positive values
    if config.simulation_dt <= 0:
        issues.append(f"Simulation timestep must be positive: {config.simulation_dt}")

    if config.control_dt <= 0:
        issues.append(f"Control interval must be positive: {config.control_dt}")

    if config.max_simulation_time < 0:
        issues.append(f"Max simulation time must be >= 0: {config.max_simulation_time}")

    # Report validation results
    if issues:
        logger.warning("Timing parameter validation failed:")
        for issue in issues:
            logger.warning(f"  - {issue}")
        return False

    return True


def print_stabilization_times(config: TimingConfig) -> None:
    """
    Print current stabilization time configuration.

    Args:
        config: TimingConfig to print
    """
    simulation_mode = (
        "ENABLED" if config.use_final_stabilization_in_simulation else "DISABLED"
    )
    logger.info("STABILIZATION CONFIGURATION:")
    logger.info(f"  Final stabilization in simulation: {simulation_mode}")
