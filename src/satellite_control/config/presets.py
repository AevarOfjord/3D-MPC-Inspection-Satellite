"""
Configuration Presets for Satellite Control System

Provides pre-configured settings optimized for different use cases:
- FAST: Aggressive control, faster movement, less stable
- BALANCED: Default balanced configuration
- STABLE: Conservative control, slower but more stable
- PRECISION: High precision, slower movement, very stable

Usage:
    from src.satellite_control.config.presets import ConfigPreset, load_preset

    # Load a preset
    config = load_preset(ConfigPreset.FAST)

    # Use in simulation
    sim = SatelliteMPCLinearizedSimulation(config_overrides=config)
"""

from typing import Dict, Any

from .models import MPCParams
from . import timing
from src.satellite_control.config.constants import Constants


class ConfigPreset:
    """Configuration preset names."""

    FAST = "fast"
    BALANCED = "balanced"
    STABLE = "stable"
    PRECISION = "precision"

    @classmethod
    def all(cls) -> list[str]:
        """Get all available preset names."""
        return [cls.FAST, cls.BALANCED, cls.STABLE, cls.PRECISION]


def _create_fast_mpc() -> MPCParams:
    """
    Create FAST preset MPC parameters.

    Characteristics:
    - High progress weight (speed)
    - Lower contouring weight (can cut corners slightly)
    - Lower smoothness weight (aggressive control)
    - Higher path speed
    """
    return MPCParams(
        prediction_horizon=40,
        control_horizon=40,
        dt=timing.CONTROL_DT,
        solver_time_limit=Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=Constants.MPC_SOLVER_TYPE,
        # MPCC Weights
        Q_contour=500.0,
        Q_progress=500.0,
        Q_smooth=1.0,
        q_angular_velocity=100.0,
        r_thrust=0.01,
        thruster_type=Constants.THRUSTER_TYPE,
        path_speed=0.5,
        enable_collision_avoidance=False,
        obstacle_margin=0.5,
    )


def _create_balanced_mpc() -> MPCParams:
    """
    Create BALANCED preset MPC parameters (default).
    """
    return MPCParams(
        prediction_horizon=Constants.MPC_PREDICTION_HORIZON,
        control_horizon=Constants.MPC_CONTROL_HORIZON,
        dt=timing.CONTROL_DT,
        solver_time_limit=Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=Constants.MPC_SOLVER_TYPE,
        # MPCC Weights
        Q_contour=1000.0,
        Q_progress=100.0,
        Q_smooth=10.0,
        q_angular_velocity=Constants.Q_ANGULAR_VELOCITY,
        r_thrust=Constants.R_THRUST,
        r_rw_torque=Constants.R_RW_TORQUE,
        thruster_type=Constants.THRUSTER_TYPE,
        path_speed=timing.DEFAULT_PATH_SPEED,
        enable_collision_avoidance=False,
        obstacle_margin=0.5,
    )


def _create_stable_mpc() -> MPCParams:
    """
    Create STABLE preset MPC parameters.
    """
    return MPCParams(
        prediction_horizon=50,
        control_horizon=50,
        dt=timing.CONTROL_DT,
        solver_time_limit=Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=Constants.MPC_SOLVER_TYPE,
        # MPCC Weights
        Q_contour=2000.0,
        Q_progress=50.0,
        Q_smooth=50.0,
        q_angular_velocity=2000.0,
        r_thrust=1.0,
        r_rw_torque=2.0,
        thruster_type=Constants.THRUSTER_TYPE,
        path_speed=0.05,
        enable_collision_avoidance=False,
        obstacle_margin=0.5,
    )


def _create_precision_mpc() -> MPCParams:
    """
    Create PRECISION preset MPC parameters.
    """
    return MPCParams(
        prediction_horizon=60,
        control_horizon=60,
        dt=timing.CONTROL_DT,
        solver_time_limit=Constants.MPC_SOLVER_TIME_LIMIT,
        solver_type=Constants.MPC_SOLVER_TYPE,
        # MPCC Weights
        Q_contour=5000.0,
        Q_progress=10.0,
        Q_smooth=100.0,
        q_angular_velocity=5000.0,
        r_thrust=5.0,
        r_rw_torque=5.0,
        thruster_type=Constants.THRUSTER_TYPE,
        path_speed=0.02,
        enable_collision_avoidance=False,
        obstacle_margin=0.5,
    )


def load_preset(preset_name: str) -> Dict[str, Any]:
    """
    Load a configuration preset.

    Args:
        preset_name: Name of preset (fast, balanced, stable, precision)

    Returns:
        Dictionary of configuration overrides compatible with AppConfig

    Raises:
        ValueError: If preset name is invalid

    Example:
        config = load_preset(ConfigPreset.FAST)
        sim = SatelliteMPCLinearizedSimulation(config_overrides=config)
    """
    preset_name = preset_name.lower()

    if preset_name == ConfigPreset.FAST:
        mpc = _create_fast_mpc()
    elif preset_name == ConfigPreset.BALANCED:
        mpc = _create_balanced_mpc()
    elif preset_name == ConfigPreset.STABLE:
        mpc = _create_stable_mpc()
    elif preset_name == ConfigPreset.PRECISION:
        mpc = _create_precision_mpc()
    else:
        available = ", ".join(ConfigPreset.all())
        raise ValueError(
            f"Invalid preset name '{preset_name}'. Available presets: {available}"
        )

    # Convert MPC params to dict for config_overrides
    return {
        "mpc": {
            "prediction_horizon": mpc.prediction_horizon,
            "control_horizon": mpc.control_horizon,
            "Q_contour": mpc.Q_contour,
            "Q_progress": mpc.Q_progress,
            "Q_smooth": mpc.Q_smooth,
            "q_angular_velocity": mpc.q_angular_velocity,
            "r_thrust": mpc.r_thrust,
            "r_rw_torque": mpc.r_rw_torque,
            "path_speed": mpc.path_speed,
        }
    }


def get_preset_description(preset_name: str) -> str:
    """
    Get a description of a preset.

    Args:
        preset_name: Name of preset

    Returns:
        Human-readable description
    """
    descriptions = {
        ConfigPreset.FAST: (
            "Fast preset: Aggressive control for rapid movement. "
            "Higher contour/progress weights, lower smoothness weight, "
            "higher path speed. Less stable but faster."
        ),
        ConfigPreset.BALANCED: (
            "Balanced preset: Default configuration with good balance "
            "between speed and stability. Recommended for most use cases."
        ),
        ConfigPreset.STABLE: (
            "Stable preset: Conservative control for smooth, stable movement. "
            "Higher contour/smoothness weights, lower path speed. "
            "Slower but more stable."
        ),
        ConfigPreset.PRECISION: (
            "Precision preset: High precision control for precise positioning. "
            "Very high contour weight, very low path speed. "
            "Slowest but most precise and stable."
        ),
    }

    preset_name = preset_name.lower()
    if preset_name not in descriptions:
        available = ", ".join(ConfigPreset.all())
        raise ValueError(
            f"Invalid preset name '{preset_name}'. Available presets: {available}"
        )

    return descriptions[preset_name]


def list_presets() -> Dict[str, str]:
    """
    List all available presets with descriptions.

    Returns:
        Dictionary mapping preset names to descriptions
    """
    return {preset: get_preset_description(preset) for preset in ConfigPreset.all()}
