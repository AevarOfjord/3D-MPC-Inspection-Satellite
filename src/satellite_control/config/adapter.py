"""
Configuration Adapter Module

Provides compatibility adapters for legacy components that expect
a flat configuration interface (like MissionReportGenerator).
"""

from typing import Any
from src.satellite_control.config.constants import Constants


class SatelliteConfigAdapter:
    """
    Compatibility adapter for report generator.

    Provides flat attribute interface that wraps SimulationConfig/MissionState/AppConfig.
    This replaces the deprecated SatelliteConfig class for report generation.
    """

    def __init__(self, simulation_config: Any):
        """Initialize adapter with SimulationConfig."""
        self.sim_config = simulation_config
        self.app_config = simulation_config.app_config
        self.mission_state = simulation_config.mission_state

    def __getattr__(self, name: str) -> Any:
        """Map flat attribute names to nested config structure."""
        # Mission state attributes

        if name == "PATH_WAYPOINTS":
            return self.mission_state.mpcc_path_waypoints if self.mission_state else []
        if name == "PATH_LENGTH":
            return (
                getattr(self.mission_state, "dxf_path_length", 0.0)
                if self.mission_state
                else 0.0
            )
        if name == "OBSTACLES_ENABLED":
            return self.mission_state.obstacles_enabled if self.mission_state else False
        if name == "OBSTACLES":
            return (
                list(self.mission_state.obstacles)
                if self.mission_state and self.mission_state.obstacles
                else []
            )

        # Simulation params
        if name == "SIMULATION_DT":
            return self.app_config.simulation.dt
        if name == "CONTROL_DT":
            return self.app_config.simulation.control_dt
        if name == "MAX_SIMULATION_TIME":
            return self.app_config.simulation.max_duration
        if name == "USE_FINAL_STABILIZATION_IN_SIMULATION":
            return self.app_config.simulation.use_final_stabilization
        if name == "HEADLESS_MODE":
            return self.app_config.simulation.headless

        # MPC params
        if name == "MPC_PREDICTION_HORIZON":
            return self.app_config.mpc.prediction_horizon
        if name == "MPC_CONTROL_HORIZON":
            return self.app_config.mpc.control_horizon
        if name == "MPC_SOLVER_TIME_LIMIT":
            return self.app_config.mpc.solver_time_limit
        if name == "MPC_SOLVER_TYPE":
            return self.app_config.mpc.solver_type
        if name == "Q_CONTOUR":
            return self.app_config.mpc.Q_contour
        if name == "Q_PROGRESS":
            return self.app_config.mpc.Q_progress
        if name == "Q_SMOOTH":
            return self.app_config.mpc.Q_smooth
        if name == "PATH_SPEED":
            return self.app_config.mpc.path_speed
        if name == "Q_ANGULAR_VELOCITY":
            return self.app_config.mpc.q_angular_velocity
        if name == "R_THRUST":
            return self.app_config.mpc.r_thrust
        if name == "R_RW_TORQUE":
            return self.app_config.mpc.r_rw_torque
        if name == "POSITION_TOLERANCE":
            return Constants.POSITION_TOLERANCE
        if name == "ANGLE_TOLERANCE":
            return Constants.ANGLE_TOLERANCE
        if name == "VELOCITY_TOLERANCE":
            return Constants.VELOCITY_TOLERANCE
        if name == "ANGULAR_VELOCITY_TOLERANCE":
            return Constants.ANGULAR_VELOCITY_TOLERANCE

        # Physics params
        if name == "TOTAL_MASS":
            return self.app_config.physics.total_mass
        if name == "MOMENT_OF_INERTIA":
            return self.app_config.physics.moment_of_inertia
        if name == "SATELLITE_SIZE":
            return self.app_config.physics.satellite_size
        if name == "COM_OFFSET":
            return self.app_config.physics.com_offset
        if name == "THRUSTER_FORCES":
            return self.app_config.physics.thruster_forces
        if name == "THRUSTER_VALVE_DELAY":
            return getattr(self.app_config.physics, "thruster_valve_delay", 0.05)
        if name == "THRUSTER_RAMPUP_TIME":
            return getattr(self.app_config.physics, "thruster_rampup_time", 0.015)
        if name == "POSITION_NOISE_STD":
            return getattr(self.app_config.physics, "position_noise_std", 0.0)
        if name == "VELOCITY_NOISE_STD":
            return getattr(self.app_config.physics, "velocity_noise_std", 0.0)
        if name == "ANGLE_NOISE_STD":
            return getattr(self.app_config.physics, "angle_noise_std", 0.0)
        if name == "ANGULAR_VELOCITY_NOISE_STD":
            return getattr(self.app_config.physics, "angular_velocity_noise_std", 0.0)
        if name == "DISTURBANCE_FORCE_STD":
            return getattr(self.app_config.physics, "disturbance_force_std", 0.0)
        if name == "DISTURBANCE_TORQUE_STD":
            return getattr(self.app_config.physics, "disturbance_torque_std", 0.0)

        raise AttributeError(
            f"'{type(self).__name__}' object has no attribute '{name}'"
        )
