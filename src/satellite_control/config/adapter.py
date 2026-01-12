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
        if name == "DXF_SHAPE_MODE_ACTIVE":
            return (
                self.mission_state.dxf_shape_mode_active
                if self.mission_state
                else False
            )
        if name == "DXF_SHAPE_PHASE":
            return (
                getattr(self.mission_state, "dxf_shape_phase", "UNKNOWN")
                if self.mission_state
                else "UNKNOWN"
            )
        if name == "DXF_SHAPE_CENTER":
            return self.mission_state.dxf_shape_center if self.mission_state else None
        if name == "DXF_SHAPE_ROTATION":
            return (
                getattr(self.mission_state, "dxf_shape_rotation", 0.0)
                if self.mission_state
                else 0.0
            )
        if name == "DXF_OFFSET_DISTANCE":
            return (
                getattr(self.mission_state, "dxf_offset_distance", 0.0)
                if self.mission_state
                else 0.0
            )
        if name == "DXF_PATH_LENGTH":
            return (
                getattr(self.mission_state, "dxf_path_length", 0.0)
                if self.mission_state
                else 0.0
            )
        if name == "DXF_BASE_SHAPE":
            return self.mission_state.dxf_base_shape if self.mission_state else []
        if name == "DXF_SHAPE_PATH":
            return self.mission_state.dxf_shape_path if self.mission_state else []
        if name == "DXF_TARGET_SPEED":
            return (
                getattr(self.mission_state, "dxf_target_speed", 0.1)
                if self.mission_state
                else 0.1
            )
        if name == "DXF_ESTIMATED_DURATION":
            return (
                getattr(self.mission_state, "dxf_estimated_duration", 0.0)
                if self.mission_state
                else 0.0
            )
        if name == "ENABLE_WAYPOINT_MODE":
            return (
                self.mission_state.enable_waypoint_mode if self.mission_state else False
            )
        if name == "WAYPOINT_TARGETS":
            return self.mission_state.waypoint_targets if self.mission_state else []
        if name == "WAYPOINT_ANGLES":
            return self.mission_state.waypoint_angles if self.mission_state else []
        if name == "OBSTACLES_ENABLED":
            return self.mission_state.obstacles_enabled if self.mission_state else False
        if name == "OBSTACLES":
            return (
                list(self.mission_state.obstacles)
                if self.mission_state and self.mission_state.obstacles
                else []
            )
        if name == "CURRENT_TARGET_INDEX":
            return (
                getattr(self.mission_state, "current_target_index", 0)
                if self.mission_state
                else 0
            )
        if name == "MULTI_POINT_PHASE":
            return (
                getattr(self.mission_state, "multi_point_phase", "")
                if self.mission_state
                else ""
            )

        # Simulation params
        if name == "SIMULATION_DT":
            return self.app_config.simulation.dt
        if name == "CONTROL_DT":
            return self.app_config.simulation.control_dt
        if name == "MAX_SIMULATION_TIME":
            return self.app_config.simulation.max_duration
        if name == "TARGET_HOLD_TIME":
            return self.app_config.simulation.target_hold_time
        if name == "WAYPOINT_FINAL_STABILIZATION_TIME":
            return self.app_config.simulation.waypoint_final_stabilization_time
        if name == "SHAPE_FINAL_STABILIZATION_TIME":
            return self.app_config.simulation.shape_final_stabilization_time
        if name == "SHAPE_POSITIONING_STABILIZATION_TIME":
            return self.app_config.simulation.shape_positioning_stabilization_time
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
        if name == "Q_POSITION":
            return self.app_config.mpc.q_position
        if name == "Q_VELOCITY":
            return self.app_config.mpc.q_velocity
        if name == "Q_ANGLE":
            return self.app_config.mpc.q_angle
        if name == "Q_ANGULAR_VELOCITY":
            return self.app_config.mpc.q_angular_velocity
        if name == "R_THRUST":
            return self.app_config.mpc.r_thrust
        if name == "MAX_VELOCITY":
            return self.app_config.mpc.max_velocity
        if name == "MAX_ANGULAR_VELOCITY":
            return self.app_config.mpc.max_angular_velocity
        if name == "POSITION_BOUNDS":
            return self.app_config.mpc.position_bounds
        if name == "DAMPING_ZONE":
            return self.app_config.mpc.damping_zone
        if name == "VELOCITY_THRESHOLD":
            return self.app_config.mpc.velocity_threshold
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
