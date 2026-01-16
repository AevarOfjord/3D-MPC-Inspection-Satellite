"""
Immutable Simulation Configuration Container

Provides a dependency-injection friendly configuration container that eliminates
the need for mutable global state. This is the foundation for better testability
and thread-safety.

Usage:
    from src.satellite_control.config.simulation_config import SimulationConfig

    # Create default config
    config = SimulationConfig.create_default()

    # Use in simulation
    sim = SatelliteMPCLinearizedSimulation(config=config)

    # Create with overrides
    config = SimulationConfig.create_with_overrides({
        "mpc": {"prediction_horizon": 60}
    })
"""

from dataclasses import dataclass
from typing import Any, Dict, Optional

from .models import AppConfig
from .mission_state import MissionState, create_mission_state
from .defaults import create_default_app_config


@dataclass(frozen=True)
class SimulationConfig:
    """
    Immutable configuration container for simulations.

    This class holds all configuration needed for a simulation run, eliminating
    the need for mutable global state. It's designed for dependency injection.

    Attributes:
        app_config: Application configuration (MPC, physics, simulation params)
        mission_state: Runtime mission state (waypoints, targets, etc.)
    """

    app_config: AppConfig
    mission_state: MissionState

    @classmethod
    def create_default(cls) -> "SimulationConfig":
        """
        Create a default simulation configuration.

        Returns:
            SimulationConfig with default settings
        """
        return cls(
            app_config=create_default_app_config(),
            mission_state=create_mission_state(),
        )

    @classmethod
    def create_with_overrides(
        cls,
        overrides: Dict[str, Dict[str, Any]],
        base_config: Optional["SimulationConfig"] = None,
    ) -> "SimulationConfig":
        """
        Create configuration with overrides applied.

        Args:
            overrides: Dictionary of configuration overrides
            base_config: Base configuration (defaults to create_default() if None)

        Returns:
            SimulationConfig with overrides applied
        """
        if base_config is None:
            base_config = cls.create_default()

        # Create new AppConfig with overrides
        app_config_dict = base_config.app_config.model_dump()

        # Apply overrides
        for section, section_overrides in overrides.items():
            if section in app_config_dict:
                app_config_dict[section].update(section_overrides)

        # Create new AppConfig from updated dict
        new_app_config = AppConfig(**app_config_dict)

        # Return new immutable config
        return cls(
            app_config=new_app_config,
            mission_state=base_config.mission_state,  # Mission state not overridden here
        )

    def clone(self) -> "SimulationConfig":
        """Create a copy of this configuration."""
        # Since it's frozen/immutable, returning self is often enough,
        # but to be safe against deep modifications in Pydantic models:
        from copy import deepcopy

        # Just return self if truly immutable, but AppConfig is Pydantic (mutable by default unless froze).
        # AppConfig in models.py is BaseModel, which is mutable.
        # So we should deepcopy.
        return deepcopy(self)

    def get_mpc_params(self) -> Dict[str, Any]:
        """Get MPC parameters as dictionary (for backward compatibility)."""
        return {
            "prediction_horizon": self.app_config.mpc.prediction_horizon,
            "control_horizon": self.app_config.mpc.control_horizon,
            "dt": self.app_config.mpc.dt,
            "solver_time_limit": self.app_config.mpc.solver_time_limit,
            "solver_type": self.app_config.mpc.solver_type,
            "q_position": self.app_config.mpc.q_position,
            "q_velocity": self.app_config.mpc.q_velocity,
            "q_angle": self.app_config.mpc.q_angle,
            "q_angular_velocity": self.app_config.mpc.q_angular_velocity,
            "r_thrust": self.app_config.mpc.r_thrust,
            "r_rw_torque": self.app_config.mpc.r_rw_torque,
            "max_velocity": self.app_config.mpc.max_velocity,
            "max_angular_velocity": self.app_config.mpc.max_angular_velocity,
            "position_bounds": self.app_config.mpc.position_bounds,
            "damping_zone": self.app_config.mpc.damping_zone,
            "velocity_threshold": self.app_config.mpc.velocity_threshold,
            "max_velocity_weight": self.app_config.mpc.max_velocity_weight,
            "thruster_type": self.app_config.mpc.thruster_type,
            "enable_rw_yaw": self.app_config.mpc.enable_rw_yaw,
            "enable_z_tilt": self.app_config.mpc.enable_z_tilt,
            "z_tilt_gain": self.app_config.mpc.z_tilt_gain,
            "z_tilt_max_deg": self.app_config.mpc.z_tilt_max_deg,
        }

    def get_physics_params(self) -> Dict[str, Any]:
        """Get physics parameters as dictionary (for backward compatibility)."""
        return {
            "total_mass": self.app_config.physics.total_mass,
            "moment_of_inertia": self.app_config.physics.moment_of_inertia,
            "satellite_size": self.app_config.physics.satellite_size,
            "com_offset": self.app_config.physics.com_offset,
            "thruster_positions": self.app_config.physics.thruster_positions,
            "thruster_directions": self.app_config.physics.thruster_directions,
            "thruster_forces": self.app_config.physics.thruster_forces,
            "use_realistic_physics": self.app_config.physics.use_realistic_physics,
            "damping_linear": self.app_config.physics.damping_linear,
            "damping_angular": self.app_config.physics.damping_angular,
        }

    def get_simulation_params(self) -> Dict[str, Any]:
        """Get simulation parameters as dictionary (for backward compatibility)."""
        return {
            "dt": self.app_config.simulation.dt,
            "max_duration": self.app_config.simulation.max_duration,
            "headless": self.app_config.simulation.headless,
            "window_width": self.app_config.simulation.window_width,
            "window_height": self.app_config.simulation.window_height,
            "use_final_stabilization": self.app_config.simulation.use_final_stabilization,
            "control_dt": self.app_config.simulation.control_dt,
            "target_hold_time": self.app_config.simulation.target_hold_time,
            "waypoint_final_stabilization_time": self.app_config.simulation.waypoint_final_stabilization_time,
            "shape_final_stabilization_time": self.app_config.simulation.shape_final_stabilization_time,
            "shape_positioning_stabilization_time": self.app_config.simulation.shape_positioning_stabilization_time,
            "default_target_speed": self.app_config.simulation.default_target_speed,
        }

    @classmethod
    def create_from_hydra_cfg(cls, cfg: Any) -> "SimulationConfig":
        """
        Create a SimulationConfig from a Hydra DictConfig.

        Args:
            cfg: Hydra configuration object (Vehicle + Control + Sim)

        Returns:
            SimulationConfig validated against Pydantic models
        """
        from .models import (
            AppConfig,
            SatellitePhysicalParams,
            MPCParams,
            SimulationParams,
        )

        # 1. Physics Parameters
        # Handle inertia: scalar vs vector
        inertia = cfg.vehicle.inertia
        if hasattr(inertia, "__len__") and len(inertia) == 3:
            # Pydantic model currently uses float (scalar inertia)
            # We take the max component for conservative worst-case or average
            moment_of_inertia = max(inertia)
        else:
            moment_of_inertia = float(inertia)

        # Thrusters: Convert List of Dicts to Dict of items
        # cfg.vehicle.thrusters is a list
        t_pos = {}
        t_dir = {}
        t_force = {}
        for i, thruster in enumerate(cfg.vehicle.thrusters):
            tid = i + 1  # 1-based IDs
            t_pos[tid] = tuple(thruster.position)
            t_dir[tid] = tuple(thruster.direction)
            t_force[tid] = float(thruster.max_thrust)

        physics_params = SatellitePhysicalParams(
            total_mass=float(cfg.vehicle.mass),
            moment_of_inertia=moment_of_inertia,
            satellite_size=getattr(
                cfg.vehicle, "size", 0.3
            ),  # Default 30cm if not in yaml
            com_offset=tuple(getattr(cfg.vehicle, "center_of_mass", [0.0, 0.0, 0.0])),
            thruster_positions=t_pos,
            thruster_directions=t_dir,
            thruster_forces=t_force,
            use_realistic_physics=False,  # cfg.sim.realistic maybe? not in main.yaml yet
        )

        # 2. MPC Parameters
        mpc_cfg = cfg.control.mpc
        # Flatten weights if structured
        weights = mpc_cfg.weights if hasattr(mpc_cfg, "weights") else mpc_cfg

        mpc_params = MPCParams(
            prediction_horizon=mpc_cfg.prediction_horizon,
            control_horizon=mpc_cfg.control_horizon,
            dt=cfg.sim.dt,  # Match simulation DT
            solver_time_limit=mpc_cfg.solver_time_limit,
            # Weights
            q_position=float(getattr(weights, "position", 1000.0)),
            q_velocity=float(getattr(weights, "velocity", 1000.0)),
            q_angle=float(getattr(weights, "angle", 1000.0)),
            q_angular_velocity=float(getattr(weights, "angular_velocity", 1000.0)),
            r_thrust=float(getattr(weights, "thrust", 0.1)),
            r_rw_torque=float(getattr(weights, "rw_torque", 0.1)),
            # Constraints (defaults if not in yaml)
            max_velocity=0.2,
            max_angular_velocity=1.0,
            position_bounds=100.0,
        )

        # 3. Simulation Parameters
        sim_params = SimulationParams(
            dt=cfg.sim.dt,
            max_duration=cfg.sim.duration,
            control_dt=cfg.sim.dt,  # Assume 1:1 for now
            headless=False,
        )

        app_config = AppConfig(
            physics=physics_params,
            mpc=mpc_params,
            simulation=sim_params,
        )

        return cls(
            app_config=app_config,
            mission_state=create_mission_state(),  # Default mission state
        )

    def to_dict(self) -> dict:
        """
        Convert to plain dictionary format.

        Returns:
            Dict mirroring the structure expected by MPCController
        """
        # 1. Vehicle Config
        physics = self.app_config.physics

        # Convert scalar inertia to list if needed
        inertia = physics.moment_of_inertia
        if isinstance(inertia, (int, float)):
            inertia_list = [float(inertia)] * 3
        else:
            inertia_list = list(inertia)

        # Reconstruct thruster list from dicts
        thrusters_list = []
        # Sort by ID to ensure consistent order
        for tid in sorted(physics.thruster_positions.keys()):
            thrusters_list.append(
                {
                    "position": list(physics.thruster_positions[tid]),
                    "direction": list(physics.thruster_directions[tid]),
                    "max_thrust": physics.thruster_forces[tid],
                }
            )

        vehicle_dict = {
            "mass": physics.total_mass,
            "inertia": inertia_list,
            "center_of_mass": list(physics.com_offset),
            "thrusters": thrusters_list,
            "reaction_wheels": [],
            "size": physics.satellite_size,
        }

        # 2. Control Config (MPC)
        mpc = self.app_config.mpc

        mpc_dict = {
            "prediction_horizon": mpc.prediction_horizon,
            "control_horizon": mpc.control_horizon,
            "solver_time_limit": mpc.solver_time_limit,
            "weights": {
                "position": mpc.q_position,
                "velocity": mpc.q_velocity,
                "angle": mpc.q_angle,
                "angular_velocity": mpc.q_angular_velocity,
                "thrust": mpc.r_thrust,
                "rw_torque": mpc.r_rw_torque,
            },
            "constraints": {
                "max_velocity": mpc.max_velocity,
                "max_angular_velocity": mpc.max_angular_velocity,
                "position_bounds": mpc.position_bounds,
            },
            "adaptive": {
                "damping_zone": mpc.damping_zone,
                "velocity_threshold": mpc.velocity_threshold,
                "max_velocity_weight": mpc.max_velocity_weight,
            },
            "settings": {
                "dt": mpc.dt,
                "thruster_type": mpc.thruster_type,
                "enable_rw_yaw": mpc.enable_rw_yaw,
                "enable_z_tilt": mpc.enable_z_tilt,
                "z_tilt_gain": mpc.z_tilt_gain,
                "z_tilt_max_deg": mpc.z_tilt_max_deg,
            },
        }

        # 3. Simulation Config
        sim = self.app_config.simulation
        sim_dict = {
            "dt": sim.dt,
            "duration": sim.max_duration,
            "headless": sim.headless,
        }

        # Assemble full config
        return {
            "vehicle": vehicle_dict,
            "control": {"mpc": mpc_dict},
            "sim": sim_dict,
            "env": "simulation",
        }
