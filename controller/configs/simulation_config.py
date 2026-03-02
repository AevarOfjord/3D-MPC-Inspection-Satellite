"""
Immutable Simulation Configuration Container

Provides a dependency-injection friendly configuration container that eliminates
the need for mutable global state. This is the foundation for better testability
and thread-safety.

Usage:
    from controller.configs.simulation_config import SimulationConfig

    # Create default config
    config = SimulationConfig.create_default()

    # Use in simulation
    sim = SatelliteMPCLinearizedSimulation(config=config)

    # Create with overrides
    config = SimulationConfig.create_with_overrides({
        "mpc": {"prediction_horizon": 60}
    })
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from controller.shared.python.mission.state import MissionState

from .defaults import create_default_app_config
from .models import AppConfig


@dataclass(frozen=True)
class SimulationConfig:
    """
    Immutable configuration container for simulations.

    This class holds all configuration needed for a simulation run, eliminating
    the need for mutable global state. It's designed for dependency injection.

    Attributes:
        app_config: Application configuration (MPC, physics, simulation params)
        mission_state: Runtime mission state (waypoints, paths, etc.)
    """

    app_config: AppConfig
    mission_state: MissionState

    @classmethod
    def create_default(cls) -> SimulationConfig:
        """
        Create a default simulation configuration.

        Returns:
            SimulationConfig with default settings
        """
        from controller.shared.python.mission.state import create_mission_state

        return cls(
            app_config=create_default_app_config(),
            mission_state=create_mission_state(),
        )

    @classmethod
    def create_with_overrides(
        cls,
        overrides: dict[str, Any],
        base_config: SimulationConfig | None = None,
    ) -> SimulationConfig:
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
            if section not in app_config_dict:
                continue
            base_value = app_config_dict[section]
            if isinstance(base_value, dict) and isinstance(section_overrides, dict):
                base_value.update(section_overrides)
            else:
                app_config_dict[section] = section_overrides

        # Create new AppConfig from updated dict
        new_app_config = AppConfig(**app_config_dict)

        # Return new immutable config
        return cls(
            app_config=new_app_config,
            mission_state=base_config.mission_state,  # Mission state not overridden here
        )

    def clone(self) -> SimulationConfig:
        """Create a copy of this configuration."""
        # Since it's frozen/immutable, returning self is often enough,
        # but to be safe against deep modifications in Pydantic models:
        from copy import deepcopy

        # Just return self if truly immutable, but AppConfig is Pydantic (mutable by default unless froze).
        # AppConfig in models.py is BaseModel, which is mutable.
        # So we should deepcopy.
        return deepcopy(self)

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
        if isinstance(inertia, int | float):
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
            "reaction_wheels": [
                {
                    "axis": list(rw.axis),
                    "max_torque": rw.max_torque,
                    "inertia": rw.inertia,
                }
                for rw in physics.reaction_wheels
            ],
            "size": physics.satellite_size,
        }

        # 2. Control Config (MPC)
        mpc = self.app_config.mpc

        mpc_dict = {
            "prediction_horizon": mpc.prediction_horizon,
            "control_horizon": mpc.control_horizon,
            "solver_time_limit": mpc.solver_time_limit,
            "weights": {
                "Q_contour": mpc.Q_contour,
                "Q_progress": mpc.Q_progress,
                "Q_lag": mpc.Q_lag,
                "Q_smooth": mpc.Q_smooth,
                "Q_attitude": mpc.Q_attitude,
                "Q_axis_align": mpc.Q_axis_align,
                "Q_terminal_pos": mpc.Q_terminal_pos,
                "Q_terminal_s": mpc.Q_terminal_s,
                "angular_velocity": mpc.q_angular_velocity,
                "thrust": mpc.r_thrust,
                "rw_torque": mpc.r_rw_torque,
            },
            "path_following": {
                "path_speed": mpc.path_speed,
            },
            "settings": {
                "dt": mpc.dt,
                "thruster_type": mpc.thruster_type,
                "max_linear_velocity": mpc.max_linear_velocity,
                "max_angular_velocity": mpc.max_angular_velocity,
                "enable_delta_u_coupling": mpc.enable_delta_u_coupling,
                "enable_gyro_jacobian": mpc.enable_gyro_jacobian,
                "enable_auto_state_bounds": mpc.enable_auto_state_bounds,
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
