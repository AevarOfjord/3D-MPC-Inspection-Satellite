"""
Unit tests for unified MPCController.

Tests the MPCController class which provides Model Predictive Control
for satellite thruster systems using linearized dynamics and OSQP.
"""

import numpy as np
import pytest

from src.satellite_control.control.mpc_controller import MPCController
from src.satellite_control.config.models import (
    AppConfig,
    SatellitePhysicalParams,
    MPCParams,
    SimulationParams,
)


def create_default_app_config() -> AppConfig:
    """Create a valid default AppConfig for testing."""
    thruster_pos = {i: (0.1, 0.0, 0.0) for i in range(1, 7)}
    thruster_dir = {i: (1.0, 0.0, 0.0) for i in range(1, 7)}
    thruster_force = {i: 1.0 for i in range(1, 7)}

    return AppConfig(
        physics=SatellitePhysicalParams(
            total_mass=10.0,
            moment_of_inertia=0.1,  # Scalar expands to diagonal
            satellite_size=0.3,
            com_offset=(0.0, 0.0, 0.0),
            thruster_positions=thruster_pos,
            thruster_directions=thruster_dir,
            thruster_forces=thruster_force,
        ),
        mpc=MPCParams(
            prediction_horizon=10,
            control_horizon=10,
            dt=0.1,
            solver_time_limit=0.01,
            q_position=10.0,
            q_velocity=1.0,
            q_angle=10.0,
            q_angular_velocity=1.0,
            r_thrust=0.1,
            max_velocity=1.0,
            max_angular_velocity=1.0,
            position_bounds=10.0,
        ),
        simulation=SimulationParams(
            dt=0.01,
            max_duration=10.0,
            headless=True,
            window_width=800,
            window_height=600,
            control_dt=0.1,
        ),
    )


class TestMPCControllerInitialization:
    """Test MPC controller initialization."""

    def test_app_config_initialization(self):
        """Test initialization with AppConfig (New Standard)."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        # Check key attributes
        assert mpc.total_mass == 10.0
        assert mpc.moment_of_inertia[0] == 0.1
        assert mpc.prediction_horizon == 10
        assert mpc.dt == 0.1
        assert mpc.nx == 13
        assert mpc.nu == 6  # 6 thrusters, 0 RWs by default in this helper

    def test_legacy_hydra_initialization(self):
        """Test initialization with legacy DotDict/Hydra config."""

        class DotDict:
            def __init__(self, d):
                for k, v in d.items():
                    if isinstance(v, dict):
                        setattr(self, k, DotDict(v))
                    elif isinstance(v, list):
                        setattr(
                            self,
                            k,
                            [DotDict(i) if isinstance(i, dict) else i for i in v],
                        )
                    else:
                        setattr(self, k, v)

        legacy_data = {
            "vehicle": {
                "mass": 10.0,
                "inertia": [0.1, 0.1, 0.1],
                "center_of_mass": [0.0, 0.0, 0.0],
                "thrusters": [
                    {
                        "position": [0.1, 0.0, 0.0],
                        "direction": [1.0, 0.0, 0.0],
                        "max_thrust": 1.0,
                    }
                ]
                * 6,
                "reaction_wheels": [],
            },
            "control": {
                "mpc": {
                    "prediction_horizon": 10,
                    "dt": 0.1,
                    "solver_time_limit": 0.01,
                    "weights": {
                        "position": 10.0,
                        "velocity": 1.0,
                        "angle": 10.0,
                        "angular_velocity": 1.0,
                        "thrust": 0.1,
                        "rw_torque": 0.1,
                    },
                    "settings": {
                        "enable_z_tilt": True,
                        "z_tilt_gain": 0.35,
                        "z_tilt_max_deg": 20.0,
                    },
                }
            },
        }

        cfg = DotDict(legacy_data)
        mpc = MPCController(cfg)

        assert mpc.total_mass == 10.0
        assert mpc.prediction_horizon == 10

    def test_thruster_precomputation(self):
        """Test that thruster forces and torques are precomputed."""
        cfg = create_default_app_config()

        # Modify specific thruster for test
        # T1 at (0.1, 0, 0) pointing (0, 1, 0)
        # Torque = r x F = (0.1, 0, 0) x (0, 1, 0) = (0, 0, 0.1)
        cfg.physics.thruster_positions[1] = (0.1, 0.0, 0.0)
        cfg.physics.thruster_directions[1] = (0.0, 1.0, 0.0)
        cfg.physics.thruster_forces[1] = 1.0

        mpc = MPCController(cfg)

        f1 = mpc.body_frame_forces[0]
        assert f1[1] == pytest.approx(1.0)

        t1 = mpc.body_frame_torques[0]
        assert t1[2] == pytest.approx(0.1)


class TestControlAction:
    """Test control action computation."""

    def test_get_control_action_runs(self):
        """Test that get_control_action calls OSQP solver and returns result."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        x_curr = np.zeros(13)
        x_target = np.zeros(13)
        x_curr[3] = 1.0  # Valid quaternion
        x_target[3] = 1.0

        u, info = mpc.get_control_action(x_curr, x_target)

        assert u is not None
        assert len(u) == mpc.nu
        assert "solve_time" in info
        assert info["status"] in [1, -1, -2]  # OSQP status codes


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
