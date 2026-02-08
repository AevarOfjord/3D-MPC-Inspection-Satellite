"""
Unit tests for unified MPCController.

Tests the MPCController class which provides Model Predictive Control
for satellite thruster systems using linearized dynamics and OSQP.
"""

import numpy as np
import pytest

from satellite_control.control.mpc_controller import MPCController
from satellite_control.config import physics as physics_cfg
from satellite_control.config.models import (
    AppConfig,
    SatellitePhysicalParams,
    MPCParams,
    SimulationParams,
)


def create_default_app_config(path_speed=0.1) -> AppConfig:
    """Create a valid default AppConfig for testing."""
    thruster_ids = range(1, len(physics_cfg.THRUSTER_POSITIONS) + 1)
    thruster_pos = {i: (0.1, 0.0, 0.0) for i in thruster_ids}
    thruster_dir = {i: (1.0, 0.0, 0.0) for i in thruster_ids}
    thruster_force = {i: 1.0 for i in thruster_ids}

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
            Q_contour=100.0,
            Q_progress=10.0,
            Q_smooth=1.0,
            q_angular_velocity=1.0,
            r_thrust=0.1,
            r_rw_torque=0.1,
            path_speed=path_speed,
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
        assert mpc.nx == 17
        expected_thrusters = len(physics_cfg.THRUSTER_POSITIONS)
        assert mpc.nu == expected_thrusters  # thrusters only in this helper

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

        x_curr = np.zeros(16)
        x_curr[3] = 1.0  # Valid quaternion

        mpc.set_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])

        u, info = mpc.get_control_action(x_curr)

        assert u is not None
        assert len(u) == mpc.nu
        assert "solve_time" in info
        assert info["status"] in [1, -1, -2]  # OSQP status codes


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
