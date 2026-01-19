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


def create_default_app_config(max_velocity=1.0) -> AppConfig:
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
            max_velocity=max_velocity,
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
        assert mpc.nx == 16
        assert mpc.nu == 6  # 6 thrusters, 0 RWs by default in this helper



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
        x_target = np.zeros(16)
        x_curr[3] = 1.0  # Valid quaternion
        x_target[3] = 1.0

        u, info = mpc.get_control_action(x_curr, x_target)

        assert u is not None
        assert len(u) == mpc.nu
        assert "solve_time" in info
        assert info["status"] in [1, -1, -2]  # OSQP status codes

    def test_get_control_action_trajectory(self):
        """Test MPC with explicit trajectory matrix input."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        x_curr = np.zeros(16)
        x_curr[3] = 1.0  # Valid quaternion
        
        # Create a trajectory matrix (N, 16)
        N = mpc.prediction_horizon
        x_traj = np.zeros((N, 16))
        
        # Simple trajectory: moving along X
        for k in range(N):
            x_traj[k, 0] = 1.0 + k * 0.1 # Increasing X position
            x_traj[k, 3] = 1.0 # Valid quaternion

        # Pass trajectory as target_trajectory to runner usually, but here directly to controller
        # Note: Controller python wrapper normally takes 'x_target_trajectory' arg
        u, info = mpc.get_control_action(x_curr, x_target=x_traj[0], x_target_trajectory=x_traj)
        
        assert u is not None
        assert len(u) == mpc.nu
        assert info["status"] in [1, -1, -2]

    def test_velocity_governor(self):
        """Test that the velocity governor prevents acceleration when overspeed."""
        # Set small max velocity
        cfg = create_default_app_config(max_velocity=0.5)
        mpc = MPCController(cfg)
        
        # Current state: moving fast in +X (1.0 > 0.5)
        x_curr = np.zeros(16)
        x_curr[3] = 1.0 
        x_curr[7] = 1.0 # vx = 1.0
        
        # Target: Far ahead in +X, would usually demand +X thrust
        x_target = np.zeros(16)
        x_target[0] = 10.0
        x_target[3] = 1.0
        
        # We need to verify that thrusters pointing in +X are NOT used.
        # Default thrusters in test helper: all point (1,0,0) with forces.
        # So ANY thruster activation yields +X force.
        # Governor should kill ALL thrust.
        
        u, info = mpc.get_control_action(x_curr, x_target)
        
        # Check sum of thrust
        total_thrust = np.sum(u)
        assert total_thrust == pytest.approx(0.0, abs=1e-4), "Governor should prevent thrust in direction of overspeed"
        
        # Counter-case: Moving fast in -X (-1.0), target in +X (10.0). 
        # Should ALLOW thrust to brake/reverse.
        # But wait, our governor implementation:
        # "if net_force_world.dot(v) > 0"
        # v = (-1, 0, 0). Thrust (+1, 0, 0). Dot = -1 < 0. ALLOWED.
        
        x_curr[7] = -1.0 # Moving left fast
        u_reverse, _ = mpc.get_control_action(x_curr, x_target)
        
        # Should likely have some thrust
        assert np.sum(u_reverse) > 0.0

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
