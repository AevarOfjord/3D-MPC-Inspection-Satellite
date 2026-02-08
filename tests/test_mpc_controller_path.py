import numpy as np
import pytest

from satellite_control.config import physics as physics_cfg
from satellite_control.config.models import (
    AppConfig,
    MPCParams,
    SatellitePhysicalParams,
    SimulationParams,
)
from satellite_control.control.mpc_controller import MPCController


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
            obstacle_margin=0.5,
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


class TestMPCPathFollowing:
    """Test MPC Path Following capabilities."""

    def test_path_following_initialization(self):
        """Test initialization with Path Following Mode enabled."""
        cfg = create_default_app_config()
        # mode_path_following is now enforced internally

        mpc = MPCController(cfg)

        assert mpc.mode_path_following is True
        assert mpc.nx == 17  # 16 + 1 (s)
        # nu might be 6 or augmented depending on C++ exposure, but python wrapper keeps it 6+RW
        assert mpc.nu == 6

    def test_set_path(self):
        """Test setting path data."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        # Define a simple path
        path = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

        mpc.set_path(path)

        # Check path length calculation
        # distance is 2.0
        assert mpc._path_length == pytest.approx(2.0)
        assert mpc.s == 0.0
        assert mpc._path_set is True

    def test_get_control_action_mpcc(self):
        """Test control action generation in MPCC mode."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        # Set a path
        path = [(0, 0, 0), (10, 0, 0)]
        mpc.set_path(path)

        # Current state: at start
        x_curr = np.zeros(16)
        x_curr[3] = 1.0  # quat

        u, info = mpc.get_control_action(x_curr)

        assert u is not None
        # Should return physical controls + extras
        # Python wrapper strips v_s from return u
        assert len(u) == mpc.nu
        assert "path_s" in info
        assert "path_v_s" in info
        assert "solve_time" in info

    def test_path_projection_progress(self):
        """Test path projection for progress and error metrics."""
        cfg = create_default_app_config()
        mpc = MPCController(cfg)

        path = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        mpc.set_path(path)

        pos = np.array([0.75, 0.2, 0.0])
        metrics = mpc.get_path_progress(pos)

        assert metrics["s"] == pytest.approx(0.75, abs=1e-2)
        assert metrics["path_error"] == pytest.approx(0.2, abs=1e-2)
