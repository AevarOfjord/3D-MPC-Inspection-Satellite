import pytest
import numpy as np
from src.satellite_control.config.orbital_config import OrbitalConfig
from src.satellite_control.physics.orbital_dynamics import (
    CWDynamics,
    compute_cw_acceleration,
    compute_cw_force,
)


@pytest.fixture
def orbital_config():
    """Default orbital configuration."""
    return OrbitalConfig()  # Use defaults


@pytest.fixture
def cw_dynamics(orbital_config):
    """CWDynamics instance."""
    return CWDynamics(orbital_config=orbital_config)


def test_cw_dynamics_initialization(cw_dynamics):
    """Test standard initialization."""
    assert cw_dynamics.n > 0
    assert cw_dynamics.n_sq > 0


def test_compute_acceleration_consistency(cw_dynamics, orbital_config):
    """Verify class method matches standalone function."""
    pos = np.array([100.0, 50.0, -20.0])
    vel = np.array([0.5, -0.2, 0.1])

    acc_class = cw_dynamics.compute_acceleration(pos, vel)
    acc_func = compute_cw_acceleration(pos, vel, mean_motion=orbital_config.mean_motion)

    np.testing.assert_allclose(acc_class, acc_func, rtol=1e-10)


def test_compute_acceleration_values(cw_dynamics, orbital_config):
    """Verify analytical correctness of CW equations."""
    n = orbital_config.mean_motion
    n_sq = n**2

    # Static hold: x=0, y=0, z=0, vel=0 -> acc=0
    pos = np.zeros(3)
    vel = np.zeros(3)
    acc = cw_dynamics.compute_acceleration(pos, vel)
    np.testing.assert_allclose(acc, np.zeros(3), atol=1e-12)

    # Pure radial offset: x=100, y=0, z=0, vel=0
    # ax = 3n^2 x
    pos = np.array([100.0, 0.0, 0.0])
    acc = cw_dynamics.compute_acceleration(pos, vel)
    expected_ax = 3 * n_sq * 100.0
    np.testing.assert_allclose(acc, [expected_ax, 0.0, 0.0], rtol=1e-10)

    # V-bar drift: x=0, y=100, z=0, vx=0, vy=0, vz=0
    # No accel (linear drift approx)
    pos = np.array([0.0, 100.0, 0.0])
    acc = cw_dynamics.compute_acceleration(pos, vel)
    np.testing.assert_allclose(acc, np.zeros(3), atol=1e-12)


def test_compute_force_wrapper(orbital_config):
    """Test the force computation wrapper."""
    pos = np.array([10.0, 0.0, 0.0])
    vel = np.zeros(3)
    mass = 12.5

    # Force = mass * accel
    accel = compute_cw_acceleration(pos, vel, orbital_config.mean_motion)
    expected_force = mass * accel

    actual_force = compute_cw_force(
        position=pos, velocity=vel, mass=mass, orbital_config=orbital_config
    )

    np.testing.assert_allclose(actual_force, expected_force, rtol=1e-10)


def test_state_matrices_dimensions(cw_dynamics):
    """Verify A and B matrix shapes."""
    dt = 0.1
    A, B = cw_dynamics.get_state_matrices(dt)

    assert A.shape == (6, 6)
    assert B.shape == (6, 3)

    # Check identity diagonal roughly (for small dt)
    np.testing.assert_allclose(np.diag(A), np.ones(6), atol=0.1)


def test_mpc_matrices_dimensions(cw_dynamics):
    """Verify MPC matrix integration."""
    dt = 0.1
    A_cw, B_cw = cw_dynamics.get_mpc_dynamics_matrices(dt)

    assert A_cw.shape == (16, 16)
    assert B_cw.shape == (16, 9)

    # Verify CW terms are in the velocity block (indices 7,8,9 affecting 7,8,9 from 0,1,2 and 7,8,9)
    # Check x -> vx coupling (3n^2 term)
    assert A_cw[7, 0] != 0.0
