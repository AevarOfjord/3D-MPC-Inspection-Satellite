from __future__ import annotations

import logging
import sys
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from config.models import AppConfig

_CPP_SIM_IMPORT_ERROR: ImportError | None = None
try:
    from cpp._cpp_sim import SatelliteParams, SimulationEngine
except ImportError as exc_src:
    _CPP_SIM_IMPORT_ERROR = exc_src
    # Allow import without compiled module for typing/testing if optional.
    SimulationEngine = None  # type: ignore
    SatelliteParams = None  # type: ignore
except Exception as exc:  # pragma: no cover - import-time runtime mismatch details
    _CPP_SIM_IMPORT_ERROR = ImportError(str(exc))
    SimulationEngine = None  # type: ignore
    SatelliteParams = None  # type: ignore

from physics.orbital_config import OrbitalConfig
from utils.orientation_utils import (
    euler_xyz_to_quat_wxyz,
    quat_wxyz_to_euler_xyz,
)

logger = logging.getLogger(__name__)


def _raise_cpp_sim_binding_import_error() -> None:
    """Raise a detailed error when C++ simulation bindings cannot be imported."""
    py_ver = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )
    original = _CPP_SIM_IMPORT_ERROR or ImportError("Unknown _cpp_sim import error")
    message = (
        "Failed to import C++ simulation bindings (`cpp._cpp_sim`). "
        f"Running interpreter: Python {py_ver}. Original error: {original}"
    )
    if "Python version mismatch" in str(original):
        message += (
            " Detected ABI mismatch between the active Python interpreter and the compiled "
            "extension. Rebuild `_cpp_sim` with the same interpreter used to run the app/tests "
            "(for this repo, Python 3.11 is the supported development target)."
        )

    raise ImportError(message) from original


class CppSatelliteSimulator:
    """
    Python wrapper for the C++ Simulation Engine.
    Implements the Python simulator interface for seamless drop-in replacement.

    Faster than the Python physics backend for orbital dynamics simulation.
    """

    def __init__(self, app_config: AppConfig):
        """
        Initialize the C++ Satellite Simulator.

        Args:
            app_config: Application configuration.
        """
        if SimulationEngine is None:
            _raise_cpp_sim_binding_import_error()

        self.app_config = app_config
        self.dt = app_config.simulation.dt

        self.simulation_time = 0.0

        # Parse params from config
        self._cpp_params = self._create_satellite_params(app_config)

        try:
            orbital_mean_motion = float(app_config.physics.orbital.mean_motion)
        except AttributeError:
            # Fallback to default LEO if not configured in AppConfig
            orbital_mean_motion = OrbitalConfig().mean_motion

        self.engine = SimulationEngine(self._cpp_params, orbital_mean_motion)

        # Local state cache (setters invalidate the cache).

        # Store for visualization integration.
        self.ax = None
        self.fig = None
        self.thruster_colors = {}
        self.force_history = []

        # Thruster state tracking for ThrusterManager integration.
        self.active_thrusters = set()
        self.thruster_activation_time = {}
        self.thruster_deactivation_time = {}

        # Pre-allocate command arrays as numpy for efficient C++ bridge crossing
        self._current_thruster_cmds = np.zeros(
            self._cpp_params.num_thrusters, dtype=np.float64
        )
        self._current_rw_torques = np.zeros(3, dtype=np.float64)

        # State cache: avoids repeated engine.get_state() calls within one timestep
        self._cached_state: np.ndarray | None = None
        self._cached_state_time: float = -1.0

    def _get_cached_state(self) -> np.ndarray:
        """Get state, reusing cache if same timestep."""
        if (
            self._cached_state is None
            or self._cached_state_time != self.simulation_time
        ):
            self._cached_state = self.engine.get_state()
            self._cached_state_time = self.simulation_time
        return self._cached_state

    def _invalidate_state_cache(self) -> None:
        """Invalidate state cache (call after any state mutation)."""
        self._cached_state = None

    def set_thruster_level(self, thruster_id: int, level: float):
        """
        Set individual thruster output level.
        Called by ThrusterManager.

        Args:
            thruster_id: 1-based index (1-N)
            level: Output level [0.0, 1.0]
        """
        # Direct numpy array indexing — no hasattr, no list growth guard
        self._current_thruster_cmds[thruster_id - 1] = level

    def _create_satellite_params(self, cfg: AppConfig):
        """Create C++ SatelliteParams from AppConfig."""
        params = SatelliteParams()
        params.dt = self.dt
        params.mass = cfg.physics.total_mass
        # Inertia from config is scalar float (isotropic assumption for now or principal axis)
        # C++ expects Vector3d.
        I_scalar = float(cfg.physics.moment_of_inertia)
        params.inertia = np.array([I_scalar, I_scalar, I_scalar], dtype=np.float64)

        # Thrusters
        t_pos_dict = cfg.physics.thruster_positions
        t_dir_dict = cfg.physics.thruster_directions

        # Sort by ID "1", "2", ...
        sorted_ids = sorted(t_pos_dict.keys(), key=lambda k: int(k))
        params.num_thrusters = len(sorted_ids)

        pos_list = []
        dir_list = []
        force_list = []

        for tid in sorted_ids:
            pos_list.append(np.array(t_pos_dict[tid]))
            dir_list.append(np.array(t_dir_dict[tid]))
            force_list.append(float(cfg.physics.thruster_forces[tid]))

        params.thruster_positions = pos_list
        params.thruster_directions = dir_list
        params.thruster_forces = force_list

        # Reaction Wheels.
        rws = list(cfg.physics.reaction_wheels)
        params.num_rw = len(rws)
        params.rw_torque_limits = [float(rw.max_torque) for rw in rws]
        params.rw_inertia = [float(rw.inertia) for rw in rws]
        if hasattr(params, "rw_speed_limits"):
            params.rw_speed_limits = [float(rw.max_speed) for rw in rws]

        params.com_offset = np.array(cfg.physics.com_offset)

        # Orbital parameters for MPC consistency (not used by simulation engine)
        try:
            orbital_cfg = getattr(cfg.physics, "orbital", None)
            if orbital_cfg is not None:
                if hasattr(params, "orbital_mu"):
                    params.orbital_mu = float(
                        getattr(orbital_cfg, "mu", OrbitalConfig().mu)
                    )
                if hasattr(params, "orbital_radius"):
                    params.orbital_radius = float(
                        getattr(
                            orbital_cfg,
                            "orbital_radius",
                            OrbitalConfig().orbital_radius,
                        )
                    )
                if hasattr(params, "orbital_mean_motion"):
                    params.orbital_mean_motion = float(
                        getattr(orbital_cfg, "mean_motion", OrbitalConfig().mean_motion)
                    )
            else:
                orbital_default = OrbitalConfig()
                if hasattr(params, "orbital_mu"):
                    params.orbital_mu = float(orbital_default.mu)
                if hasattr(params, "orbital_radius"):
                    params.orbital_radius = float(orbital_default.orbital_radius)
                if hasattr(params, "orbital_mean_motion"):
                    params.orbital_mean_motion = float(orbital_default.mean_motion)
        except Exception:
            logger.warning(
                "Failed to load orbital config, using Earth LEO defaults", exc_info=True
            )
            if hasattr(params, "orbital_mu"):
                params.orbital_mu = 3.986004418e14
            if hasattr(params, "orbital_radius"):
                params.orbital_radius = 6.778e6
            if hasattr(params, "orbital_mean_motion"):
                params.orbital_mean_motion = 0.0
        if hasattr(params, "use_two_body"):
            params.use_two_body = getattr(cfg.physics, "use_two_body_gravity", True)
        return params

    @property
    def position(self) -> np.ndarray:
        """Get position [x, y, z]."""
        return self._get_cached_state()[0:3]

    @position.setter
    def position(self, value: np.ndarray):
        s = self.engine.get_state()
        s[0:3] = value
        self.engine.reset(s)
        self._invalidate_state_cache()

    @property
    def quaternion(self) -> np.ndarray:
        """Get quaternion [w, x, y, z]."""
        return self._get_cached_state()[3:7]

    @property
    def angle(self) -> tuple[float, float, float]:
        """Get Euler angles (roll, pitch, yaw)."""
        q = self.quaternion
        return quat_wxyz_to_euler_xyz(q)

    @angle.setter
    def angle(self, euler: tuple[float, float, float]):
        """Set orientation from Euler angles."""
        q = euler_xyz_to_quat_wxyz(euler)
        s = self.engine.get_state()
        s[3:7] = q
        self.engine.reset(s)
        self._invalidate_state_cache()

    @property
    def velocity(self) -> np.ndarray:
        """Get velocity [vx, vy, vz]."""
        return self._get_cached_state()[7:10]

    @velocity.setter
    def velocity(self, value: np.ndarray):
        s = self.engine.get_state()
        s[7:10] = value
        self.engine.reset(s)
        self._invalidate_state_cache()

    @property
    def angular_velocity(self) -> np.ndarray:
        """Get body angular velocity [wx, wy, wz]."""
        return self._get_cached_state()[10:13]

    @angular_velocity.setter
    def angular_velocity(self, value: float | tuple[float, float, float]):
        if isinstance(value, int | float):
            # Scalar (Yaw rate only)
            w = np.array([0.0, 0.0, float(value)])
        else:
            w = np.array(value)
        s = self.engine.get_state()
        s[10:13] = w
        self.engine.reset(s)
        self._invalidate_state_cache()

    @property
    def wheel_speeds(self) -> np.ndarray:
        """Get reaction wheel speeds [rad/s]."""
        return self.engine.get_rw_speeds()

    def apply_force(self, force: list[float]):
        """
        Set thruster duty cycles for the NEXT step.
        Legacy Python simulator applied forces during integration.
        Here we store them for update_physics.
        """
        self._current_thruster_cmds[:] = force

    def set_reaction_wheel_torque(self, torque: np.ndarray):
        """Set reaction wheel torques for the NEXT step."""
        self._current_rw_torques[:] = torque

    def update_physics(self, dt: float):
        """
        Step the physics simulation.

        Args:
            dt: Time to step (RK4 handles arbitrary dt).
        """
        self.engine.step(dt, self._current_thruster_cmds, self._current_rw_torques)
        self.simulation_time += dt
        self._invalidate_state_cache()

    def update_physics_batch(self, steps: int, dt: float):
        """
        Step the physics simulation multiple times with constant inputs.

        Args:
            steps: Number of physics steps to take
            dt: Time step per physics step
        """
        if steps <= 0:
            return

        self.engine.step_batch(
            steps, dt, self._current_thruster_cmds, self._current_rw_torques
        )
        self.simulation_time += dt * steps
        self._invalidate_state_cache()

    # Visualization Compat (Headless Mocks)
    def is_viewer_paused(self):
        return False

    def consume_viewer_step(self):
        return False

    def sync_viewer(self):
        pass
