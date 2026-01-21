from typing import Any, Dict, List, Optional, Tuple, Union

import numpy as np
import logging

try:
    from satellite_control.cpp._cpp_sim import SimulationEngine, SatelliteParams
except ImportError:
    # Allow import without compiled module for typing/testing if optional
    SimulationEngine = None  # type: ignore
    SatelliteParams = None  # type: ignore

from src.satellite_control.utils.orientation_utils import (
    quat_wxyz_to_euler_xyz,
    euler_xyz_to_quat_wxyz,
)
from src.satellite_control.config.orbital_config import OrbitalConfig
from src.satellite_control.config.timing import SIMULATION_DT

logger = logging.getLogger(__name__)


class CppSatelliteSimulator:
    """
    Python wrapper for the C++ Simulation Engine.
    Mimics the legacy Python simulator interface for seamless drop-in replacement.

    Faster than the legacy Python physics backend for orbital dynamics simulation.
    """

    def __init__(self, app_config: Any):
        """
        Initialize the C++ Satellite Simulator.

        Args:
            app_config: Application configuration (Hydra DictConfig)
        """
        if SimulationEngine is None:
            raise ImportError(
                "C++ Simulation Engine module (_cpp_sim) not found. "
                "Please compile with 'pip install -e .'"
            )

        self.app_config = app_config
        # Get physics dt from simulation config (preferred) then mpc, then fallback
        if hasattr(app_config, "simulation") and hasattr(app_config.simulation, "dt"):
            self.dt = app_config.simulation.dt
        elif hasattr(app_config, "mpc") and hasattr(app_config.mpc, "dt"):
            self.dt = app_config.mpc.dt
        else:
            self.dt = SIMULATION_DT  # Single source of truth from timing.py

        self.simulation_time = 0.0

        # Parse params from config
        self._cpp_params = self._create_satellite_params(app_config)

        try:
            orbital_mean_motion = float(app_config.physics.orbital.mean_motion)
        except AttributeError:
            # Fallback to default LEO if not configured in AppConfig
            orbital_mean_motion = OrbitalConfig().mean_motion

        self.engine = SimulationEngine(self._cpp_params, orbital_mean_motion)

        # Local state cache to match legacy access patterns if needed,
        # or just fetch from engine on demand.
        # But setters need to update engine.

        # Store for visualizations compatibility
        self.ax = None
        self.fig = None
        self.thruster_colors = {}
        self.force_history = []

        # Thruster state tracking (for ThrusterManager compatibility)
        self.active_thrusters = set()
        self.thruster_activation_time = {}
        self.thruster_deactivation_time = {}

    def set_thruster_level(self, thruster_id: int, level: float):
        """
        Set individual thruster output level.
        Called by ThrusterManager.

        Args:
            thruster_id: 1-based index (1-N)
            level: Output level [0.0, 1.0]
        """
        if not hasattr(self, "_current_thruster_cmds"):
            self._current_thruster_cmds = [0.0] * self._cpp_params.num_thrusters

        # Ensure list is long enough
        while len(self._current_thruster_cmds) < thruster_id:
            self._current_thruster_cmds.append(0.0)

        # Update command (0-indexed internally)
        self._current_thruster_cmds[thruster_id - 1] = level

    def _create_satellite_params(self, cfg: Any):
        """Create C++ SatelliteParams from Hydra config."""
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

        # Reaction Wheels (if config has them)
        params.num_rw = 0
        params.rw_torque_limits = []

        # Check if cfg has vehicle attribute (DictConfig) or we are using AppConfig
        if hasattr(cfg, "reaction_wheels") and cfg.reaction_wheels:
            rws = cfg.reaction_wheels
            params.num_rw = len(rws)
            params.rw_torque_limits = [float(rw.max_torque) for rw in rws]
            params.rw_inertia = [float(rw.inertia) if hasattr(rw, "inertia") else 0.001 for rw in rws]
        elif hasattr(cfg, "mpc") and hasattr(cfg.mpc, "r_rw_torque"):
             # Basic fallback if needed, but safe to leave 0
             pass

        params.com_offset = np.array(cfg.physics.com_offset)
        return params

    @property
    def position(self) -> np.ndarray:
        """Get position [x, y, z]."""
        return self.engine.get_state()[0:3]

    @position.setter
    def position(self, value: np.ndarray):
        s = self.engine.get_state()
        s[0:3] = value
        self.engine.reset(s)

    @property
    def quaternion(self) -> np.ndarray:
        """Get quaternion [w, x, y, z]."""
        return self.engine.get_state()[3:7]

    @property
    def angle(self) -> Tuple[float, float, float]:
        """Get Euler angles (roll, pitch, yaw)."""
        q = self.quaternion
        return quat_wxyz_to_euler_xyz(q)

    @angle.setter
    def angle(self, euler: Tuple[float, float, float]):
        """Set orientation from Euler angles."""
        q = euler_xyz_to_quat_wxyz(euler)
        s = self.engine.get_state()
        s[3:7] = q
        self.engine.reset(s)

    @property
    def velocity(self) -> np.ndarray:
        """Get velocity [vx, vy, vz]."""
        return self.engine.get_state()[7:10]

    @velocity.setter
    def velocity(self, value: np.ndarray):
        s = self.engine.get_state()
        s[7:10] = value
        self.engine.reset(s)

    @property
    def angular_velocity(self) -> np.ndarray:
        """Get body angular velocity [wx, wy, wz]."""
        return self.engine.get_state()[10:13]

    @angular_velocity.setter
    def angular_velocity(self, value: Union[float, Tuple[float, float, float]]):
        if isinstance(value, (int, float)):
            # Scalar (Yaw rate only)
            w = np.array([0.0, 0.0, float(value)])
        else:
            w = np.array(value)
        s = self.engine.get_state()
        s[10:13] = w
        self.engine.reset(s)

    @property
    def wheel_speeds(self) -> np.ndarray:
        """Get reaction wheel speeds [rad/s]."""
        return self.engine.get_rw_speeds()

    def apply_force(self, force: List[float]):
        """
        Set thruster duty cycles for the NEXT step.
        Legacy Python simulator applied forces during integration.
        Here we store them for update_physics.
        """
        self._current_thruster_cmds = force

    def set_reaction_wheel_torque(self, torque: np.ndarray):
        """Set reaction wheel torques for the NEXT step."""
        self._current_rw_torques = torque

    def update_physics(self, dt: float):
        """
        Step the physics simulation.

        Args:
            dt: Time to step (must match params.dt usually, or RK4 handles arbitrary?)
                RK4 handles arbitrary dt.
        """
        # Get pending commands (default to zero if not set)
        if not hasattr(self, "_current_thruster_cmds"):
            self._current_thruster_cmds = [0.0] * self._cpp_params.num_thrusters

        if not hasattr(self, "_current_rw_torques"):
            self._current_rw_torques = [0.0] * 3

        # Call Engine Step
        self.engine.step(dt, self._current_thruster_cmds, self._current_rw_torques)
        self.simulation_time += dt


    # Visualization Compat (Headless Mocks)
    def is_viewer_paused(self):
        return False

    def consume_viewer_step(self):
        return False

    def sync_viewer(self):
        pass
