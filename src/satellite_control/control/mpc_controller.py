"""
Unified MPC Controller using OSQP.

Combines logic from previous BaseMPC and PWMMPC into a single, optimized class.
"""

import logging
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import osqp
import scipy.sparse as sp

# from src.satellite_control.config import mpc_params
# from src.satellite_control.config.reaction_wheel_config import get_reaction_wheel_config
# from src.satellite_control.config.models import MPCParams, SatellitePhysicalParams
from src.satellite_control.core.error_handling import with_error_context
from src.satellite_control.core.exceptions import (
    OptimizationError,
)  # , SolverTimeoutError
from src.satellite_control.utils.caching import cached
from src.satellite_control.utils.orientation_utils import (
    euler_xyz_to_quat_wxyz,
    quat_wxyz_to_euler_xyz,
)

from .base import Controller

logger = logging.getLogger(__name__)


class MPCController(Controller):
    """
    Satellite Model Predictive Controller using OSQP (3D).

    Unified implementation for 6-DOF control.
    State: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz] (13 elements).
    Control: [τ_rw_x, τ_rw_y, τ_rw_z, u1, ..., uN] (RW torques + thrusters).
    """

    def __init__(
        self,
        cfg: Any,  # Expects the full Hydra DictConfig or a specific sub-config
    ):
        """
        Initialize MPC controller with OSQP and pre-allocate matrices.
        """
        # Unwrap if passed the root config
        if hasattr(cfg, "control") and hasattr(cfg, "vehicle"):
            self.mpc_cfg = cfg.control.mpc
            self.vehicle_cfg = cfg.vehicle
        else:
            # Assume we got passed standard objects or dicts (legacy support or direct injection)
            # This part requires careful migration. For now, let's assume strict Hydra usage
            # or simple duck-typing if needed.
            raise ValueError(
                "MPCController now requires a valid Hydra-like config object with .control.mpc and .vehicle"
            )

        # Helper to safely access attributes (Omegaconf/Dataclass)
        def get(obj, attr, default=None):
            return getattr(obj, attr, default)

        # Satellite physical parameters
        self.total_mass = self.vehicle_cfg.mass
        # Ensure inertia is 3D array
        inertia = self.vehicle_cfg.inertia
        if hasattr(inertia, "__len__") and len(inertia) == 3:
            self.moment_of_inertia = np.array(inertia, dtype=float)
        else:
            # scalar to vector
            val = float(inertia)
            self.moment_of_inertia = np.array([val, val, val], dtype=float)

        self.com_offset = np.array(self.vehicle_cfg.center_of_mass)

        # Thruster Configuration
        self.thrusters = self.vehicle_cfg.thrusters
        self.num_thrusters = len(self.thrusters)
        self.thruster_ids = list(range(self.num_thrusters))

        # Extract thruster parameters for easy access
        self.thruster_positions = [t.position for t in self.thrusters]
        self.thruster_directions = [t.direction for t in self.thrusters]
        self.thruster_forces = [float(t.max_thrust) for t in self.thrusters]

        # Reaction wheel configuration
        self.reaction_wheels = self.vehicle_cfg.reaction_wheels
        self.num_rw_axes = len(
            self.reaction_wheels
        )  # Should be 3 ideally for this code, or adaptation needed

        # RW Logic (Assuming 3 orthogonal wheels aligned with axes for now, as per linear model assumption)
        # Verify assumption or adapt. The 6U yaml defines 3 wheels.
        self.enable_rw_yaw = True  # Configurable?

        # Max torque per axis
        # Assuming axis 0 is X, 1 is Y, 2 is Z
        if self.num_rw_axes >= 3:
            self.max_rw_torque = self.reaction_wheels[0].max_torque  # Simplified
        else:
            self.max_rw_torque = 0.0

        self.rw_torque_limits = np.array([rw.max_torque for rw in self.reaction_wheels])

        # MPC parameters
        self.N = self.mpc_cfg.prediction_horizon
        self._dt = self.mpc_cfg.settings.dt
        self.solver_time_limit = self.mpc_cfg.solver_time_limit

        # Cost weights
        w = self.mpc_cfg.weights
        self.Q_pos = w.position
        self.Q_vel = w.velocity
        self.Q_ang = w.angle
        self.Q_angvel = w.angular_velocity
        self.R_thrust = w.thrust
        self.R_rw_torque = w.rw_torque
        self.R_switch = w.switch

        # Constraints
        c = self.mpc_cfg.constraints
        self.max_velocity = c.max_velocity
        self.max_angular_velocity = c.max_angular_velocity
        self.position_bounds = c.position_bounds

        # Adaptive settings
        if hasattr(self.mpc_cfg, "adaptive"):
            a = self.mpc_cfg.adaptive
            self.damping_zone = a.damping_zone
            self.velocity_threshold = a.velocity_threshold
            self.max_velocity_weight = a.max_velocity_weight

        # Z translation and other settings
        s = self.mpc_cfg.settings
        self.thruster_type = s.thruster_type
        self.enable_rw_yaw = s.enable_rw_yaw
        self.enable_z_tilt = s.enable_z_tilt
        self.z_tilt_gain = s.z_tilt_gain
        self.z_tilt_max_deg = s.z_tilt_max_deg
        self.z_tilt_max_rad = np.deg2rad(self.z_tilt_max_deg)

        # Verbose
        self.verbose = s.verbose_mpc

        # State dimension: [p(3), q(4), v(3), w(3)]
        self.nx = 13
        # Control dimension: RW torques + thrusters
        self.nu = self.num_rw_axes + self.num_thrusters

        self.control_lower = np.concatenate(
            [np.full(self.num_rw_axes, -1.0), np.zeros(self.num_thrusters)]
        )
        self.control_upper = np.concatenate(
            [np.full(self.num_rw_axes, 1.0), np.ones(self.num_thrusters)]
        )

        # Performance tracking
        self.solve_times: list[float] = []

        # Precompute thruster forces (body frame)
        self._precompute_thruster_forces()

        # Precompute Q_diag (used frequently in get_control_action)
        self.Q_diag = np.concatenate(
            [
                np.full(3, self.Q_pos),
                np.full(4, self.Q_ang),
                np.full(3, self.Q_vel),
                np.full(3, self.Q_angvel),
            ]
        )

        # Use verbose flag from mpc_params
        # Use verbose flag from mpc_params or config
        self.verbose = getattr(self.mpc_cfg, "verbose_mpc", False)

        if self.verbose:
            print("OSQP MPC Controller Initializing (3D)...")

        # Problem dimensions
        self.n_vars = (self.N + 1) * self.nx + self.N * self.nu

        # Constraints counts
        n_dyn = self.N * self.nx
        n_init = self.nx
        n_bounds_x = (self.N + 1) * self.nx
        n_bounds_u = self.N * self.nu
        self.n_constraints = n_dyn + n_init + n_bounds_x + n_bounds_u

        # OSQP Solver instance
        self.prob = osqp.OSQP()

        # Initialize Persistent Matrices
        self._init_solver_structures()

        # State tracking for updates
        self.prev_quat = np.array([-999.0] * 4)  # Forces update

        if self.verbose:
            print("OSQP MPC Ready.")

    @property
    def dt(self) -> float:
        """Control update interval in seconds (Controller interface)."""
        return self._dt

    @property
    def prediction_horizon(self) -> Optional[int]:
        """Prediction horizon (Controller interface)."""
        return self.N

    def _precompute_thruster_forces(self) -> None:
        """Precompute thruster forces and torques in body frame (3D)."""
        # Forces: N x 3
        # Torques: N x 3
        self.body_frame_forces = np.zeros((self.num_thrusters, 3), dtype=np.float64)
        self.body_frame_torques = np.zeros((self.num_thrusters, 3), dtype=np.float64)

        for i, thruster_id in enumerate(self.thruster_ids):
            pos = np.array(self.thruster_positions[thruster_id])
            rel_pos = pos - self.com_offset
            direction = np.array(self.thruster_directions[thruster_id])
            force = self.thruster_forces[thruster_id] * direction

            self.body_frame_forces[i] = force
            self.body_frame_torques[i] = np.cross(rel_pos, force)

    @cached(maxsize=128)
    def _compute_rotation_matrix(self, quat_tuple: tuple) -> np.ndarray:
        """
        Compute rotation matrix from quaternion (cached).

        Args:
            quat_tuple: Quaternion as tuple (for hashing)

        Returns:
            3x3 rotation matrix
        """
        import mujoco

        qv = np.array(quat_tuple)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, qv)
        return R.reshape(3, 3)

    def linearize_dynamics(
        self, x_current: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linearize dynamics around current state (quaternion).

        Uses caching for expensive rotation matrix computation.
        """
        # x = [px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        # Indices:
        # P: 0-2 (3)
        # Q: 3-6 (4)
        # V: 7-9 (3)
        # W: 10-12 (3)

        qv = x_current[3:7]  # Current quaternion

        # Round quaternion to reduce cache size (binning for similar orientations)
        quat_rounded = tuple(np.round(qv, decimals=3))

        # Use cached rotation matrix computation
        R = self._compute_rotation_matrix(quat_rounded)

        # State transition A (13 x 13)
        A = np.eye(13)

        # dPos/dVel = I * dt
        A[0, 7] = self.dt
        A[1, 8] = self.dt
        A[2, 9] = self.dt

        # dQuat/dOmega = 0.5 * G(q) * dt
        w, x, y, z = qv[0], qv[1], qv[2], qv[3]
        G = 0.5 * np.array([[-x, -y, -z], [w, -z, y], [z, w, -x], [-y, x, w]]) * self.dt
        A[3:7, 10:13] = G

        # Control input B (13 x (RW + thrusters))
        B = np.zeros((13, self.nu))

        # Reaction wheel torque effect (body frame)
        for i in range(self.num_rw_axes):
            if self.rw_torque_limits[i] == 0.0:
                continue
            # Handle 3D inertia (diagonal)
            B[10 + i, i] = (
                self.rw_torque_limits[i] / self.moment_of_inertia[i] * self.dt
            )

        thruster_offset = self.num_rw_axes
        for i in range(self.num_thrusters):
            # Velocity update (World frame) -> Force rotated
            F_body = self.body_frame_forces[i]
            F_world = R @ F_body
            B[7:10, thruster_offset + i] = F_world / self.total_mass * self.dt

            # Angular velocity update (Body frame) -> Torque in body
            T_body = self.body_frame_torques[i]
            B[10:13, thruster_offset + i] = T_body / self.moment_of_inertia * self.dt

        return A, B

    def _init_solver_structures(self):
        """Build initial solver structures."""
        # Cost Matrix P
        # Weights: [pos(3), quat(4), vel(3), angvel(3)]
        # Quat weight? If Target is [1,0,0,0], penalize x,y,z components heavily?
        # Q_ang applies to all 4 or just vector part?
        # Let's apply uniformly for now.

        Q_diag = np.concatenate(
            [
                np.full(3, self.Q_pos),
                np.full(4, self.Q_ang),
                np.full(3, self.Q_vel),
                np.full(3, self.Q_angvel),
            ]
        )  # 13 elements

        R_diag = np.concatenate(
            [
                np.full(self.num_rw_axes, self.R_rw_torque),
                np.full(self.num_thrusters, self.R_thrust),
            ]
        )

        diags = []
        for _ in range(self.N):
            diags.append(Q_diag)
        diags.append(Q_diag * 10.0)  # Terminal
        for _ in range(self.N):
            diags.append(R_diag)

        self.P = sp.diags(np.concatenate(diags), format="csc")

        # Linear Cost q
        self.q = np.zeros(self.n_vars)

        # Equality Constraints A (Dynamics)
        # B mapping: A matrix data depends on B, which depends on 'q' (orientation).
        self.B_idx_map: Dict[Tuple[int, int], List[int]] = {}
        for r in range(self.nx):  # 13 rows for dynamics
            for c in range(self.nu):  # control columns
                self.B_idx_map[(r, c)] = []

        row_idx = 0
        dummy_state = np.zeros(13)
        dummy_state[3] = 1.0  # Valid quat
        A_template, B_template = self.linearize_dynamics(dummy_state)

        triples = []

        # Dyn: -A x_k + x_{k+1} - B u_k = 0
        for k in range(self.N):
            x_k_idx = k * self.nx
            x_kp1_idx = (k + 1) * self.nx
            u_k_idx = (self.N + 1) * self.nx + k * self.nu

            # -A
            for r in range(self.nx):
                for c in range(self.nx):
                    if A_template[r, c] != 0:
                        triples.append((row_idx + r, x_k_idx + c, -A_template[r, c]))

            # +I
            for r in range(self.nx):
                triples.append((row_idx + r, x_kp1_idx + r, 1.0))

            # -B
            for r in range(self.nx):
                for c in range(self.nu):
                    # We store -1.0 placeholder, update later
                    triples.append((row_idx + r, u_k_idx + c, -B_template[r, c]))

            row_idx += self.nx

        # Init
        for r in range(self.nx):
            triples.append((row_idx + r, r, 1.0))
        row_idx += self.nx

        # Bounds X
        for k in range(self.N + 1):
            x_k_idx = k * self.nx
            for r in range(self.nx):
                triples.append((row_idx + r, x_k_idx + r, 1.0))
            row_idx += self.nx

        # Bounds U
        for k in range(self.N):
            u_k_idx = (self.N + 1) * self.nx + k * self.nu
            for r in range(self.nu):
                triples.append((row_idx + r, u_k_idx + r, 1.0))
            row_idx += self.nu

        # Build A
        rows = [t[0] for t in triples]
        cols = [t[1] for t in triples]
        vals = [t[2] for t in triples]

        self.A = sp.csc_matrix(
            (vals, (rows, cols)), shape=(self.n_constraints, self.n_vars)
        )
        self.A.sort_indices()

        # Map B indices
        u_start_idx = (self.N + 1) * self.nx
        for k in range(self.N):
            current_u_idx = u_start_idx + k * self.nu
            current_row_base = k * self.nx
            for c in range(self.nu):
                col = current_u_idx + c
                start_ptr = self.A.indptr[col]
                end_ptr = self.A.indptr[col + 1]
                col_rows = self.A.indices[start_ptr:end_ptr]
                for r in range(self.nx):
                    target_row = current_row_base + r
                    match = np.where(col_rows == target_row)[0]
                    if len(match) > 0:
                        self.B_idx_map[(r, c)].append(start_ptr + match[0])

        # Bounds l/u
        self.l = np.zeros(self.n_constraints)
        self.u = np.zeros(self.n_constraints)

        # Control bounds (RW torques + thrusters)
        ctrl_idx = self.n_constraints - self.N * self.nu
        for k in range(self.N):
            start = ctrl_idx + k * self.nu
            self.l[start : start + self.nu] = self.control_lower
            self.u[start : start + self.nu] = self.control_upper

        # State Bounds
        # default to inf
        state_idx_start = self.N * self.nx + self.nx
        state_idx_end = state_idx_start + (self.N + 1) * self.nx

        self.l[state_idx_start:state_idx_end] = -1e20
        self.u[state_idx_start:state_idx_end] = 1e20

        # Setup OSQP
        self.prob.setup(
            self.P,
            self.q,
            self.A,
            self.l,
            self.u,
            verbose=False,
            time_limit=self.solver_time_limit,
        )

    def _update_A_data(self, B_dyn: np.ndarray):
        """Update B-matrix part of A."""
        for r in range(self.nx):
            for c in range(self.nu):
                val = -B_dyn[r, c]
                indices = self.B_idx_map[(r, c)]
                for idx in indices:
                    self.A.data[idx] = val

    def _apply_z_tilt_target(
        self, x_current: np.ndarray, x_target: np.ndarray
    ) -> np.ndarray:
        """Tilt target roll/pitch to enable Z translation with planar thrusters."""
        if not self.enable_z_tilt or x_target.shape[0] < 7:
            return x_target

        z_err = x_target[2] - x_current[2]
        if abs(z_err) < 1e-4:
            return x_target

        q = np.array(x_target[3:7], dtype=float)
        if np.linalg.norm(q) == 0:
            q = np.array([1.0, 0.0, 0.0, 0.0])

        roll_t, pitch_t, yaw_t = quat_wxyz_to_euler_xyz(q)

        dx = x_target[0] - x_current[0]
        dy = x_target[1] - x_current[1]
        norm_xy = np.hypot(dx, dy)

        if norm_xy < 1e-6:
            d_xy_body = np.array([1.0, 0.0])
        else:
            d_xy_world = np.array([dx, dy]) / norm_xy
            cos_yaw = np.cos(-yaw_t)
            sin_yaw = np.sin(-yaw_t)
            d_xy_body = np.array(
                [
                    cos_yaw * d_xy_world[0] - sin_yaw * d_xy_world[1],
                    sin_yaw * d_xy_world[0] + cos_yaw * d_xy_world[1],
                ]
            )

        tilt_mag = min(abs(z_err) * self.z_tilt_gain, self.z_tilt_max_rad)
        if tilt_mag == 0.0:
            return x_target

        sign_z = 1.0 if z_err > 0 else -1.0
        roll_offset = sign_z * tilt_mag * d_xy_body[1]
        pitch_offset = -sign_z * tilt_mag * d_xy_body[0]

        if roll_offset == 0.0 and pitch_offset == 0.0:
            return x_target

        new_quat = euler_xyz_to_quat_wxyz(
            (roll_t + roll_offset, pitch_t + pitch_offset, yaw_t)
        )
        x_new = x_target.copy()
        x_new[3:7] = new_quat
        return x_new

    def split_control(self, control: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Split control vector into RW torques and thruster commands."""
        rw = control[: self.num_rw_axes]
        thrusters = control[self.num_rw_axes : self.num_rw_axes + self.num_thrusters]
        return rw, thrusters

    def get_solver_stats(self) -> Dict[str, Any]:
        """Get solver performance statistics (Controller interface)."""
        if not self.solve_times:
            return {
                "solve_count": 0,
                "average_solve_time": 0.0,
                "max_solve_time": 0.0,
            }
        return {
            "solve_times": self.solve_times.copy(),
            "solve_count": len(self.solve_times),
            "average_solve_time": sum(self.solve_times) / len(self.solve_times),
            "max_solve_time": max(self.solve_times),
        }

    @with_error_context("MPC solve", reraise=True)
    def get_control_action(
        self,
        x_current: np.ndarray,
        x_target: np.ndarray,
        previous_thrusters: Optional[np.ndarray] = None,
        x_target_trajectory: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """Compute optimal control action."""
        start_time = time.time()

        # Ensure x_current is 13-dim
        if len(x_current) < 13:
            # Expand? Assuming caller (simulation.py) already updated.
            pass

        # Prepare targets (identity mapping now)
        x_curr_mpc = x_current  # StateConverter.sim_to_mpc(x_current) is identity
        # Planar-translation mode: if tilt is disabled, do not try to chase Z.
        # With body-frame planar thrusters, the system could create world-Z
        # acceleration by tilting. If you want effectively planar translation,
        # keep Z as a free/uncontrolled state and align target Z to current.
        if not self.enable_z_tilt and len(x_target) >= 3:
            x_target = x_target.copy()
            x_target[2] = float(x_current[2])

        x_targ_mpc = self._apply_z_tilt_target(x_current, x_target)

        if x_target_trajectory is not None and self.enable_z_tilt:
            tilted_trajectory = x_target_trajectory.copy()
            for k in range(tilted_trajectory.shape[0]):
                tilted_trajectory[k] = self._apply_z_tilt_target(
                    x_current, tilted_trajectory[k]
                )
            x_target_trajectory = tilted_trajectory

        # Dynamics update if Quat changed significantly
        quat = x_current[3:7]
        if np.linalg.norm(quat - self.prev_quat) > 0.05:
            _, B_dyn = self.linearize_dynamics(x_current)
            self._update_A_data(B_dyn)
            self.prob.update(Ax=self.A.data)
            self.prev_quat = quat.copy()

        # Update Cost Vector q
        # J = 0.5 x' P x + q' x
        # min ||x - x_ref||^2_Q = x'Qx - 2 x_ref' Q x
        # So q = -Q * x_ref

        self.q.fill(0.0)

        # Use precomputed Q_diag
        Q_diag = self.Q_diag

        # Terminal weight
        Q_term = Q_diag * 10.0

        if x_target_trajectory is not None:
            # Track trajectory
            for k in range(self.N):
                idx = k * self.nx
                ref = x_target_trajectory[min(k, len(x_target_trajectory) - 1)]
                self.q[idx : idx + self.nx] = -Q_diag * ref
            # Terminal
            idx = self.N * self.nx
            ref = x_target_trajectory[-1]
            self.q[idx : idx + self.nx] = -Q_term * ref
        else:
            # Fixed setpoint
            ref = x_targ_mpc
            for k in range(self.N):
                idx = k * self.nx
                self.q[idx : idx + self.nx] = -Q_diag * ref
            idx = self.N * self.nx
            self.q[idx : idx + self.nx] = -Q_term * ref

        # Update init constraint
        init_start = self.N * self.nx
        self.l[init_start : init_start + self.nx] = x_curr_mpc
        self.u[init_start : init_start + self.nx] = x_curr_mpc

        self.prob.update(q=self.q, l=self.l, u=self.u)

        # Solve
        try:
            res = self.prob.solve()
        except Exception as e:
            logger.error(f"OSQP solver raised exception: {e}")
            raise OptimizationError("solver_exception", f"OSQP solver failed: {e}")

        solve_time = time.time() - start_time
        self.solve_times.append(solve_time)

        # Check for timeout
        if solve_time > self.solver_time_limit:
            logger.warning(
                f"MPC solve time ({solve_time:.3f}s) exceeded limit ({self.solver_time_limit:.3f}s)"
            )
            # Don't raise - use fallback instead
            return self._get_fallback_control(x_curr_mpc, x_targ_mpc), {
                "status": -1,
                "solve_time": solve_time,
                "timeout": True,
            }

        if res.info.status not in ["solved", "solved_inaccurate"]:
            status_msg = res.info.status if hasattr(res.info, "status") else "unknown"
            logger.warning(f"MPC solver status: {status_msg}")
            raise OptimizationError(status_msg, f"Solver returned status: {status_msg}")

        u_idx = (self.N + 1) * self.nx
        u_opt = res.x[u_idx : u_idx + self.nu]
        u_opt = np.clip(u_opt, self.control_lower, self.control_upper)

        return u_opt, {"status": 1, "solve_time": solve_time}

    def _get_fallback_control(
        self, x_current: np.ndarray, x_target: np.ndarray
    ) -> np.ndarray:
        """3D PD Controller fallback."""
        u = np.zeros(self.nu)
        # Placeholder: Zero thrust
        return u
