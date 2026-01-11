"""
Reaction Wheel MPC Controller

MPC controller for satellite with reaction wheels (attitude) + thrusters (translation).

State Vector (16 elements):
    [x, y, z, qw, qx, qy, qz, vx, vy, vz, ωx, ωy, ωz, ω_rw_x, ω_rw_y, ω_rw_z]

Control Vector (9 elements):
    [τ_rw_x, τ_rw_y, τ_rw_z, F_px, F_mx, F_py, F_my, F_pz, F_mz]

Dynamics:
    - Position: ṗ = v
    - Velocity: v̇ = (1/m) * Σ F_thrusters
    - Orientation: q̇ = 0.5 * q ⊗ ω
    - Angular velocity: I * ω̇ = -ω × (I*ω) - τ_rw  (reaction torque from wheels)
    - Wheel speeds: ω̇_rw = τ_rw / I_rw
"""

import logging
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np
import osqp
import scipy.sparse as sp

from src.satellite_control.config.actuator_config import ActuatorConfig, ActuatorMode
from src.satellite_control.config.reaction_wheel_config import ReactionWheelArrayConfig
from src.satellite_control.core.error_handling import with_error_context
from src.satellite_control.core.exceptions import OptimizationError
from src.satellite_control.utils.orientation_utils import closest_symmetric_quat

from .base import Controller

logger = logging.getLogger(__name__)


class ReactionWheelMPCController(Controller):
    """
    MPC Controller for satellite with reaction wheels + thrusters.

    Optimizes:
        - Reaction wheel torques for attitude control
        - Thruster forces for translation control

    Constraints:
        - Wheel torque limits (±max_torque)
        - Wheel speed limits (saturation avoidance)
        - Thruster force limits (0 to max_force)
    """

    def __init__(
        self,
        actuator_config: Optional[ActuatorConfig] = None,
        prediction_horizon: int = 500,  # 25s horizon for single-thruster planning
        dt: float = 0.05,
        satellite_mass: float = 10.0,
        satellite_inertia: float = 0.14,
        solver_time_limit: float = 0.04,
        mpc_params: Optional[Any] = None,
    ):
        """
        Initialize reaction wheel MPC controller.

        Args:
            actuator_config: Actuator configuration (default: reaction wheels)
            prediction_horizon: Number of prediction steps N
            dt: Control timestep in seconds
            satellite_mass: Satellite mass in kg
            satellite_inertia: Satellite moment of inertia in kg·m²
            solver_time_limit: Maximum solver time in seconds
        """
        # Actuator configuration
        if actuator_config is None:
            from src.satellite_control.config.actuator_config import create_actuator_config

            actuator_config = create_actuator_config()

        if actuator_config.mode not in [ActuatorMode.REACTION_WHEELS, ActuatorMode.ONE_THRUSTER_RW]:
            raise ValueError(
                "ReactionWheelMPCController requires reaction_wheels or one_thruster_rw mode"
            )

        self.actuator_config = actuator_config
        self.rw_config = actuator_config.reaction_wheels
        self.thruster_config = actuator_config.thrusters
        self.mode = actuator_config.mode

        # Satellite parameters
        self.mass = satellite_mass
        self.inertia = satellite_inertia  # Scalar (assume spherical for now)

        # MPC parameters
        self.N = prediction_horizon
        self._dt = dt
        self.solver_time_limit = solver_time_limit

        # State and control dimensions
        self.nx = 16  # [pos(3), quat(4), vel(3), angvel(3), wheel_speed(3)]

        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            self.nu = 5  # [rw_torque(3), thruster_fwd(1), thruster_brake(1)]
        else:
            self.nu = 9  # [rw_torque(3), thruster_force(6)]

        # Cost weights (defaults, can be overridden by config)
        self.Q_pos = 1000.0  # Position tracking
        self.Q_vel = 100.0  # Velocity tracking
        # For single-thruster mode, lower orientation weight (focus on position)
        # Need some orientation tracking to rotate and face target for thrust
        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            self.Q_ang = 100.0  # Lower weight - just enough to track thrust direction
        else:
            self.Q_ang = 2000.0  # Orientation tracking for multi-thruster modes
        self.Q_angvel = 200.0  # Angular velocity tracking - for damping
        self.Q_wheel = 0.0  # No wheel speed penalty (allow momentum buildup)
        self.R_torque = 0.01  # Very low torque penalty for aggressive control
        self.R_thrust = 0.1  # Thruster usage penalty

        # Optional MPC config overrides
        self.max_velocity: Optional[float] = None
        self.max_angular_velocity: Optional[float] = None
        if mpc_params is not None:
            self.Q_pos = float(getattr(mpc_params, "q_position", self.Q_pos))
            self.Q_vel = float(getattr(mpc_params, "q_velocity", self.Q_vel))
            self.Q_ang = float(getattr(mpc_params, "q_angle", self.Q_ang))
            self.Q_angvel = float(getattr(mpc_params, "q_angular_velocity", self.Q_angvel))
            self.R_thrust = float(getattr(mpc_params, "r_thrust", self.R_thrust))
            self.max_velocity = getattr(mpc_params, "max_velocity", None)
            self.max_angular_velocity = getattr(mpc_params, "max_angular_velocity", None)

        # Build Q diagonal
        self.Q_diag = np.concatenate(
            [
                np.full(3, self.Q_pos),  # Position
                np.full(4, self.Q_ang),  # Quaternion
                np.full(3, self.Q_vel),  # Velocity
                np.full(3, self.Q_angvel),  # Angular velocity
                np.full(3, self.Q_wheel),  # Wheel speeds
            ]
        )

        # Build R diagonal
        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            self.R_diag = np.concatenate(
                [
                    np.full(3, self.R_torque),  # RW torques
                    np.full(2, self.R_thrust),  # Forward + brake thrusters
                ]
            )
        else:
            self.R_diag = np.concatenate(
                [
                    np.full(3, self.R_torque),  # RW torques
                    np.full(6, self.R_thrust),  # Thruster forces
                ]
            )

        # Actuator limits
        self.max_rw_torque = self.rw_config.wheel_x.max_torque
        self.max_rw_speed = self.rw_config.wheel_x.max_speed
        self.max_thrust = self.thruster_config.force

        # Performance tracking
        self.solve_times: list[float] = []

        # Problem dimensions
        self.n_vars = (self.N + 1) * self.nx + self.N * self.nu

        # Initialize OSQP solver
        self.prob = osqp.OSQP()
        self._init_solver_structures()

        # Previous quaternion for dynamics update
        self.prev_quat = np.array([-999.0] * 4)

        logger.info(f"ReactionWheelMPCController initialized in {self.mode.value} mode")

    @property
    def dt(self) -> float:
        """Control update interval in seconds."""
        return self._dt

    @property
    def prediction_horizon(self) -> Optional[int]:
        """Prediction horizon N."""
        return self.N

    def _get_thrust_directions(self) -> np.ndarray:
        """Get thrust direction vectors for each thruster."""
        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            # Dual thrusters: forward (+X) and braking (-X)
            # Thruster 1: rear face (-X position), pushes +X (forward)
            # Thruster 2: front face (+X position), pushes -X (braking)
            return np.array(
                [
                    [1.0, 0.0, 0.0],  # Forward thruster
                    [-1.0, 0.0, 0.0],  # Brake thruster
                ],
                dtype=np.float64,
            )

        # Standard 6-thruster setup
        directions = np.array(
            [
                [-1, 0, 0],  # px: pushes -X
                [1, 0, 0],  # mx: pushes +X
                [0, -1, 0],  # py: pushes -Y
                [0, 1, 0],  # my: pushes +Y
                [0, 0, -1],  # pz: pushes -Z
                [0, 0, 1],  # mz: pushes +Z
            ],
            dtype=np.float64,
        )
        return directions

    def linearize_dynamics(self, x_current: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Linearize dynamics around current state.

        Args:
            x_current: Current state (16 elements)

        Returns:
            (A, B) matrices for discrete-time dynamics x_{k+1} = A*x_k + B*u_k
        """
        # Extract current quaternion
        q = x_current[3:7]
        w, qx, qy, qz = q[0], q[1], q[2], q[3]

        # State transition matrix A (16x16)
        A = np.eye(16)

        # Position dynamics: ṗ = v
        A[0, 7] = self.dt  # dx/dvx
        A[1, 8] = self.dt  # dy/dvy
        A[2, 9] = self.dt  # dz/dvz

        # Quaternion dynamics: q̇ = 0.5 * G(q) * ω
        # G(q) = [-qx, -qy, -qz; qw, -qz, qy; qz, qw, -qx; -qy, qx, qw]
        G = 0.5 * np.array(
            [
                [-qx, -qy, -qz],
                [w, -qz, qy],
                [qz, w, -qx],
                [-qy, qx, w],
            ]
        )
        A[3:7, 10:13] = G * self.dt

        # Control input matrix B (16x9 or 16x4)
        B = np.zeros((16, self.nu))

        # Velocity from thrusters: v̇ = (1/m) * Σ F
        # Control indices 3-(nu-1) are thruster forces
        thrust_dirs = self._get_thrust_directions()
        num_thrusters = thrust_dirs.shape[0]

        for i in range(num_thrusters):
            # Transform local thrust direction to world frame using quaternion?
            # Wait, linear dynamics usually assume linearization around an orientation.
            # But here `thrust_dirs` are in BODY frame.
            # v (velocity) is usually in WORLD frame in this state vector?
            # Looking at state vector: [x, y, z, qw, qx, qy, qz, vx, vy, vz, ...]
            # Yes, v is world frame.
            # SO B matrix needs to depend on current orientation q.

            # Rotation matrix from body to world R(q)
            # For small angles, B is approx constant, but here we rotate freely.
            # The provided linearization code didn't rotate B!
            # It just used `thrust_dirs` directly.
            # This implies the velocity state might be body-frame OR the previous code was simplified.
            # Let's check `simulation.py`: pos=s.position (world), vel=s.velocity (world).
            # So the previous code was actually incorrect for rotated states if it didn't rotate the thrust vector!
            # However, `linearize_dynamics` is called with `x_template` where q=[1,0,0,0] (identity).
            # And then `linearize_dynamics` is re-called if quaternion changes significantly.
            # BUT: The code below doesn't apply rotation to `thrust_dirs`.

            # FIX: We must always apply rotation to B for thruster forces if v is world velocity.
            # Since we are modifying this file anyway, I should respect the existing "re-linearize on change" logic
            # but I MUST add the rotation to make it correct.

            # Rotate body force vector to world frame
            # R = ...
            pass  # (logic continues below)

            # Re-implement B fill with rotation support:

        # Simplified Rotation Matrix from Quaternion (Body -> World)
        # R = [1-2y^2-2z^2, 2xy-2wz, 2xz+2wy; ...]
        # Actually, let's just implement the rotation of the thrust vector.

        yy = qy * qy
        zz = qz * qz
        xx = qx * qx
        re_q = w

        # We need full rotation matrix R to rotate body-frame thrust vector to world frame
        # Col 0 (X-axis in world): [1-2y2-2z2, 2xy+2wz, 2xz-2wy]
        # Col 1 (Y-axis in world): [2xy-2wz, 1-2x2-2z2, 2yz+2wx]
        # Col 2 (Z-axis in world): [2xz+2wy, 2yz-2wx, 1-2x2-2y2]

        R = np.array(
            [
                [1 - 2 * (yy + zz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)],
                [2 * (qx * qy + qz * w), 1 - 2 * (xx + zz), 2 * (qy * qz - qx * w)],
                [2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (xx + yy)],
            ]
        )

        for i in range(num_thrusters):
            # Force in body frame
            f_body = thrust_dirs[i] * self.max_thrust

            # Force in world frame
            f_world = R @ f_body

            # dv = F_world / m * dt
            B[7:10, 3 + i] = f_world / self.mass * self.dt

        # Angular velocity from reaction wheel torque
        # ω̇ = -τ_rw / I_sat (reaction torque is opposite to motor torque)
        for i in range(3):
            B[10 + i, i] = -self.max_rw_torque / self.inertia * self.dt

        # Wheel speed dynamics: ω̇_rw = τ_rw / I_rw
        rw_inertia = self.rw_config.wheel_x.inertia
        for i in range(3):
            B[13 + i, i] = self.max_rw_torque / rw_inertia * self.dt

        return A, B

    def _init_solver_structures(self):
        """Initialize OSQP solver with constraint matrices."""
        # Cost matrix P (quadratic cost)
        Q_diag_full = np.concatenate([self.Q_diag] * self.N + [self.Q_diag * 10])  # Terminal weight
        R_diag_full = np.concatenate([self.R_diag] * self.N)
        P_diag = np.concatenate([Q_diag_full, R_diag_full])
        self.P = sp.diags(P_diag, format="csc")

        # Linear cost q (will be updated each solve)
        self.q = np.zeros(self.n_vars)

        # Build constraint matrices
        x_template = np.zeros(16)
        x_template[3] = 1.0  # Valid quaternion
        A_dyn, B_dyn = self.linearize_dynamics(x_template)

        # Store initial A_dyn and B_dyn for later comparison
        self._A_dyn = A_dyn.copy()
        self._B_dyn = B_dyn.copy()

        # Count constraints
        n_dynamics = self.N * self.nx
        n_init = self.nx
        n_state_bounds = (self.N + 1) * self.nx
        n_control_bounds = self.N * self.nu
        self.n_constraints = n_dynamics + n_init + n_state_bounds + n_control_bounds

        # Build constraint matrix A_con (sparse)
        # Track indices for dynamic updates
        triples = []
        row = 0

        # Store mapping: (constraint_row, matrix_type, i, j) -> index in data array
        # This allows us to update A and B values later
        self._dynamics_indices = []  # List of (data_index, 'A' or 'B', i, j) tuples

        # Dynamics constraints: -A*x_k + x_{k+1} - B*u_k = 0
        for k in range(self.N):
            x_k_start = k * self.nx
            x_kp1_start = (k + 1) * self.nx
            u_k_start = (self.N + 1) * self.nx + k * self.nu

            # -A term (always add entries even if zero, to maintain sparsity pattern)
            for i in range(self.nx):
                for j in range(self.nx):
                    val = -A_dyn[i, j]
                    if abs(val) > 1e-12 or self._is_dynamic_entry("A", i, j):
                        data_idx = len(triples)
                        triples.append((row + i, x_k_start + j, val))
                        self._dynamics_indices.append((data_idx, "A", i, j, k))

            # +I term (x_{k+1}) - constant, no need to track
            for i in range(self.nx):
                triples.append((row + i, x_kp1_start + i, 1.0))

            # -B term (always add entries even if zero for dynamic entries)
            for i in range(self.nx):
                for j in range(self.nu):
                    val = -B_dyn[i, j]
                    if abs(val) > 1e-12 or self._is_dynamic_entry("B", i, j):
                        data_idx = len(triples)
                        triples.append((row + i, u_k_start + j, val))
                        self._dynamics_indices.append((data_idx, "B", i, j, k))

            row += self.nx

        # Initial state constraint: x_0 = x_current
        for i in range(self.nx):
            triples.append((row + i, i, 1.0))
        row += self.nx

        # State bounds: x_min <= x <= x_max
        for k in range(self.N + 1):
            x_k_start = k * self.nx
            for i in range(self.nx):
                triples.append((row + i, x_k_start + i, 1.0))
            row += self.nx

        # Control bounds: u_min <= u <= u_max
        for k in range(self.N):
            u_k_start = (self.N + 1) * self.nx + k * self.nu
            for i in range(self.nu):
                triples.append((row + i, u_k_start + i, 1.0))
            row += self.nu

        # Build sparse matrix
        rows, cols, vals = zip(*triples)
        self.A_con = sp.csc_matrix((vals, (rows, cols)), shape=(self.n_constraints, self.n_vars))
        self.A_con.sort_indices()

        # Build lookup table for fast updates: map (row, col) -> index in A_con.data
        # After sorting, we need to rebuild the index mapping
        self._build_sparse_index_map()

        # Initialize bounds
        self.l = np.zeros(self.n_constraints)
        self.u = np.zeros(self.n_constraints)

        # Dynamics constraints (already 0)

        # Initial state (set in loop)

        # State bounds
        state_start = n_dynamics + n_init
        self.l[state_start : state_start + n_state_bounds] = -1e10
        self.u[state_start : state_start + n_state_bounds] = 1e10

        # Velocity and angular velocity bounds (if configured)
        if self.max_velocity is not None:
            for k in range(self.N + 1):
                vel_start = state_start + k * self.nx + 7
                self.l[vel_start : vel_start + 3] = -self.max_velocity
                self.u[vel_start : vel_start + 3] = self.max_velocity

        if self.max_angular_velocity is not None:
            for k in range(self.N + 1):
                ang_start = state_start + k * self.nx + 10
                self.l[ang_start : ang_start + 3] = -self.max_angular_velocity
                self.u[ang_start : ang_start + 3] = self.max_angular_velocity

        # Wheel speed limits
        for k in range(self.N + 1):
            wheel_start = state_start + k * self.nx + 13
            self.l[wheel_start : wheel_start + 3] = -self.max_rw_speed * 0.95
            self.u[wheel_start : wheel_start + 3] = self.max_rw_speed * 0.95

        # Control bounds
        # [0-2]: RW Torque (-1 to 1)
        # [3-End]: Thruster Force (0 to 1)
        ctrl_start = state_start + n_state_bounds

        for k in range(self.N):
            base = ctrl_start + k * self.nu
            # RW torques: [-1, 1] normalized
            self.l[base : base + 3] = -1.0
            self.u[base : base + 3] = 1.0

            # Thruster forces: [0, 1] normalized
            self.l[base + 3 : base + self.nu] = 0.0
            self.u[base + 3 : base + self.nu] = 1.0

        # Setup OSQP
        self.prob.setup(
            self.P,
            self.q,
            self.A_con,
            self.l,
            self.u,
            verbose=False,
            time_limit=self.solver_time_limit,
        )

    def _is_dynamic_entry(self, matrix_type: str, i: int, j: int) -> bool:
        """
        Check if this A or B matrix entry can change with quaternion.

        Dynamic entries are:
        - A[3:7, 10:13]: quaternion dynamics depend on current quaternion
        - B[7:10, 3:]: thrust direction in world frame depends on orientation
        """
        if matrix_type == "A":
            # Quaternion dynamics block: rows 3-6, cols 10-12
            return (3 <= i < 7) and (10 <= j < 13)
        elif matrix_type == "B":
            # Thruster force to velocity block: rows 7-9, cols 3+
            return (7 <= i < 10) and (j >= 3)
        return False

    def _build_sparse_index_map(self):
        """Build a mapping from (row, col) to index in A_con.data for fast updates."""
        self._sparse_lookup = {}
        csc = self.A_con.tocsc()
        for col in range(csc.shape[1]):
            for idx in range(csc.indptr[col], csc.indptr[col + 1]):
                row = csc.indices[idx]
                self._sparse_lookup[(row, col)] = idx

    def _update_dynamics_matrices(self, A_dyn: np.ndarray, B_dyn: np.ndarray):
        """
        Update the OSQP constraint matrix with new A and B dynamics matrices.

        Args:
            A_dyn: New state transition matrix (16x16)
            B_dyn: New control input matrix (16xnu)
        """
        # Get the data array from the sparse matrix
        new_Ax = self.A_con.data.copy()

        # Update each dynamics constraint block
        for k in range(self.N):
            x_k_start = k * self.nx
            u_k_start = (self.N + 1) * self.nx + k * self.nu
            row_offset = k * self.nx

            # Update -A entries
            for i in range(self.nx):
                for j in range(self.nx):
                    if self._is_dynamic_entry("A", i, j) or abs(A_dyn[i, j]) > 1e-12:
                        key = (row_offset + i, x_k_start + j)
                        if key in self._sparse_lookup:
                            idx = self._sparse_lookup[key]
                            new_Ax[idx] = -A_dyn[i, j]

            # Update -B entries
            for i in range(self.nx):
                for j in range(self.nu):
                    if self._is_dynamic_entry("B", i, j) or abs(B_dyn[i, j]) > 1e-12:
                        key = (row_offset + i, u_k_start + j)
                        if key in self._sparse_lookup:
                            idx = self._sparse_lookup[key]
                            new_Ax[idx] = -B_dyn[i, j]

        # Update OSQP with new constraint matrix values
        self.prob.update(Ax=new_Ax)

        # Store current matrices
        self._A_dyn = A_dyn.copy()
        self._B_dyn = B_dyn.copy()

    def _compute_thrust_aligned_target(
        self, x_current: np.ndarray, x_target: np.ndarray
    ) -> np.ndarray:
        """
        Compute target state with orientation aligned for single-thruster navigation.

        For a single rear thruster that pushes in +X body direction, the satellite
        must point its +X axis toward the target to make progress. This method
        computes the quaternion that rotates +X body to point toward target position.

        Args:
            x_current: Current state [16 elements]
            x_target: Original target state [16 elements]

        Returns:
            Modified target state with thrust-aligned quaternion
        """
        pos_current = x_current[:3]
        pos_target = x_target[:3]

        # Direction from current to target
        direction = pos_target - pos_current
        distance = np.linalg.norm(direction)

        # If very close to target, keep original target orientation (no thrust needed)
        if distance < 0.01:
            return x_target

        # Normalize direction vector
        dir_unit = direction / distance

        # We want to align either +X or -X body axis with the target direction.
        # Choose the orientation that requires the smallest rotation from current.
        def _quat_from_dir(vec: np.ndarray) -> np.ndarray:
            body_x = np.array([1.0, 0.0, 0.0])
            dot = np.dot(body_x, vec)
            cross = np.cross(body_x, vec)

            if dot < -0.999:
                return np.array([0.0, 0.0, 1.0, 0.0])
            if dot > 0.999:
                return np.array([1.0, 0.0, 0.0, 0.0])

            w = 1.0 + dot
            quat = np.array([w, cross[0], cross[1], cross[2]])
            return quat / np.linalg.norm(quat)

        quat_pos = _quat_from_dir(dir_unit)
        q_current = x_current[3:7]
        quat = closest_symmetric_quat(quat_pos, q_current)

        # Create modified target state
        x_target_modified = x_target.copy()
        x_target_modified[3:7] = quat  # Set quaternion

        # Also set target velocity in thrust direction using a braking-aware profile.
        # This keeps the approach speed below what we can stop within the remaining distance.
        a_max = self.max_thrust / self.mass
        v_stop = np.sqrt(max(0.0, 2.0 * a_max * distance))
        v_cap = self.max_velocity if self.max_velocity is not None else v_stop
        approach_speed = min(v_stop, v_cap)
        if distance > 0.1:
            x_target_modified[7:10] = dir_unit * approach_speed
        else:
            x_target_modified[7:10] = 0.0  # Zero velocity when close

        return x_target_modified

    def get_solver_stats(self) -> Dict[str, Any]:
        """Get solver performance statistics."""
        if not self.solve_times:
            return {"solve_count": 0, "average_solve_time": 0.0, "max_solve_time": 0.0}
        return {
            "solve_times": self.solve_times.copy(),
            "solve_count": len(self.solve_times),
            "average_solve_time": sum(self.solve_times) / len(self.solve_times),
            "max_solve_time": max(self.solve_times),
        }

    @with_error_context("RW MPC solve", reraise=True)
    def get_control_action(
        self,
        x_current: np.ndarray,
        x_target: np.ndarray,
        previous_thrusters: Optional[np.ndarray] = None,
        x_target_trajectory: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Compute optimal control action.

        Args:
            x_current: Current state (16 elements)
            x_target: Target state (16 elements)
            previous_thrusters: Previous control (unused for compatibility)
            x_target_trajectory: Optional trajectory to track

        Returns:
            Tuple of (control_action, info_dict)
            control_action: [τx, τy, τz, F...] normalized
        """
        start_time = time.time()

        # Ensure state dimensions
        if len(x_current) < 16:
            x_current = np.concatenate([x_current, np.zeros(16 - len(x_current))])
        if len(x_target) < 16:
            x_target = np.concatenate([x_target, np.zeros(16 - len(x_target))])

        # For single-thruster mode, compute target orientation to point +X body
        # axis toward target position (since thruster only pushes in +X body)
        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            x_target = self._compute_thrust_aligned_target(x_current, x_target)

        # Store for alignment gate (used after solve)
        self._x_current = x_current.copy()
        self._x_target = x_target.copy()

        # Precompute alignment for turn-in-place guidance in one-thruster mode
        align = None
        align_dist = None
        v_ref_vec = None
        if self.mode == ActuatorMode.ONE_THRUSTER_RW:
            dir_vec = x_target[:3] - x_current[:3]
            align_dist = np.linalg.norm(dir_vec)
            if align_dist is not None and align_dist > 1e-9:
                dir_unit = dir_vec / align_dist
                a_max = self.max_thrust / self.mass
                v_stop = np.sqrt(max(0.0, 2.0 * a_max * align_dist))
                v_cap = self.max_velocity if self.max_velocity is not None else v_stop
                v_ref = min(v_stop, v_cap)
                v_ref_vec = dir_unit * v_ref
                w, qx, qy, qz = x_current[3:7]
                body_x_world = np.array(
                    [
                        1 - 2 * (qy * qy + qz * qz),
                        2 * (qx * qy + qz * w),
                        2 * (qx * qz - qy * w),
                    ]
                )
                align = abs(float(np.dot(body_x_world, dir_unit)))
            else:
                align = 1.0

        # Update dynamics if quaternion changed significantly
        quat = x_current[3:7]
        if np.linalg.norm(quat - self.prev_quat) > 0.05:
            A_dyn, B_dyn = self.linearize_dynamics(x_current)
            # Update OSQP constraint matrix with new dynamics
            self._update_dynamics_matrices(A_dyn, B_dyn)
            self.prev_quat = quat.copy()

        # Update linear cost q = -Q * x_ref
        self.q.fill(0.0)
        ref = x_target
        Q_term = self.Q_diag * 10  # Terminal weight

        for k in range(self.N):
            idx = k * self.nx
            # If trajectory provided
            if x_target_trajectory is not None and k < len(x_target_trajectory):
                curr_ref = x_target_trajectory[k]
                # Ensure dim
                if len(curr_ref) < 16:
                    curr_ref = np.concatenate([curr_ref, np.zeros(16 - len(curr_ref))])
            else:
                curr_ref = ref.copy()

            # If no velocity is specified, apply braking-aware reference in one-thruster mode.
            if (
                self.mode == ActuatorMode.ONE_THRUSTER_RW
                and v_ref_vec is not None
                and np.linalg.norm(curr_ref[7:10]) < 1e-6
            ):
                curr_ref = curr_ref.copy()
                curr_ref[7:10] = v_ref_vec

            # Turn-in-place guidance for one-thruster mode:
            # hold position/velocity while aligning to target direction.
            if (
                self.mode == ActuatorMode.ONE_THRUSTER_RW
                and align is not None
                and align_dist is not None
                and align_dist > 0.1
                and align < 0.95
            ):
                curr_ref = curr_ref.copy()
                curr_ref[:3] = x_current[:3]
                curr_ref[7:10] = 0.0
                curr_ref[10:13] = 0.0

            self.q[idx : idx + self.nx] = -self.Q_diag * curr_ref

        # Terminal state
        idx = self.N * self.nx
        term_ref = ref.copy()
        if (
            self.mode == ActuatorMode.ONE_THRUSTER_RW
            and align is not None
            and align_dist is not None
            and align_dist > 0.1
            and align < 0.95
        ):
            term_ref[:3] = x_current[:3]
            term_ref[7:10] = 0.0
            term_ref[10:13] = 0.0
        self.q[idx : idx + self.nx] = -Q_term * term_ref

        # Update initial state constraint
        n_dynamics = self.N * self.nx
        init_start = n_dynamics
        self.l[init_start : init_start + self.nx] = x_current
        self.u[init_start : init_start + self.nx] = x_current

        # Tighten terminal velocity bounds near target to reduce overshoot
        if self.mode == ActuatorMode.ONE_THRUSTER_RW and self.max_velocity is not None:
            state_start = n_dynamics + self.nx
            terminal_start = state_start + self.N * self.nx
            if align_dist is not None and align_dist < 0.5:
                v_ref_speed = float(np.linalg.norm(v_ref_vec)) if v_ref_vec is not None else 0.0
                v_term_max = max(0.02, min(self.max_velocity, v_ref_speed))
            else:
                v_term_max = self.max_velocity
            self.l[terminal_start + 7 : terminal_start + 10] = -v_term_max
            self.u[terminal_start + 7 : terminal_start + 10] = v_term_max

        # Update solver
        self.prob.update(q=self.q, l=self.l, u=self.u)

        # Solve
        try:
            res = self.prob.solve()
        except Exception as e:
            logger.error(f"OSQP solver exception: {e}")
            raise OptimizationError("solver_exception", str(e))

        solve_time = time.time() - start_time
        self.solve_times.append(solve_time)

        # Check result
        if res.info.status not in ["solved", "solved_inaccurate"]:
            logger.warning(f"RW MPC solver status: {res.info.status}")
            return np.zeros(self.nu), {"status": -1, "solve_time": solve_time}

        # Extract first control action
        u_start = (self.N + 1) * self.nx
        u_opt = res.x[u_start : u_start + self.nu]

        # Clip to bounds
        u_opt[:3] = np.clip(u_opt[:3], -1.0, 1.0)  # RW torques
        u_opt[3:] = np.clip(u_opt[3:], 0.0, 1.0)  # Thruster forces

        # In one-thruster mode, suppress thrust until the body +X axis is aligned
        # with the target direction to avoid curved "sideways" accelerations.
        if self.mode == ActuatorMode.ONE_THRUSTER_RW and u_opt.size > 3:
            pos = self._x_current[:3]
            target_pos = self._x_target[:3]
            dir_vec = target_pos - pos
            dist = np.linalg.norm(dir_vec)
            if dist > 0.1:
                dir_unit = dir_vec / dist
                w, qx, qy, qz = self._x_current[3:7]
                body_x_world = np.array(
                    [
                        1 - 2 * (qy * qy + qz * qz),
                        2 * (qx * qy + qz * w),
                        2 * (qx * qz - qy * w),
                    ]
                )
                align = abs(float(np.dot(body_x_world, dir_unit)))
                if align < 0.95:
                    u_opt[3:] = 0.0

        return u_opt, {"status": 1, "solve_time": solve_time}

    def denormalize_control(self, u_normalized: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Convert normalized control to physical units.

        Args:
            u_normalized: Normalized control [nu elements]

        Returns:
            Dict with 'rw_torques' [N·m] and 'thruster_forces' [N]
        """
        return {
            "rw_torques": u_normalized[:3] * self.max_rw_torque,
            "thruster_forces": u_normalized[3:] * self.max_thrust,
        }
