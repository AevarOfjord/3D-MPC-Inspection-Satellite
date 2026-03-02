"""
CasADi Symbolic MPCC Cost Functions.

Defines every cost component used by the SQP-MPC as CasADi symbolic
expressions. CasADi automatic differentiation produces exact Hessians
and gradients for the QP sub-problem — no hand derivation needed.

Cost components:
    - Contouring error (cross-track distance to path)
    - Lag error (along-track deviation)
    - Progress tracking (virtual speed regulation)
    - Velocity alignment (velocity along path tangent)
    - Attitude tracking (quaternion error w.r.t. reference)
    - Angular velocity damping
    - Control effort (RW torque + thrusters)
    - Smoothness (Δu penalty)
    - Opposing-thruster co-fire penalty
    - Terminal cost (position pull + progress pull + DARE)
"""

from __future__ import annotations

import casadi as ca
import numpy as np

# ---------------------------------------------------------------------------
# Quaternion error (scalar-first convention)
# ---------------------------------------------------------------------------


def quat_error_vec(q: ca.SX, q_ref: ca.SX) -> ca.SX:
    """
    Compute the 3-vector part of the quaternion error δq = q_ref⁻¹ ⊗ q.

    For small errors this approximates the rotation error half-angle vector.
    The sign of the scalar part is enforced positive to avoid the double-cover
    ambiguity (always take the shorter rotation).

    Args:
        q:     Current quaternion [qw, qx, qy, qz] (4,)
        q_ref: Reference quaternion [qw, qx, qy, qz] (4,)

    Returns:
        3-vector error (SX of shape (3,1))
    """
    # δq = q_ref* ⊗ q  (quaternion conjugate = inverse for unit quaternions)
    qr_conj = ca.vertcat(q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3])

    dq_w = qr_conj[0] * q[0] - qr_conj[1] * q[1] - qr_conj[2] * q[2] - qr_conj[3] * q[3]
    dq_x = qr_conj[0] * q[1] + qr_conj[1] * q[0] + qr_conj[2] * q[3] - qr_conj[3] * q[2]
    dq_y = qr_conj[0] * q[2] - qr_conj[1] * q[3] + qr_conj[2] * q[0] + qr_conj[3] * q[1]
    dq_z = qr_conj[0] * q[3] + qr_conj[1] * q[2] - qr_conj[2] * q[1] + qr_conj[3] * q[0]

    dq_vec = ca.vertcat(dq_x, dq_y, dq_z)

    # Ensure shortest path (sign of scalar part)
    sign = ca.sign(dq_w + 1e-12)  # small eps to avoid sign(0)
    return sign * dq_vec


# ---------------------------------------------------------------------------
# Individual cost terms
# ---------------------------------------------------------------------------


def contouring_cost(
    pos: ca.SX,
    p_ref: ca.SX,
    t_ref: ca.SX,
    Q_contour: ca.SX | float,
) -> ca.SX:
    """
    Contouring error: squared cross-track distance to the path.

    e_c = (p - p_ref) - [(p - p_ref)·t_ref] t_ref
    cost = Q_contour * ||e_c||²
    """
    dp = pos - p_ref
    along = ca.dot(dp, t_ref) * t_ref
    e_c = dp - along
    return Q_contour * ca.dot(e_c, e_c)


def lag_cost(
    pos: ca.SX,
    p_ref: ca.SX,
    t_ref: ca.SX,
    Q_lag: ca.SX | float,
) -> ca.SX:
    """
    Lag error: squared along-track deviation.

    e_l = (p - p_ref)·t_ref
    cost = Q_lag * e_l²
    """
    dp = pos - p_ref
    e_l = ca.dot(dp, t_ref)
    return Q_lag * e_l * e_l


def progress_cost(
    v_s: ca.SX,
    v_ref: ca.SX | float,
    Q_progress: ca.SX | float,
) -> ca.SX:
    """
    Progress tracking: penalise deviation of virtual speed from reference.

    cost = Q_progress * (v_s - v_ref)²
    """
    e_v = v_s - v_ref
    return Q_progress * e_v * e_v


def progress_reward_cost(
    v_s: ca.SX,
    reward: ca.SX | float,
) -> ca.SX:
    """
    Linear reward for forward progress (encourages faster traversal).

    cost = -2 * reward * v_s
    """
    return -2.0 * reward * v_s


def velocity_alignment_cost(
    vel: ca.SX,
    t_ref: ca.SX,
    v_ref_speed: ca.SX | float,
    Q_vel_align: ca.SX | float,
) -> ca.SX:
    """
    Velocity alignment: penalise deviation of velocity from tangent * speed.

    cost = Q_vel_align * ||v - v_ref * t_ref||²
    """
    v_desired = v_ref_speed * t_ref
    e_v = vel - v_desired
    return Q_vel_align * ca.dot(e_v, e_v)


def s_anchor_cost(
    s: ca.SX,
    s_ref: ca.SX | float,
    Q_s_anchor: ca.SX | float,
) -> ca.SX:
    """
    Progress anchor: penalise drift of virtual arc-length from current estimate.

    cost = Q_s_anchor * (s - s_ref)²
    """
    e_s = s - s_ref
    return Q_s_anchor * e_s * e_s


def attitude_cost(
    q: ca.SX,
    q_ref: ca.SX,
    Q_attitude: ca.SX | float,
) -> ca.SX:
    """
    Attitude tracking: squared quaternion error vector norm.

    cost = Q_attitude * ||δq_vec||²
    """
    dq = quat_error_vec(q, q_ref)
    return Q_attitude * ca.dot(dq, dq)


def quat_norm_cost(
    q: ca.SX,
    q_current: ca.SX,
    Q_quat_norm: ca.SX | float,
) -> ca.SX:
    """
    Soft quaternion normalisation: penalise deviation from measured quaternion.

    Helps the QP keep quaternions near the unit sphere.
    cost = Q_quat_norm * ||q - q_current||²
    """
    dq = q - q_current
    return Q_quat_norm * ca.dot(dq, dq)


def angular_velocity_cost(
    omega: ca.SX,
    Q_angvel: ca.SX | float,
) -> ca.SX:
    """
    Angular velocity damping: penalise angular rate magnitude.

    cost = Q_angvel * ||ω||²
    """
    return Q_angvel * ca.dot(omega, omega)


def control_effort_cost(
    tau_rw: ca.SX,
    u_thr: ca.SX,
    R_rw: ca.SX | float,
    R_thr: ca.SX | float,
) -> ca.SX:
    """
    Control effort: weighted norm of RW torques and thruster commands.

    cost = R_rw * ||τ_rw||² + R_thr * ||u_thr||²
    """
    return R_rw * ca.dot(tau_rw, tau_rw) + R_thr * ca.dot(u_thr, u_thr)


def smoothness_cost(
    u_k: ca.SX,
    u_prev: ca.SX,
    Q_smooth: ca.SX | float,
) -> ca.SX:
    """
    Smoothness (Δu penalty): penalise control increments.

    cost = Q_smooth * ||u_k - u_{k-1}||²
    """
    du = u_k - u_prev
    return Q_smooth * ca.dot(du, du)


def opposing_thruster_cost(
    u_thr: ca.SX,
    pairs: list[tuple[int, int]],
    weight: ca.SX | float,
) -> ca.SX:
    """
    Opposing-thruster co-fire penalty.

    For each pair (i, j): cost += weight * (u_i + u_j)²
    """
    cost = ca.SX(0)
    for i, j in pairs:
        s = u_thr[i] + u_thr[j]
        cost += weight * s * s
    return cost


def fuel_bias_cost(
    u_thr: ca.SX,
    weight: ca.SX | float,
) -> ca.SX:
    """
    L1 fuel bias: linear penalty on thruster usage (promotes coasting).

    Approximated as: cost = weight * Σ u_thr_i  (since u_thr ≥ 0)
    """
    return weight * ca.sum1(u_thr)


# ---------------------------------------------------------------------------
# Stage cost builder (combines all terms)
# ---------------------------------------------------------------------------


class MPCCStageCost:
    """
    Builds the complete MPCC stage cost as a CasADi function.

    The stage cost is a scalar function of:
        x_k, u_k, u_{k-1}, p_ref_k, t_ref_k, q_ref_k, s_ref_k, weights

    where weights is a packed vector of all cost weights.
    """

    # Default opposing-thruster pairs for a 6-thruster configuration
    # (pairs along each body axis: +X/-X, +Y/-Y, +Z/-Z)
    DEFAULT_6THR_PAIRS: list[tuple[int, int]] = [(0, 1), (2, 3), (4, 5)]

    def __init__(
        self,
        num_thrusters: int = 6,
        num_rw: int = 3,
        thruster_pairs: list[tuple[int, int]] | None = None,
    ):
        self.num_thrusters = num_thrusters
        self.num_rw = num_rw
        self.nu = num_rw + num_thrusters + 1
        self.nx = 17

        if thruster_pairs is None:
            self.thruster_pairs = self.DEFAULT_6THR_PAIRS
        else:
            self.thruster_pairs = thruster_pairs

        self._build()

    # ----- Weight vector layout ------------------------------------------
    # Packed as:
    #   Q_contour, Q_lag, Q_progress, progress_reward,
    #   Q_vel_align, Q_s_anchor, Q_attitude, Q_quat_norm,
    #   Q_angvel, R_rw, R_thr, Q_smooth, thrust_pair_weight,
    #   thrust_l1_weight, v_ref_speed
    # = 15 scalars

    N_WEIGHTS = 15

    @staticmethod
    def pack_weights(
        Q_contour: float = 2400.0,
        Q_lag: float = 4000.0,
        Q_progress: float = 70.0,
        progress_reward: float = 0.0,
        Q_vel_align: float = 160.0,
        Q_s_anchor: float = 500.0,
        Q_attitude: float = 3500.0,
        Q_quat_norm: float = 20.0,
        Q_angvel: float = 1200.0,
        R_rw: float = 0.003,
        R_thr: float = 0.02,
        Q_smooth: float = 20.0,
        thrust_pair_weight: float = 0.8,
        thrust_l1_weight: float = 0.0,
        v_ref_speed: float = 0.1,
    ) -> np.ndarray:
        """Pack weights into the flat vector expected by the stage cost."""
        return np.array(
            [
                Q_contour,
                Q_lag,
                Q_progress,
                progress_reward,
                Q_vel_align,
                Q_s_anchor,
                Q_attitude,
                Q_quat_norm,
                Q_angvel,
                R_rw,
                R_thr,
                Q_smooth,
                thrust_pair_weight,
                thrust_l1_weight,
                v_ref_speed,
            ],
            dtype=np.float64,
        )

    def _build(self):
        """Build the symbolic stage cost function."""
        # Symbolic inputs
        x = ca.SX.sym("x", self.nx)
        u = ca.SX.sym("u", self.nu)
        u_prev = ca.SX.sym("u_prev", self.nu)
        p_ref = ca.SX.sym("p_ref", 3)  # path reference position
        t_ref = ca.SX.sym("t_ref", 3)  # path tangent
        q_ref = ca.SX.sym("q_ref", 4)  # reference quaternion
        s_ref = ca.SX.sym("s_ref")  # anchor arc-length
        q_current = ca.SX.sym("q_current", 4)  # measured quat (for norm penalty)
        w = ca.SX.sym("w", self.N_WEIGHTS)  # weight vector

        # Unpack state
        pos = x[0:3]
        quat = x[3:7]
        vel = x[7:10]
        omega = x[10:13]
        s = x[16]

        # Unpack control
        tau_rw = u[0 : self.num_rw]
        u_thr = u[self.num_rw : self.num_rw + self.num_thrusters]
        v_s = u[self.num_rw + self.num_thrusters]

        # Unpack weights
        W_contour = w[0]
        W_lag = w[1]
        W_progress = w[2]
        W_progress_reward = w[3]
        W_vel_align = w[4]
        W_s_anchor = w[5]
        W_attitude = w[6]
        W_quat_norm = w[7]
        W_angvel = w[8]
        W_R_rw = w[9]
        W_R_thr = w[10]
        W_smooth = w[11]
        W_pair = w[12]
        W_l1 = w[13]
        W_v_ref = w[14]  # reference path speed

        # Build cost
        cost = ca.SX(0)
        cost += contouring_cost(pos, p_ref, t_ref, W_contour)
        cost += lag_cost(pos, p_ref, t_ref, W_lag)
        cost += progress_cost(v_s, W_v_ref, W_progress)
        cost += progress_reward_cost(v_s, W_progress_reward)
        cost += velocity_alignment_cost(vel, t_ref, W_v_ref, W_vel_align)
        cost += s_anchor_cost(s, s_ref, W_s_anchor)
        cost += attitude_cost(quat, q_ref, W_attitude)
        cost += quat_norm_cost(quat, q_current, W_quat_norm)
        cost += angular_velocity_cost(omega, W_angvel)
        cost += control_effort_cost(tau_rw, u_thr, W_R_rw, W_R_thr)
        cost += smoothness_cost(u, u_prev, W_smooth)
        cost += opposing_thruster_cost(u_thr, self.thruster_pairs, W_pair)
        cost += fuel_bias_cost(u_thr, W_l1)

        # Build CasADi function
        inputs = [x, u, u_prev, p_ref, t_ref, q_ref, s_ref, q_current, w]
        input_names = [
            "x",
            "u",
            "u_prev",
            "p_ref",
            "t_ref",
            "q_ref",
            "s_ref",
            "q_current",
            "w",
        ]
        self.stage_cost = ca.Function(
            "stage_cost",
            inputs,
            [cost],
            input_names,
            ["cost"],
        )

        # Gradient and Hessian w.r.t. (x, u) for QP construction
        z = ca.vertcat(x, u)
        grad = ca.gradient(cost, z)
        hess, _ = ca.hessian(cost, z)

        self.stage_cost_grad = ca.Function(
            "stage_cost_grad",
            inputs,
            [grad],
            input_names,
            ["grad"],
        )

        self.stage_cost_hess = ca.Function(
            "stage_cost_hess",
            inputs,
            [hess],
            input_names,
            ["hess"],
        )

        # Store for external use
        self._sym_x = x
        self._sym_u = u
        self._sym_u_prev = u_prev
        self._sym_p_ref = p_ref
        self._sym_t_ref = t_ref
        self._sym_q_ref = q_ref
        self._sym_s_ref = s_ref
        self._sym_q_current = q_current
        self._sym_w = w


# ---------------------------------------------------------------------------
# Terminal cost builder
# ---------------------------------------------------------------------------


class MPCCTerminalCost:
    """
    Builds the terminal cost function.

    Terminal cost = position pull to endpoint + progress pull + DARE-based
    physical-state terminal penalty.
    """

    def __init__(self, num_thrusters: int = 6, num_rw: int = 3):
        self.nx = 17
        self.num_thrusters = num_thrusters
        self.num_rw = num_rw
        self._build()

    def _build(self):
        """Build terminal cost function."""
        x = ca.SX.sym("x", self.nx)
        p_end = ca.SX.sym("p_end", 3)  # path endpoint
        s_target = ca.SX.sym("s_target")  # target arc-length (path length)
        q_ref = ca.SX.sym("q_ref", 4)  # terminal reference quaternion

        # Terminal weights: [Q_pos, Q_s, Q_att, Q_angvel, Q_vel]
        w_term = ca.SX.sym("w_term", 5)

        # Optional DARE diagonal (physical states: 16)
        P_dare_diag = ca.SX.sym("P_dare_diag", 16)
        # Reference for DARE (physical states: 16)
        x_ref_phys = ca.SX.sym("x_ref_phys", 16)

        # Unpack state
        pos = x[0:3]
        quat = x[3:7]
        vel = x[7:10]
        omega = x[10:13]
        s = x[16]

        # Terminal weights
        Q_pos = w_term[0]
        Q_s = w_term[1]
        Q_att = w_term[2]
        Q_angvel = w_term[3]
        Q_vel = w_term[4]

        cost = ca.SX(0)

        # Position pull to endpoint
        dp = pos - p_end
        cost += Q_pos * ca.dot(dp, dp)

        # Progress pull to path length
        ds = s - s_target
        cost += Q_s * ds * ds

        # Terminal attitude
        dq = quat_error_vec(quat, q_ref)
        cost += Q_att * ca.dot(dq, dq)

        # Terminal angular velocity damping
        cost += Q_angvel * ca.dot(omega, omega)

        # Terminal velocity damping
        cost += Q_vel * ca.dot(vel, vel)

        # DARE-based terminal cost (diagonal approximation)
        x_phys = x[0:16]
        dx_phys = x_phys - x_ref_phys
        cost += ca.dot(P_dare_diag * dx_phys, dx_phys)

        self.terminal_cost = ca.Function(
            "terminal_cost",
            [x, p_end, s_target, q_ref, w_term, P_dare_diag, x_ref_phys],
            [cost],
            ["x", "p_end", "s_target", "q_ref", "w_term", "P_dare_diag", "x_ref_phys"],
            ["cost"],
        )

        # Gradient and Hessian w.r.t. x for QP
        grad_x = ca.gradient(cost, x)
        hess_x, _ = ca.hessian(cost, x)

        self.terminal_cost_grad = ca.Function(
            "terminal_cost_grad",
            [x, p_end, s_target, q_ref, w_term, P_dare_diag, x_ref_phys],
            [grad_x],
            ["x", "p_end", "s_target", "q_ref", "w_term", "P_dare_diag", "x_ref_phys"],
            ["grad_x"],
        )

        self.terminal_cost_hess = ca.Function(
            "terminal_cost_hess",
            [x, p_end, s_target, q_ref, w_term, P_dare_diag, x_ref_phys],
            [hess_x],
            ["x", "p_end", "s_target", "q_ref", "w_term", "P_dare_diag", "x_ref_phys"],
            ["hess_x"],
        )
