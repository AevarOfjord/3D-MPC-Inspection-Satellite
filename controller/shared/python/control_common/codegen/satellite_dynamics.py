"""
CasADi Symbolic Satellite Dynamics Model.

Full nonlinear 6DOF dynamics (translational + rotational) for a satellite
with reaction wheels and thrusters.  CasADi automatic differentiation
provides exact Jacobians — no hand-derived linearization required.

State vector (nx=17, MPCC-augmented):
    x = [p(3), q(4), v(3), ω(3), ω_rw(3), s(1)]
        p     : position in inertial frame [m]
        q     : quaternion (scalar-first) [qw, qx, qy, qz]
        v     : velocity in inertial frame [m/s]
        ω     : body angular velocity [rad/s]
        ω_rw  : reaction wheel speeds [rad/s]
        s     : virtual path arc-length parameter [m]

Control vector (nu=10 for default 3RW + 6THR):
    u = [τ_rw(3), u_thr(6), v_s(1)]
        τ_rw  : normalized RW torque commands [-1, 1]
        u_thr : thruster duty cycles [0, 1]
        v_s   : virtual path speed [m/s]
"""

from __future__ import annotations

import casadi as ca
import numpy as np

# ---------------------------------------------------------------------------
# Quaternion helpers (scalar-first convention: q = [qw, qx, qy, qz])
# ---------------------------------------------------------------------------


def quat_mult(p: ca.SX, q: ca.SX) -> ca.SX:
    """Hamilton quaternion product p ⊗ q (scalar-first)."""
    pw, px, py, pz = p[0], p[1], p[2], p[3]
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        pw * qw - px * qx - py * qy - pz * qz,
        pw * qx + px * qw + py * qz - pz * qy,
        pw * qy - px * qz + py * qw + pz * qx,
        pw * qz + px * qy - py * qx + pz * qw,
    )


def quat_conj(q: ca.SX) -> ca.SX:
    """Quaternion conjugate (inverse for unit quaternions)."""
    return ca.vertcat(q[0], -q[1], -q[2], -q[3])


def quat_rotate(q: ca.SX, v: ca.SX) -> ca.SX:
    """Rotate vector v by quaternion q: v' = q ⊗ [0,v] ⊗ q*."""
    v_quat = ca.vertcat(0, v)
    return quat_mult(quat_mult(q, v_quat), quat_conj(q))[1:]


def quat_Xi(q: ca.SX) -> ca.SX:
    """
    Ξ(q) matrix for quaternion kinematics: q̇ = ½ Ξ(q) ω.

    Returns 4×3 matrix such that q̇ = 0.5 * Xi(q) @ omega.
    Scalar-first convention.
    """
    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(-qx, -qy, -qz),
        ca.horzcat(qw, -qz, qy),
        ca.horzcat(qz, qw, -qx),
        ca.horzcat(-qy, qx, qw),
    )


def skew(v: ca.SX) -> ca.SX:
    """Skew-symmetric matrix [v]× for cross product."""
    return ca.vertcat(
        ca.horzcat(0, -v[2], v[1]),
        ca.horzcat(v[2], 0, -v[0]),
        ca.horzcat(-v[1], v[0], 0),
    )


# ---------------------------------------------------------------------------
# Symbolic dynamics builder
# ---------------------------------------------------------------------------


class SatelliteDynamicsSymbolic:
    """
    Builds the full nonlinear continuous-time and RK4-discretised dynamics
    as CasADi symbolic functions, along with their exact Jacobians.

    Satellite parameters (mass, inertia, thruster geometry, …) are injected
    as CasADi *parameters* so one compiled function handles any configuration.
    """

    def __init__(
        self,
        num_thrusters: int = 6,
        num_rw: int = 3,
    ):
        self.num_thrusters = num_thrusters
        self.num_rw = num_rw
        self.nx = 17  # 3 pos + 4 quat + 3 vel + 3 ω + 3 ω_rw + 1 s
        self.nu = num_rw + num_thrusters + 1  # RW torques + thrusters + v_s

        # ----- Symbolic state & control -----------------------------------
        self.x = ca.SX.sym("x", self.nx)
        self.u = ca.SX.sym("u", self.nu)
        self.dt_sym = ca.SX.sym("dt")

        # ----- Symbolic satellite parameters (packed into one vector) ------
        # Layout:
        #   mass(1) + inertia_diag(3) +
        #   for each thruster: pos(3) + dir(3) + force(1) = 7 per thruster
        #   for each RW: axis(3) + torque_limit(1) + inertia(1) = 5 per RW
        #   orbital_mu(1) + orbital_radius(1)
        self.np_ = 1 + 3 + num_thrusters * 7 + num_rw * 5 + 2
        self.p = ca.SX.sym("p", self.np_)

        # Build the continuous dynamics ẋ = f(x, u, p)
        self._xdot = self._build_continuous_dynamics()

        # Build CasADi functions
        self._build_functions()

    # ----- Parameter unpacking -------------------------------------------

    def _unpack_params(self) -> dict:
        """Unpack the flat parameter vector into named components."""
        p = self.p
        idx = 0

        mass = p[idx]
        idx += 1
        inertia_diag = p[idx : idx + 3]
        idx += 3

        thruster_positions = []
        thruster_directions = []
        thruster_forces = []
        for _ in range(self.num_thrusters):
            thruster_positions.append(p[idx : idx + 3])
            idx += 3
            thruster_directions.append(p[idx : idx + 3])
            idx += 3
            thruster_forces.append(p[idx])
            idx += 1

        rw_axes = []
        rw_torque_limits = []
        rw_inertias = []
        for _ in range(self.num_rw):
            rw_axes.append(p[idx : idx + 3])
            idx += 3
            rw_torque_limits.append(p[idx])
            idx += 1
            rw_inertias.append(p[idx])
            idx += 1

        orbital_mu = p[idx]
        idx += 1
        orbital_radius = p[idx]
        idx += 1

        return {
            "mass": mass,
            "inertia_diag": inertia_diag,
            "thruster_positions": thruster_positions,
            "thruster_directions": thruster_directions,
            "thruster_forces": thruster_forces,
            "rw_axes": rw_axes,
            "rw_torque_limits": rw_torque_limits,
            "rw_inertias": rw_inertias,
            "orbital_mu": orbital_mu,
            "orbital_radius": orbital_radius,
        }

    # ----- State unpacking -----------------------------------------------

    def _unpack_state(self):
        """Unpack state vector into named components."""
        x = self.x
        pos = x[0:3]
        quat = x[3:7]  # [qw, qx, qy, qz]
        vel = x[7:10]
        omega = x[10:13]
        omega_rw = x[13:16]
        s = x[16]
        return pos, quat, vel, omega, omega_rw, s

    def _unpack_control(self):
        """Unpack control vector into named components."""
        u = self.u
        tau_rw = u[0 : self.num_rw]  # normalized RW torques [-1, 1]
        u_thr = u[self.num_rw : self.num_rw + self.num_thrusters]  # [0, 1]
        v_s = u[self.num_rw + self.num_thrusters]  # virtual path speed
        return tau_rw, u_thr, v_s

    # ----- Continuous dynamics -------------------------------------------

    def _build_continuous_dynamics(self) -> ca.SX:
        """
        Build ẋ = f(x, u, p) — continuous-time nonlinear 6DOF dynamics.

        Returns:
            xdot: SX expression for the time derivative of the state vector.
        """
        pos, quat, vel, omega, omega_rw, s = self._unpack_state()
        tau_rw_cmd, u_thr, v_s = self._unpack_control()
        params = self._unpack_params()

        mass = params["mass"]
        I_diag = params["inertia_diag"]
        I_body = ca.diag(I_diag)
        I_body_inv = ca.diag(1.0 / I_diag)
        mu = params["orbital_mu"]
        R_orbit = params["orbital_radius"]

        # ----- Translational dynamics -----

        # Clohessy-Wiltshire (Hill's) equations — linearised gravity
        # in LVLH frame matching the simulation physics engine.
        # Mean motion: n = √(μ / R³)
        n = ca.sqrt(mu / (R_orbit**3))
        n_sq = n * n

        # CW gravity-gradient accelerations:
        #   ax =  3n²x + 2n·vy   (radial)
        #   ay = -2n·vx           (along-track)
        #   az = -n²z             (cross-track)
        a_grav = ca.vertcat(
            3.0 * n_sq * pos[0] + 2.0 * n * vel[1],
            -2.0 * n * vel[0],
            -n_sq * pos[2],
        )

        # Thruster forces in body frame → world frame
        F_body = ca.SX.zeros(3)
        tau_body_thr = ca.SX.zeros(3)
        for i in range(self.num_thrusters):
            t_pos = params["thruster_positions"][i]
            t_dir = params["thruster_directions"][i]
            t_force_max = params["thruster_forces"][i]

            # Force in body frame
            F_i = t_dir * t_force_max * u_thr[i]
            F_body += F_i

            # Torque from thruster (lever arm × force)
            tau_body_thr += ca.cross(t_pos, F_i)

        # Rotate body force to world frame
        F_world = quat_rotate(quat, F_body)
        a_thrust = F_world / mass

        # Total translational acceleration
        a_total = a_grav + a_thrust

        # ----- Rotational dynamics (Euler's equation) -----

        # Reaction wheel torques (physical, in body frame)
        # Convention: positive cmd → positive torque on wheel → NEGATIVE
        # reaction torque on body (Newton's 3rd law).  This matches the
        # simulation engine where body_torque -= rw_torque.
        tau_rw_body = ca.SX.zeros(3)
        for i in range(self.num_rw):
            axis = params["rw_axes"][i]
            tau_max = params["rw_torque_limits"][i]
            # Torque command is normalized [-1, 1], scale to physical
            tau_rw_body -= axis * tau_max * tau_rw_cmd[i]

        # Total RW angular momentum (about body axes)
        h_rw = ca.SX.zeros(3)
        for i in range(self.num_rw):
            axis = params["rw_axes"][i]
            I_rw_i = params["rw_inertias"][i]
            h_rw += axis * I_rw_i * omega_rw[i]

        # Euler's equation:
        # I ω̇ = -ω × (Iω + h_rw) + τ_thr + τ_rw
        # (τ_rw acts on body, reaction on wheels)
        I_omega = I_body @ omega
        gyro_torque = -ca.cross(omega, I_omega + h_rw)
        tau_total = gyro_torque + tau_body_thr + tau_rw_body

        omega_dot = I_body_inv @ tau_total

        # ----- Quaternion kinematics (full nonlinear) -----
        # q̇ = ½ Ξ(q) ω
        quat_dot = 0.5 * quat_Xi(quat) @ omega

        # ----- Reaction wheel dynamics -----
        # ω̇_rw_i = +τ_rw_i / I_rw_i  (positive cmd accelerates wheel)
        # Matches simulation: dxdt(13+i) = rw_torque(i) / inertia
        omega_rw_dot = ca.SX.zeros(self.num_rw)
        for i in range(self.num_rw):
            tau_max = params["rw_torque_limits"][i]
            I_rw_i = params["rw_inertias"][i]
            omega_rw_dot[i] = tau_max * tau_rw_cmd[i] / I_rw_i

        # ----- Path augmentation -----
        s_dot = v_s

        # ----- Assemble ẋ -----
        xdot = ca.vertcat(
            vel,  # ṗ = v
            quat_dot,  # q̇ = ½ Ξ(q) ω
            a_total,  # v̇ = a_grav + a_thrust
            omega_dot,  # ω̇ = I⁻¹ τ_total
            omega_rw_dot,  # ω̇_rw
            s_dot,  # ṡ = v_s
        )
        return xdot

    # ----- RK4 discretization -------------------------------------------

    def _rk4_step(self, x: ca.SX, u: ca.SX, p: ca.SX, dt: ca.SX) -> ca.SX:
        """Single RK4 integration step with quaternion re-normalisation."""
        # f(x, u, p)
        f = ca.Function("f_cont", [self.x, self.u, self.p], [self._xdot])

        k1 = f(x, u, p)
        k2 = f(x + dt / 2 * k1, u, p)
        k3 = f(x + dt / 2 * k2, u, p)
        k4 = f(x + dt * k3, u, p)

        x_next = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        # Re-normalise quaternion
        q_next = x_next[3:7]
        # Use smooth norm regularization for stable derivatives in NLP mode.
        q_norm = ca.sqrt(ca.dot(q_next, q_next) + 1e-12)
        q_normalised = q_next / q_norm
        x_next = ca.vertcat(
            x_next[0:3],  # pos
            q_normalised,  # quat (normalised)
            x_next[7:],  # vel, omega, omega_rw, s
        )
        return x_next

    # ----- Build CasADi functions ----------------------------------------

    def _build_functions(self):
        """Build all CasADi function objects."""

        # 1) Continuous dynamics: f_continuous(x, u, p) -> xdot
        self.f_continuous = ca.Function(
            "f_continuous",
            [self.x, self.u, self.p],
            [self._xdot],
            ["x", "u", "p"],
            ["xdot"],
        )

        # 2) Discrete dynamics: f_discrete(x, u, p, dt) -> x_next (RK4)
        x_next = self._rk4_step(self.x, self.u, self.p, self.dt_sym)
        self.f_discrete = ca.Function(
            "f_discrete",
            [self.x, self.u, self.p, self.dt_sym],
            [x_next],
            ["x", "u", "p", "dt"],
            ["x_next"],
        )

        # 3) Jacobians via CasADi AD (exact)
        A = ca.jacobian(x_next, self.x)  # ∂f/∂x  (nx × nx)
        B = ca.jacobian(x_next, self.u)  # ∂f/∂u  (nx × nu)

        self.jac_f_x = ca.Function(
            "jac_f_x",
            [self.x, self.u, self.p, self.dt_sym],
            [A],
            ["x", "u", "p", "dt"],
            ["A"],
        )

        self.jac_f_u = ca.Function(
            "jac_f_u",
            [self.x, self.u, self.p, self.dt_sym],
            [B],
            ["x", "u", "p", "dt"],
            ["B"],
        )

        # 4) Fused: f_and_jacs(x, u, p, dt) -> (x_next, A, B)
        self.f_and_jacs = ca.Function(
            "f_and_jacs",
            [self.x, self.u, self.p, self.dt_sym],
            [x_next, A, B],
            ["x", "u", "p", "dt"],
            ["x_next", "A", "B"],
        )

        # 5) Linearised affine term: d = f(x̄, ū) - A x̄ - B ū
        d = x_next - A @ self.x - B @ self.u
        self.affine_term = ca.Function(
            "affine_term",
            [self.x, self.u, self.p, self.dt_sym],
            [d],
            ["x", "u", "p", "dt"],
            ["d"],
        )

    # ----- Parameter packing helper (for Python callers) -----------------

    @staticmethod
    def pack_params(
        mass: float,
        inertia_diag: np.ndarray,
        thruster_positions: list[np.ndarray],
        thruster_directions: list[np.ndarray],
        thruster_forces: list[float],
        rw_axes: list[np.ndarray],
        rw_torque_limits: list[float],
        rw_inertias: list[float],
        orbital_mu: float = 3.986004418e14,
        orbital_radius: float = 6.778e6,
    ) -> np.ndarray:
        """
        Pack satellite parameters into the flat vector expected by CasADi functions.

        Returns:
            1D numpy array of length np_.
        """
        parts = [
            np.array([mass]),
            np.asarray(inertia_diag).ravel()[:3],
        ]
        for pos, dirn, force in zip(
            thruster_positions, thruster_directions, thruster_forces
        ):
            parts.append(np.asarray(pos).ravel()[:3])
            parts.append(np.asarray(dirn).ravel()[:3])
            parts.append(np.array([force]))
        for axis, tau_max, I_rw in zip(rw_axes, rw_torque_limits, rw_inertias):
            parts.append(np.asarray(axis).ravel()[:3])
            parts.append(np.array([tau_max]))
            parts.append(np.array([I_rw]))
        parts.append(np.array([orbital_mu, orbital_radius]))
        return np.concatenate(parts)

    # ----- Code generation -----------------------------------------------

    def generate_c_code(self, output_dir: str = "codegen_cache") -> list[str]:
        """
        Generate C source files for all dynamics functions.

        Args:
            output_dir: Directory to write generated .c files.

        Returns:
            List of generated file paths.
        """
        import os

        os.makedirs(output_dir, exist_ok=True)

        generated_files = []
        opts = {"with_header": True}

        for name, func in [
            ("f_discrete", self.f_discrete),
            ("jac_f_x", self.jac_f_x),
            ("jac_f_u", self.jac_f_u),
            ("f_and_jacs", self.f_and_jacs),
            ("affine_term", self.affine_term),
        ]:
            func.generate(f"{name}.c", opts)
            # CasADi generates to cwd; move to output_dir
            src = f"{name}.c"
            hdr = f"{name}.h"
            dst_c = os.path.join(output_dir, src)
            dst_h = os.path.join(output_dir, hdr)
            if os.path.exists(src) and os.path.abspath(src) != os.path.abspath(dst_c):
                os.replace(src, dst_c)
            if os.path.exists(hdr) and os.path.abspath(hdr) != os.path.abspath(dst_h):
                os.replace(hdr, dst_h)
            generated_files.append(dst_c)
            generated_files.append(dst_h)

        return generated_files
