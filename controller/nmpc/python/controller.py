"""
True Nonlinear MPC controller profile using CasADi Opti + IPOPT.

Unlike the three RTI-SQP profiles (hybrid, nonlinear, linear), this controller
solves the full nonlinear program (NLP) directly at every control step — no
linearization of the dynamics. IPOPT finds a local optimum of the exact
nonlinear problem, making this a genuine NMPC implementation.

Cost structure and all weights are identical to the RTI-SQP profiles so that
results are directly comparable. The only difference is the solution method:
  RTI-SQP profiles: linearize dynamics → solve convex QP (OSQP, ~7ms)
  This profile:     retain full nonlinear dynamics → solve NLP (IPOPT, ~50-500ms)
"""

from __future__ import annotations

import logging
import time
from collections import deque
from typing import Any

import casadi as ca
import numpy as np

from controller.configs.models import AppConfig
from controller.shared.python.control_common.base import Controller
from controller.shared.python.control_common.codegen.cost_functions import (
    angular_velocity_cost,
    attitude_cost,
    contouring_cost,
    control_effort_cost,
    lag_cost,
    progress_cost,
    progress_reward_cost,
    quat_norm_cost,
    smoothness_cost,
    velocity_alignment_cost,
)
from controller.shared.python.control_common.codegen.satellite_dynamics import (
    SatelliteDynamicsSymbolic,
)
from controller.shared.python.control_common.mpc_controller import SharedMPCContract
from controller.shared.python.control_common.profile_params import (
    resolve_effective_mpc_profile_contract,
)

logger = logging.getLogger(__name__)


class NmpcController(Controller):
    """
    True Nonlinear MPC: CasADi Opti + IPOPT.

    Solves the full NLP every control step with N=50 horizon (same as
    RTI-SQP profiles). No linearization — dynamics constraints are the
    exact RK4-discretized nonlinear equations from SatelliteDynamicsSymbolic.
    """

    controller_profile = "nmpc"
    controller_core = "casadi-opti-ipopt"
    solver_type = "NMPC-IPOPT"
    solver_backend = "CasADi+IPOPT"
    linearization_mode = "none"
    cpp_module_name = None

    def __init__(self, cfg: AppConfig) -> None:
        if not isinstance(cfg, AppConfig):
            raise TypeError(
                f"NmpcController requires AppConfig, got {type(cfg).__name__}"
            )

        self._cfg = cfg
        self._extract_params(cfg)

        # Fairness hashes — same mechanism as RTI-SQP profiles
        self.effective_contract = resolve_effective_mpc_profile_contract(cfg, "nmpc")
        self.shared_params_hash = str(self.effective_contract.shared_signature)
        self.effective_params_hash = str(self.effective_contract.effective_signature)
        self.profile_override_diff = dict(self.effective_contract.override_diff)
        self.profile_specific_params = dict(self.effective_contract.profile_specific)
        self.shared_contract = SharedMPCContract.from_app_config(cfg)

        # Path state
        self._path_data: list[list[float]] = []
        self._path_length: float = 0.0
        self._path_set: bool = False
        self.s: float = 0.0
        self._runtime_mode: str = "TRACK"
        self._scan_attitude_enabled: bool = False
        self._scan_attitude_target: np.ndarray | None = None

        # Warm-start cache
        self._last_X_sol: np.ndarray | None = None
        self._last_U_sol: np.ndarray | None = None
        # Mirrors MPCController._last_path_projection — consumed by reference.py
        self._last_path_projection: dict[str, Any] = {}

        # Persistent frame-continuity axes — mirrors C++ scan_ctx_.ref_prev_{y,z}.
        # Updated after every call to _tangent_to_quat so consecutive steps get a
        # smooth, hemisphere-consistent reference frame (same logic as C++ build_reference_quaternion).
        self._ref_prev_z: np.ndarray | None = None
        self._ref_prev_y: np.ndarray | None = None
        self._ref_initialized: bool = False

        self.solve_times: deque[float] = deque(maxlen=10_000)
        self._step_count: int = 0

        # Build the CasADi Opti NLP (done once at construction)
        self._build_opti_problem()

        logger.info(
            "NmpcController initialized: N=%d, nu=%d, ipopt_max_iter=%d",
            self.N,
            self.nu,
            self._ipopt_max_iter,
        )

    # ------------------------------------------------------------------
    # Controller ABC properties
    # ------------------------------------------------------------------

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def prediction_horizon(self) -> int | None:
        return self.N

    # ------------------------------------------------------------------
    # Parameter extraction (mirrors MPCController._extract_params)
    # ------------------------------------------------------------------

    def _extract_params(self, cfg: AppConfig) -> None:
        physics = cfg.physics
        mpc = cfg.mpc

        self.total_mass = physics.total_mass
        I_val = physics.moment_of_inertia
        self.moment_of_inertia = np.array([I_val, I_val, I_val], dtype=float)

        sorted_ids = sorted(physics.thruster_positions.keys())
        self.num_thrusters = len(sorted_ids)
        self.thruster_positions = [physics.thruster_positions[i] for i in sorted_ids]
        self.thruster_directions = [physics.thruster_directions[i] for i in sorted_ids]
        self.thruster_forces = [physics.thruster_forces[i] for i in sorted_ids]
        # Default opposing pairs for 6-thruster config: (+X/-X, +Y/-Y, +Z/-Z)
        if self.num_thrusters == 6:
            self._thruster_pairs: list[tuple[int, int]] = [(0, 1), (2, 3), (4, 5)]
        else:
            # Generic: pair consecutive thrusters (best-effort)
            self._thruster_pairs = [
                (i, i + 1) for i in range(0, self.num_thrusters - 1, 2)
            ]

        if physics.reaction_wheels:
            self.num_rw_axes = len(physics.reaction_wheels)
            self.rw_torque_limits = [
                float(rw.max_torque) for rw in physics.reaction_wheels
            ]
            self.rw_inertia = [float(rw.inertia) for rw in physics.reaction_wheels]
            self.rw_axes = []
            for rw in physics.reaction_wheels:
                axis = np.array(getattr(rw, "axis", (1, 0, 0)), dtype=float)
                n = np.linalg.norm(axis)
                self.rw_axes.append(
                    axis / n if n > 1e-12 else np.array([1.0, 0.0, 0.0])
                )
        else:
            self.num_rw_axes = 0
            self.rw_torque_limits = []
            self.rw_inertia = []
            self.rw_axes = []

        self.N = int(mpc.prediction_horizon)
        self._dt = float(mpc.dt)
        self.nu = self.num_rw_axes + self.num_thrusters + 1  # RW + thrusters + v_s
        self.nx = 17  # augmented state with path parameter s

        self.Q_contour = float(mpc.Q_contour)
        _q_lag = float(mpc.Q_lag)
        if _q_lag <= 0.0:
            _q_lag = float(getattr(mpc, "Q_lag_default", 0.0) or 0.0)
        self.Q_lag = _q_lag
        self.Q_progress = float(mpc.Q_progress)
        self.progress_reward = float(mpc.progress_reward)
        self.Q_velocity_align = float(mpc.Q_velocity_align)
        self.Q_s_anchor = float(mpc.Q_s_anchor)
        self.Q_smooth = float(mpc.Q_smooth)
        self.Q_terminal_pos = float(mpc.Q_terminal_pos)
        self.Q_terminal_s = float(mpc.Q_terminal_s)
        self.Q_angvel = float(mpc.q_angular_velocity)
        self.Q_attitude = float(mpc.Q_attitude)
        self.Q_quat_norm = float(mpc.Q_quat_norm)
        self.R_thrust = float(mpc.r_thrust)
        self.R_rw_torque = float(mpc.r_rw_torque)
        self.thrust_pair_weight = float(mpc.thrust_pair_weight)
        self.path_speed = float(mpc.path_speed)
        self.path_speed_min = float(mpc.path_speed_min)
        self.path_speed_max = float(mpc.path_speed_max)

        try:
            from controller.shared.python.physics.orbital_config import OrbitalConfig

            orb = OrbitalConfig()
            self.orbital_mu = float(orb.mu)
            self.orbital_radius = float(orb.orbital_radius)
        except Exception:
            self.orbital_mu = 3.986004418e14
            self.orbital_radius = 6.778e6

        # IPOPT-specific: read from profile_specific defaults
        self._ipopt_max_iter = 3000

        # Pack CasADi parameter vector (satellite physical constants)
        self._casadi_params = SatelliteDynamicsSymbolic.pack_params(
            mass=self.total_mass,
            inertia_diag=self.moment_of_inertia,
            thruster_positions=[np.array(p) for p in self.thruster_positions],
            thruster_directions=[np.array(d) for d in self.thruster_directions],
            thruster_forces=list(self.thruster_forces),
            rw_axes=self.rw_axes,
            rw_torque_limits=self.rw_torque_limits,
            rw_inertias=self.rw_inertia,
            orbital_mu=self.orbital_mu,
            orbital_radius=self.orbital_radius,
        )

    # ------------------------------------------------------------------
    # NLP construction (called once at __init__)
    # ------------------------------------------------------------------

    def _build_opti_problem(self) -> None:
        """Construct the CasADi Opti NLP. Called once; parameters set per-solve."""
        dyn = SatelliteDynamicsSymbolic(
            num_thrusters=self.num_thrusters,
            num_rw=self.num_rw_axes,
        )

        opti = ca.Opti()

        # Decision variables
        X = opti.variable(self.nx, self.N + 1)  # state trajectory
        U = opti.variable(self.nu, self.N)  # control trajectory

        # Parameters updated each solve
        x0_p = opti.parameter(self.nx)  # initial state
        sat_p = opti.parameter(dyn.np_)  # satellite physical params
        # Reference trajectory: [p_ref(3), t_ref(3), q_ref(4)] per stage
        refs_p = opti.parameter(10, self.N + 1)

        # ── Dynamics constraints (exact nonlinear RK4) ──────────────
        for k in range(self.N):
            x_next = dyn.f_discrete(X[:, k], U[:, k], sat_p, self._dt)
            opti.subject_to(X[:, k + 1] == x_next)

        # ── Initial state constraint ────────────────────────────────
        opti.subject_to(X[:, 0] == x0_p)

        # ── Control bounds (element-wise via vec) ───────────────────
        # CasADi Opti requires element-wise constraints; use ca.vec to flatten.
        # RW torques: normalized [-1, 1]
        opti.subject_to(opti.bounded(-1.0, ca.vec(U[: self.num_rw_axes, :]), 1.0))
        # Thruster duty cycles: [0, 1]
        thr_slice = U[self.num_rw_axes : self.num_rw_axes + self.num_thrusters, :]
        opti.subject_to(opti.bounded(0.0, ca.vec(thr_slice), 1.0))
        # Virtual path speed
        vs_slice = U[self.num_rw_axes + self.num_thrusters :, :]
        opti.subject_to(
            opti.bounded(self.path_speed_min, ca.vec(vs_slice), self.path_speed_max)
        )

        # ── Cost function ───────────────────────────────────────────
        # Opti variables are MX; cost accumulator must also be MX.
        cost = ca.MX(0)
        for k in range(self.N):
            x_k = X[:, k]
            u_k = U[:, k]
            pos_k = x_k[0:3]
            q_k = x_k[3:7]
            vel_k = x_k[7:10]
            omega_k = x_k[10:13]
            tau_rw_k = u_k[: self.num_rw_axes]
            u_thr_k = u_k[self.num_rw_axes : self.num_rw_axes + self.num_thrusters]
            v_s_k = u_k[self.num_rw_axes + self.num_thrusters]

            p_ref_k = refs_p[0:3, k]
            t_ref_k = refs_p[3:6, k]
            q_ref_k = refs_p[6:10, k]

            cost += contouring_cost(pos_k, p_ref_k, t_ref_k, self.Q_contour)
            cost += lag_cost(pos_k, p_ref_k, t_ref_k, self.Q_lag)
            cost += progress_cost(v_s_k, self.path_speed, self.Q_progress)
            cost += progress_reward_cost(v_s_k, self.progress_reward)
            cost += velocity_alignment_cost(
                vel_k, t_ref_k, self.path_speed, self.Q_velocity_align
            )
            cost += attitude_cost(q_k, q_ref_k, self.Q_attitude)
            cost += quat_norm_cost(q_k, q_k, self.Q_quat_norm)
            cost += angular_velocity_cost(omega_k, self.Q_angvel)
            cost += control_effort_cost(
                tau_rw_k, u_thr_k, self.R_rw_torque, self.R_thrust
            )
            # Inline opposing-thruster penalty (opposing_thruster_cost uses SX internally,
            # which cannot mix with the MX Opti graph; replicate the logic here).
            for _i, _j in self._thruster_pairs:
                _s = u_thr_k[_i] + u_thr_k[_j]
                cost += self.thrust_pair_weight * _s * _s

            if k > 0:
                u_prev = U[:, k - 1]
                cost += smoothness_cost(u_k, u_prev, self.Q_smooth)

        # Terminal cost: position pull toward end of path
        x_N = X[:, self.N]
        p_ref_N = refs_p[0:3, self.N]
        q_ref_N = refs_p[6:10, self.N]
        cost += self.Q_terminal_pos * ca.sumsqr(x_N[0:3] - p_ref_N)
        cost += attitude_cost(x_N[3:7], q_ref_N, self.Q_attitude)

        opti.minimize(cost)

        # ── Solver configuration ────────────────────────────────────
        opti.solver(
            "ipopt",
            {"print_time": False},
            {
                "max_iter": self._ipopt_max_iter,
                "print_level": 0,
                "warm_start_init_point": "yes",
                "warm_start_bound_push": 1e-2,
                "warm_start_mult_bound_push": 1e-2,
                "tol": 1e-2,
                "acceptable_tol": 1e-2,
                "acceptable_iter": 5,
            },
        )

        self._opti = opti
        self._X = X
        self._U = U
        self._x0_p = x0_p
        self._sat_p = sat_p
        self._refs_p = refs_p

    # ------------------------------------------------------------------
    # Core control action
    # ------------------------------------------------------------------

    def get_control_action(
        self,
        x_current: np.ndarray,
        previous_thrusters: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        t_start = time.perf_counter()
        self._step_count += 1

        # --- State augmentation (mirrors MPCController.get_control_action) ---
        s_for_state = float(self.s)
        x_arr = np.array(x_current, dtype=float).ravel()
        if x_arr.shape[0] == 16:
            x_aug = np.append(x_arr, s_for_state)
        elif x_arr.shape[0] == 17:
            x_aug = x_arr.copy()
            x_aug[16] = s_for_state
        elif x_arr.shape[0] == 13:
            x_aug = np.concatenate([x_arr, np.zeros(3), [s_for_state]])
        else:
            x_aug = np.zeros(17)
            x_aug[: min(len(x_arr), 17)] = x_arr[: min(len(x_arr), 17)]
            x_aug[16] = s_for_state

        # Build reference trajectory for full horizon
        refs = self._build_reference_trajectory(x_aug)

        # Set parameter values
        self._opti.set_value(self._x0_p, x_aug)
        self._opti.set_value(self._sat_p, self._casadi_params)
        self._opti.set_value(self._refs_p, refs)

        # Warm-start from previous solution (shift trajectory by one step)
        if self._last_X_sol is not None:
            X_warm = np.hstack([self._last_X_sol[:, 1:], self._last_X_sol[:, [-1]]])
            U_warm = np.hstack([self._last_U_sol[:, 1:], self._last_U_sol[:, [-1]]])
            self._opti.set_initial(self._X, X_warm)
            self._opti.set_initial(self._U, U_warm)
        else:
            X_init = np.tile(x_aug.reshape(-1, 1), (1, self.N + 1))
            U_init = np.zeros((self.nu, self.N))
            U_init[self.num_rw_axes + self.num_thrusters, :] = self.path_speed
            self._opti.set_initial(self._X, X_init)
            self._opti.set_initial(self._U, U_init)

        # --- Solve ---
        solve_success = False
        ipopt_iters = 0
        ipopt_status = "unknown"
        u_opt = np.zeros(self.nu)
        u_opt[self.num_rw_axes + self.num_thrusters] = (
            self.path_speed
        )  # safe default v_s

        try:
            sol = self._opti.solve()
            u_full = np.array(sol.value(self._U), dtype=float)
            u_opt = u_full[:, 0]
            self._last_X_sol = np.array(sol.value(self._X), dtype=float)
            self._last_U_sol = u_full
            solve_success = True
            stats = self._opti.stats()
            ipopt_iters = int(stats.get("iter_count", 0))
            ipopt_status = "optimal" if stats.get("success", False) else "acceptable"
        except Exception as exc:
            logger.warning("IPOPT solve failed at step %d: %s", self._step_count, exc)
            try:
                stats = self._opti.stats()
                ipopt_iters = int(stats.get("iter_count", 0))
                ipopt_status = "failed"
            except Exception:
                pass
            self._last_X_sol = None
            self._last_U_sol = None

        solve_time = time.perf_counter() - t_start
        self.solve_times.append(solve_time)

        # --- Path s advancement (mirrors C++ update_from_runtime_mode + s clamping) ---
        # SETTLE/HOLD/COMPLETE: freeze s (C++ sets active_path_speed_ = 0)
        # TRACK/RECOVER: advance by solved v_s, clamped to [0, path_length]
        v_s = float(u_opt[self.num_rw_axes + self.num_thrusters])
        mode = self._runtime_mode.upper()
        if mode not in {"SETTLE", "HOLD", "COMPLETE"}:
            path_s = max(0.0, s_for_state + v_s * self._dt)
            if self._path_length > 0.0:
                path_s = min(path_s, self._path_length)
        else:
            path_s = s_for_state
            v_s = 0.0

        self.s = path_s
        path_len = float(self._path_length) if self._path_length > 0.0 else 0.0
        progress = path_s / path_len if path_len > 1e-9 else 0.0
        remaining = max(0.0, path_len - path_s)

        # Mirrors MPCController._last_path_projection — consumed by reference.py
        self._last_path_projection = {
            "s": path_s,
            "s_proj": None,
            "path_error": None,
            "endpoint_error": None,
            "s_pred": path_s + v_s * self._dt,
            "path_v_s": v_s,
        }

        # Return RW torques + thruster commands (strip v_s)
        control_out = u_opt[: self.num_rw_axes + self.num_thrusters]

        info: dict[str, Any] = {
            "controller_profile": self.controller_profile,
            "controller_core": self.controller_core,
            "solver_type": self.solver_type,
            "solver_backend": self.solver_backend,
            "linearization_mode": self.linearization_mode,
            "cpp_backend_module": None,
            "solver_success": solve_success,
            "solver_fallback": not solve_success,
            "solver_fallback_reason": None if solve_success else "ipopt_failed",
            "iterations": ipopt_iters,
            "solve_time": solve_time,
            "time_limit_exceeded": False,
            "objective_value": None,
            "sqp_iterations": 1,
            "sqp_kkt_residual": 0.0,
            "sqp_outer_iterations": 1,
            "sqp_outer_residual": None,
            "sqp_outer_converged": solve_success,
            "ipopt_status": ipopt_status,
            "ipopt_iterations": ipopt_iters,
            "shared_params_hash": self.shared_params_hash,
            "effective_params_hash": self.effective_params_hash,
            "override_diff": dict(self.profile_override_diff),
            "profile_specific_params": dict(self.profile_specific_params),
            "path_s": path_s,
            "path_s_proj": None,
            "path_v_s": v_s,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": None,
            "path_endpoint_error": None,
            "path_s_pred": path_s + v_s * self._dt,
            "timing_linearization_s": 0.0,
            "timing_cost_update_s": 0.0,
            "timing_constraint_update_s": 0.0,
            "timing_matrix_update_s": 0.0,
            "timing_warmstart_s": 0.0,
            "timing_solve_only_s": solve_time,
            "fallback_active": False,
            "fallback_age_s": 0.0,
            "fallback_scale": 0.0,
            "ref_heading_step_deg": 0.0,
            "ref_quat_step_deg_max_horizon": 0.0,
            "ref_slew_limited_fraction": 0.0,
            "linearization_attempted_stages": 0,
            "linearization_failed_stages": 0,
            "linearization_integrity_failure": False,
            "linearization_integrity_reason": None,
            "linearization_used_stale_fallback": False,
        }

        return control_out, info

    # ------------------------------------------------------------------
    # Reference trajectory builder
    # ------------------------------------------------------------------

    def _tangent_to_quat(
        self,
        tangent: np.ndarray,
        q_current: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Build a reference quaternion that aligns the body +X axis with `tangent`.

        Mirrors C++ SQPController::build_reference_quaternion exactly:
          - First call: z_seed = current satellite Z (from q_current)
          - Subsequent calls: z_seed = _ref_prev_z (persistent across steps)
          - y hemisphere continuity enforced via _ref_prev_y
          - Shortest-path flip to q_current
          - Persists _ref_prev_{y,z} for next call
        """
        from controller.shared.python.utils.orientation_utils import (
            quat_wxyz_from_basis,
        )

        x_body = np.array(tangent, dtype=float)
        n = np.linalg.norm(x_body)
        x_body = x_body / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])

        # --- Choose z seed (same precedence as C++ build_reference_quaternion) ---
        # Priority 1: scan enabled → z_seed = scan axis (always, every call)
        # Priority 2: frame initialized → z_seed = previous reference z-axis
        # Priority 3: first call → bootstrap from current satellite z-axis
        if self._scan_attitude_enabled and self._scan_attitude_target is not None:
            axis_norm = np.linalg.norm(self._scan_attitude_target)
            z_seed = (
                self._scan_attitude_target / axis_norm
                if axis_norm > 1e-9
                else np.array([0.0, 0.0, 1.0])
            )
        elif (
            self._ref_initialized
            and self._ref_prev_z is not None
            and np.linalg.norm(self._ref_prev_z) > 1e-9
        ):
            z_seed = self._ref_prev_z / np.linalg.norm(self._ref_prev_z)
        elif q_current is not None and np.linalg.norm(q_current) > 1e-9:
            qn = q_current / np.linalg.norm(q_current)
            qw, qx, qy, qz = qn
            z_seed = np.array(
                [
                    2 * (qx * qz + qw * qy),
                    2 * (qy * qz - qw * qx),
                    1 - 2 * (qx * qx + qy * qy),
                ]
            )
        else:
            z_seed = np.array([0.0, 0.0, 1.0])

        # --- Project z_seed orthogonal to x_body ---
        z_body = z_seed - np.dot(z_seed, x_body) * x_body
        z_norm = np.linalg.norm(z_body)
        if z_norm <= 1e-6:
            # Deterministic least-aligned world axis fallback (same as C++)
            best, best_dot = np.array([1.0, 0.0, 0.0]), abs(x_body[0])
            for cand in (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])):
                d = abs(float(np.dot(cand, x_body)))
                if d < best_dot:
                    best, best_dot = cand, d
            z_body = best - np.dot(best, x_body) * x_body
            z_norm = np.linalg.norm(z_body)
        z_body = z_body / max(z_norm, 1e-12)

        y_body = np.cross(z_body, x_body)
        y_norm = np.linalg.norm(y_body)
        if y_norm <= 1e-9:
            y_body = (
                np.array([0.0, 1.0, 0.0])
                - np.dot(np.array([0.0, 1.0, 0.0]), x_body) * x_body
            )
            y_norm = np.linalg.norm(y_body)
        y_body = y_body / max(y_norm, 1e-12)
        z_body = np.cross(x_body, y_body)
        z_body = z_body / max(np.linalg.norm(z_body), 1e-12)

        # --- Y hemisphere continuity (same as C++) ---
        if (
            self._ref_initialized
            and self._ref_prev_y is not None
            and np.linalg.norm(self._ref_prev_y) > 1e-9
        ):
            if float(np.dot(self._ref_prev_y, y_body)) < 0.0:
                y_body = -y_body
                z_body = -z_body

        # --- Build quaternion from orthonormal basis ---
        q_ref = quat_wxyz_from_basis(x_body, y_body, z_body)

        # --- Shortest-path flip to q_current ---
        if q_current is not None and np.dot(q_ref, q_current) < 0:
            q_ref = -q_ref

        # --- Persist axes for next call ---
        self._ref_prev_z = z_body.copy()
        self._ref_prev_y = y_body.copy()
        self._ref_initialized = True

        return q_ref

    def _build_reference_trajectory(self, x_aug: np.ndarray) -> np.ndarray:
        """
        Build a (10, N+1) reference array: [p_ref(3), t_ref(3), q_ref(4)] per stage.
        Interpolates along the stored path data at s + k*dt*path_speed.
        q_ref aligns body +X with the path tangent (mirrors C++ build_reference_quaternion).
        """
        refs = np.zeros((10, self.N + 1))
        refs[6, :] = 1.0  # default: identity quaternion

        if not self._path_set or len(self._path_data) < 2:
            return refs

        q_current = x_aug[3:7]
        # Stage 0 uses the satellite's current quaternion as bootstrap for _tangent_to_quat
        # (only matters on the very first call when _ref_initialized is False).
        # Subsequent stages use the persistent _ref_prev_{y,z} axes for continuity.
        for k in range(self.N + 1):
            s_k = self.s + k * self._dt * self.path_speed
            s_k = min(s_k, self._path_length)
            p_ref, t_ref = self._interpolate_path(s_k)
            q_ref = self._tangent_to_quat(t_ref, q_current if k == 0 else None)
            refs[0:3, k] = p_ref
            refs[3:6, k] = t_ref
            refs[6:10, k] = q_ref

        return refs

    def _interpolate_path(self, s_query: float) -> tuple[np.ndarray, np.ndarray]:
        """Linear interpolation along path data to find p_ref and tangent t_ref."""
        data = self._path_data
        if s_query <= data[0][0]:
            p = np.array(data[0][1:4])
            if len(data) > 1:
                t = np.array(data[1][1:4]) - np.array(data[0][1:4])
                n = np.linalg.norm(t)
                t = t / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
            else:
                t = np.array([1.0, 0.0, 0.0])
            return p, t
        if s_query >= data[-1][0]:
            p = np.array(data[-1][1:4])
            if len(data) > 1:
                t = np.array(data[-1][1:4]) - np.array(data[-2][1:4])
                n = np.linalg.norm(t)
                t = t / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
            else:
                t = np.array([1.0, 0.0, 0.0])
            return p, t

        for i in range(len(data) - 1):
            s0, s1 = data[i][0], data[i + 1][0]
            if s0 <= s_query <= s1:
                alpha = (s_query - s0) / (s1 - s0) if s1 > s0 else 0.0
                p0 = np.array(data[i][1:4])
                p1 = np.array(data[i + 1][1:4])
                p = p0 + alpha * (p1 - p0)
                t = p1 - p0
                n = np.linalg.norm(t)
                t = t / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
                return p, t

        return np.array(data[-1][1:4]), np.array([1.0, 0.0, 0.0])

    # ------------------------------------------------------------------
    # Path and mode interface (mirrors MPCController)
    # ------------------------------------------------------------------

    def set_path(self, path_points: list[tuple[float, float, float]]) -> None:
        if not path_points or len(path_points) < 2:
            logger.warning("NMPC: path must have at least 2 points")
            return
        self._path_data = []
        s = 0.0
        prev = None
        for pt in path_points:
            if prev is not None:
                dx, dy, dz = pt[0] - prev[0], pt[1] - prev[1], pt[2] - prev[2]
                s += (dx**2 + dy**2 + dz**2) ** 0.5
            self._path_data.append([s, float(pt[0]), float(pt[1]), float(pt[2])])
            prev = pt
        self._path_length = s
        self._path_set = True
        self.s = 0.0
        self._last_X_sol = None
        self._last_U_sol = None
        self._ref_prev_z = None
        self._ref_prev_y = None
        self._ref_initialized = False
        logger.info("NMPC path set: %d points, length=%.3fm", len(path_points), s)

    def set_runtime_mode(self, mode: str | None) -> None:
        self._runtime_mode = str(mode or "TRACK").upper()

    def set_current_path_s(self, s_value: float) -> None:
        self.s = float(max(0.0, s_value))

    def set_scan_attitude_context(
        self,
        center: tuple[float, float, float] | None,
        axis: tuple[float, float, float] | None,
        direction: str = "CW",
    ) -> None:
        if axis is None:
            self._scan_attitude_enabled = False
            self._scan_attitude_target = None
            # Reset frame continuity so non-scan free-twist mode restarts cleanly
            self._ref_prev_z = None
            self._ref_prev_y = None
            self._ref_initialized = False
            return
        self._scan_attitude_enabled = True
        # Store normalized scan axis — used as z_seed in _tangent_to_quat (mirrors C++ scan_ctx_.axis)
        axis_arr = np.array(axis, dtype=float)
        n = np.linalg.norm(axis_arr)
        self._scan_attitude_target = (
            axis_arr / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        )

    def get_path_progress(self, position: np.ndarray | None = None) -> dict[str, float]:
        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        return {
            "s": float(self.s),
            "progress": float(self.s / path_len) if path_len > 1e-9 else 0.0,
            "remaining": float(max(0.0, path_len - self.s)),
            "path_error": float("inf"),
            "endpoint_error": float("inf"),
        }

    def split_control(self, control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Split flat control vector into (rw_torques, thruster_commands)."""
        return control[: self.num_rw_axes], control[self.num_rw_axes :]

    def get_path_reference_state(
        self,
        s_query: float | None = None,
        q_current: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (pos_ref, tangent, q_ref) at current or queried arc-length."""
        q_curr = np.array(
            q_current if q_current is not None else [1.0, 0.0, 0.0, 0.0], dtype=float
        )
        if not self._path_set or len(self._path_data) < 2:
            return np.zeros(3), np.zeros(3), q_curr
        s_val = float(self.s if s_query is None else s_query)
        s_val = max(0.0, min(s_val, self._path_length))
        pos, tangent = self._interpolate_path(s_val)
        q_ref = self._tangent_to_quat(tangent, q_curr)
        return pos, tangent, q_ref

    def get_path_reference(
        self, s_query: float | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (pos_ref, tangent) at current or queried arc-length."""
        pos, tangent, _ = self.get_path_reference_state(s_query=s_query)
        return pos, tangent

    def reset(self) -> None:
        self.s = 0.0
        self._last_X_sol = None
        self._last_U_sol = None
        self._last_path_projection.clear()
        self._step_count = 0
        self.solve_times.clear()
        self._ref_prev_z = None
        self._ref_prev_y = None
        self._ref_initialized = False

    def get_solver_stats(self) -> dict[str, Any]:
        if not self.solve_times:
            return {"solve_count": 0, "average_solve_time": 0.0, "max_solve_time": 0.0}
        times = list(self.solve_times)
        return {
            "solve_times": times,
            "solve_count": len(times),
            "average_solve_time": sum(times) / len(times),
            "max_solve_time": max(times),
        }
