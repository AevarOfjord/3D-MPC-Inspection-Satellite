"""
Shared acados OCP base controller for acados_rti and acados_sqp profiles.

Both profiles share identical OCP formulation, dynamics, cost, parameter
handling, path interface, warm-start strategy, and reference building.
The only difference is the nlp_solver_type class variable:
  AcadosRtiController  → "SQP_RTI"  (1 SQP step/step, like RTI-SQP but exact NL)
  AcadosSqpController  → "SQP"      (iterate to convergence, like NMPC-IPOPT but faster)

Solver: acados + HPIPM QP backend.
Dynamics: exact RK4 via SatelliteDynamicsSymbolic._rk4_step (DISCRETE integrator).
Cost: EXTERNAL type — required for nonlinear quaternion attitude cost.
Note: smoothness cost (u_k - u_{k-1}) is omitted because EXTERNAL cost at stage k
      only has access to x_k, u_k, p_k — not u_{k-1}.  This is an acknowledged
      asymmetry vs. NmpcController; document in the paper.
"""

from __future__ import annotations

import ctypes
import logging
import os
import sys
import time
from collections import deque
from typing import Any

import numpy as np

from controller.configs.models import AppConfig
from controller.shared.python.control_common.base import Controller
from controller.shared.python.control_common.codegen.cost_functions import (
    quat_error_vec,
)
from controller.shared.python.control_common.codegen.satellite_dynamics import (
    SatelliteDynamicsSymbolic,
)
from controller.shared.python.control_common.mpc_controller import SharedMPCContract
from controller.shared.python.control_common.profile_params import (
    resolve_effective_mpc_profile_contract,
)

logger = logging.getLogger(__name__)


class AcadosBaseController(Controller):
    """
    Base class for acados-backed NMPC controllers.

    Subclasses set controller_profile, solver_type, and _acados_nlp_solver_type.
    All OCP construction, cost formulation, path interface, and solve logic live here.
    """

    # --- Subclasses override these three ---
    controller_profile: str = ""
    solver_type: str = ""
    _acados_nlp_solver_type: str = ""  # "SQP_RTI" or "SQP"

    # --- Delta-u (control-rate) bounds: max allowed change per 50ms step ---
    # These hard constraints prevent bang-bang RW saturation that would occur
    # when the QP aggressive-step solver (RTI/SQP) pre-rotates for future
    # attitude targets.  Subclasses may override for different missions.
    _delta_u_max_rw: float = 0.08  # normalized RW change/step  (~1.6/s)
    _delta_u_max_thr: float = 0.15  # thruster duty-cycle change/step
    _delta_u_max_vs: float = 0.10  # virtual path-speed change/step

    # --- Globalization (SQP only; RTI always uses FIXED_STEP) ---
    _acados_globalization: str = (
        "FIXED_STEP"  # SQP subclass overrides to MERIT_BACKTRACKING
    )

    # --- Stage attitude weight factor (≤ 1.0) ---
    # The stage attitude cost drives aggressive pre-rotation when the RTI/SQP
    # global QP step looks ahead and sees large future attitude errors.
    # Reducing it to a small fraction (keeping full weight only at terminal)
    # eliminates this without breaking path-following or attitude stability.
    # The path *angular velocity* reference (omega_ref) handles attitude control.
    _Q_attitude_stage_factor: float = 0.02

    # --- Terminal angular velocity bound ---
    # Forces the OCP to plan deceleration within the horizon so the predicted
    # state at k=N has |ω_N| ≤ ω_max_term.  Without this the QP commits to
    # one-way angular-momentum build-up (spin-up only), whereas the correct
    # optimal profile is symmetric (spin-up then coast/spin-down).
    _omega_max_terminal: float = 0.35  # rad/s ≈ 20°/s at horizon end

    controller_core: str = "acados"
    solver_backend: str = "acados+HPIPM"
    linearization_mode: str = "none"
    cpp_module_name = None

    def __init__(self, cfg: AppConfig) -> None:
        if not isinstance(cfg, AppConfig):
            raise TypeError(
                f"AcadosBaseController requires AppConfig, got {type(cfg).__name__}"
            )

        self._cfg = cfg
        self._extract_params(cfg)

        # Fairness hashes — same mechanism as NmpcController and RTI-SQP profiles
        self.effective_contract = resolve_effective_mpc_profile_contract(
            cfg, self.controller_profile
        )
        self.shared_params_hash = str(self.effective_contract.shared_signature)
        self.effective_params_hash = str(self.effective_contract.effective_signature)
        self.profile_override_diff = dict(self.effective_contract.override_diff)
        self.profile_specific_params = dict(self.effective_contract.profile_specific)
        self.shared_contract = SharedMPCContract.from_app_config(cfg)

        # acados-specific tolerances from profile_specific
        self._acados_max_iter = int(
            self.profile_specific_params.get("acados_max_iter", 50)
        )
        self._acados_tol_stat = float(
            self.profile_specific_params.get("acados_tol_stat", 1e-2)
        )
        self._acados_tol_eq = float(
            self.profile_specific_params.get("acados_tol_eq", 1e-2)
        )
        self._acados_tol_ineq = float(
            self.profile_specific_params.get("acados_tol_ineq", 1e-2)
        )

        # Path state (mirrors NmpcController exactly)
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
        self._last_applied_u: np.ndarray | None = None
        # Mirrors MPCController._last_path_projection — consumed by reference.py
        self._last_path_projection: dict[str, Any] = {}

        # Persistent frame-continuity axes (mirrors C++ ref_prev_{y,z})
        self._ref_prev_z: np.ndarray | None = None
        self._ref_prev_y: np.ndarray | None = None
        self._ref_initialized: bool = False

        self.solve_times: deque[float] = deque(maxlen=10_000)
        self._step_count: int = 0

        # Build and compile the acados OCP solver (done once at construction)
        self._build_acados_solver()

        logger.info(
            "%s initialized: N=%d, nu=%d, solver=%s, max_iter=%d",
            self.__class__.__name__,
            self.N,
            self.nu,
            self._acados_nlp_solver_type,
            self._acados_max_iter,
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
    # Parameter extraction (mirrors NmpcController._extract_params)
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
        if self.num_thrusters == 6:
            self._thruster_pairs: list[tuple[int, int]] = [(0, 1), (2, 3), (4, 5)]
        else:
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
        self.nu = self.num_rw_axes + self.num_thrusters + 1
        self.nx = 17

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
    # acados OCP construction (called once at __init__)
    # ------------------------------------------------------------------

    @staticmethod
    def _shared_lib_extension() -> str:
        return ".dylib" if sys.platform == "darwin" else ".so"

    @classmethod
    def _resolve_acados_lib_dirs(cls, acados_template_module: Any) -> list[str]:
        """Resolve likely acados runtime lib directories in priority order."""
        candidates: list[str] = []

        acados_source_dir = os.environ.get("ACADOS_SOURCE_DIR", "").strip()
        if acados_source_dir:
            candidates.append(os.path.join(acados_source_dir, "lib"))

        guessed_root = os.path.abspath(
            os.path.join(os.path.dirname(acados_template_module.__file__), "..", "..")
        )
        candidates.append(os.path.join(guessed_root, "lib"))

        # Known local install fallbacks.
        candidates.extend(
            [
                "/opt/homebrew/opt/acados/lib",
                "/usr/local/opt/acados/lib",
            ]
        )

        resolved: list[str] = []
        seen: set[str] = set()
        for path in candidates:
            abs_path = os.path.abspath(path)
            if abs_path in seen:
                continue
            seen.add(abs_path)
            if os.path.isdir(abs_path):
                resolved.append(abs_path)
        return resolved

    @classmethod
    def _preload_acados_runtime_libs(cls, lib_dirs: list[str]) -> None:
        """
        Preload acados runtime dependencies with RTLD_GLOBAL.

        On macOS this avoids fragile @rpath resolution during dynamic loading.
        """
        lib_ext = cls._shared_lib_extension()
        ordered_libs = [
            f"libblasfeo{lib_ext}",
            f"libhpipm{lib_ext}",
            f"libqpOASES_e{lib_ext}",
            f"libacados{lib_ext}",
        ]
        mode = getattr(ctypes, "RTLD_GLOBAL", ctypes.DEFAULT_MODE)
        attempted: list[str] = []

        for lib_name in ordered_libs:
            loaded = False
            failures: list[str] = []

            for lib_dir in lib_dirs:
                candidate = os.path.join(lib_dir, lib_name)
                attempted.append(candidate)
                if not os.path.isfile(candidate):
                    continue
                try:
                    ctypes.CDLL(candidate, mode=mode)
                    loaded = True
                    break
                except OSError as exc:
                    failures.append(f"{candidate}: {exc}")

            if loaded:
                continue

            attempted.append(lib_name)
            try:
                ctypes.CDLL(lib_name, mode=mode)
                loaded = True
            except OSError as exc:
                failures.append(f"{lib_name}: {exc}")

            if not loaded:
                attempted_text = "\n  - ".join(attempted)
                failure_text = "\n  - ".join(failures) if failures else "none"
                raise RuntimeError(
                    "Failed to preload required acados runtime library "
                    f"{lib_name!r}. Attempted:\n  - {attempted_text}\n"
                    f"Load errors:\n  - {failure_text}\n"
                    "Set ACADOS_SOURCE_DIR so <ACADOS_SOURCE_DIR>/lib contains "
                    "libacados + HPIPM/BLASFEO/qpOASES runtime libraries."
                )

    def _build_acados_solver(self) -> None:
        """
        Construct and compile the acados OCP solver. Called once at construction.

        Per-stage parameter layout:
            p_stage = [sat_params(np_), p_ref(3), t_ref(3), q_ref(4)]
        Set at each solve via solver.set(k, "p", p_k) for k in 0..N.
        """
        import casadi as ca

        try:
            from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
        except ImportError as exc:
            raise RuntimeError(
                "acados_template is not installed. Install it from source:\n"
                "  pip install git+https://github.com/acados/acados.git"
                "#subdirectory=interfaces/acados_template\n"
                "You also need the acados C library compiled and "
                "ACADOS_SOURCE_DIR set. See https://docs.acados.org/installation/"
            ) from exc

        # Resolve runtime lib locations and preload dependencies before solver init.
        import acados_template as _at

        acados_lib_dirs = self._resolve_acados_lib_dirs(_at)
        if not acados_lib_dirs:
            raise RuntimeError(
                "acados runtime library directory not found. Checked:\n"
                f"  - ACADOS_SOURCE_DIR/lib (ACADOS_SOURCE_DIR={os.environ.get('ACADOS_SOURCE_DIR', '')!r})\n"
                "  - acados_template-adjacent lib\n"
                "  - /opt/homebrew/opt/acados/lib\n"
                "  - /usr/local/opt/acados/lib\n"
                "Install acados from source and set ACADOS_SOURCE_DIR:\n"
                "  https://docs.acados.org/installation/"
            )
        self._preload_acados_runtime_libs(acados_lib_dirs)

        dyn = SatelliteDynamicsSymbolic(
            num_thrusters=self.num_thrusters,
            num_rw=self.num_rw_axes,
        )

        # Per-stage parameter vector: satellite physics + reference trajectory
        n_sat_p = dyn.np_
        n_ref_p = 10  # p_ref(3) + t_ref(3) + q_ref(4)
        n_stage_p = n_sat_p + n_ref_p
        p_stage_sym = ca.SX.sym("p_stage", n_stage_p)

        sat_p = p_stage_sym[:n_sat_p]
        p_ref_sym = p_stage_sym[n_sat_p : n_sat_p + 3]
        t_ref_sym = p_stage_sym[n_sat_p + 3 : n_sat_p + 6]
        q_ref_sym = p_stage_sym[n_sat_p + 6 : n_sat_p + 10]

        # ── Augmented state: append u_prev for true intra-horizon smoothness ──
        # x_aug = [x_sat(17), u_prev(nu)]
        # At stage k the OCP can access u_{k-1} via x_aug[17:].
        # Augmented dynamics: u_prev_next = u_k (copy current control per step).
        # This replicates NMPC's smoothness cost ||u_k - u_{k-1}||^2 exactly.
        u_prev_aug = ca.SX.sym("u_prev_aug", self.nu)
        x_aug_sym = ca.vertcat(dyn.x, u_prev_aug)
        nx_aug = x_aug_sym.shape[0]  # 17 + nu

        # ── acados model ────────────────────────────────────────────
        model = AcadosModel()
        model.name = f"satellite_{self.controller_profile}"
        model.x = x_aug_sym  # [x_sat(17), u_prev(nu)]
        model.u = dyn.u
        model.p = p_stage_sym
        # Augmented discrete dynamics:
        # - First 17: standard satellite RK4
        # - Last nu:  u_prev_next = u_k  (track previous control in state)
        model.disc_dyn_expr = ca.vertcat(
            dyn._rk4_step(dyn.x, dyn.u, sat_p, self._dt),
            dyn.u,
        )

        # ── OCP ─────────────────────────────────────────────────────
        ocp = AcadosOcp()
        ocp.model = model
        ocp.solver_options.N_horizon = self.N
        ocp.dims.np = n_stage_p

        # ── NONLINEAR_LS cost via residual vector ─────────────────────
        # NONLINEAR_LS + GAUSS_NEWTON gives a PSD Hessian (J^T W J) by
        # construction — essential for HPIPM's Cholesky to succeed with the
        # nonlinear quaternion attitude cost (exact Hessian is indefinite).
        #
        # Residual layout (stage):
        #   r = [e_contour(3), e_lag(1), e_progress(1), e_vel_align(3),
        #        e_attitude(3), e_omega(3), e_tau_rw(nrw), e_u_thr(nthr),
        #        e_v_s(1), e_pair(npairs)]
        # Weight matrix W = diag(w²) so cost = ||r||²_W = r^T W r.
        # Smoothness via state augmentation: u_prev_aug = x_aug[17:] is the
        # previous stage's control, propagated by the augmented dynamics.
        # This gives identical intra-horizon smoothness to the NMPC formulation.
        x_k = dyn.x
        u_k = dyn.u
        pos_k = x_k[0:3]
        q_k = x_k[3:7]
        vel_k = x_k[7:10]
        omega_k = x_k[10:13]
        tau_rw_k = u_k[: self.num_rw_axes]
        u_thr_k = u_k[self.num_rw_axes : self.num_rw_axes + self.num_thrusters]
        v_s_k = u_k[self.num_rw_axes + self.num_thrusters]

        # --- Contouring residual (3): cross-track error vector
        dp = pos_k - p_ref_sym
        e_contour = dp - ca.dot(dp, t_ref_sym) * t_ref_sym

        # --- Lag residual (1): along-track scalar
        e_lag = ca.dot(dp, t_ref_sym)

        # --- Progress residual (1): virtual speed deviation
        e_progress = v_s_k - self.path_speed

        # --- Velocity alignment residual (3)
        e_vel_align = vel_k - self.path_speed * t_ref_sym

        # --- Attitude residual (3): quaternion error vector (non-linear but
        #     Gauss-Newton J^T J is PSD because it only uses first-order terms)
        e_attitude = quat_error_vec(q_k, q_ref_sym)

        # --- Angular velocity residual (3)
        e_omega = omega_k

        # --- Control effort residuals
        e_tau_rw = tau_rw_k
        e_u_thr = u_thr_k

        # --- Virtual speed effort residual (1)
        e_v_s = v_s_k

        # --- Opposing thruster pair residuals
        e_pairs = ca.vertcat(
            *[u_thr_k[_i] + u_thr_k[_j] for _i, _j in self._thruster_pairs]
        )

        # --- Smoothness residual (nu): true intra-horizon control rate cost
        # u_prev_aug holds u_{k-1} (propagated via augmented dynamics u_prev_next=u_k).
        e_smooth = dyn.u - u_prev_aug

        stage_res = ca.vertcat(
            e_contour,
            e_lag,
            e_progress,
            e_vel_align,
            e_attitude,
            e_omega,
            e_tau_rw,
            e_u_thr,
            e_v_s,
            e_pairs,
            e_smooth,
        )
        nr_stage = stage_res.shape[0]

        # Weight vector: sqrt of each penalty weight so ||r||²_W = r^T W r
        import math as _math

        # Stage attitude weight is reduced to avoid aggressive pre-rotation:
        # the full Q_attitude is applied only at the terminal stage.
        # The required attitude is enforced via omega_ref tracking at each stage.
        _Q_att_stage = self.Q_attitude * self._Q_attitude_stage_factor
        w_stage = np.array(
            [
                *([_math.sqrt(self.Q_contour)] * 3),  # contouring (3)
                _math.sqrt(self.Q_lag),  # lag (1)
                _math.sqrt(self.Q_progress),  # progress (1)
                *([_math.sqrt(self.Q_velocity_align)] * 3),  # vel align (3)
                *([_math.sqrt(_Q_att_stage)] * 3),  # attitude: LOW STAGE WEIGHT
                *(
                    [_math.sqrt(self.Q_angvel / 100.0)] * 3
                ),  # angular vel (3) — reduced ÷100
                *([_math.sqrt(self.R_rw_torque)] * self.num_rw_axes),  # RW effort
                *([_math.sqrt(self.R_thrust)] * self.num_thrusters),  # thr effort
                0.0,  # v_s effort (no penalty)
                *([_math.sqrt(self.thrust_pair_weight)] * len(self._thruster_pairs)),
                *([_math.sqrt(self.Q_smooth)] * self.nu),  # smoothness (nu)
            ],
            dtype=float,
        )
        W_stage = np.diag(w_stage**2)

        # Terminal residual: position error + attitude + angular velocity + velocity
        # Angular velocity MUST be in the terminal cost so the Gauss-Newton
        # Hessian has a non-zero ∂e_omega_N / ∂u_{N-1} path via dynamics.
        # (Stage e_omega_k has zero Jacobian w.r.t. u_k — only future stages see it.)
        x_e = dyn.x  # terminal stage uses same symbols
        e_pos_term = x_e[0:3] - p_ref_sym
        e_att_term = quat_error_vec(x_e[3:7], q_ref_sym)
        e_omega_term = x_e[10:13]  # drive angular velocity to zero at horizon end
        e_vel_term = x_e[7:10]  # drive velocity to zero at horizon end
        terminal_res = ca.vertcat(e_pos_term, e_att_term, e_omega_term, e_vel_term)
        nr_term = terminal_res.shape[0]

        # Root-cause of the pre-rotation tumbling:
        #   Q_angvel = 1200, so W_term_omega = Q_angvel × N = 60,000
        #   W_term_att = Q_attitude = 3,500  →  ratio 17:1 AGAINST attitude
        # omega_ref_traj was injecting the path angular velocity into yref, so the
        # 60,000-weight terminal penalty demanded SPIN-UP to the path rate — far
        # more important than reaching the attitude target.  This caused monotonic
        # angular acceleration on every horizon solve.
        #
        # Fix (zero recompile): reduce omega weights by ÷100 (stage) and ÷200
        # (terminal) in the W matrices.  In acados NONLINEAR_LS + GAUSS_NEWTON,
        # W is stored only in the JSON config (not compiled into the C library),
        # so changing it here updates the JSON without triggering codegen/compile.
        # omega_ref is set to 0 in get_control_action; with:
        #   stage omega weight = Q_angvel / 100 = 12
        #   terminal omega weight = Q_angvel × N / 200 = 300
        #   terminal attitude weight = Q_attitude = 3,500  (dominant driver)
        # the OCP plans a triangular velocity profile (spin-up + spin-down) to
        # reach the terminal attitude while decelerating to ω≈0 at horizon end.
        _Q_angvel_stage_eff = self.Q_angvel / 100.0  # 1200 → 12
        _Q_angvel_term_eff = self.Q_angvel * self.N / 200.0  # 60,000 → 300
        w_term = np.array(
            [
                *([_math.sqrt(self.Q_terminal_pos)] * 3),
                *([_math.sqrt(self.Q_attitude)] * 3),
                *(
                    [_math.sqrt(_Q_angvel_term_eff)] * 3
                ),  # reduced: 300 ≈ 1/12 of attitude
                *([_math.sqrt(self.Q_velocity_align * self.N)] * 3),
            ],
            dtype=float,
        )
        W_term = np.diag(w_term**2)

        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_0 = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        model.cost_y_expr = stage_res
        model.cost_y_expr_0 = stage_res
        model.cost_y_expr_e = terminal_res
        ocp.cost.yref = np.zeros(nr_stage)
        ocp.cost.yref_0 = np.zeros(nr_stage)
        ocp.cost.yref_e = np.zeros(nr_term)  # drive all terminal residuals to zero
        ocp.cost.W = W_stage
        ocp.cost.W_0 = W_stage
        ocp.cost.W_e = W_term

        # ── Control bounds ────────────────────────────────────────────
        lb_u = np.concatenate(
            [
                np.full(self.num_rw_axes, -1.0),
                np.zeros(self.num_thrusters),
                [self.path_speed_min],
            ]
        )
        ub_u = np.concatenate(
            [
                np.full(self.num_rw_axes, 1.0),
                np.ones(self.num_thrusters),
                [self.path_speed_max],
            ]
        )
        ocp.constraints.lbu = lb_u
        ocp.constraints.ubu = ub_u
        ocp.constraints.idxbu = np.arange(self.nu)

        # ── Delta-u (control-rate) constraints ────────────────────────────────
        # Hard bounds on per-step control change: |u_k - u_{k-1}| ≤ Δu_max
        # u_prev_k = x_aug_k[17:] is tracked via augmented dynamics.
        # This is the standard RTI-MPC approach for preventing bang-bang
        # saturation when the QP global optimum prefers aggressive control.
        du_expr = dyn.u - u_prev_aug  # u_k - u_prev_k  (shape: nu)
        ocp.model.con_h_expr = du_expr
        ocp.model.con_h_expr_0 = du_expr
        delta_u_lb = np.concatenate(
            [
                np.full(self.num_rw_axes, -self._delta_u_max_rw),
                np.full(self.num_thrusters, -self._delta_u_max_thr),
                [-self._delta_u_max_vs],
            ]
        )
        delta_u_ub = np.concatenate(
            [
                np.full(self.num_rw_axes, self._delta_u_max_rw),
                np.full(self.num_thrusters, self._delta_u_max_thr),
                [self._delta_u_max_vs],
            ]
        )
        ocp.constraints.lh = delta_u_lb
        ocp.constraints.uh = delta_u_ub
        ocp.constraints.lh_0 = delta_u_lb
        ocp.constraints.uh_0 = delta_u_ub

        # ── Terminal angular-velocity state bounds ─────────────────────────────
        # Indices 10-12 in the augmented state are ω_x, ω_y, ω_z.
        # Bounding them at the TERMINAL stage forces the QP to also plan a
        # deceleration phase: the OCP must produce a bang-coast-bang profile
        # instead of the unconstrained spin-up the QP naively prefers.
        _omega_t = self._omega_max_terminal
        ocp.constraints.lbx_e = np.full(3, -_omega_t)
        ocp.constraints.ubx_e = np.full(3, _omega_t)
        ocp.constraints.idxbx_e = np.array([10, 11, 12])

        # ── Initial state equality constraint ─────────────────────────
        # Declared here with zeros; overridden at each solve via solver.set(0, "x", ...)
        ocp.constraints.x0 = np.zeros(nx_aug)

        # ── Parameter default values (required by acados >= 0.4) ─────
        ocp.parameter_values = np.zeros(n_stage_p)

        # ── Solver options ────────────────────────────────────────────
        ocp.solver_options.tf = self.N * self._dt
        ocp.solver_options.integrator_type = "DISCRETE"
        ocp.solver_options.nlp_solver_type = self._acados_nlp_solver_type
        ocp.solver_options.nlp_solver_max_iter = self._acados_max_iter
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        # GAUSS_NEWTON is required for NONLINEAR_LS cost: forms J^T W J which
        # is PSD by construction, so HPIPM Cholesky always succeeds.
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.nlp_solver_tol_stat = self._acados_tol_stat
        ocp.solver_options.nlp_solver_tol_eq = self._acados_tol_eq
        ocp.solver_options.nlp_solver_tol_ineq = self._acados_tol_ineq
        ocp.solver_options.print_level = 0
        if self._acados_globalization != "FIXED_STEP":
            ocp.solver_options.globalization = self._acados_globalization

        # ── Code generation directory (mirrors codegen_cache/ convention) ─
        build_dir = os.path.join("codegen_cache", self.controller_profile)
        os.makedirs(build_dir, exist_ok=True)
        json_file = os.path.join(build_dir, "acados_ocp.json")

        # ── Smart caching: skip codegen+compile when library is already current ──
        # acados always writes the compiled shared library to:
        #   c_generated_code/libacados_ocp_solver_{model_name}{ext}
        # (relative to CWD, because code_export_directory defaults to c_generated_code/)
        import hashlib
        import sys as _sys

        _model_name = f"satellite_{self.controller_profile}"
        _lib_ext = ".dylib" if _sys.platform == "darwin" else ".so"
        _lib_path = os.path.join(
            "c_generated_code", f"libacados_ocp_solver_{_model_name}{_lib_ext}"
        )
        _hash_file = os.path.join(build_dir, "ocp_hash.txt")

        # Hash covers all parameters that affect the compiled OCP.
        # If anything changes the solver must be rebuilt.
        _hash_params = {
            "N": self.N,
            "dt": self._dt,
            "num_thrusters": self.num_thrusters,
            "num_rw_axes": self.num_rw_axes,
            "nlp_solver_type": self._acados_nlp_solver_type,
            "max_iter": self._acados_max_iter,
            "tol_stat": self._acados_tol_stat,
            "tol_eq": self._acados_tol_eq,
            "tol_ineq": self._acados_tol_ineq,
            "Q_contour": self.Q_contour,
            "Q_lag": self.Q_lag,
            "Q_progress": self.Q_progress,
            "Q_velocity_align": self.Q_velocity_align,
            "Q_attitude": self.Q_attitude,
            "Q_angvel": self.Q_angvel,
            "Q_terminal_pos": self.Q_terminal_pos,
            "R_rw_torque": self.R_rw_torque,
            "R_thrust": self.R_thrust,
            "thrust_pair_weight": self.thrust_pair_weight,
            "path_speed": self.path_speed,
            "path_speed_min": self.path_speed_min,
            "path_speed_max": self.path_speed_max,
            "thruster_pairs": str(sorted(self._thruster_pairs)),
            "Q_smooth": self.Q_smooth,
            "nx_aug": 17 + self.nu,  # state augmentation: x_sat + u_prev
            "delta_u_max_rw": self._delta_u_max_rw,
            "delta_u_max_thr": self._delta_u_max_thr,
            "delta_u_max_vs": self._delta_u_max_vs,
            "globalization": self._acados_globalization,
            "Q_attitude_stage_factor": self._Q_attitude_stage_factor,
            "omega_max_terminal": self._omega_max_terminal,
        }
        _current_hash = hashlib.sha256(
            str(sorted(_hash_params.items())).encode()
        ).hexdigest()[:16]

        _need_build = True
        if (
            os.path.isfile(_lib_path)
            and os.path.isfile(json_file)
            and os.path.isfile(_hash_file)
        ):
            try:
                with open(_hash_file) as _hf:
                    _stored_hash = _hf.read().strip()
                if _stored_hash == _current_hash:
                    _need_build = False
                    logger.info(
                        "[%s] acados solver cache hit (hash=%s) — "
                        "reusing compiled library, skipping codegen+compile.",
                        self.controller_profile,
                        _current_hash,
                    )
            except Exception:
                pass  # corrupted hash file → rebuild

        if _need_build:
            logger.info(
                "[%s] acados solver cache miss (hash=%s) — "
                "running codegen+compile for N=%d (this may take ~1–2 min)...",
                self.controller_profile,
                _current_hash,
                self.N,
            )

        self._acados_solver = AcadosOcpSolver(
            ocp,
            json_file=json_file,
            build=_need_build,
            generate=_need_build,
        )

        if _need_build:
            try:
                with open(_hash_file, "w") as _hf:
                    _hf.write(_current_hash)
                logger.info(
                    "[%s] acados solver compiled successfully — hash %s stored.",
                    self.controller_profile,
                    _current_hash,
                )
            except Exception as _e:
                logger.warning(
                    "[%s] Could not write ocp_hash.txt: %s",
                    self.controller_profile,
                    _e,
                )

        # --- Store solver metadata ---
        self._n_stage_p = n_stage_p
        self._nx_aug = nx_aug  # 17 + nu — size of augmented state in solver
        self._nr_stage = int(nr_stage)  # for per-stage yref in get_control_action
        self._nr_term = int(nr_term)  # for terminal yref
        # Precompute omega indices in stage / terminal yref vectors
        # e_contour(3) e_lag(1) e_progress(1) e_vel_align(3) e_attitude(3) e_omega(3) ...
        self._stage_omega_idx = 3 + 1 + 1 + 3 + 3  # = 11
        # Terminal: e_pos(3) e_att(3) e_omega(3) e_vel(3)
        self._term_omega_idx = 3 + 3  # = 6

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

        # --- State augmentation (mirrors NmpcController exactly) ---
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

        # Build reference trajectory for full horizon (uses first 17 components only)
        refs = self._build_reference_trajectory(x_aug)  # (10, N+1)

        # --- Augmented initial state: [x_sat(17), u_prev_0(nu)] ---
        # u_prev_0 = the actual control applied in the previous timestep.
        # At k>0 the augmented dynamics (u_prev_next = u_k) propagate u_prev
        # automatically — no per-stage parameter needed.
        u_prev_0 = np.zeros(self.nu)
        u_prev_0[self.num_rw_axes + self.num_thrusters] = self.path_speed
        if self._last_applied_u is not None:
            u_prev_0 = self._last_applied_u.copy()
        x_aug_full = np.concatenate([x_aug, u_prev_0])

        # --- Pin initial state (set both x guess and hard x0 bounds at stage 0) ---
        self._acados_solver.set(0, "x", x_aug_full)
        self._acados_solver.set(0, "lbx", x_aug_full)
        self._acados_solver.set(0, "ubx", x_aug_full)

        # --- Set per-stage parameters: [sat_params, p_ref, t_ref, q_ref] ---
        for k in range(self.N + 1):
            p_k = np.concatenate([self._casadi_params, refs[:, k]])
            self._acados_solver.set(k, "p", p_k)

        # --- Zero omega reference (omega_ref = 0 everywhere) -----------------------
        # omega_ref = path angular velocity was set here previously, which caused
        # monotonic spin-up (see detailed comment above W_term in _build_acados_solver).
        # W is already corrected at build time (omega weights ÷100/÷200 in the JSON).
        # Setting omega_ref = 0 lets the TERMINAL ATTITUDE cost drive all rotation.
        yref_stage = np.zeros(self._nr_stage)
        yref_term = np.zeros(self._nr_term)
        for k in range(self.N):
            self._acados_solver.set(k, "yref", yref_stage)
        self._acados_solver.set(self.N, "yref", yref_term)

        # --- Warm-start from previous solution (shifted trajectory) ---
        # IMPORTANT: stage 0 must always reflect the ACTUAL current state (x_aug_full).
        # We explicitly pin x0 via lbx/ubx above; keep warm-start from overwriting
        # stage 0 by starting this shift at k=1.
        if self._last_X_sol is not None:
            for k in range(
                1, self.N + 1
            ):  # k=0 already pinned above — do NOT overwrite
                src_k = min(k + 1, self.N)
                self._acados_solver.set(k, "x", self._last_X_sol[:, src_k])
            for k in range(self.N):
                src_k = min(k + 1, self.N - 1)
                self._acados_solver.set(k, "u", self._last_U_sol[:, src_k])
        else:
            for k in range(self.N + 1):
                self._acados_solver.set(
                    k, "x", x_aug_full
                )  # augmented init for all stages
            u_init = np.zeros(self.nu)
            u_init[self.num_rw_axes + self.num_thrusters] = self.path_speed
            for k in range(self.N):
                self._acados_solver.set(k, "u", u_init)

        # --- Solve ---
        u_opt = np.zeros(self.nu)
        solve_success = False
        acados_status = -1
        acados_iters = 0

        try:
            acados_status = self._acados_solver.solve()
            # 0 = optimal, 2 = max_iter reached but feasible — both usable
            solve_success = acados_status in (0, 2)

            if solve_success:
                u_opt = self._acados_solver.get(0, "u")
                self._last_applied_u = u_opt.copy()

                # Cache full trajectory for warm-start next step
                # X_sol rows are (nx_aug=17+nu) — augmented state with u_prev
                X_sol = np.zeros((self._nx_aug, self.N + 1))
                U_sol = np.zeros((self.nu, self.N))
                for k in range(self.N + 1):
                    X_sol[:, k] = self._acados_solver.get(k, "x")
                for k in range(self.N):
                    U_sol[:, k] = self._acados_solver.get(k, "u")
                self._last_X_sol = X_sol
                self._last_U_sol = U_sol
            else:
                # Solver failed (e.g. ACADOS_MINSTEP) — discard warm-start so
                # the next step initialises from scratch rather than from an
                # infeasible iterate.
                self._last_X_sol = None
                self._last_U_sol = None
                self._last_applied_u = np.zeros(self.nu)
                logger.debug(
                    "%s solver status %d at step %d",
                    self.__class__.__name__,
                    acados_status,
                    self._step_count,
                )

            try:
                stats = self._acados_solver.get_stats("sqp_iter")
                acados_iters = int(stats) if stats is not None else 0
            except Exception:
                acados_iters = 0

        except Exception as exc:
            logger.warning(
                "%s solve failed at step %d: %s",
                self.__class__.__name__,
                self._step_count,
                exc,
            )
            self._last_X_sol = None
            self._last_U_sol = None
            self._last_applied_u = np.zeros(self.nu)

        solve_time = time.perf_counter() - t_start
        self.solve_times.append(solve_time)

        # --- Path s advancement with safe fallback handling ---
        v_s = float(u_opt[self.num_rw_axes + self.num_thrusters])
        mode = self._runtime_mode.upper()
        if not solve_success:
            # Safe hold fallback: do not inject virtual progress.
            v_s = 0.0
            path_s = s_for_state
        elif mode not in {"SETTLE", "HOLD", "COMPLETE"}:
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
        path_s_proj: float | None = None
        path_error: float | None = None
        endpoint_error: float | None = None
        if self._path_set and len(self._path_data) >= 2:
            path_s_proj, _, path_error, endpoint_error = self._project_onto_path(
                x_aug[:3]
            )

        self._last_path_projection = {
            "s": path_s,
            "s_proj": path_s_proj,
            "path_error": path_error,
            "endpoint_error": endpoint_error,
            "s_pred": path_s + v_s * self._dt,
            "path_v_s": v_s,
        }

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
            "solver_fallback_reason": None if solve_success else "acados_failed",
            "iterations": acados_iters,
            "solve_time": solve_time,
            "time_limit_exceeded": False,
            "objective_value": None,
            "sqp_iterations": acados_iters,
            "sqp_kkt_residual": 0.0,
            "sqp_outer_iterations": acados_iters,
            "sqp_outer_residual": None,
            "sqp_outer_converged": solve_success,
            # acados-specific
            "acados_status": acados_status,
            "acados_iterations": acados_iters,
            # NMPC-compatible keys (zeroed — no IPOPT here)
            "ipopt_status": None,
            "ipopt_iterations": 0,
            # Fairness contract
            "shared_params_hash": self.shared_params_hash,
            "effective_params_hash": self.effective_params_hash,
            "override_diff": dict(self.profile_override_diff),
            "profile_specific_params": dict(self.profile_specific_params),
            # Path
            "path_s": path_s,
            "path_s_proj": float(path_s_proj) if path_s_proj is not None else None,
            "path_v_s": v_s,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": float(path_error) if path_error is not None else None,
            "path_endpoint_error": (
                float(endpoint_error) if endpoint_error is not None else None
            ),
            "path_s_pred": path_s + v_s * self._dt,
            # Timing breakdown
            "timing_linearization_s": 0.0,
            "timing_cost_update_s": 0.0,
            "timing_constraint_update_s": 0.0,
            "timing_matrix_update_s": 0.0,
            "timing_warmstart_s": 0.0,
            "timing_solve_only_s": solve_time,
            # Fallback / reference keys
            "fallback_active": not solve_success,
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
    # Reference trajectory builder (verbatim from NmpcController)
    # ------------------------------------------------------------------

    def _tangent_to_quat(
        self,
        tangent: np.ndarray,
        q_current: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Build a reference quaternion that aligns the body +X axis with `tangent`.

        Mirrors C++ SQPController::build_reference_quaternion exactly:
          - Priority 1: scan enabled → z_seed = scan axis
          - Priority 2: frame initialized → z_seed = _ref_prev_z
          - Priority 3: first call → bootstrap from current satellite z-axis
          - Y hemisphere continuity enforced via _ref_prev_y
          - Shortest-path flip to q_current
        """
        from controller.shared.python.utils.orientation_utils import (
            quat_wxyz_from_basis,
        )

        x_body = np.array(tangent, dtype=float)
        n = np.linalg.norm(x_body)
        x_body = x_body / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])

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

        z_body = z_seed - np.dot(z_seed, x_body) * x_body
        z_norm = np.linalg.norm(z_body)
        if z_norm <= 1e-6:
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

        if (
            self._ref_initialized
            and self._ref_prev_y is not None
            and np.linalg.norm(self._ref_prev_y) > 1e-9
        ):
            if float(np.dot(self._ref_prev_y, y_body)) < 0.0:
                y_body = -y_body
                z_body = -z_body

        q_ref = quat_wxyz_from_basis(x_body, y_body, z_body)

        if q_current is not None and np.dot(q_ref, q_current) < 0:
            q_ref = -q_ref

        self._ref_prev_z = z_body.copy()
        self._ref_prev_y = y_body.copy()
        self._ref_initialized = True

        return q_ref

    def _build_reference_trajectory(self, x_aug: np.ndarray) -> np.ndarray:
        """Build (10, N+1) reference array: [p_ref(3), t_ref(3), q_ref(4)] per stage.

        IMPORTANT: _tangent_to_quat() has persistent side-effects on _ref_prev_z,
        _ref_prev_y, _ref_initialized.  We must ensure that only the k=0 call
        updates these frame-continuity variables — the k=1..N calls are purely
        look-ahead and must not advance the persistent frame (otherwise the frame
        races N steps ahead each control step, driving the reference ~50x too fast).
        """
        refs = np.zeros((10, self.N + 1))
        refs[6, :] = 1.0  # default: identity quaternion

        if not self._path_set or len(self._path_data) < 2:
            return refs

        q_current = x_aug[3:7]

        # --- Stage k=0: update persistent frame (one step per control cycle) ---
        s_0 = min(self.s, self._path_length)
        p_ref_0, t_ref_0 = self._interpolate_path(s_0)
        q_ref_0 = self._tangent_to_quat(t_ref_0, q_current)
        refs[0:3, 0] = p_ref_0
        refs[3:6, 0] = t_ref_0
        refs[6:10, 0] = q_ref_0

        # Save the frame state after k=0 so look-ahead stages can compute without
        # permanently advancing _ref_prev_z / _ref_prev_y.
        saved_prev_z = self._ref_prev_z.copy() if self._ref_prev_z is not None else None
        saved_prev_y = self._ref_prev_y.copy() if self._ref_prev_y is not None else None
        saved_ref_init = self._ref_initialized

        # --- Stages k=1..N: look-ahead only (no persistent frame mutation) ---
        for k in range(1, self.N + 1):
            s_k = min(self.s + k * self._dt * self.path_speed, self._path_length)
            p_ref, t_ref = self._interpolate_path(s_k)
            q_ref = self._tangent_to_quat(t_ref, None)  # propagates from k-1 frame
            refs[0:3, k] = p_ref
            refs[3:6, k] = t_ref
            refs[6:10, k] = q_ref

        # Restore the persistent frame to the k=0 state so next control step
        # starts from the current satellite orientation, not the horizon end.
        self._ref_prev_z = saved_prev_z
        self._ref_prev_y = saved_prev_y
        self._ref_initialized = saved_ref_init

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

    @staticmethod
    def _quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """Convert wxyz quaternion to 3×3 rotation matrix (body→world)."""
        qw, qx, qy, qz = q
        return np.array(
            [
                [
                    1 - 2 * (qy * qy + qz * qz),
                    2 * (qx * qy - qz * qw),
                    2 * (qx * qz + qy * qw),
                ],
                [
                    2 * (qx * qy + qz * qw),
                    1 - 2 * (qx * qx + qz * qz),
                    2 * (qy * qz - qx * qw),
                ],
                [
                    2 * (qx * qz - qy * qw),
                    2 * (qy * qz + qx * qw),
                    1 - 2 * (qx * qx + qy * qy),
                ],
            ]
        )

    def _compute_omega_ref_traj(self, refs: np.ndarray) -> np.ndarray:
        """
        Compute required body-frame angular velocity at each horizon stage.

        When the path tangent rotates from t_k to t_{k+1} over one timestep dt,
        the satellite must spin at:
            ω_world ≈ cross(t_k, t_{k+1}) / dt
        Transformed to body frame via the reference quaternion q_ref_k:
            ω_body = R(q_ref_k)^T @ ω_world

        Args:
            refs: (10, N+1) reference array [p_ref(3), t_ref(3), q_ref(4)]
        Returns:
            omega_ref: (3, N+1) body-frame angular velocity reference
        """
        omega_ref = np.zeros((3, self.N + 1))
        for k in range(self.N):
            t_k = refs[3:6, k]
            t_k1 = refs[3:6, k + 1]
            q_k = refs[6:10, k]
            omega_world = np.cross(t_k, t_k1) / self._dt
            R = self._quat_to_rotation_matrix(q_k)
            omega_ref[:, k] = R.T @ omega_world
        omega_ref[:, self.N] = omega_ref[:, self.N - 1]  # terminal: extrapolate
        return omega_ref

    # ------------------------------------------------------------------
    # Path and mode interface (verbatim from NmpcController)
    # ------------------------------------------------------------------

    def set_path(self, path_points: list[tuple[float, float, float]]) -> None:
        if not path_points or len(path_points) < 2:
            logger.warning(
                "%s: path must have at least 2 points", self.__class__.__name__
            )
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
        self._last_applied_u = None
        self._ref_prev_z = None
        self._ref_prev_y = None
        self._ref_initialized = False
        logger.info(
            "%s path set: %d points, length=%.3fm",
            self.__class__.__name__,
            len(path_points),
            s,
        )

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
            self._ref_prev_z = None
            self._ref_prev_y = None
            self._ref_initialized = False
            return
        self._scan_attitude_enabled = True
        axis_arr = np.array(axis, dtype=float)
        n = np.linalg.norm(axis_arr)
        self._scan_attitude_target = (
            axis_arr / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])
        )

    def _project_onto_path(
        self, position: np.ndarray
    ) -> tuple[float, np.ndarray, float, float]:
        """
        Project position onto current polyline path.

        Returns:
            (s_projection, closest_point, contour_error, endpoint_error)
        """
        if not self._path_set or len(self._path_data) < 2:
            return 0.0, np.zeros(3, dtype=float), float("inf"), float("inf")

        pos = np.array(position, dtype=float).ravel()[:3]
        endpoint = np.array(self._path_data[-1][1:4], dtype=float)
        endpoint_error = float(np.linalg.norm(pos - endpoint))

        best_s = 0.0
        best_point = np.array(self._path_data[0][1:4], dtype=float)
        best_dist = float("inf")

        for i in range(len(self._path_data) - 1):
            s0 = float(self._path_data[i][0])
            s1 = float(self._path_data[i + 1][0])
            p0 = np.array(self._path_data[i][1:4], dtype=float)
            p1 = np.array(self._path_data[i + 1][1:4], dtype=float)
            seg = p1 - p0
            seg_norm_sq = float(np.dot(seg, seg))

            if seg_norm_sq <= 1e-12:
                proj = p0
                alpha = 0.0
            else:
                alpha = float(np.dot(pos - p0, seg) / seg_norm_sq)
                alpha = float(np.clip(alpha, 0.0, 1.0))
                proj = p0 + alpha * seg

            dist = float(np.linalg.norm(pos - proj))
            if dist < best_dist:
                best_dist = dist
                best_point = proj
                best_s = s0 + alpha * (s1 - s0)

        return best_s, best_point, best_dist, endpoint_error

    def get_path_progress(self, position: np.ndarray | None = None) -> dict[str, float]:
        if not self._path_set or len(self._path_data) < 2:
            return {
                "s": 0.0,
                "progress": 0.0,
                "remaining": 0.0,
                "path_error": float("inf"),
                "endpoint_error": float("inf"),
            }

        if position is None:
            s_val = float(self.s)
            path_error_raw = self._last_path_projection.get("path_error", float("inf"))
            endpoint_error_raw = self._last_path_projection.get(
                "endpoint_error", float("inf")
            )
            path_error = (
                float(path_error_raw) if path_error_raw is not None else float("inf")
            )
            endpoint_error = (
                float(endpoint_error_raw)
                if endpoint_error_raw is not None
                else float("inf")
            )
        else:
            s_val, _, path_error, endpoint_error = self._project_onto_path(position)

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        return {
            "s": float(s_val),
            "progress": float(s_val / path_len) if path_len > 1e-9 else 0.0,
            "remaining": float(max(0.0, path_len - s_val)),
            "path_error": float(path_error),
            "endpoint_error": float(endpoint_error),
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
        self._last_applied_u = None
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
