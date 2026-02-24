"""
MPC Controller — CasADi + OSQP RTI-SQP Backend.

Drop-in replacement for MPCController (legacy) that uses:
  - CasADi-generated exact Jacobians for dynamics linearisation
  - CasADi symbolic cost functions with exact Hessians
  - OSQP as the QP back-end inside an RTI-SQP loop

The public interface (Controller ABC) is identical to the original, so it plugs
straight into MPCRunner / simulation_loop without changes.
"""

import logging
import sys
from typing import Any

import numpy as np
from config.models import AppConfig

from .base import Controller

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# C++ backend import
# --------------------------------------------------------------------------
_IMPORT_ERROR: ImportError | None = None
try:
    from cpp import _cpp_mpc as _cpp_module

    SQPControllerCpp = _cpp_module.SQPController
    SatelliteParams = _cpp_module.SatelliteParams
    MPCParamsCpp = _cpp_module.MPCV2Params
except ImportError as exc:
    _IMPORT_ERROR = exc
    SQPControllerCpp = None  # type: ignore[assignment]
    SatelliteParams = None  # type: ignore[assignment]
    MPCParamsCpp = None  # type: ignore[assignment]

# --------------------------------------------------------------------------
# CasADi codegen (optional — gracefully skip if not yet generated)
# --------------------------------------------------------------------------
_CASADI_DYNAMICS = None
try:
    from .codegen.satellite_dynamics import SatelliteDynamicsSymbolic

    _CASADI_DYNAMICS = SatelliteDynamicsSymbolic
except ImportError:
    pass


def _raise_import_error() -> None:
    assert _IMPORT_ERROR is not None
    py_ver = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )
    raise RuntimeError(
        f"Failed to import MPC bindings (cpp._cpp_mpc). "
        f"Python {py_ver}. Original: {_IMPORT_ERROR}"
    ) from _IMPORT_ERROR


class MPCController(Controller):
    """
    RTI-SQP Satellite MPC Controller (CasADi + OSQP).

    State:  [p(3), q(4), v(3), ω(3), ω_rw(3), s(1)]  (17 elements)
    Control: [τ_rw(3), u_thr(N), v_s(1)]  (num_rw + num_thrusters + 1)
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(self, cfg: AppConfig):
        if _IMPORT_ERROR is not None:
            _raise_import_error()

        if not isinstance(cfg, AppConfig):
            raise TypeError(
                f"MPCController requires AppConfig, got {type(cfg).__name__}"
            )

        self._cfg = cfg
        self._extract_params(cfg)

        # Build C++ SatelliteParams (same struct as V1)
        sat = SatelliteParams()
        sat.dt = self._dt
        sat.mass = self.total_mass
        sat.inertia = self.moment_of_inertia
        sat.num_thrusters = self.num_thrusters
        sat.num_rw = self.num_rw_axes
        sat.thruster_positions = [np.array(p) for p in self.thruster_positions]
        sat.thruster_directions = [np.array(d) for d in self.thruster_directions]
        sat.thruster_forces = self.thruster_forces
        sat.rw_torque_limits = self.rw_torque_limits
        sat.rw_inertia = self.rw_inertia
        sat.rw_speed_limits = self.rw_speed_limits
        sat.rw_axes = [np.array(a) for a in self.rw_axes]
        sat.com_offset = self.com_offset
        sat.orbital_mean_motion = self.orbital_mean_motion
        sat.orbital_mu = self.orbital_mu
        sat.orbital_radius = self.orbital_radius
        sat.use_two_body = self.use_two_body
        self._sat_params = sat

        # Build MPC parameters
        mpc_p = MPCParamsCpp()
        mpc_p.prediction_horizon = self.N
        mpc_p.control_horizon = self.control_horizon
        mpc_p.dt = self._dt
        mpc_p.solver_time_limit = self.solver_time_limit
        mpc_p.verbose = bool(self.verbose_mpc)

        # Cost weights
        mpc_p.Q_contour = self.Q_contour
        mpc_p.Q_lag = self.Q_lag
        mpc_p.Q_progress = self.Q_progress
        mpc_p.progress_reward = float(self.progress_reward)
        mpc_p.Q_velocity_align = float(self.Q_velocity_align)
        mpc_p.Q_s_anchor = float(self.Q_s_anchor)
        mpc_p.Q_attitude = self.Q_attitude
        mpc_p.Q_axis_align = float(self.Q_axis_align)
        mpc_p.Q_quat_norm = float(self.Q_quat_norm)
        mpc_p.Q_angvel = self.Q_angvel
        mpc_p.R_thrust = self.R_thrust
        mpc_p.R_rw_torque = self.R_rw_torque
        mpc_p.Q_smooth = self.Q_smooth
        mpc_p.thrust_pair_weight = float(self.thrust_pair_weight)
        mpc_p.thrust_l1_weight = float(self.thrust_l1_weight)

        # Path
        mpc_p.path_speed = self.path_speed
        mpc_p.path_speed_min = float(self.path_speed_min)
        mpc_p.path_speed_max = float(self.path_speed_max)

        # Terminal
        mpc_p.Q_terminal_pos = self.Q_terminal_pos
        mpc_p.Q_terminal_s = self.Q_terminal_s
        mpc_p.enable_dare_terminal = bool(self.enable_online_dare_terminal)
        mpc_p.dare_update_period_steps = int(self.dare_update_period_steps)
        mpc_p.terminal_cost_profile = str(self.terminal_cost_profile)

        # Robustness
        mpc_p.robustness_mode = str(self.robustness_mode)
        mpc_p.constraint_tightening_scale = float(self.constraint_tightening_scale)
        mpc_p.tube_feedback_gain_scale = float(self.tube_feedback_gain_scale)
        mpc_p.tube_feedback_max_correction = float(self.tube_feedback_max_correction)

        # Bounds
        mpc_p.max_linear_velocity = float(self.max_linear_velocity)
        mpc_p.max_angular_velocity = float(self.max_angular_velocity)

        # Progress policy
        mpc_p.progress_policy = str(self.progress_policy)
        mpc_p.error_priority_min_vs = float(self.error_priority_min_vs)
        mpc_p.error_priority_error_speed_gain = float(
            self.error_priority_error_speed_gain
        )

        # OSQP
        mpc_p.osqp_max_iter = 4000
        mpc_p.osqp_eps_abs = 5e-3
        mpc_p.osqp_eps_rel = 5e-3
        mpc_p.osqp_warm_start = True

        # Mode scaling
        mpc_p.recover_contour_scale = float(self.recover_contour_scale)
        mpc_p.recover_lag_scale = float(self.recover_lag_scale)
        mpc_p.recover_progress_scale = float(self.recover_progress_scale)
        mpc_p.recover_attitude_scale = float(self.recover_attitude_scale)
        mpc_p.settle_progress_scale = float(self.settle_progress_scale)
        mpc_p.settle_terminal_pos_scale = float(self.settle_terminal_pos_scale)
        mpc_p.settle_terminal_attitude_scale = float(
            self.settle_terminal_attitude_scale
        )
        mpc_p.settle_velocity_align_scale = float(self.settle_velocity_align_scale)
        mpc_p.settle_angular_velocity_scale = float(self.settle_angular_velocity_scale)
        mpc_p.hold_smoothness_scale = float(self.hold_smoothness_scale)
        mpc_p.hold_thruster_pair_scale = float(self.hold_thruster_pair_scale)
        mpc_p.solver_fallback_hold_s = float(self.solver_fallback_hold_s)
        mpc_p.solver_fallback_decay_s = float(self.solver_fallback_decay_s)
        mpc_p.solver_fallback_zero_after_s = float(self.solver_fallback_zero_after_s)

        self._cpp = SQPControllerCpp(sat, mpc_p)

        # Build CasADi dynamics evaluator (if available)
        self._dynamics = None
        self._f_and_jacs = None
        if _CASADI_DYNAMICS is not None:
            try:
                self._dynamics = _CASADI_DYNAMICS(
                    num_thrusters=self.num_thrusters,
                    num_rw=self.num_rw_axes,
                )
                self._f_and_jacs = self._dynamics.f_and_jacs
                logger.info("CasADi dynamics evaluator initialised.")
            except Exception:
                logger.warning(
                    "CasADi dynamics init failed — C++ will use internal linearisation",
                    exc_info=True,
                )

        self._casadi_params = np.array(self._cpp.casadi_params, dtype=float)

        # Tracking
        self.solve_times: list[float] = []
        self.s = 0.0
        self._path_data: list[list[float]] = []
        self._path_set = False
        self._path_length = 0.0
        self._last_path_projection: dict[str, Any] = {}
        self._scan_attitude_enabled = False
        self._runtime_mode = "TRACK"
        self._fallback_log_interval_s = 5.0
        self._last_fallback_log_at: dict[str, float] = {}

        self.nx = 17
        self.nu = self.num_rw_axes + self.num_thrusters

        # Convenience scalar used by legacy code paths for RW denormalization.
        # Per-wheel limits are in self.rw_torque_limits (preferred).
        self.max_rw_torque = (
            max(self.rw_torque_limits) if self.rw_torque_limits else 0.0
        )

        logger.info(
            "MPC Controller init (RTI-SQP, CasADi+OSQP). "
            "Thrusters=%d, RW=%d, N=%d, M=%d, dt=%.3f",
            self.num_thrusters,
            self.num_rw_axes,
            self.N,
            self.control_horizon,
            self._dt,
        )

    # ------------------------------------------------------------------
    # Controller ABC
    # ------------------------------------------------------------------

    @property
    def dt(self) -> float:
        return self._dt

    @property
    def prediction_horizon(self) -> int:
        return self.N

    def get_control_action(
        self,
        x_current: np.ndarray,
        previous_thrusters: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Compute optimal control via RTI-SQP (CasADi linearise → C++ QP)."""

        # --- state augmentation ---
        s_for_state = float(self.s)
        if self._path_set:
            try:
                s_for_state = float(self._cpp.current_path_s)
                self.s = s_for_state
            except Exception:
                pass

        if len(x_current) == 16:
            x_input = np.append(x_current, s_for_state)
        elif len(x_current) == 17:
            x_input = np.array(x_current, dtype=float).copy()
            x_input[16] = s_for_state
        else:
            x_input = x_current

        if previous_thrusters is not None:
            try:
                self._cpp.set_warm_start_control(
                    np.array(previous_thrusters, dtype=float)
                )
            except Exception:
                pass

        # --- CasADi linearisation (Python-side) ---
        if self._f_and_jacs is not None:
            self._evaluate_casadi_linearisation(x_input)

        # --- Solve QP in C++ ---
        result = self._cpp.get_control_action(x_input)

        self.solve_times.append(result.solve_time)

        u_out = np.array(result.u)
        v_s = float(u_out[-1]) if u_out.size > 0 else 0.0

        path_s = float(getattr(result, "path_s", s_for_state))
        path_s_proj = getattr(result, "path_s_proj", None)
        path_error = getattr(result, "path_error", None)
        endpoint_error = getattr(result, "path_endpoint_error", None)
        path_s_pred_raw = getattr(result, "path_s_pred", None)
        path_s_pred = (
            float(path_s_pred_raw)
            if path_s_pred_raw is not None
            else (path_s + v_s * self._dt)
        )

        self.s = path_s
        self._last_path_projection = {
            "s": path_s,
            "s_proj": float(path_s_proj) if path_s_proj is not None else None,
            "path_error": float(path_error) if path_error is not None else None,
            "endpoint_error": float(endpoint_error)
            if endpoint_error is not None
            else None,
            "s_pred": path_s_pred,
            "path_v_s": v_s,
        }

        # Strip virtual control
        u_phys = u_out[:-1] if u_out.size > 0 else u_out

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        progress = float(path_s / path_len) if path_len > 1e-9 else 0.0
        remaining = float(path_len - path_s) if path_len > 0 else 0.0
        status_code = int(getattr(result, "status", -1))
        solver_status = getattr(result, "solver_status", None)
        timeout = bool(getattr(result, "timeout", False))
        solve_time = float(getattr(result, "solve_time", 0.0))
        time_limit_exceeded = timeout or (solve_time > float(self.solver_time_limit))

        extras: dict[str, Any] = {
            "status": status_code,
            "status_name": "SUCCESS" if status_code == 1 else "FAILED",
            "solver_status": solver_status,
            "timeout": timeout,
            "solve_time": solve_time,
            "iterations": getattr(result, "iterations", None),
            "objective_value": getattr(result, "objective", None),
            "solver_type": "RTI-SQP",
            "solver_backend": "CasADi+OSQP",
            "solver_time_limit": self.solver_time_limit,
            "solver_fallback": status_code != 1,
            "solver_fallback_reason": self._classify_fallback_reason(
                status_code, solver_status, timeout, time_limit_exceeded
            ),
            "solver_success": status_code == 1,
            "time_limit_exceeded": time_limit_exceeded,
            "fallback_active": bool(getattr(result, "fallback_active", False)),
            "fallback_age_s": float(getattr(result, "fallback_age_s", 0.0)),
            "fallback_scale": float(getattr(result, "fallback_scale", 0.0)),
            "timing_linearization_s": float(getattr(result, "t_linearization_s", 0.0)),
            "timing_cost_update_s": float(getattr(result, "t_cost_update_s", 0.0)),
            "timing_constraint_update_s": float(
                getattr(result, "t_constraint_update_s", 0.0)
            ),
            "timing_matrix_update_s": float(getattr(result, "t_matrix_update_s", 0.0)),
            "timing_warmstart_s": float(getattr(result, "t_warmstart_s", 0.0)),
            "timing_solve_only_s": float(getattr(result, "t_solve_only_s", 0.0)),
            "sqp_iterations": getattr(result, "sqp_iterations", 1),
            "sqp_kkt_residual": getattr(result, "sqp_kkt_residual", 0.0),
            "controller_core": "v2-sqp",
            "path_s": path_s,
            "path_s_proj": float(path_s_proj) if path_s_proj is not None else None,
            "path_v_s": v_s,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": float(path_error) if path_error is not None else None,
            "path_endpoint_error": float(endpoint_error)
            if endpoint_error is not None
            else None,
            "path_s_pred": path_s_pred,
        }
        return u_phys, extras

    def reset(self) -> None:
        """Reset controller state."""
        self.solve_times.clear()
        self.s = 0.0
        self._path_data.clear()
        self._path_set = False
        self._path_length = 0.0
        self._last_path_projection.clear()

    def get_solver_stats(self) -> dict[str, Any]:
        if not self.solve_times:
            return {"solve_count": 0, "average_solve_time": 0.0, "max_solve_time": 0.0}
        return {
            "solve_times": self.solve_times.copy(),
            "solve_count": len(self.solve_times),
            "average_solve_time": sum(self.solve_times) / len(self.solve_times),
            "max_solve_time": max(self.solve_times),
        }

    # ------------------------------------------------------------------
    # Path & mode configuration (same interface as V1)
    # ------------------------------------------------------------------

    def set_path(self, path_points: list[tuple[float, float, float]]) -> None:
        if not path_points or len(path_points) < 2:
            logger.warning("Path must have at least 2 points")
            return

        self._path_data = []
        s = 0.0
        prev = None
        for pt in path_points:
            if prev is not None:
                dx, dy, dz = pt[0] - prev[0], pt[1] - prev[1], pt[2] - prev[2]
                s += (dx**2 + dy**2 + dz**2) ** 0.5
            self._path_data.append([s, pt[0], pt[1], pt[2]])
            prev = pt

        self._path_length = s
        self._cpp.set_path_data(self._path_data)
        self._path_set = True
        self.s = 0.0
        logger.info("Path set: %d pts, length=%.3fm", len(path_points), s)

    def set_scan_attitude_context(
        self,
        center: tuple[float, float, float] | None,
        axis: tuple[float, float, float] | None,
        direction: str = "CW",
    ) -> None:
        if axis is None:
            self._cpp.clear_scan_attitude_context()
            self._scan_attitude_enabled = False
            return
        c = np.array(center if center else [np.nan] * 3, dtype=float)
        a = np.array(axis, dtype=float)
        self._cpp.set_scan_attitude_context(c, a, str(direction))
        self._scan_attitude_enabled = True

    def set_runtime_mode(self, mode: str | None) -> None:
        mode_name = str(mode or "TRACK").upper()
        self._runtime_mode = mode_name
        try:
            self._cpp.set_runtime_mode(mode_name)
        except Exception:
            logger.debug("Failed to set runtime mode in SQP core", exc_info=True)

    def get_path_progress(self, position: np.ndarray | None = None) -> dict[str, float]:
        if not self._path_data or len(self._path_data) < 2:
            return {
                "s": 0.0,
                "progress": 0.0,
                "remaining": 0.0,
                "path_error": float("inf"),
                "endpoint_error": float("inf"),
            }

        if position is None:
            s_val = float(self.s)
            pe = float(self._last_path_projection.get("path_error", float("inf")))
            ee = float(self._last_path_projection.get("endpoint_error", float("inf")))
        else:
            pos = np.array(position, dtype=float).ravel()[:3]
            try:
                s_val, _, pe, ee = self._cpp.project_onto_path(pos)
            except Exception:
                s_val, pe, ee = 0.0, float("inf"), float("inf")

        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        return {
            "s": float(s_val),
            "progress": float(s_val / path_len) if path_len > 1e-9 else 0.0,
            "remaining": float(path_len - s_val) if path_len > 0 else 0.0,
            "path_error": float(pe),
            "endpoint_error": float(ee),
        }

    def get_path_reference(
        self, s_query: float | None = None
    ) -> tuple[np.ndarray, np.ndarray]:
        q_guess = np.array([1.0, 0.0, 0.0, 0.0])
        pos, tan, _ = self.get_path_reference_state(s_query=s_query, q_current=q_guess)
        return pos, tan

    def get_path_reference_state(
        self,
        s_query: float | None = None,
        q_current: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self._path_data or len(self._path_data) < 2:
            return (np.zeros(3), np.zeros(3), np.array([1, 0, 0, 0], dtype=float))

        s_val = float(self.s if s_query is None else s_query)
        s_val = max(0.0, min(s_val, float(self._path_length)))
        q_curr = np.array(
            q_current if q_current is not None else [1, 0, 0, 0], dtype=float
        )

        try:
            pos, tangent, q_ref = self._cpp.get_reference_at_s(s_val, q_curr)
            return np.asarray(pos), np.asarray(tangent), np.asarray(q_ref)
        except Exception:
            return np.zeros(3), np.zeros(3), q_curr

    def split_control(self, control: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        return control[: self.num_rw_axes], control[self.num_rw_axes :]

    # ------------------------------------------------------------------
    # CasADi linearisation bridge
    # ------------------------------------------------------------------

    def _evaluate_casadi_linearisation(self, x_current: np.ndarray) -> None:
        """Evaluate CasADi dynamics at each horizon stage and inject into C++."""
        if self._f_and_jacs is None:
            return

        N = self._cpp.prediction_horizon
        p = self._casadi_params

        for k in range(N):
            x_k = np.array(self._cpp.get_stage_state(k), dtype=float)
            u_k = np.array(self._cpp.get_stage_control(k), dtype=float)

            # On first solve, stage states may be all-zero — use current
            # state as fallback to avoid gravitational singularity at r=0.
            if np.linalg.norm(x_k[:3]) < 1e-6:
                x_k = np.array(x_current, dtype=float)

            # CasADi f_and_jacs returns: (x_next, A_k, B_k)
            try:
                result = self._f_and_jacs(x_k, u_k, p, self._dt)
                x_next = np.array(result[0]).ravel()
                A_k = np.array(result[1])
                B_k = np.array(result[2])

                # Guard: skip injection if any matrix contains NaN/Inf
                if not (
                    np.all(np.isfinite(A_k))
                    and np.all(np.isfinite(B_k))
                    and np.all(np.isfinite(x_next))
                ):
                    continue

                # Affine term: d = x_next - A_k @ x_k - B_k @ u_k
                d_k = x_next - A_k @ x_k - B_k @ u_k

                self._cpp.set_stage_linearisation(k, A_k, B_k, d_k)
            except Exception:
                logger.debug("CasADi eval failed at stage %d", k, exc_info=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _classify_fallback_reason(
        status: int, solver_status: Any, timeout: bool, time_limit_exceeded: bool
    ) -> str | None:
        if int(status) == 1:
            return None
        if timeout or time_limit_exceeded:
            return "solver_timeout"
        if solver_status is not None:
            return f"solver_non_success_{solver_status}"
        return "solver_non_success"

    # ------------------------------------------------------------------
    # Config extraction  (mirrors V1)
    # ------------------------------------------------------------------

    def _extract_params(self, cfg: AppConfig) -> None:
        physics = cfg.physics
        mpc = cfg.mpc
        mpc_core = cfg.mpc_core
        contracts = cfg.controller_contracts

        self.total_mass = physics.total_mass
        I_val = physics.moment_of_inertia
        self.moment_of_inertia = np.array([I_val, I_val, I_val], dtype=float)
        self.com_offset = np.array(physics.com_offset)

        sorted_ids = sorted(physics.thruster_positions.keys())
        self.num_thrusters = len(sorted_ids)
        self.thruster_positions = [physics.thruster_positions[i] for i in sorted_ids]
        self.thruster_directions = [physics.thruster_directions[i] for i in sorted_ids]
        self.thruster_forces = [physics.thruster_forces[i] for i in sorted_ids]

        if physics.reaction_wheels:
            self.num_rw_axes = len(physics.reaction_wheels)
            self.rw_torque_limits = [
                float(rw.max_torque) for rw in physics.reaction_wheels
            ]
            self.rw_inertia = [float(rw.inertia) for rw in physics.reaction_wheels]
            self.rw_speed_limits = [
                float(getattr(rw, "max_speed", 0.0)) for rw in physics.reaction_wheels
            ]
            self.rw_axes = []
            for rw in physics.reaction_wheels:
                axis = np.array(getattr(rw, "axis", (1, 0, 0)), dtype=float)
                n = np.linalg.norm(axis)
                self.rw_axes.append(
                    axis / n if n > 1e-12 else np.array([1, 0, 0], dtype=float)
                )
        else:
            self.num_rw_axes = 0
            self.rw_torque_limits = []
            self.rw_inertia = []
            self.rw_speed_limits = []
            self.rw_axes = []

        self.N = mpc.prediction_horizon
        self._dt = mpc.dt
        self.solver_time_limit = mpc.solver_time_limit
        self.control_horizon = max(1, min(int(mpc.control_horizon), self.N))
        self.verbose_mpc = mpc.verbose_mpc

        self.Q_contour = mpc.Q_contour
        # Auto-resolve Q_lag: when 0 (default), use Q_lag_default
        _q_lag = mpc.Q_lag
        if _q_lag <= 0.0:
            _q_lag = float(getattr(mpc, "Q_lag_default", 0.0) or 0.0)
        self.Q_lag = _q_lag
        self.Q_progress = mpc.Q_progress
        self.progress_reward = mpc.progress_reward
        self.Q_velocity_align = mpc.Q_velocity_align
        self.Q_s_anchor = mpc.Q_s_anchor
        self.Q_smooth = mpc.Q_smooth
        self.Q_terminal_pos = mpc.Q_terminal_pos
        self.Q_terminal_s = mpc.Q_terminal_s
        self.Q_angvel = mpc.q_angular_velocity
        self.Q_attitude = mpc.Q_attitude
        self.Q_axis_align = mpc.Q_axis_align
        self.Q_quat_norm = mpc.Q_quat_norm
        self.R_thrust = mpc.r_thrust
        self.R_rw_torque = mpc.r_rw_torque
        self.thrust_l1_weight = mpc.thrust_l1_weight
        self.thrust_pair_weight = mpc.thrust_pair_weight
        self.max_linear_velocity = mpc.max_linear_velocity
        self.max_angular_velocity = mpc.max_angular_velocity
        self.enable_online_dare_terminal = mpc.enable_online_dare_terminal
        self.dare_update_period_steps = mpc.dare_update_period_steps
        self.terminal_cost_profile = mpc.terminal_cost_profile
        self.robustness_mode = mpc.robustness_mode
        self.constraint_tightening_scale = mpc.constraint_tightening_scale
        self.tube_feedback_gain_scale = mpc.tube_feedback_gain_scale
        self.tube_feedback_max_correction = mpc.tube_feedback_max_correction
        self.progress_policy = mpc.progress_policy
        self.error_priority_min_vs = mpc.error_priority_min_vs
        self.error_priority_error_speed_gain = mpc.error_priority_error_speed_gain
        self.path_speed = mpc.path_speed
        self.path_speed_min = mpc.path_speed_min
        self.path_speed_max = mpc.path_speed_max

        self.recover_contour_scale = mpc_core.recover_contour_scale
        self.recover_lag_scale = mpc_core.recover_lag_scale
        self.recover_progress_scale = mpc_core.recover_progress_scale
        self.recover_attitude_scale = mpc_core.recover_attitude_scale
        self.settle_progress_scale = mpc_core.settle_progress_scale
        self.settle_terminal_pos_scale = mpc_core.settle_terminal_pos_scale
        self.settle_terminal_attitude_scale = mpc_core.settle_terminal_attitude_scale
        self.settle_velocity_align_scale = mpc_core.settle_velocity_align_scale
        self.settle_angular_velocity_scale = mpc_core.settle_angular_velocity_scale
        self.hold_smoothness_scale = mpc_core.hold_smoothness_scale
        self.hold_thruster_pair_scale = mpc_core.hold_thruster_pair_scale
        self.solver_fallback_hold_s = contracts.solver_fallback_hold_s
        self.solver_fallback_decay_s = contracts.solver_fallback_decay_s
        self.solver_fallback_zero_after_s = contracts.solver_fallback_zero_after_s

        try:
            from physics.orbital_config import OrbitalConfig

            orb = OrbitalConfig()
            self.orbital_mu = float(orb.mu)
            self.orbital_radius = float(orb.orbital_radius)
            self.orbital_mean_motion = float(orb.mean_motion)
            self.use_two_body = True
        except Exception:
            self.orbital_mu = 3.986004418e14
            self.orbital_radius = 6.778e6
            self.orbital_mean_motion = 0.0
            self.use_two_body = True
