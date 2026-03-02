"""
MPC Controller — CasADi + OSQP RTI-SQP Backend.

Drop-in replacement for MPCController (legacy) that uses:
  - CasADi-generated exact Jacobians for dynamics linearisation
  - CasADi symbolic cost functions with exact Hessians
  - OSQP as the QP back-end inside an RTI-SQP loop

The public interface (Controller ABC) is identical to the original, so it plugs
straight into MPCRunner / simulation_loop without changes.
"""

from __future__ import annotations

import hashlib
import json
import logging
import sys
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass
from types import MappingProxyType
from typing import Any

import numpy as np

from controller.configs.models import AppConfig

from .base import Controller
from .profile_params import (
    EffectiveMPCProfileContract,
    resolve_effective_mpc_profile_contract,
)

logger = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# OSQP solver defaults
# --------------------------------------------------------------------------
_OSQP_MAX_ITER_DEFAULT: int = 4000
_OSQP_EPS_DEFAULT: float = 5e-3

# --------------------------------------------------------------------------
# C++ backend import
# --------------------------------------------------------------------------
_DEFAULT_CPP_MPC_MODULE = "_cpp_mpc"

# --------------------------------------------------------------------------
# CasADi codegen (optional — gracefully skip if not yet generated)
# --------------------------------------------------------------------------
_CASADI_DYNAMICS = None
try:
    from .codegen.satellite_dynamics import SatelliteDynamicsSymbolic

    _CASADI_DYNAMICS = SatelliteDynamicsSymbolic
except ImportError:
    pass


def _resolve_cpp_bindings(module_name: str) -> tuple[type, type, type, str]:
    """
    Resolve C++ MPC bindings for a profile-specific module with safe fallback.

    Returns:
        (SatelliteParams class, MPCV2Params class, SQPController class, loaded module name)
    """
    from controller.shared.python import cpp as cpp_extensions

    requested = str(module_name or _DEFAULT_CPP_MPC_MODULE).strip()
    candidates: list[str] = [requested]
    if requested != _DEFAULT_CPP_MPC_MODULE:
        candidates.append(_DEFAULT_CPP_MPC_MODULE)

    import_errors: list[str] = []
    for candidate in candidates:
        module = getattr(cpp_extensions, candidate, None)
        if module is None:
            import_errors.append(f"{candidate}: module not loaded")
            continue
        try:
            sat_cls = module.SatelliteParams
            mpc_params_cls = module.MPCV2Params
            controller_cls = module.SQPController
            return sat_cls, mpc_params_cls, controller_cls, candidate
        except Exception as exc:
            import_errors.append(f"{candidate}: {exc}")

    py_ver = (
        f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    )
    tried = ", ".join(candidates)
    details = "; ".join(import_errors) if import_errors else "no modules attempted"
    raise RuntimeError(
        f"Failed to resolve MPC bindings for modules [{tried}] on Python {py_ver}. "
        f"Details: {details}"
    )


def _stable_payload_hash(payload: dict[str, Any]) -> str:
    """Deterministic hash used to guard shared MPC contract integrity."""
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _freeze_payload(value: Any) -> Any:
    """Recursively freeze payload values to prevent in-process mutation."""
    if isinstance(value, dict):
        frozen = {key: _freeze_payload(val) for key, val in value.items()}
        return MappingProxyType(frozen)
    if isinstance(value, list):
        return tuple(_freeze_payload(item) for item in value)
    return value


@dataclass(frozen=True)
class SharedMPCContract:
    """Immutable shared-parameter contract consumed by all controller profiles."""

    physics: MappingProxyType[str, Any]
    mpc: MappingProxyType[str, Any]
    mpc_core: MappingProxyType[str, Any]
    controller_contracts: MappingProxyType[str, Any]
    signature: str

    @classmethod
    def from_app_config(cls, cfg: AppConfig) -> SharedMPCContract:
        mpc_core_payload = cfg.mpc_core.model_dump()
        # Selector is routing metadata, not part of fairness-tunable MPC knobs.
        mpc_core_payload.pop("controller_profile", None)
        payload = {
            "physics": cfg.physics.model_dump(),
            "mpc": cfg.mpc.model_dump(),
            "mpc_core": mpc_core_payload,
            "controller_contracts": cfg.controller_contracts.model_dump(),
        }
        return cls(
            physics=_freeze_payload(payload["physics"]),
            mpc=_freeze_payload(payload["mpc"]),
            mpc_core=_freeze_payload(payload["mpc_core"]),
            controller_contracts=_freeze_payload(payload["controller_contracts"]),
            signature=_stable_payload_hash(payload),
        )


@dataclass
class LinearizationInfo:
    """Structured per-step linearization telemetry."""

    linearization_mode: str
    attempted_stages: int = 0
    failed_stages: int = 0
    integrity_failure: bool = False
    integrity_reason: str | None = None
    strict_mode: bool = False
    used_stale_fallback: bool = False


class _LinearizationStrategy(ABC):
    """Profile-specific linearization behavior."""

    linearization_mode: str

    @abstractmethod
    def prepare_linearization(
        self, controller: MPCController, x_current: np.ndarray
    ) -> LinearizationInfo:
        """Evaluate and inject linearization data for the upcoming control solve."""

    @abstractmethod
    def handle_linearization_failure(
        self, controller: MPCController, info: LinearizationInfo
    ) -> bool:
        """Return True when execution should fail closed before solving."""

    def annotate_solver_metadata(
        self,
        extras: dict[str, Any],
        info: LinearizationInfo,
    ) -> None:
        extras["linearization_mode"] = info.linearization_mode
        extras["linearization_attempted_stages"] = int(info.attempted_stages)
        extras["linearization_failed_stages"] = int(info.failed_stages)
        extras["linearization_integrity_failure"] = bool(info.integrity_failure)
        extras["linearization_integrity_reason"] = info.integrity_reason
        extras["linearization_used_stale_fallback"] = bool(info.used_stale_fallback)


class _HybridLinearizationStrategy(_LinearizationStrategy):
    linearization_mode = "hybrid_tolerant_stage"

    def prepare_linearization(
        self, controller: MPCController, x_current: np.ndarray
    ) -> LinearizationInfo:
        return controller._evaluate_casadi_linearisation_stagewise(
            x_current=x_current,
            strict_integrity=not bool(
                getattr(controller, "allow_stale_stage_reuse", True)
            ),
            linearization_mode=self.linearization_mode,
        )

    def handle_linearization_failure(
        self, controller: MPCController, info: LinearizationInfo
    ) -> bool:
        return False


class _NonlinearLinearizationStrategy(_LinearizationStrategy):
    linearization_mode = "nonlinear_exact_stage"

    def prepare_linearization(
        self, controller: MPCController, x_current: np.ndarray
    ) -> LinearizationInfo:
        return controller._evaluate_casadi_linearisation_stagewise(
            x_current=x_current,
            strict_integrity=bool(
                getattr(controller, "strict_linearization_integrity", True)
            ),
            linearization_mode=self.linearization_mode,
        )

    def handle_linearization_failure(
        self, controller: MPCController, info: LinearizationInfo
    ) -> bool:
        return bool(info.integrity_failure)


class _LinearFrozenLinearizationStrategy(_LinearizationStrategy):
    linearization_mode = "linear_frozen_step"

    def prepare_linearization(
        self, controller: MPCController, x_current: np.ndarray
    ) -> LinearizationInfo:
        return controller._evaluate_casadi_linearisation_frozen_step(
            x_current=x_current,
            linearization_mode=self.linearization_mode,
        )

    def handle_linearization_failure(
        self, controller: MPCController, info: LinearizationInfo
    ) -> bool:
        return False


class MPCController(Controller):
    """
    RTI-SQP Satellite MPC Controller (CasADi + OSQP).

    State:  [p(3), q(4), v(3), ω(3), ω_rw(3), s(1)]  (17 elements)
    Control: [τ_rw(3), u_thr(N), v_s(1)]  (num_rw + num_thrusters + 1)
    """

    controller_profile: str = "hybrid"
    controller_core: str = "v2-sqp"
    solver_type: str = "RTI-SQP"
    solver_backend: str = "CasADi+OSQP"
    linearization_mode: str = "hybrid_tolerant_stage"
    cpp_module_name: str = _DEFAULT_CPP_MPC_MODULE

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------

    def __init__(self, cfg: AppConfig):
        if not isinstance(cfg, AppConfig):
            raise TypeError(
                f"MPCController requires AppConfig, got {type(cfg).__name__}"
            )

        self._cfg = cfg
        # Profile metadata can be overridden by subclasses/factory adapters.
        self.controller_profile = str(
            getattr(self, "controller_profile", self.__class__.controller_profile)
        )
        self.controller_core = str(
            getattr(self, "controller_core", self.__class__.controller_core)
        )
        self.solver_type = str(getattr(self, "solver_type", self.__class__.solver_type))
        self.solver_backend = str(
            getattr(self, "solver_backend", self.__class__.solver_backend)
        )
        self.linearization_mode = str(
            getattr(self, "linearization_mode", self.__class__.linearization_mode)
        )
        self.cpp_module_name = str(
            getattr(self, "cpp_module_name", self.__class__.cpp_module_name)
        )

        self.effective_contract: EffectiveMPCProfileContract = (
            resolve_effective_mpc_profile_contract(
                cfg=cfg,
                profile=self.controller_profile,
            )
        )
        self.shared_params_hash = str(self.effective_contract.shared_signature)
        self.effective_params_hash = str(self.effective_contract.effective_signature)
        self.profile_override_diff = dict(self.effective_contract.override_diff)
        self.profile_specific_params = dict(self.effective_contract.profile_specific)
        self._effective_contract_signature = self.effective_params_hash
        self.shared_contract = SharedMPCContract.from_app_config(cfg)
        self._shared_contract_signature = self.shared_contract.signature
        self._extract_params(
            cfg,
            effective_mpc=dict(self.effective_contract.effective_mpc),
            profile_specific=dict(self.effective_contract.profile_specific),
        )
        (
            self._satellite_params_cls,
            self._mpc_params_cls,
            self._sqp_controller_cls,
            self._cpp_backend_module,
        ) = _resolve_cpp_bindings(self.cpp_module_name)
        self._linearization_strategy = self._build_linearization_strategy(
            self.linearization_mode
        )

        # Build C++ SatelliteParams (same struct as V1)
        sat = self._satellite_params_cls()
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
        mpc_p = self._mpc_params_cls()
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
        mpc_p.ref_tangent_lookahead_m = float(self.ref_tangent_lookahead_m)
        mpc_p.ref_tangent_lookback_m = float(self.ref_tangent_lookback_m)
        mpc_p.ref_quat_max_rate_rad_s = float(self.ref_quat_max_rate_rad_s)
        mpc_p.ref_quat_terminal_rate_scale = float(self.ref_quat_terminal_rate_scale)

        # Terminal
        mpc_p.Q_terminal_pos = self.Q_terminal_pos
        mpc_p.Q_terminal_s = self.Q_terminal_s
        mpc_p.Q_terminal_vel = float(self.Q_terminal_vel)
        mpc_p.enable_dare_terminal = bool(self.enable_online_dare_terminal)
        mpc_p.dare_update_period_steps = int(self.dare_update_period_steps)
        mpc_p.terminal_cost_profile = str(self.terminal_cost_profile)

        # Bounds
        mpc_p.max_linear_velocity = float(self.max_linear_velocity)
        mpc_p.max_angular_velocity = float(self.max_angular_velocity)

        # Progress policy
        mpc_p.progress_policy = str(self.progress_policy)
        mpc_p.error_priority_min_vs = float(self.error_priority_min_vs)
        mpc_p.error_priority_error_speed_gain = float(
            self.error_priority_error_speed_gain
        )

        # SQP
        mpc_p.sqp_max_iter = int(self.sqp_max_iter)
        mpc_p.sqp_tol = float(self.sqp_tol)

        # OSQP
        mpc_p.osqp_max_iter = _OSQP_MAX_ITER_DEFAULT
        mpc_p.osqp_eps_abs = _OSQP_EPS_DEFAULT
        mpc_p.osqp_eps_rel = _OSQP_EPS_DEFAULT
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

        self._cpp = self._sqp_controller_cls(sat, mpc_p)

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

        # Tracking (bounded ring-buffer — retains most recent 10 000 solve times)
        self.solve_times: deque[float] = deque(maxlen=10_000)
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
        self._linear_frozen_cache: (
            tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]] | None
        ) = None
        self._linear_step_counter = 0

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

    def _build_linearization_strategy(
        self, linearization_mode: str
    ) -> _LinearizationStrategy:
        if linearization_mode == "nonlinear_exact_stage":
            return _NonlinearLinearizationStrategy()
        if linearization_mode == "linear_frozen_step":
            return _LinearFrozenLinearizationStrategy()
        return _HybridLinearizationStrategy()

    def _assert_shared_contract_integrity(self) -> None:
        if self.shared_contract.signature != self._shared_contract_signature:
            raise RuntimeError("Shared MPC contract integrity guard failed.")
        if (
            self.effective_contract.effective_signature
            != self._effective_contract_signature
        ):
            raise RuntimeError("Effective MPC profile contract integrity guard failed.")

    def get_control_action(
        self,
        x_current: np.ndarray,
        previous_thrusters: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Compute optimal control via RTI-SQP (CasADi linearise → C++ QP)."""
        self._assert_shared_contract_integrity()

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

        # --- CasADi linearisation (Python-side) + profile-specific solve orchestration ---
        linearization_info = LinearizationInfo(
            linearization_mode=self.linearization_mode
        )
        nonlinear_outer_iterations = 1
        nonlinear_outer_residual: float | None = None
        nonlinear_outer_converged = False
        result = None

        # Nonlinear profile: run a true outer loop in Python orchestration.
        # Each iteration re-evaluates stage Jacobians, solves once, and
        # warm-starts the next iteration.
        if (
            self.linearization_mode == "nonlinear_exact_stage"
            and self._f_and_jacs is not None
            and int(self.sqp_max_iter) > 1
        ):
            max_outer_iter = max(1, int(self.sqp_max_iter))
            total_attempted_stages = 0
            total_failed_stages = 0
            used_stale_fallback = False
            total_solve_time = 0.0
            last_u_full: np.ndarray | None = None

            for outer_idx in range(max_outer_iter):
                iter_info = self._linearization_strategy.prepare_linearization(
                    controller=self,
                    x_current=x_input,
                )
                total_attempted_stages += int(iter_info.attempted_stages)
                total_failed_stages += int(iter_info.failed_stages)
                used_stale_fallback = used_stale_fallback or bool(
                    iter_info.used_stale_fallback
                )
                linearization_info = iter_info

                if self._linearization_strategy.handle_linearization_failure(
                    controller=self,
                    info=iter_info,
                ):
                    linearization_info.attempted_stages = int(total_attempted_stages)
                    linearization_info.failed_stages = int(total_failed_stages)
                    linearization_info.used_stale_fallback = bool(used_stale_fallback)
                    return self._build_integrity_failure_fallback(
                        previous_thrusters=previous_thrusters,
                        s_for_state=s_for_state,
                        linearization_info=linearization_info,
                    )

                iter_result = self._cpp.get_control_action(x_input)
                result = iter_result
                nonlinear_outer_iterations = int(outer_idx + 1)
                total_solve_time += float(getattr(iter_result, "solve_time", 0.0))

                u_candidate = np.array(iter_result.u, dtype=float)
                if last_u_full is not None and u_candidate.shape == last_u_full.shape:
                    nonlinear_outer_residual = float(
                        np.linalg.norm(u_candidate - last_u_full, ord=np.inf)
                    )
                    if nonlinear_outer_residual <= float(self.sqp_tol):
                        nonlinear_outer_converged = True
                        break
                last_u_full = u_candidate

                if outer_idx < max_outer_iter - 1:
                    try:
                        self._cpp.set_warm_start_control(u_candidate)
                    except Exception:
                        pass
                    if self._path_set:
                        try:
                            # Keep run-level path progress consistent while iterating
                            # internally within one control step.
                            self._cpp.set_current_path_s(float(s_for_state))
                            self.s = float(s_for_state)
                        except Exception:
                            pass

            linearization_info.attempted_stages = int(total_attempted_stages)
            linearization_info.failed_stages = int(total_failed_stages)
            linearization_info.used_stale_fallback = bool(used_stale_fallback)

            if result is None:
                result = self._cpp.get_control_action(x_input)
                nonlinear_outer_iterations = 1
            try:
                result.solve_time = float(total_solve_time)
            except Exception:
                pass
            try:
                result.sqp_iterations = int(nonlinear_outer_iterations)
            except Exception:
                pass
            if nonlinear_outer_residual is not None:
                try:
                    result.sqp_kkt_residual = float(nonlinear_outer_residual)
                except Exception:
                    pass
        else:
            if self._f_and_jacs is not None:
                linearization_info = self._linearization_strategy.prepare_linearization(
                    controller=self,
                    x_current=x_input,
                )
                if self._linearization_strategy.handle_linearization_failure(
                    controller=self,
                    info=linearization_info,
                ):
                    return self._build_integrity_failure_fallback(
                        previous_thrusters=previous_thrusters,
                        s_for_state=s_for_state,
                        linearization_info=linearization_info,
                    )

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
            "solver_type": self.solver_type,
            "solver_backend": self.solver_backend,
            "linearization_mode": self.linearization_mode,
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
            "sqp_outer_iterations": int(nonlinear_outer_iterations),
            "sqp_outer_residual": nonlinear_outer_residual,
            "sqp_outer_converged": bool(nonlinear_outer_converged),
            "controller_core": self.controller_core,
            "controller_profile": self.controller_profile,
            "cpp_backend_module": self._cpp_backend_module,
            "shared_params_hash": self.shared_params_hash,
            "effective_params_hash": self.effective_params_hash,
            "override_diff": dict(self.profile_override_diff),
            "profile_specific_params": dict(self.profile_specific_params),
            "sqp_max_iter_config": int(self.sqp_max_iter),
            "sqp_tol_config": float(self.sqp_tol),
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
            "ref_heading_step_deg": float(
                getattr(result, "ref_heading_step_deg", 0.0) or 0.0
            ),
            "ref_quat_step_deg_max_horizon": float(
                getattr(result, "ref_quat_step_deg_max_horizon", 0.0) or 0.0
            ),
            "ref_slew_limited_fraction": float(
                getattr(result, "ref_slew_limited_fraction", 0.0) or 0.0
            ),
            "terminal_progress_reward_active": bool(
                getattr(result, "terminal_progress_reward_active", False)
            ),
            "degenerate_tangent_fallback_count": int(
                getattr(result, "degenerate_tangent_fallback_count", 0) or 0
            ),
        }
        self._linearization_strategy.annotate_solver_metadata(
            extras=extras,
            info=linearization_info,
        )
        if self.linearization_mode == "linear_frozen_step":
            extras["linear_frozen_refresh_interval_steps"] = int(
                self.freeze_refresh_interval_steps
            )
            extras["linear_frozen_refreshed"] = bool(
                linearization_info.attempted_stages > 0
            )
        if self.linearization_mode == "hybrid_tolerant_stage":
            extras["hybrid_allow_stale_stage_reuse"] = bool(
                self.allow_stale_stage_reuse
            )
        if self.linearization_mode == "nonlinear_exact_stage":
            extras["nonlinear_strict_integrity"] = bool(
                self.strict_linearization_integrity
            )
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
            "solve_times": list(self.solve_times),
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
            if not self._scan_attitude_enabled:
                return
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

    def set_current_path_s(self, s_value: float) -> None:
        s_next = float(max(0.0, s_value))
        self.s = s_next
        try:
            self._cpp.set_current_path_s(s_next)
        except Exception:
            logger.debug("Failed to set current path s in SQP core", exc_info=True)

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

    def _evaluate_casadi_linearisation_stagewise(
        self,
        x_current: np.ndarray,
        strict_integrity: bool,
        linearization_mode: str,
    ) -> LinearizationInfo:
        """Evaluate CasADi dynamics at each horizon stage and inject into C++."""
        info = LinearizationInfo(
            linearization_mode=linearization_mode,
            strict_mode=bool(strict_integrity),
        )
        if self._f_and_jacs is None:
            return info
        if not hasattr(self._cpp, "set_stage_linearisation"):
            return info

        N = self._cpp.prediction_horizon
        p = self._casadi_params
        info.attempted_stages = int(N)

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
            except Exception:
                info.failed_stages += 1
                info.used_stale_fallback = True
                logger.debug("CasADi eval failed at stage %d", k, exc_info=True)
                if strict_integrity:
                    info.integrity_failure = True
                    info.integrity_reason = f"casadi_eval_exception_stage_{k}"
                    return info
                continue

            if not (
                np.all(np.isfinite(A_k))
                and np.all(np.isfinite(B_k))
                and np.all(np.isfinite(x_next))
            ):
                info.failed_stages += 1
                info.used_stale_fallback = True
                logger.warning(
                    "CasADi linearization at stage %d produced NaN/Inf; "
                    "reusing stale A/B matrix from previous iteration.",
                    k,
                )
                if strict_integrity:
                    info.integrity_failure = True
                    info.integrity_reason = f"non_finite_linearization_stage_{k}"
                    return info
                continue

            # Affine term: d = x_next - A_k @ x_k - B_k @ u_k
            d_k = x_next - A_k @ x_k - B_k @ u_k
            self._cpp.set_stage_linearisation(k, A_k, B_k, d_k)

        return info

    def _evaluate_casadi_linearisation_frozen_step(
        self,
        x_current: np.ndarray,
        linearization_mode: str,
    ) -> LinearizationInfo:
        """Freeze Jacobians once per step and reuse them across the horizon."""
        info = LinearizationInfo(
            linearization_mode=linearization_mode,
            strict_mode=False,
        )
        if self._f_and_jacs is None:
            return info
        if not hasattr(self._cpp, "set_all_linearisations"):
            return info

        N = self._cpp.prediction_horizon
        p = self._casadi_params
        refresh_interval = max(
            1, int(getattr(self, "freeze_refresh_interval_steps", 1))
        )
        self._linear_step_counter += 1
        should_refresh = (
            self._linear_frozen_cache is None
            or refresh_interval == 1
            or ((self._linear_step_counter - 1) % refresh_interval == 0)
        )

        if not should_refresh and self._linear_frozen_cache is not None:
            As, Bs, ds = self._linear_frozen_cache
            self._cpp.set_all_linearisations(As, Bs, ds)
            info.attempted_stages = 0
            return info

        info.attempted_stages = int(N)

        x0 = np.array(self._cpp.get_stage_state(0), dtype=float)
        u0 = np.array(self._cpp.get_stage_control(0), dtype=float)
        if np.linalg.norm(x0[:3]) < 1e-6:
            x0 = np.array(x_current, dtype=float)

        try:
            result = self._f_and_jacs(x0, u0, p, self._dt)
            x_next0 = np.array(result[0]).ravel()
            A0 = np.array(result[1])
            B0 = np.array(result[2])
            if not (
                np.all(np.isfinite(A0))
                and np.all(np.isfinite(B0))
                and np.all(np.isfinite(x_next0))
            ):
                raise ValueError("Frozen-step linearization produced non-finite values")
            d0 = x_next0 - A0 @ x0 - B0 @ u0
            As = [A0] * N
            Bs = [B0] * N
            ds = [d0] * N
            self._cpp.set_all_linearisations(As, Bs, ds)
            self._linear_frozen_cache = (As, Bs, ds)
        except Exception:
            info.failed_stages = int(N)
            info.used_stale_fallback = True
            info.integrity_reason = "frozen_step_linearization_failed"
            logger.debug("Frozen-step CasADi eval failed", exc_info=True)

        return info

    def _build_integrity_failure_fallback(
        self,
        previous_thrusters: np.ndarray | None,
        s_for_state: float,
        linearization_info: LinearizationInfo,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        """Fail-closed fallback used by strict nonlinear profile."""
        rw_fallback = np.zeros(self.num_rw_axes, dtype=float)
        if (
            previous_thrusters is not None
            and len(previous_thrusters) == self.num_thrusters
        ):
            thruster_fallback = np.clip(
                np.array(previous_thrusters, dtype=float),
                0.0,
                1.0,
            )
        else:
            thruster_fallback = np.zeros(self.num_thrusters, dtype=float)
        u_phys = np.concatenate((rw_fallback, thruster_fallback)).astype(
            float, copy=False
        )
        path_len = float(self._path_length) if self._path_length > 0 else 0.0
        path_s = float(self.s if self._path_set else s_for_state)
        progress = float(path_s / path_len) if path_len > 1e-9 else 0.0
        remaining = float(path_len - path_s) if path_len > 0 else 0.0
        extras: dict[str, Any] = {
            "status": -1,
            "status_name": "FAILED",
            "solver_status": "linearization_integrity_failed",
            "timeout": False,
            "solve_time": 0.0,
            "iterations": None,
            "objective_value": None,
            "solver_type": self.solver_type,
            "solver_backend": self.solver_backend,
            "linearization_mode": self.linearization_mode,
            "solver_time_limit": self.solver_time_limit,
            "solver_fallback": True,
            "solver_fallback_reason": "linearization_integrity_failed",
            "solver_success": False,
            "time_limit_exceeded": False,
            "fallback_active": True,
            "fallback_age_s": 0.0,
            "fallback_scale": 1.0,
            "timing_linearization_s": 0.0,
            "timing_cost_update_s": 0.0,
            "timing_constraint_update_s": 0.0,
            "timing_matrix_update_s": 0.0,
            "timing_warmstart_s": 0.0,
            "timing_solve_only_s": 0.0,
            "sqp_iterations": 0,
            "sqp_kkt_residual": 0.0,
            "sqp_outer_iterations": 0,
            "sqp_outer_residual": None,
            "sqp_outer_converged": False,
            "controller_core": self.controller_core,
            "controller_profile": self.controller_profile,
            "cpp_backend_module": self._cpp_backend_module,
            "shared_params_hash": self.shared_params_hash,
            "effective_params_hash": self.effective_params_hash,
            "override_diff": dict(self.profile_override_diff),
            "profile_specific_params": dict(self.profile_specific_params),
            "sqp_max_iter_config": int(self.sqp_max_iter),
            "sqp_tol_config": float(self.sqp_tol),
            "path_s": path_s,
            "path_s_proj": self._last_path_projection.get("s_proj"),
            "path_v_s": 0.0,
            "path_progress": progress,
            "path_remaining": remaining,
            "path_error": self._last_path_projection.get("path_error"),
            "path_endpoint_error": self._last_path_projection.get("endpoint_error"),
            "path_s_pred": path_s,
            "ref_heading_step_deg": 0.0,
            "ref_quat_step_deg_max_horizon": 0.0,
            "ref_slew_limited_fraction": 0.0,
            "terminal_progress_reward_active": False,
            "degenerate_tangent_fallback_count": 0,
        }
        self._linearization_strategy.annotate_solver_metadata(
            extras=extras,
            info=linearization_info,
        )
        if self.linearization_mode == "linear_frozen_step":
            extras["linear_frozen_refresh_interval_steps"] = int(
                self.freeze_refresh_interval_steps
            )
            extras["linear_frozen_refreshed"] = bool(
                linearization_info.attempted_stages > 0
            )
        if self.linearization_mode == "hybrid_tolerant_stage":
            extras["hybrid_allow_stale_stage_reuse"] = bool(
                self.allow_stale_stage_reuse
            )
        if self.linearization_mode == "nonlinear_exact_stage":
            extras["nonlinear_strict_integrity"] = bool(
                self.strict_linearization_integrity
            )
        return u_phys, extras

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

    def _extract_params(
        self,
        cfg: AppConfig,
        effective_mpc: dict[str, Any] | None = None,
        profile_specific: dict[str, Any] | None = None,
    ) -> None:
        physics = cfg.physics
        mpc = cfg.mpc
        mpc_core = cfg.mpc_core
        contracts = cfg.controller_contracts
        resolved_mpc = dict(effective_mpc or {})
        resolved_specific = dict(profile_specific or {})

        def mpc_val(name: str, fallback: Any = None) -> Any:
            if name in resolved_mpc:
                return resolved_mpc[name]
            if hasattr(mpc, name):
                return getattr(mpc, name)
            return fallback

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

        self.N = int(mpc_val("prediction_horizon"))
        self._dt = float(mpc_val("dt"))
        self.solver_time_limit = float(mpc_val("solver_time_limit"))
        self.control_horizon = max(1, min(int(mpc_val("control_horizon")), self.N))
        self.verbose_mpc = bool(mpc_val("verbose_mpc"))

        self.Q_contour = float(mpc_val("Q_contour"))
        # Auto-resolve Q_lag: when 0 (default), use Q_lag_default
        _q_lag = float(mpc_val("Q_lag"))
        if _q_lag <= 0.0:
            _q_lag = float(mpc_val("Q_lag_default", 0.0) or 0.0)
            logger.debug("Q_lag=0 detected; substituting Q_lag_default=%s", _q_lag)
        self.Q_lag = _q_lag
        self.Q_progress = float(mpc_val("Q_progress"))
        self.progress_reward = float(mpc_val("progress_reward"))
        self.Q_velocity_align = float(mpc_val("Q_velocity_align"))
        self.Q_s_anchor = float(mpc_val("Q_s_anchor"))
        self.Q_smooth = float(mpc_val("Q_smooth"))
        self.Q_terminal_pos = float(mpc_val("Q_terminal_pos"))
        self.Q_terminal_s = float(mpc_val("Q_terminal_s"))
        self.Q_terminal_vel = float(mpc_val("Q_terminal_vel"))
        self.Q_angvel = float(mpc_val("q_angular_velocity"))
        self.Q_attitude = float(mpc_val("Q_attitude"))
        self.Q_axis_align = float(mpc_val("Q_axis_align"))
        self.Q_quat_norm = float(mpc_val("Q_quat_norm"))
        self.R_thrust = float(mpc_val("r_thrust"))
        self.R_rw_torque = float(mpc_val("r_rw_torque"))
        self.thrust_l1_weight = float(mpc_val("thrust_l1_weight"))
        self.thrust_pair_weight = float(mpc_val("thrust_pair_weight"))
        self.max_linear_velocity = float(mpc_val("max_linear_velocity"))
        self.max_angular_velocity = float(mpc_val("max_angular_velocity"))
        self.enable_online_dare_terminal = bool(mpc_val("enable_online_dare_terminal"))
        self.dare_update_period_steps = int(mpc_val("dare_update_period_steps"))
        self.terminal_cost_profile = str(mpc_val("terminal_cost_profile"))
        self.progress_policy = str(mpc_val("progress_policy"))
        self.error_priority_min_vs = float(mpc_val("error_priority_min_vs"))
        self.error_priority_error_speed_gain = float(
            mpc_val("error_priority_error_speed_gain")
        )
        self.path_speed = float(mpc_val("path_speed"))
        self.path_speed_min = float(mpc_val("path_speed_min"))
        self.path_speed_max = float(mpc_val("path_speed_max"))
        self.ref_tangent_lookahead_m = float(mpc_val("ref_tangent_lookahead_m", 0.35))
        self.ref_tangent_lookback_m = float(mpc_val("ref_tangent_lookback_m", 0.10))
        self.ref_quat_max_rate_rad_s = float(mpc_val("ref_quat_max_rate_rad_s", 1.57))
        self.ref_quat_terminal_rate_scale = float(
            mpc_val("ref_quat_terminal_rate_scale", 2.0)
        )

        self.sqp_max_iter = int(max(1, resolved_specific.get("sqp_max_iter", 1)))
        self.sqp_tol = float(max(1e-9, resolved_specific.get("sqp_tol", 1e-4)))
        self.strict_linearization_integrity = bool(
            resolved_specific.get(
                "strict_integrity",
                self.linearization_mode == "nonlinear_exact_stage",
            )
        )
        self.allow_stale_stage_reuse = bool(
            resolved_specific.get(
                "allow_stale_stage_reuse",
                self.linearization_mode != "nonlinear_exact_stage",
            )
        )
        self.freeze_refresh_interval_steps = int(
            max(1, resolved_specific.get("freeze_refresh_interval_steps", 1))
        )

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
            from controller.shared.python.physics.orbital_config import OrbitalConfig

            orb = OrbitalConfig()
            self.orbital_mu = float(orb.mu)
            self.orbital_radius = float(orb.orbital_radius)
            self.orbital_mean_motion = float(orb.mean_motion)
            self.use_two_body = True
        except Exception:
            logger.warning(
                "OrbitalConfig load failed; using hardcoded Earth defaults "
                "(mu=3.986e14, r=6.778e6 m). Check physics.orbital_config.",
                exc_info=True,
            )
            self.orbital_mu = 3.986004418e14
            self.orbital_radius = 6.778e6
            self.orbital_mean_motion = 0.0
            self.use_two_body = True
