"""V6 controller runtime primitives: modes, terminal gate, actuator policy, and speed planning."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Literal

import numpy as np

ModeName = Literal["TRACK", "RECOVER", "SETTLE", "HOLD", "COMPLETE"]


@dataclass
class MissionRuntimePlanV6:
    """Compiled mission runtime metadata for V6 diagnostics and duration policy."""

    path_length_m: float
    path_speed_mps: float
    estimated_eta_s: float
    required_duration_s: float
    hold_duration_s: float
    duration_margin_s: float
    speed_policy: str
    non_hold_segment_speed_caps: list[float] = field(default_factory=list)


@dataclass
class ReferenceSliceV6:
    """Reference knot slice for diagnostics/replay (N+1 states)."""

    dt: float
    knots: list[dict[str, Any]]


@dataclass
class ModeProfileV6:
    """Mode-specific cost profile multipliers used by V6 diagnostics/policy."""

    contour_scale: float = 1.0
    lag_scale: float = 1.0
    progress_scale: float = 1.0
    attitude_scale: float = 1.0
    terminal_pos_scale: float = 1.0
    terminal_attitude_scale: float = 1.0
    velocity_align_scale: float = 1.0
    angular_velocity_scale: float = 1.0
    smoothness_scale: float = 1.0
    thruster_pair_scale: float = 1.0


@dataclass
class ModeStateV6:
    """Current controller mode and transition timing."""

    current_mode: ModeName = "TRACK"
    entered_at_s: float = 0.0
    time_in_mode_s: float = 0.0


@dataclass
class CompletionGateStatusV6:
    """Strict terminal gate status for V6 completion contract."""

    progress_ok: bool
    position_ok: bool
    angle_ok: bool
    velocity_ok: bool
    angular_velocity_ok: bool
    all_thresholds_ok: bool
    gate_ok: bool
    hold_elapsed_s: float
    hold_required_s: float
    last_breach_reason: str | None
    complete: bool


@dataclass
class SolverHealthV6:
    """Solver health counters surfaced via telemetry/artifacts."""

    status: str = "ok"
    fallback_count: int = 0
    hard_limit_breaches: int = 0
    last_fallback_reason: str | None = None
    fallback_reasons: dict[str, int] = field(default_factory=dict)
    fallback_active: bool = False
    fallback_age_s: float = 0.0
    fallback_scale: float = 0.0


@dataclass
class ControlResultV6:
    """Normalized control-step result contract emitted by MPC runtime."""

    controller_core: str
    solver_backend: str
    solver_status: int | None
    solver_success: bool
    solver_fallback: bool
    solver_fallback_reason: str | None
    solve_time_s: float
    timeout: bool

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class QualityContractReportV6:
    """Serializable contract report payload for quality harness outputs."""

    schema_version: str
    generated_at: str
    scenario: str
    run_id: str
    run_dir: str
    command: list[str]
    return_code: int
    metrics: dict[str, Any]
    contracts: dict[str, dict[str, Any]]
    passed: bool
    breaches: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class ControllerModeManagerV6:
    """Finite-state mode manager for TRACK/RECOVER/SETTLE/HOLD transitions."""

    def __init__(
        self,
        recover_enter_error_m: float = 0.20,
        recover_enter_hold_s: float = 0.5,
        recover_exit_error_m: float = 0.10,
        recover_exit_hold_s: float = 1.0,
        recover_contour_scale: float = 2.0,
        recover_lag_scale: float = 2.0,
        recover_progress_scale: float = 0.6,
        recover_attitude_scale: float = 0.8,
        settle_progress_scale: float = 0.0,
        settle_terminal_pos_scale: float = 2.0,
        settle_terminal_attitude_scale: float = 1.5,
        settle_velocity_align_scale: float = 1.5,
        settle_angular_velocity_scale: float = 2.0,
        hold_smoothness_scale: float = 1.5,
        hold_thruster_pair_scale: float = 1.2,
    ):
        self.recover_enter_error_m = max(0.0, float(recover_enter_error_m))
        self.recover_enter_hold_s = max(0.0, float(recover_enter_hold_s))
        self.recover_exit_error_m = max(0.0, float(recover_exit_error_m))
        self.recover_exit_hold_s = max(0.0, float(recover_exit_hold_s))
        self.recover_contour_scale = max(0.0, float(recover_contour_scale))
        self.recover_lag_scale = max(0.0, float(recover_lag_scale))
        self.recover_progress_scale = max(0.0, float(recover_progress_scale))
        self.recover_attitude_scale = max(0.0, float(recover_attitude_scale))
        self.settle_progress_scale = max(0.0, float(settle_progress_scale))
        self.settle_terminal_pos_scale = max(0.0, float(settle_terminal_pos_scale))
        self.settle_terminal_attitude_scale = max(
            0.0, float(settle_terminal_attitude_scale)
        )
        self.settle_velocity_align_scale = max(
            0.0, float(settle_velocity_align_scale)
        )
        self.settle_angular_velocity_scale = max(
            0.0, float(settle_angular_velocity_scale)
        )
        self.hold_smoothness_scale = max(0.0, float(hold_smoothness_scale))
        self.hold_thruster_pair_scale = max(0.0, float(hold_thruster_pair_scale))

        self.state = ModeStateV6()
        self._recover_enter_candidate_since: float | None = None
        self._recover_exit_candidate_since: float | None = None

    def reset(self, sim_time_s: float = 0.0) -> None:
        self.state = ModeStateV6(current_mode="TRACK", entered_at_s=float(sim_time_s))
        self._recover_enter_candidate_since = None
        self._recover_exit_candidate_since = None

    def profile_for_mode(self, mode: ModeName) -> ModeProfileV6:
        if mode == "RECOVER":
            return ModeProfileV6(
                contour_scale=self.recover_contour_scale,
                lag_scale=self.recover_lag_scale,
                progress_scale=self.recover_progress_scale,
                attitude_scale=self.recover_attitude_scale,
            )
        if mode == "SETTLE":
            return ModeProfileV6(
                progress_scale=self.settle_progress_scale,
                terminal_pos_scale=self.settle_terminal_pos_scale,
                terminal_attitude_scale=self.settle_terminal_attitude_scale,
                velocity_align_scale=self.settle_velocity_align_scale,
                angular_velocity_scale=self.settle_angular_velocity_scale,
            )
        if mode == "HOLD":
            return ModeProfileV6(
                progress_scale=self.settle_progress_scale,
                terminal_pos_scale=self.settle_terminal_pos_scale,
                terminal_attitude_scale=self.settle_terminal_attitude_scale,
                velocity_align_scale=self.settle_velocity_align_scale,
                angular_velocity_scale=self.settle_angular_velocity_scale,
                smoothness_scale=self.hold_smoothness_scale,
                thruster_pair_scale=self.hold_thruster_pair_scale,
            )
        return ModeProfileV6()

    def _switch_mode(self, new_mode: ModeName, sim_time_s: float) -> None:
        if self.state.current_mode == new_mode:
            return
        self.state.current_mode = new_mode
        self.state.entered_at_s = float(sim_time_s)
        self.state.time_in_mode_s = 0.0

    def update(
        self,
        *,
        sim_time_s: float,
        contour_error_m: float,
        path_s: float,
        path_len: float,
        position_tolerance_m: float,
        completion_gate_state_ok: bool,
        completion_reached: bool,
        solver_degraded: bool = False,
        solver_fallback_reason: str | None = None,
    ) -> ModeStateV6:
        """Update mode state based on error/progress/gate conditions."""
        sim_time_s = float(sim_time_s)
        contour_error_m = _safe_float(contour_error_m, default=0.0)
        path_s = _safe_float(path_s, default=0.0)
        path_len = _safe_float(path_len, default=0.0)
        pos_tol = max(1e-6, _safe_float(position_tolerance_m, default=0.1))

        if completion_reached:
            self._switch_mode("COMPLETE", sim_time_s)
            self.state.time_in_mode_s = max(0.0, sim_time_s - self.state.entered_at_s)
            return self.state

        at_path_end = path_len > 0.0 and path_s >= (path_len - pos_tol)
        mode = self.state.current_mode

        if mode in ("TRACK", "RECOVER") and at_path_end:
            self._switch_mode("SETTLE", sim_time_s)
            mode = self.state.current_mode
            self._recover_enter_candidate_since = None
            self._recover_exit_candidate_since = None

        solver_reason = str(solver_fallback_reason or "").strip()
        solver_recover_trigger = bool(
            solver_degraded and mode == "TRACK" and not at_path_end
        )

        if mode == "TRACK":
            if contour_error_m >= self.recover_enter_error_m or solver_recover_trigger:
                if self._recover_enter_candidate_since is None:
                    self._recover_enter_candidate_since = sim_time_s
                required_hold = self.recover_enter_hold_s
                if solver_recover_trigger and solver_reason:
                    required_hold = min(required_hold, 0.25)
                if (sim_time_s - self._recover_enter_candidate_since) >= required_hold:
                    self._switch_mode("RECOVER", sim_time_s)
                    self._recover_enter_candidate_since = None
            else:
                self._recover_enter_candidate_since = None

        elif mode == "RECOVER":
            if contour_error_m <= self.recover_exit_error_m:
                if self._recover_exit_candidate_since is None:
                    self._recover_exit_candidate_since = sim_time_s
                if (sim_time_s - self._recover_exit_candidate_since) >= self.recover_exit_hold_s:
                    self._switch_mode("TRACK", sim_time_s)
                    self._recover_exit_candidate_since = None
            else:
                self._recover_exit_candidate_since = None

        elif mode == "SETTLE":
            if completion_gate_state_ok and at_path_end:
                self._switch_mode("HOLD", sim_time_s)
            elif not at_path_end:
                self._switch_mode("TRACK", sim_time_s)

        elif mode == "HOLD":
            if not (completion_gate_state_ok and at_path_end):
                self._switch_mode("SETTLE", sim_time_s)

        self.state.time_in_mode_s = max(0.0, sim_time_s - self.state.entered_at_s)
        return self.state


class TerminalSupervisorV6:
    """Strict terminal gate supervisor with continuous hold timer."""

    def __init__(self, hold_required_s: float = 10.0):
        self.hold_required_s = max(0.0, float(hold_required_s))
        self.hold_started_at_s: float | None = None
        self.last_breach_reason: str | None = None

    def reset(self) -> None:
        self.hold_started_at_s = None
        self.last_breach_reason = None

    def evaluate(
        self,
        *,
        sim_time_s: float,
        progress_ok: bool,
        position_ok: bool,
        angle_ok: bool,
        velocity_ok: bool,
        angular_velocity_ok: bool,
    ) -> CompletionGateStatusV6:
        sim_time_s = float(sim_time_s)
        all_thresholds_ok = bool(position_ok and angle_ok and velocity_ok and angular_velocity_ok)
        gate_ok = bool(progress_ok and all_thresholds_ok)

        if gate_ok:
            if self.hold_started_at_s is None:
                self.hold_started_at_s = sim_time_s
            hold_elapsed = max(0.0, sim_time_s - self.hold_started_at_s)
            self.last_breach_reason = None
        else:
            hold_elapsed = 0.0
            self.hold_started_at_s = None
            if not progress_ok:
                self.last_breach_reason = "path_progress"
            elif not position_ok:
                self.last_breach_reason = "position_error"
            elif not angle_ok:
                self.last_breach_reason = "angle_error"
            elif not velocity_ok:
                self.last_breach_reason = "velocity_error"
            elif not angular_velocity_ok:
                self.last_breach_reason = "angular_velocity_error"
            else:
                self.last_breach_reason = "unknown"

        complete = gate_ok and hold_elapsed >= self.hold_required_s

        return CompletionGateStatusV6(
            progress_ok=bool(progress_ok),
            position_ok=bool(position_ok),
            angle_ok=bool(angle_ok),
            velocity_ok=bool(velocity_ok),
            angular_velocity_ok=bool(angular_velocity_ok),
            all_thresholds_ok=all_thresholds_ok,
            gate_ok=gate_ok,
            hold_elapsed_s=hold_elapsed,
            hold_required_s=self.hold_required_s,
            last_breach_reason=self.last_breach_reason,
            complete=bool(complete),
        )


class ActuatorPolicyV6:
    """Deterministic actuator shaping with hysteresis and terminal bypass band."""

    def __init__(
        self,
        *,
        enable_thruster_hysteresis: bool = True,
        thruster_hysteresis_on: float = 0.015,
        thruster_hysteresis_off: float = 0.007,
        terminal_bypass_band_m: float = 0.20,
    ):
        self.enable_thruster_hysteresis = bool(enable_thruster_hysteresis)
        self.thruster_hysteresis_on = max(0.0, float(thruster_hysteresis_on))
        self.thruster_hysteresis_off = max(0.0, float(thruster_hysteresis_off))
        self.terminal_bypass_band_m = max(0.0, float(terminal_bypass_band_m))
        if self.thruster_hysteresis_on <= self.thruster_hysteresis_off:
            self.enable_thruster_hysteresis = False

    def _apply_hysteresis(
        self,
        thruster_action: np.ndarray,
        previous_thrusters: np.ndarray,
    ) -> np.ndarray:
        prev = np.array(previous_thrusters, dtype=np.float64, copy=False).reshape(-1)
        cmd = np.array(thruster_action, dtype=np.float64, copy=False).reshape(-1)

        if prev.size != cmd.size:
            prev = np.zeros(cmd.size, dtype=np.float64)

        prev_active = prev >= self.thruster_hysteresis_off
        turn_on = (~prev_active) & (cmd >= self.thruster_hysteresis_on)
        stay_on = prev_active & (cmd >= self.thruster_hysteresis_off)
        active_mask = turn_on | stay_on
        return np.where(active_mask, cmd, 0.0).astype(np.float64, copy=False)

    def apply(
        self,
        thruster_action: np.ndarray,
        previous_thrusters: np.ndarray,
        *,
        mode: ModeName,
        endpoint_error_m: float | None,
    ) -> np.ndarray:
        cmd = np.array(thruster_action, dtype=np.float64, copy=True).reshape(-1)
        np.clip(cmd, 0.0, 1.0, out=cmd)

        if mode == "COMPLETE":
            return np.zeros_like(cmd)

        if not self.enable_thruster_hysteresis:
            return cmd

        mode_uses_hysteresis = mode in ("TRACK", "RECOVER")
        if mode in ("SETTLE", "HOLD"):
            endpoint_error = _safe_float(endpoint_error_m, default=float("inf"))
            mode_uses_hysteresis = endpoint_error > self.terminal_bypass_band_m

        if not mode_uses_hysteresis:
            return cmd

        return self._apply_hysteresis(cmd, previous_thrusters)


class ReferenceSchedulerV6:
    """Reference horizon scheduler used for V6 diagnostics and replay."""

    def build_slice(
        self,
        *,
        sim: Any,
        current_state: np.ndarray,
        mode: ModeName,
        horizon: int,
        dt: float,
    ) -> ReferenceSliceV6:
        knots: list[dict[str, Any]] = []

        try:
            s0 = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
            path_len = float(sim._get_mission_path_length(compute_if_missing=True) or 0.0)
        except Exception:
            s0 = 0.0
            path_len = 0.0

        mpc_cfg = getattr(getattr(sim.simulation_config, "app_config", None), "mpc", None)
        v_nominal = _safe_float(getattr(mpc_cfg, "path_speed", 0.0), default=0.0)
        if mode in ("SETTLE", "HOLD", "COMPLETE"):
            v_nominal = 0.0

        q_curr = (
            np.array(current_state[3:7], dtype=float)
            if isinstance(current_state, np.ndarray) and current_state.size >= 7
            else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        )

        for k in range(max(1, int(horizon)) + 1):
            s_query = s0 + v_nominal * float(dt) * float(k)
            if path_len > 0.0:
                s_query = float(np.clip(s_query, 0.0, path_len))
            try:
                pos, tangent, quat = sim.mpc_controller.get_path_reference_state(
                    s_query=s_query,
                    q_current=q_curr,
                )
                knot = {
                    "k": k,
                    "s": float(s_query),
                    "position": [float(pos[0]), float(pos[1]), float(pos[2])],
                    "tangent": [float(tangent[0]), float(tangent[1]), float(tangent[2])],
                    "quaternion": [
                        float(quat[0]),
                        float(quat[1]),
                        float(quat[2]),
                        float(quat[3]),
                    ],
                    "velocity_target": [
                        float(tangent[0] * v_nominal),
                        float(tangent[1] * v_nominal),
                        float(tangent[2] * v_nominal),
                    ],
                }
            except Exception:
                knot = {
                    "k": k,
                    "s": float(s_query),
                    "position": [0.0, 0.0, 0.0],
                    "tangent": [0.0, 0.0, 0.0],
                    "quaternion": [1.0, 0.0, 0.0, 0.0],
                    "velocity_target": [0.0, 0.0, 0.0],
                }
            knots.append(knot)

        return ReferenceSliceV6(dt=float(dt), knots=knots)


def extract_non_hold_speed_caps(mission: Any) -> list[float]:
    """Extract explicit speed_max caps from non-hold unified mission segments."""
    caps: list[float] = []
    segments = getattr(mission, "segments", None)
    if segments is None and isinstance(mission, dict):
        segments = mission.get("segments")
    if not isinstance(segments, list):
        return caps

    for segment in segments:
        seg_type = None
        constraints = None
        if isinstance(segment, dict):
            seg_type = str(segment.get("type", ""))
            constraints = segment.get("constraints", {})
            speed = constraints.get("speed_max") if isinstance(constraints, dict) else None
        else:
            seg_type = str(getattr(segment, "type", ""))
            constraints = getattr(segment, "constraints", None)
            speed = getattr(constraints, "speed_max", None)
        if "HOLD" in str(seg_type).upper():
            continue
        speed_val = _safe_float(speed, default=0.0)
        if speed_val > 0.0:
            caps.append(speed_val)

    return caps


def compute_runtime_path_speed(
    *,
    non_hold_segment_caps: list[float],
    default_path_speed: float,
    path_speed_min: float,
    path_speed_max: float,
) -> float:
    """Apply V6 speed policy: min non-hold cap, then clamp to MPC bounds."""
    speed_candidate = min(non_hold_segment_caps) if non_hold_segment_caps else float(default_path_speed)
    speed = float(speed_candidate)

    min_bound = _safe_float(path_speed_min, default=0.0)
    max_bound = _safe_float(path_speed_max, default=0.0)
    if max_bound > 0.0:
        speed = min(speed, max_bound)
    if min_bound > 0.0:
        speed = max(speed, min_bound)

    return max(1e-6, speed)


def estimate_required_duration_s(
    *,
    path_length_m: float,
    path_speed_mps: float,
    hold_duration_s: float,
    margin_s: float = 30.0,
) -> float:
    """Compute minimum duration for completion contract feasibility."""
    path_len = max(0.0, float(path_length_m))
    speed = max(1e-6, float(path_speed_mps))
    hold_s = max(0.0, float(hold_duration_s))
    margin = max(0.0, float(margin_s))
    return (path_len / speed) + hold_s + margin


def compile_mission_runtime_plan_v6(
    *,
    mission: Any,
    path_length_m: float,
    default_path_speed: float,
    path_speed_min: float,
    path_speed_max: float,
    hold_duration_s: float,
    margin_s: float = 30.0,
) -> MissionRuntimePlanV6:
    """Compile V6 runtime plan metadata from mission and path characteristics."""
    caps = extract_non_hold_speed_caps(mission)
    path_speed = compute_runtime_path_speed(
        non_hold_segment_caps=caps,
        default_path_speed=default_path_speed,
        path_speed_min=path_speed_min,
        path_speed_max=path_speed_max,
    )
    eta = max(0.0, float(path_length_m)) / max(path_speed, 1e-6)
    required_duration = estimate_required_duration_s(
        path_length_m=path_length_m,
        path_speed_mps=path_speed,
        hold_duration_s=hold_duration_s,
        margin_s=margin_s,
    )

    return MissionRuntimePlanV6(
        path_length_m=max(0.0, float(path_length_m)),
        path_speed_mps=float(path_speed),
        estimated_eta_s=float(eta),
        required_duration_s=float(required_duration),
        hold_duration_s=max(0.0, float(hold_duration_s)),
        duration_margin_s=max(0.0, float(margin_s)),
        speed_policy="min_non_hold_segment_speed_then_mpc_clamp",
        non_hold_segment_speed_caps=[float(x) for x in caps],
    )
