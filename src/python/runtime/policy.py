"""Controller runtime primitives: modes, terminal gate, actuator policy, and speed planning."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Literal

import numpy as np

ModeName = Literal["TRACK", "RECOVER", "SETTLE", "HOLD", "COMPLETE"]


@dataclass
class MissionRuntimePlan:
    """Compiled mission runtime metadata for diagnostics and duration policy."""

    path_length_m: float
    path_speed_mps: float
    estimated_eta_s: float
    required_duration_s: float
    hold_duration_s: float
    duration_margin_s: float
    speed_policy: str
    non_hold_segment_speed_caps: list[float] = field(default_factory=list)


@dataclass
class ReferenceSlice:
    """Reference knot slice for diagnostics/replay (N+1 states)."""

    dt: float
    knots: list[dict[str, Any]]


@dataclass
class ModeProfile:
    """Mode-specific cost profile multipliers used by diagnostics/policy."""

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
class ModeState:
    """Current controller mode and transition timing."""

    current_mode: ModeName = "TRACK"
    entered_at_s: float = 0.0
    time_in_mode_s: float = 0.0


@dataclass
class CompletionGateStatus:
    """Strict terminal gate status for completion contract."""

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
class SolverHealth:
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
class PointingContext:
    """Resolved runtime pointing context for the current path region."""

    axis_world: np.ndarray
    center_world: np.ndarray | None
    direction_cw: bool
    source: str
    span_index: int | None = None
    source_segment_index: int | None = None


@dataclass
class PointingGuardrailStatus:
    """Latched guardrail status for pointing contract monitoring."""

    breached: bool = False
    breach_since_s: float | None = None
    clear_since_s: float | None = None
    last_reason: str | None = None


@dataclass
class QualityContractReport:
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


def _normalize_vec(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    arr = np.array(vec, dtype=float).reshape(-1)
    if arr.size < 3:
        return np.array(fallback, dtype=float)
    arr3 = arr[:3]
    norm = float(np.linalg.norm(arr3))
    if norm <= 1e-9 or not np.isfinite(norm):
        return np.array(fallback, dtype=float)
    return arr3 / norm


def _quat_rotate_vec_wxyz(quat_wxyz: np.ndarray, vec: np.ndarray) -> np.ndarray:
    q = np.array(quat_wxyz, dtype=float).reshape(-1)
    if q.size < 4:
        return np.array(vec, dtype=float)
    q4 = q[:4]
    q_norm = float(np.linalg.norm(q4))
    if q_norm <= 1e-12:
        return np.array(vec, dtype=float)
    q4 = q4 / q_norm
    w, x, y, z = q4
    q_vec = np.array([x, y, z], dtype=float)
    vv = np.array(vec, dtype=float)
    uv = np.cross(q_vec, vv)
    uuv = np.cross(q_vec, uv)
    return vv + 2.0 * (w * uv + uuv)


def _angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    a_u = _normalize_vec(a, np.array([1.0, 0.0, 0.0], dtype=float))
    b_u = _normalize_vec(b, np.array([1.0, 0.0, 0.0], dtype=float))
    cos_val = float(np.clip(np.dot(a_u, b_u), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_val)))


def _extract_mission_state(sim: Any) -> Any:
    mission_state = None
    mission_state_getter = getattr(sim, "_get_mission_state", None)
    if callable(mission_state_getter):
        try:
            mission_state = mission_state_getter()
        except Exception:
            mission_state = None
    if mission_state is None:
        mission_state = getattr(
            getattr(sim, "simulation_config", None), "mission_state", None
        )
    return mission_state


def resolve_pointing_context(
    *,
    sim: Any,
    current_state: np.ndarray,
    path_s: float,
) -> PointingContext:
    """Resolve segment-aware pointing context for the current path arc-length."""
    mission_state = _extract_mission_state(sim)
    spans = list(getattr(mission_state, "pointing_path_spans", []) or [])
    s_query = max(0.0, float(path_s))

    def _axis_from_span(span: dict[str, Any] | None) -> np.ndarray | None:
        if not isinstance(span, dict):
            return None
        axis_raw = span.get("scan_axis")
        if not isinstance(axis_raw, list | tuple | np.ndarray) or len(axis_raw) < 3:
            return None
        axis_candidate = np.array(axis_raw[:3], dtype=float)
        if not np.all(np.isfinite(axis_candidate)):
            return None
        axis_norm = float(np.linalg.norm(axis_candidate))
        if axis_norm <= 1e-9:
            return None
        return axis_candidate / axis_norm

    def _center_from_span(span: dict[str, Any] | None) -> np.ndarray | None:
        if not isinstance(span, dict):
            return None
        center_raw = span.get("scan_center")
        if not isinstance(center_raw, list | tuple | np.ndarray) or len(center_raw) < 3:
            return None
        center_candidate = np.array(center_raw[:3], dtype=float)
        if not np.all(np.isfinite(center_candidate)):
            return None
        return center_candidate

    def _span_distance(span: dict[str, Any]) -> float:
        s_start = _safe_float(span.get("s_start"), 0.0)
        s_end = _safe_float(span.get("s_end"), s_start)
        if s_start <= s_query <= s_end:
            return 0.0
        return min(abs(s_query - s_start), abs(s_query - s_end))

    selected_span: dict[str, Any] | None = None
    selected_idx: int | None = None
    if spans:
        for idx, span in enumerate(spans):
            if not isinstance(span, dict):
                continue
            s_start = _safe_float(span.get("s_start"), 0.0)
            s_end = _safe_float(span.get("s_end"), s_start)
            if s_start <= s_query <= s_end + 1e-9:
                selected_span = span
                selected_idx = idx
                break
        if selected_span is None:
            best_dist = float("inf")
            for idx, span in enumerate(spans):
                if not isinstance(span, dict):
                    continue
                s_start = _safe_float(span.get("s_start"), 0.0)
                s_end = _safe_float(span.get("s_end"), s_start)
                dist = min(abs(s_query - s_start), abs(s_query - s_end))
                if dist < best_dist:
                    best_dist = dist
                    selected_span = span
                    selected_idx = idx

    chosen_span = selected_span
    axis_vec = _axis_from_span(chosen_span)
    center_world = _center_from_span(chosen_span)

    valid_scan_spans: list[dict[str, Any]] = []
    for span in spans:
        if not isinstance(span, dict):
            continue
        if _axis_from_span(span) is not None:
            valid_scan_spans.append(span)

    initial_axis = np.array(
        getattr(mission_state, "scan_attitude_axis", (0.0, 0.0, 0.0)),
        dtype=float,
    )
    has_initial_scan_axis = bool(
        initial_axis.size >= 3
        and np.all(np.isfinite(initial_axis[:3]))
        and float(np.linalg.norm(initial_axis[:3])) > 1e-9
    )
    has_scan_context = bool(valid_scan_spans) or has_initial_scan_axis

    if axis_vec is None and valid_scan_spans:
        nearest_span = min(valid_scan_spans, key=_span_distance)
        chosen_span = nearest_span
        axis_vec = _axis_from_span(nearest_span)
        if center_world is None:
            center_world = _center_from_span(nearest_span)

    if axis_vec is None and has_scan_context:
        if has_initial_scan_axis:
            axis_vec = initial_axis[:3] / float(np.linalg.norm(initial_axis[:3]))
        else:
            axis_vec = np.array([0.0, 0.0, 1.0], dtype=float)

    if axis_vec is None:
        # Fallback for missions without any scan-axis context.
        pos = (
            np.array(current_state[:3], dtype=float)
            if current_state.size >= 3
            else np.zeros(3, dtype=float)
        )
        path_frame = (
            str(getattr(mission_state, "path_frame", "LVLH")).upper()
            if mission_state is not None
            else "LVLH"
        )
        if path_frame == "LVLH":
            origin = np.array(
                getattr(mission_state, "frame_origin", (0.0, 0.0, 0.0)), dtype=float
            )
            if origin.size >= 3:
                pos = pos + origin[:3]
        axis_vec = _normalize_vec(pos, np.array([0.0, 0.0, 1.0], dtype=float))
        center_world = None
        source_label = "lvlh_radial_fallback"
    else:
        source_label = "segment_span"
        if isinstance(chosen_span, dict):
            raw_source = str(chosen_span.get("context_source", "")).strip().lower()
            if raw_source == "transfer_next_scan":
                source_label = "transfer_next_scan"
            elif raw_source == "transfer_previous_scan":
                source_label = "transfer_previous_scan"

    direction_raw = (
        "CW" if chosen_span is None else str(chosen_span.get("scan_direction", "CW"))
    )
    direction_cw = direction_raw.strip().upper() != "CCW"
    source_segment_index = (
        None
        if chosen_span is None
        else int(_safe_float(chosen_span.get("source_segment_index"), -1.0))
    )

    return PointingContext(
        axis_world=np.array(axis_vec, dtype=float),
        center_world=(
            None if center_world is None else np.array(center_world, dtype=float)
        ),
        direction_cw=bool(direction_cw),
        source=source_label,
        span_index=selected_idx,
        source_segment_index=(
            source_segment_index
            if isinstance(source_segment_index, int) and source_segment_index >= 0
            else None
        ),
    )


def compute_pointing_errors_deg(
    *,
    current_quat_wxyz: np.ndarray,
    reference_quat_wxyz: np.ndarray,
) -> tuple[float, float]:
    """Return (x_axis_error_deg, z_axis_error_deg) between current and reference body frames."""
    curr_x = _quat_rotate_vec_wxyz(
        current_quat_wxyz, np.array([1.0, 0.0, 0.0], dtype=float)
    )
    curr_z = _quat_rotate_vec_wxyz(
        current_quat_wxyz, np.array([0.0, 0.0, 1.0], dtype=float)
    )
    ref_x = _quat_rotate_vec_wxyz(
        reference_quat_wxyz, np.array([1.0, 0.0, 0.0], dtype=float)
    )
    ref_z = _quat_rotate_vec_wxyz(
        reference_quat_wxyz, np.array([0.0, 0.0, 1.0], dtype=float)
    )
    return _angle_between_deg(curr_x, ref_x), _angle_between_deg(curr_z, ref_z)


def resolve_object_visible_side(
    *,
    current_state: np.ndarray,
    context: PointingContext | None,
) -> str | None:
    """Return which camera side is object-facing (+Y / -Y) when center is available."""
    if context is None or context.center_world is None:
        return None
    if not isinstance(current_state, np.ndarray) or current_state.size < 7:
        return None
    pos = np.array(current_state[:3], dtype=float)
    radial = context.center_world - pos
    radial = radial - float(np.dot(radial, context.axis_world)) * context.axis_world
    radial_norm = float(np.linalg.norm(radial))
    if radial_norm <= 1e-9:
        return None
    radial_dir = radial / radial_norm
    curr_y = _quat_rotate_vec_wxyz(
        current_state[3:7], np.array([0.0, 1.0, 0.0], dtype=float)
    )
    return "+Y" if float(np.dot(curr_y, radial_dir)) >= 0.0 else "-Y"


class PointingGuardrail:
    """Continuous hold/reset guardrail for pointing-axis error bounds."""

    def __init__(
        self,
        *,
        enabled: bool = True,
        z_error_deg_max: float = 4.0,
        x_error_deg_max: float = 6.0,
        breach_hold_s: float = 0.30,
        clear_hold_s: float = 0.80,
    ):
        self.enabled = bool(enabled)
        self.z_error_deg_max = max(0.0, float(z_error_deg_max))
        self.x_error_deg_max = max(0.0, float(x_error_deg_max))
        self.breach_hold_s = max(0.0, float(breach_hold_s))
        self.clear_hold_s = max(0.0, float(clear_hold_s))
        self.status = PointingGuardrailStatus()

    def reset(self) -> None:
        self.status = PointingGuardrailStatus()

    def update(
        self,
        *,
        sim_time_s: float,
        x_error_deg: float | None,
        z_error_deg: float | None,
    ) -> PointingGuardrailStatus:
        if not self.enabled:
            self.reset()
            return self.status

        x_err = _safe_float(x_error_deg, default=0.0)
        z_err = _safe_float(z_error_deg, default=0.0)
        x_breach = x_err > self.x_error_deg_max
        z_breach = z_err > self.z_error_deg_max
        exceeds = x_breach or z_breach
        reason = (
            "z_axis_error"
            if z_breach and z_err >= x_err
            else ("x_axis_error" if x_breach else None)
        )

        if exceeds:
            self.status.clear_since_s = None
            if self.status.breach_since_s is None:
                self.status.breach_since_s = float(sim_time_s)
            if (
                self.status.breached
                or (float(sim_time_s) - self.status.breach_since_s)
                >= self.breach_hold_s
            ):
                self.status.breached = True
                self.status.last_reason = reason
        else:
            self.status.breach_since_s = None
            if self.status.breached:
                if self.status.clear_since_s is None:
                    self.status.clear_since_s = float(sim_time_s)
                if (float(sim_time_s) - self.status.clear_since_s) >= self.clear_hold_s:
                    self.status.breached = False
                    self.status.clear_since_s = None
                    self.status.last_reason = None
            else:
                self.status.clear_since_s = None

        return self.status


class ControllerModeManager:
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
        self.settle_velocity_align_scale = max(0.0, float(settle_velocity_align_scale))
        self.settle_angular_velocity_scale = max(
            0.0, float(settle_angular_velocity_scale)
        )
        self.hold_smoothness_scale = max(0.0, float(hold_smoothness_scale))
        self.hold_thruster_pair_scale = max(0.0, float(hold_thruster_pair_scale))

        self.state = ModeState()
        self._recover_enter_candidate_since: float | None = None
        self._recover_exit_candidate_since: float | None = None

    def reset(self, sim_time_s: float = 0.0) -> None:
        self.state = ModeState(current_mode="TRACK", entered_at_s=float(sim_time_s))
        self._recover_enter_candidate_since = None
        self._recover_exit_candidate_since = None

    def profile_for_mode(self, mode: ModeName) -> ModeProfile:
        if mode == "RECOVER":
            return ModeProfile(
                contour_scale=self.recover_contour_scale,
                lag_scale=self.recover_lag_scale,
                progress_scale=self.recover_progress_scale,
                attitude_scale=self.recover_attitude_scale,
            )
        if mode == "SETTLE":
            return ModeProfile(
                progress_scale=self.settle_progress_scale,
                terminal_pos_scale=self.settle_terminal_pos_scale,
                terminal_attitude_scale=self.settle_terminal_attitude_scale,
                velocity_align_scale=self.settle_velocity_align_scale,
                angular_velocity_scale=self.settle_angular_velocity_scale,
            )
        if mode == "HOLD":
            return ModeProfile(
                progress_scale=self.settle_progress_scale,
                terminal_pos_scale=self.settle_terminal_pos_scale,
                terminal_attitude_scale=self.settle_terminal_attitude_scale,
                velocity_align_scale=self.settle_velocity_align_scale,
                angular_velocity_scale=self.settle_angular_velocity_scale,
                smoothness_scale=self.hold_smoothness_scale,
                thruster_pair_scale=self.hold_thruster_pair_scale,
            )
        return ModeProfile()

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
    ) -> ModeState:
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
                if (
                    sim_time_s - self._recover_exit_candidate_since
                ) >= self.recover_exit_hold_s:
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


class TerminalSupervisor:
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
    ) -> CompletionGateStatus:
        sim_time_s = float(sim_time_s)
        all_thresholds_ok = bool(
            position_ok and angle_ok and velocity_ok and angular_velocity_ok
        )
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

        return CompletionGateStatus(
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


class ActuatorPolicy:
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


class ReferenceScheduler:
    """Reference horizon scheduler used for diagnostics and replay."""

    def build_slice(
        self,
        *,
        sim: Any,
        current_state: np.ndarray,
        mode: ModeName,
        horizon: int,
        dt: float,
    ) -> ReferenceSlice:
        knots: list[dict[str, Any]] = []

        try:
            s0 = float(getattr(sim.mpc_controller, "s", 0.0) or 0.0)
            path_len = float(
                sim._get_mission_path_length(compute_if_missing=True) or 0.0
            )
        except Exception:
            s0 = 0.0
            path_len = 0.0

        mpc_cfg = getattr(
            getattr(sim.simulation_config, "app_config", None), "mpc", None
        )
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
                    "tangent": [
                        float(tangent[0]),
                        float(tangent[1]),
                        float(tangent[2]),
                    ],
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

        return ReferenceSlice(dt=float(dt), knots=knots)


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
            speed = (
                constraints.get("speed_max") if isinstance(constraints, dict) else None
            )
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
    """Apply speed policy: min non-hold cap, then clamp to MPC bounds."""
    speed_candidate = (
        min(non_hold_segment_caps)
        if non_hold_segment_caps
        else float(default_path_speed)
    )
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


def compile_mission_runtime_plan(
    *,
    mission: Any,
    path_length_m: float,
    default_path_speed: float,
    path_speed_min: float,
    path_speed_max: float,
    hold_duration_s: float,
    margin_s: float = 30.0,
) -> MissionRuntimePlan:
    """Compile runtime plan metadata from mission and path characteristics."""
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

    return MissionRuntimePlan(
        path_length_m=max(0.0, float(path_length_m)),
        path_speed_mps=float(path_speed),
        estimated_eta_s=float(eta),
        required_duration_s=float(required_duration),
        hold_duration_s=max(0.0, float(hold_duration_s)),
        duration_margin_s=max(0.0, float(margin_s)),
        speed_policy="min_non_hold_segment_speed_then_mpc_clamp",
        non_hold_segment_speed_caps=[float(x) for x in caps],
    )
