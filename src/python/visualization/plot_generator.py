"""
Plot Generator for Visualization

Coordinates static plot generation for simulation analysis.

Most plotting implementations live in helper modules; this class keeps the
public plotting API and call ordering.
"""

import csv
import json
import re
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from config.constants import Constants
from config.models import AppConfig
from scipy.spatial.transform import Rotation
from simulation.artifact_paths import artifact_relative_path
from utils.orientation_utils import quat_angle_error
from visualization.actuator_plots import (
    generate_actuator_limits_plot,
    generate_command_vs_valve_tracking_plot,
    generate_control_effort_plot,
    generate_cumulative_impulse_delta_v_proxy_plot,
    generate_pwm_quantization_plot,
    generate_reaction_wheel_output_plot,
    generate_thruster_impulse_proxy_plot,
    generate_thruster_usage_plot,
    generate_thruster_valve_activity_plot,
)
from visualization.command_utils import (
    get_thruster_count as infer_thruster_count,
)
from visualization.diagnostics_plots import (
    generate_error_vs_solve_time_scatter_plot,
    generate_mpc_performance_plot,
    generate_path_shaping_note_plot,
    generate_solver_health_plot,
    generate_solver_iterations_and_status_timeline_plot,
    generate_timing_intervals_plot,
    generate_waypoint_progress_plot,
)
from visualization.plot_catalog import GROUP_BY_ID, PLOT_GROUPS, PLOT_SPECS
from visualization.plot_data_utils import (
    get_control_time_axis,
    resolve_data_frame_and_columns,
)
from visualization.plot_style import PlotStyle
from visualization.state_plots import (
    generate_constraint_violations_plot,
    generate_phase_attitude_rate_plot,
    generate_phase_position_velocity_plot,
    generate_translation_attitude_coupling_plot,
    generate_velocity_magnitude_plot,
    generate_velocity_tracking_plot,
)
from visualization.threshold_overlays import add_limit_overlay
from visualization.trajectory_plots import (
    generate_trajectory_3d_interactive_plot,
    generate_trajectory_3d_orientation_plot,
    generate_trajectory_plot,
)


class PlotSkippedError(RuntimeError):
    """Raised when a plot cannot be produced due to missing inputs."""


class PlotGenerator:
    """
    Generates performance analysis plots from simulation data.

    This class handles all static plot generation, separating plotting
    logic from data management and animation generation.
    """

    def __init__(
        self,
        data_accessor: Any,
        dt: float,
        system_title: str = "Satellite Control System",
        app_config: AppConfig | None = None,
    ):
        """
        Initialize plot generator.

        Args:
            data_accessor: Object with data access methods (_col, _row, _get_len)
            dt: Simulation timestep in seconds
            system_title: Title for plots
            app_config: Optional AppConfig for accessing configuration (v3.0.0)
        """
        self.data_accessor = data_accessor
        self.dt = dt
        self.system_title = system_title
        self.app_config = app_config
        self.last_plot_manifest: dict[str, Any] | None = None
        self._frame_meta_cache: tuple[str, np.ndarray] | None = None

    def _col(self, name: str) -> np.ndarray:
        """Get column data from data accessor."""
        return self.data_accessor._col(name)

    def _row(self, idx: int) -> dict[str, Any]:
        """Get row data from data accessor."""
        return self.data_accessor._row(idx)

    def _get_len(self) -> int:
        """Get data length from data accessor."""
        return self.data_accessor._get_len()

    def _get_quaternion_series(self, prefix: str) -> np.ndarray:
        """
        Get continuous quaternion series [N,4] in wxyz order.

        Prefers logged quaternion columns and falls back to Euler->quaternion
        conversion for legacy data files.
        """
        n = self._get_len()
        if n <= 0:
            return np.zeros((0, 4), dtype=float)

        qw = self._col(f"{prefix}_QW")
        qx = self._col(f"{prefix}_QX")
        qy = self._col(f"{prefix}_QY")
        qz = self._col(f"{prefix}_QZ")
        has_quat_cols = len(qw) == n and len(qx) == n and len(qy) == n and len(qz) == n

        if has_quat_cols:
            q = np.column_stack((qw, qx, qy, qz)).astype(float, copy=False)
        else:
            r = self._col(f"{prefix}_Roll")
            p = self._col(f"{prefix}_Pitch")
            y = self._col(f"{prefix}_Yaw")
            if len(r) != n or len(p) != n or len(y) != n:
                return np.zeros((0, 4), dtype=float)
            q_xyzw = Rotation.from_euler(
                "xyz", np.column_stack((r, p, y)), degrees=False
            ).as_quat()
            q = np.column_stack(
                (q_xyzw[:, 3], q_xyzw[:, 0], q_xyzw[:, 1], q_xyzw[:, 2])
            )

        # Normalize and enforce sign continuity (q and -q represent same rotation).
        norms = np.linalg.norm(q, axis=1, keepdims=True)
        norms[norms <= 1e-12] = 1.0
        q = q / norms
        for i in range(1, len(q)):
            if float(np.dot(q[i], q[i - 1])) < 0.0:
                q[i] = -q[i]
        return q

    def _get_euler_series_unwrapped(self, prefix: str) -> np.ndarray:
        """
        Get continuous Euler xyz series [N,3] in radians for display plots.

        Uses quaternion columns when available, then unwraps each component to avoid
        artificial +/-180 or 360 display discontinuities.
        """
        n = self._get_len()
        if n <= 0:
            return np.zeros((0, 3), dtype=float)

        q = self._get_quaternion_series(prefix)
        if len(q) == n:
            q_xyzw = np.column_stack((q[:, 1], q[:, 2], q[:, 3], q[:, 0]))
            e = Rotation.from_quat(q_xyzw).as_euler("xyz", degrees=False)
        else:
            r = self._col(f"{prefix}_Roll")
            p = self._col(f"{prefix}_Pitch")
            y = self._col(f"{prefix}_Yaw")
            if len(r) != n or len(p) != n or len(y) != n:
                return np.zeros((0, 3), dtype=float)
            e = np.column_stack((r, p, y)).astype(float, copy=False)

        return np.unwrap(e, axis=0)

    def generate_all_plots(self, plot_dir: Path) -> None:
        """
        Generate full post-run plot suite using declarative catalog order.

        Args:
            plot_dir: Directory to save plots
        """
        print("Generating performance analysis plots...")
        PlotStyle.apply_global_theme()
        plot_dir.mkdir(parents=True, exist_ok=True)
        print(f" Created Plots directory: {plot_dir}")

        grouped_dirs = {
            group.id: plot_dir / group.folder
            for group in sorted(PLOT_GROUPS, key=lambda g: g.order)
        }
        for out_dir in grouped_dirs.values():
            out_dir.mkdir(parents=True, exist_ok=True)

        run_dir = self._run_dir_for_outputs(plot_dir)
        files_manifest: list[dict[str, Any]] = []
        specs_manifest: list[dict[str, Any]] = []
        failures: list[dict[str, Any]] = []

        for spec in sorted(PLOT_SPECS, key=lambda s: s.order):
            group = GROUP_BY_ID[spec.group_id]
            output_path = grouped_dirs[group.id] / spec.filename
            output_path.parent.mkdir(parents=True, exist_ok=True)
            renderer = getattr(self, spec.renderer, None)
            if renderer is None:
                reason = f"Renderer {spec.renderer} not found"
                specs_manifest.append(
                    {
                        "plot_id": spec.plot_id,
                        "title": spec.title,
                        "group_id": spec.group_id,
                        "order": spec.order,
                        "status": "failed",
                        "reason": reason,
                        "outputs": [],
                    }
                )
                failures.append({"plot_id": spec.plot_id, "reason": reason})
                continue

            try:
                produced = renderer(output_path, spec=spec) or []
                if not produced and output_path.exists():
                    produced = [output_path]
                if not produced:
                    raise PlotSkippedError("No output artifact generated")

                output_rel_paths: list[str] = []
                for idx, produced_path in enumerate(produced):
                    rel_path = str(produced_path.relative_to(run_dir))
                    output_rel_paths.append(rel_path)
                    plot_id = spec.plot_id
                    if len(produced) > 1:
                        plot_id = self._build_multi_plot_id(
                            base_plot_id=spec.plot_id,
                            artifact_path=produced_path,
                            fallback_index=idx + 1,
                        )
                    files_manifest.append(
                        {
                            "plot_id": plot_id,
                            "title": spec.title,
                            "path": rel_path,
                            "order": spec.order * 100 + idx,
                            "group_id": spec.group_id,
                            "format": produced_path.suffix.lower().lstrip("."),
                            "interactive": produced_path.suffix.lower() == ".html",
                            "status": "ok",
                        }
                    )

                specs_manifest.append(
                    {
                        "plot_id": spec.plot_id,
                        "title": spec.title,
                        "group_id": spec.group_id,
                        "order": spec.order,
                        "status": "ok",
                        "reason": None,
                        "outputs": output_rel_paths,
                    }
                )
            except PlotSkippedError as exc:
                reason = str(exc)
                specs_manifest.append(
                    {
                        "plot_id": spec.plot_id,
                        "title": spec.title,
                        "group_id": spec.group_id,
                        "order": spec.order,
                        "status": "skipped",
                        "reason": reason,
                        "outputs": [],
                    }
                )
                failures.append({"plot_id": spec.plot_id, "reason": reason})
            except Exception as exc:
                reason = str(exc)
                specs_manifest.append(
                    {
                        "plot_id": spec.plot_id,
                        "title": spec.title,
                        "group_id": spec.group_id,
                        "order": spec.order,
                        "status": "failed",
                        "reason": reason,
                        "outputs": [],
                    }
                )
                failures.append({"plot_id": spec.plot_id, "reason": reason})

        groups_manifest = []
        for group in sorted(PLOT_GROUPS, key=lambda g: g.order):
            group_path = plot_dir / group.folder
            groups_manifest.append(
                {
                    "id": group.id,
                    "order": group.order,
                    "title": group.title,
                    "path": str(group_path.relative_to(run_dir)),
                    "file_count": sum(
                        1 for item in files_manifest if item["group_id"] == group.id
                    ),
                }
            )

        manifest = {
            "schema_version": "plot_manifest_v2",
            "suite_version": "postrun_v2_full",
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "groups": groups_manifest,
            "specs": specs_manifest,
            "files": sorted(files_manifest, key=lambda item: int(item["order"])),
            "failures": failures,
        }
        self.last_plot_manifest = manifest
        (plot_dir / "plot_manifest.json").write_text(
            json.dumps(manifest, indent=2),
            encoding="utf-8",
        )
        print(f"Performance plots saved to: {plot_dir}")

    def _build_multi_plot_id(
        self, *, base_plot_id: str, artifact_path: Path, fallback_index: int
    ) -> str:
        match = re.search(r"(\d+)", artifact_path.stem)
        if match:
            return f"{base_plot_id}.{int(match.group(1)):02d}"
        return f"{base_plot_id}.{fallback_index:02d}"

    def _run_dir_for_outputs(self, plot_dir: Path) -> Path:
        if plot_dir.name == "Plots":
            return plot_dir.parent
        if plot_dir.parent.name == "Plots":
            return plot_dir.parent.parent
        output_dir = getattr(self.data_accessor, "output_dir", None)
        if output_dir:
            return Path(output_dir)
        return plot_dir.parent

    def _run_dir(self) -> Path | None:
        output_dir = getattr(self.data_accessor, "output_dir", None)
        if output_dir:
            return Path(output_dir)
        csv_path = getattr(self.data_accessor, "csv_path", None)
        if csv_path is not None:
            return Path(csv_path).parent
        return None

    def _load_json_artifact(self, name: str) -> dict[str, Any]:
        run_dir = self._run_dir()
        if run_dir is None:
            return {}
        candidates = [run_dir / artifact_relative_path(name), run_dir / name]
        path = next(
            (candidate for candidate in candidates if candidate.exists()), candidates[0]
        )
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(payload, dict):
                return payload
            return {}
        except Exception:
            return {}

    def _load_csv_artifact_rows(self, name: str) -> list[dict[str, str]]:
        run_dir = self._run_dir()
        if run_dir is None:
            return []
        candidates = [run_dir / artifact_relative_path(name), run_dir / name]
        path = next(
            (candidate for candidate in candidates if candidate.exists()), candidates[0]
        )
        if not path.exists():
            return []
        try:
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                return [dict(row) for row in reader]
        except Exception:
            return []

    def _load_jsonl_artifact_rows(self, name: str) -> list[dict[str, Any]]:
        run_dir = self._run_dir()
        if run_dir is None:
            return []
        candidates = [run_dir / artifact_relative_path(name), run_dir / name]
        path = next(
            (candidate for candidate in candidates if candidate.exists()), candidates[0]
        )
        if not path.exists():
            return []
        rows: list[dict[str, Any]] = []
        try:
            for line in path.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if not line:
                    continue
                payload = json.loads(line)
                if isinstance(payload, dict):
                    rows.append(payload)
        except Exception:
            return []
        return rows

    def _get_time_axis(self) -> np.ndarray:
        df, cols = resolve_data_frame_and_columns(self.data_accessor)
        return np.array(
            get_control_time_axis(
                df=df,
                cols=cols,
                fallback_len=self._get_len(),
                dt=float(self.dt),
            ),
            dtype=float,
        )

    def _col_float(self, name: str, size: int | None = None) -> np.ndarray:
        raw = self._col(name)
        if size is None:
            size = len(raw)
        out = np.zeros(int(size), dtype=float)
        if len(raw) == 0:
            return out
        limit = min(len(raw), int(size))
        for idx in range(limit):
            try:
                out[idx] = float(raw[idx])
            except (ValueError, TypeError):
                out[idx] = 0.0
        return out

    def _resolve_frame_metadata(self) -> tuple[str, np.ndarray]:
        if self._frame_meta_cache is not None:
            return self._frame_meta_cache

        metadata = self._load_json_artifact("mission_metadata.json")
        frame = str(metadata.get("planned_path_frame", "LVLH")).upper()
        if frame not in {"ECI", "LVLH"}:
            frame = "LVLH"

        origin_raw = metadata.get("frame_origin", [0.0, 0.0, 0.0])
        origin = np.zeros(3, dtype=float)
        if isinstance(origin_raw, list | tuple) and len(origin_raw) >= 3:
            for i in range(3):
                try:
                    origin[i] = float(origin_raw[i])
                except (ValueError, TypeError):
                    origin[i] = 0.0

        self._frame_meta_cache = (frame, origin)
        return self._frame_meta_cache

    def _position_series_lvlh(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        time = self._get_time_axis()
        n = len(time)
        cx = self._col_float("Current_X", n)
        cy = self._col_float("Current_Y", n)
        cz = self._col_float("Current_Z", n)
        rx = self._col_float("Reference_X", n)
        ry = self._col_float("Reference_Y", n)
        rz = self._col_float("Reference_Z", n)

        frame, origin_const = self._resolve_frame_metadata()
        ox = self._col_float("Frame_Origin_X", n)
        oy = self._col_float("Frame_Origin_Y", n)
        oz = self._col_float("Frame_Origin_Z", n)
        if not np.any(np.abs(ox)) and not np.any(np.abs(oy)) and not np.any(np.abs(oz)):
            ox = np.full(n, origin_const[0], dtype=float)
            oy = np.full(n, origin_const[1], dtype=float)
            oz = np.full(n, origin_const[2], dtype=float)

        def looks_absolute(
            arr_x: np.ndarray, arr_y: np.ndarray, arr_z: np.ndarray
        ) -> bool:
            if len(arr_x) == 0:
                return False
            stacked = np.vstack((arr_x, arr_y, arr_z))
            return float(np.nanmax(np.abs(stacked))) > 1.0e5 and (
                np.any(np.abs(ox) > 1.0)
                or np.any(np.abs(oy) > 1.0)
                or np.any(np.abs(oz) > 1.0)
            )

        should_convert = (
            frame == "ECI" or looks_absolute(cx, cy, cz) or looks_absolute(rx, ry, rz)
        )
        if should_convert:
            cx = cx - ox
            cy = cy - oy
            cz = cz - oz
            rx = rx - ox
            ry = ry - oy
            rz = rz - oz
        return cx, cy, cz, rx, ry, rz

    def _thresholds(self) -> dict[str, float]:
        pos_tol = float(Constants.POSITION_TOLERANCE)
        angle_tol_deg = float(np.degrees(Constants.ANGLE_TOLERANCE))
        max_linear_velocity = 0.0
        max_angular_velocity_degps = 0.0
        solver_limit_s = float(Constants.MPC_SOLVER_TIME_LIMIT)

        if self.app_config is not None and getattr(self.app_config, "mpc", None):
            mpc_cfg = self.app_config.mpc
            max_linear_velocity = float(getattr(mpc_cfg, "max_linear_velocity", 0.0))
            max_angular_velocity_degps = float(
                np.degrees(float(getattr(mpc_cfg, "max_angular_velocity", 0.0)))
            )
            solver_limit_s = float(
                getattr(mpc_cfg, "solver_time_limit", solver_limit_s)
            )

        constraints = self._load_json_artifact("constraint_violations.json")
        limits = constraints.get("limits", {}) if isinstance(constraints, dict) else {}
        if max_linear_velocity <= 0.0:
            try:
                max_linear_velocity = float(limits.get("max_linear_velocity_mps", 0.0))
            except (ValueError, TypeError):
                max_linear_velocity = 0.0
        if max_angular_velocity_degps <= 0.0:
            try:
                max_angular_velocity_degps = float(
                    np.degrees(float(limits.get("max_angular_velocity_radps", 0.0)))
                )
            except (ValueError, TypeError):
                max_angular_velocity_degps = 0.0
        if solver_limit_s <= 0.0:
            try:
                solver_limit_s = float(limits.get("solver_time_limit_s", 0.0))
            except (ValueError, TypeError):
                solver_limit_s = 0.0

        return {
            "position_tolerance_m": max(pos_tol, 0.0),
            "attitude_tolerance_deg": max(angle_tol_deg, 0.0),
            "velocity_limit_mps": max(max_linear_velocity, 0.0),
            "angular_rate_limit_degps": max(max_angular_velocity_degps, 0.0),
            "solver_time_limit_s": max(solver_limit_s, 0.0),
        }

    def _rename_output(self, source: Path, target: Path) -> Path:
        if not source.exists():
            raise PlotSkippedError(f"Expected artifact missing: {source.name}")
        if source.resolve() == target.resolve():
            return target
        if target.exists():
            target.unlink()
        source.rename(target)
        return target

    def _render_from_legacy(
        self,
        output_path: Path,
        *,
        legacy_renderer,
        legacy_filename: str,
    ) -> list[Path]:
        legacy_renderer(output_path.parent)
        source = output_path.parent / legacy_filename
        return [self._rename_output(source, output_path)]

    def _render_mission_overview(self, output_path: Path, *, spec: Any) -> list[Path]:
        kpi = self._load_json_artifact("kpi_summary.json")
        constraints = self._load_json_artifact("constraint_violations.json")
        run_status = self._load_json_artifact("run_status.json")
        controller_health = self._load_json_artifact("controller_health.json")

        status = str(run_status.get("status", "unknown"))
        path_completed = bool(kpi.get("path_completed", False))
        final_pos = float(kpi.get("final_position_error_m", 0.0) or 0.0)
        final_ang = float(kpi.get("final_angle_error_deg", 0.0) or 0.0)
        final_time = float(kpi.get("final_time_s", 0.0) or 0.0)
        solve_mean = float(kpi.get("mpc_mean_solve_time_ms", 0.0) or 0.0)
        solve_max = float(kpi.get("mpc_max_solve_time_ms", 0.0) or 0.0)
        dominant_failure = ""
        violations = (
            constraints.get("violations", []) if isinstance(constraints, dict) else []
        )
        if isinstance(violations, list) and violations:
            dominant_failure = str(violations[0].get("type", "constraint_violation"))
        else:
            dominant_failure = str(run_status.get("status_detail", "")) or "none"

        fig, ax = plt.subplots(1, 1, figsize=(12, 6))
        ax.axis("off")
        lines = [
            "Mission Debrief Overview",
            "",
            f"Run Status: {status}",
            f"Path Completed: {'YES' if path_completed else 'NO'}",
            f"Constraint Pass: {'YES' if constraints.get('pass', True) else 'NO'}",
            f"Final Position Error: {final_pos:.4f} m",
            f"Final Attitude Error: {final_ang:.3f} deg",
            f"Final Time: {final_time:.3f} s",
            f"MPC Solve Mean/Max: {solve_mean:.3f} / {solve_max:.3f} ms",
            "",
            f"Dominant Failure Reason: {dominant_failure}",
        ]
        fallback_count = (
            controller_health.get("solver_health", {}).get("fallback_count")
            if isinstance(controller_health, dict)
            else 0
        )
        lines.append(f"Fallback Count: {int(fallback_count or 0)}")
        ax.text(
            0.02,
            0.98,
            "\n".join(lines),
            transform=ax.transAxes,
            va="top",
            ha="left",
            fontsize=12,
            family="monospace",
            bbox=PlotStyle.TEXTBOX_STYLE,
        )
        ax.set_title("Mission Overview", fontsize=16)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_constraints_overview(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        constraints = self._load_json_artifact("constraint_violations.json")
        violations = (
            constraints.get("violations", []) if isinstance(constraints, dict) else []
        )
        limits = constraints.get("limits", {}) if isinstance(constraints, dict) else {}
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle("Constraints Overview")

        if isinstance(violations, list) and violations:
            labels = [str(item.get("type", "unknown")) for item in violations]
            counts = [int(float(item.get("count", 0) or 0)) for item in violations]
            axes[0].bar(labels, counts, color=PlotStyle.COLOR_ERROR, alpha=0.85)
            axes[0].set_ylabel("Violation Count", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            axes[0].set_title("Violation Counts by Type")
            axes[0].tick_params(axis="x", rotation=25)
            axes[0].grid(True, axis="y", alpha=PlotStyle.GRID_ALPHA)
        else:
            axes[0].text(
                0.5,
                0.5,
                "No violations recorded",
                ha="center",
                va="center",
                transform=axes[0].transAxes,
                fontsize=PlotStyle.ANNOTATION_SIZE,
            )
            axes[0].set_title("Violation Counts by Type")
            axes[0].axis("off")

        axes[1].axis("off")
        lines = [
            "Configured Thresholds",
            "",
            f"Max Linear Velocity: {float(limits.get('max_linear_velocity_mps', 0.0) or 0.0):.4f} m/s",
            f"Max Angular Velocity: {float(np.degrees(float(limits.get('max_angular_velocity_radps', 0.0) or 0.0))):.4f} deg/s",
            f"Solver Time Limit: {float(limits.get('solver_time_limit_s', 0.0) or 0.0):.4f} s",
            "",
            f"Constraint Pass: {'YES' if constraints.get('pass', True) else 'NO'}",
        ]
        axes[1].text(
            0.02,
            0.98,
            "\n".join(lines),
            transform=axes[1].transAxes,
            va="top",
            ha="left",
            fontsize=11,
            family="monospace",
            bbox=PlotStyle.TEXTBOX_STYLE,
        )
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_controller_health_overview(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        controller_health = self._load_json_artifact("controller_health.json")
        kpi = self._load_json_artifact("kpi_summary.json")

        fallback_count = int(
            (
                controller_health.get("solver_health", {}).get("fallback_count", 0)
                if isinstance(controller_health, dict)
                else 0
            )
            or 0
        )
        hard_breaches = int(
            (
                controller_health.get("solver_health", {}).get("hard_limit_breaches", 0)
                if isinstance(controller_health, dict)
                else 0
            )
            or 0
        )
        status = (
            str(controller_health.get("solver_health", {}).get("status", "unknown"))
            if isinstance(controller_health, dict)
            else "unknown"
        )
        solve_mean = float(kpi.get("mpc_mean_solve_time_ms", 0.0) or 0.0)
        solve_p95 = float(kpi.get("mpc_p95_solve_time_ms", 0.0) or 0.0)
        solve_max = float(kpi.get("mpc_max_solve_time_ms", 0.0) or 0.0)

        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        fig.suptitle("Controller Health Overview")

        axes[0].bar(
            ["fallback", "hard_breach"],
            [fallback_count, hard_breaches],
            color=[PlotStyle.COLOR_SIGNAL_ANG, PlotStyle.COLOR_ERROR],
            alpha=0.85,
        )
        axes[0].set_title("Fallback / Breach Counts")
        axes[0].grid(True, axis="y", alpha=PlotStyle.GRID_ALPHA)

        axes[1].bar(
            ["mean", "p95", "max"],
            [solve_mean, solve_p95, solve_max],
            color=PlotStyle.COLOR_SIGNAL_POS,
            alpha=0.85,
        )
        axes[1].set_title("Solve Time (ms)")
        axes[1].grid(True, axis="y", alpha=PlotStyle.GRID_ALPHA)

        axes[2].axis("off")
        axes[2].text(
            0.02,
            0.98,
            "\n".join(
                [
                    f"Solver Status: {status}",
                    f"Fallback Active: {bool(controller_health.get('solver_health', {}).get('fallback_active', False))}",
                    f"Controller Core: {controller_health.get('controller_core', 'unknown')}",
                    f"Solver Backend: {controller_health.get('solver_backend', 'unknown')}",
                ]
            ),
            transform=axes[2].transAxes,
            va="top",
            ha="left",
            fontsize=11,
            family="monospace",
            bbox=PlotStyle.TEXTBOX_STYLE,
        )
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_trajectory_projection(
        self,
        output_path: Path,
        *,
        axis_a: str,
        axis_b: str,
        title: str,
    ) -> list[Path]:
        cx, cy, cz, rx, ry, rz = self._position_series_lvlh()
        data_map = {
            "X": (cx, rx),
            "Y": (cy, ry),
            "Z": (cz, rz),
        }
        cur_a, ref_a = data_map[axis_a]
        cur_b, ref_b = data_map[axis_b]

        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.plot(
            cur_a,
            cur_b,
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH_THICK,
            label="Measured",
        )
        if len(cur_a) > 0:
            ax.scatter(
                cur_a[0],
                cur_b[0],
                s=45,
                color=PlotStyle.COLOR_SUCCESS,
                label="Start",
                zorder=4,
            )
            ax.scatter(
                cur_a[-1],
                cur_b[-1],
                s=45,
                color=PlotStyle.COLOR_ERROR,
                label="End",
                zorder=4,
            )
        if len(ref_a) > 0:
            ax.scatter(
                ref_a[-1],
                ref_b[-1],
                s=70,
                color=PlotStyle.COLOR_TARGET,
                marker="x",
                label="Reference",
                zorder=5,
            )
        ax.set_xlabel(f"{axis_a} LVLH (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel(f"{axis_b} LVLH (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title(title)
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        ax.set_aspect("equal", adjustable="box")
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_trajectory_xy_lvlh(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_trajectory_projection(
            output_path,
            axis_a="X",
            axis_b="Y",
            title="Trajectory XY - LVLH",
        )

    def _render_trajectory_xz_lvlh(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_trajectory_projection(
            output_path,
            axis_a="X",
            axis_b="Z",
            title="Trajectory XZ - LVLH",
        )

    def _render_trajectory_yz_lvlh(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_trajectory_projection(
            output_path,
            axis_a="Y",
            axis_b="Z",
            title="Trajectory YZ - LVLH",
        )

    def _render_trajectory_3d_orientation(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        cx, cy, cz, *_ = self._position_series_lvlh()
        if len(cx) == 0:
            raise PlotSkippedError("No trajectory data available")
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(cx, cy, cz, color=PlotStyle.COLOR_SIGNAL_POS, linewidth=2, label="Path")
        ax.scatter(
            cx[0], cy[0], cz[0], color=PlotStyle.COLOR_SUCCESS, s=40, label="Start"
        )
        ax.scatter(
            cx[-1], cy[-1], cz[-1], color=PlotStyle.COLOR_ERROR, s=40, label="End"
        )

        q_cur = self._get_quaternion_series("Current")
        n = min(len(cx), len(q_cur))
        if n > 1:
            idxs = np.arange(0, n, max(n // 50, 1))
            q_xyzw = np.column_stack(
                (q_cur[idxs, 1], q_cur[idxs, 2], q_cur[idxs, 3], q_cur[idxs, 0])
            )
            dirs = Rotation.from_quat(q_xyzw).apply(np.array([1.0, 0.0, 0.0]))
            ax.quiver(
                cx[idxs],
                cy[idxs],
                cz[idxs],
                dirs[:, 0],
                dirs[:, 1],
                dirs[:, 2],
                length=0.15,
                normalize=True,
                color=PlotStyle.COLOR_MUTED,
                alpha=0.6,
            )

        ax.set_xlabel("X LVLH (m)")
        ax.set_ylabel("Y LVLH (m)")
        ax.set_zlabel("Z LVLH (m)")
        ax.set_title("3D Trajectory Orientation - LVLH")
        ax.legend()
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_trajectory_3d_interactive(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        generate_trajectory_3d_interactive_plot(self, output_path.parent)
        source = output_path.parent / "trajectory_3d_interactive.html"
        return [self._rename_output(source, output_path)]

    def _render_position_tracking_xyz(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        cx, cy, cz, rx, ry, rz = self._position_series_lvlh()
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        fig.suptitle(f"Position Tracking XYZ - {self.system_title}")
        axes_data = [
            ("X", cx, rx),
            ("Y", cy, ry),
            ("Z", cz, rz),
        ]
        for ax, (label, cur, ref) in zip(axes, axes_data):
            ax.plot(
                time, cur, color=PlotStyle.COLOR_SIGNAL_POS, label=f"Current {label}"
            )
            ax.plot(
                time,
                ref,
                color=PlotStyle.COLOR_SIGNAL_ANG,
                linestyle="--",
                label=f"Reference {label}",
            )
            ax.set_ylabel(f"{label} LVLH (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[-1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_position_error_xyz_with_limits(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        cx, cy, cz, rx, ry, rz = self._position_series_lvlh()
        thresholds = self._thresholds()
        pos_tol = thresholds["position_tolerance_m"]
        errs = [cx - rx, cy - ry, cz - rz]
        labels = ["X", "Y", "Z"]

        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        fig.suptitle(f"Position Error XYZ With Limits - {self.system_title}")
        for ax, err, label in zip(axes, errs, labels):
            ax.plot(time, err, color=PlotStyle.COLOR_ERROR, label=f"{label} Error")
            add_limit_overlay(
                ax,
                time_s=time,
                values=err,
                limit=pos_tol,
                label=f"Tolerance ±{pos_tol:.3f} m",
                symmetric=True,
            )
            ax.set_ylabel(f"{label} Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[-1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_position_error_norm_with_limit(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        cx, cy, cz, rx, ry, rz = self._position_series_lvlh()
        pos_tol = self._thresholds()["position_tolerance_m"]
        err_norm = np.sqrt((cx - rx) ** 2 + (cy - ry) ** 2 + (cz - rz) ** 2)

        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.plot(
            time,
            err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Position Error Norm",
        )
        add_limit_overlay(
            ax,
            time_s=time,
            values=err_norm,
            limit=pos_tol,
            label=f"Tolerance {pos_tol:.3f} m",
            symmetric=False,
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Error Norm (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title("Position Error Norm With Limit")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_velocity_tracking_xyz(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        n = len(time)
        cvx = self._col_float("Current_VX", n)
        cvy = self._col_float("Current_VY", n)
        cvz = self._col_float("Current_VZ", n)
        rvx = self._col_float("Reference_VX", n)
        rvy = self._col_float("Reference_VY", n)
        rvz = self._col_float("Reference_VZ", n)
        v_limit = self._thresholds()["velocity_limit_mps"]

        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        fig.suptitle(f"Velocity Tracking XYZ - {self.system_title}")
        axes_data = [("VX", cvx, rvx), ("VY", cvy, rvy), ("VZ", cvz, rvz)]
        for ax, (label, cur, ref) in zip(axes, axes_data):
            ax.plot(
                time, cur, color=PlotStyle.COLOR_SIGNAL_POS, label=f"Current {label}"
            )
            ax.plot(
                time,
                ref,
                color=PlotStyle.COLOR_SIGNAL_ANG,
                linestyle="--",
                label=f"Reference {label}",
            )
            if v_limit > 0.0:
                add_limit_overlay(
                    ax,
                    time_s=time,
                    values=cur,
                    limit=v_limit,
                    label=f"Velocity Limit ±{v_limit:.3f} m/s",
                    symmetric=True,
                )
            ax.set_ylabel(f"{label} (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[-1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_velocity_error_norm_with_limit(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        n = len(time)
        evx = self._col_float("Error_VX", n)
        evy = self._col_float("Error_VY", n)
        evz = self._col_float("Error_VZ", n)
        err_norm = np.sqrt(evx**2 + evy**2 + evz**2)
        vel_limit = self._thresholds()["velocity_limit_mps"]

        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.plot(
            time, err_norm, color=PlotStyle.COLOR_ERROR, label="Velocity Error Norm"
        )
        add_limit_overlay(
            ax,
            time_s=time,
            values=err_norm,
            limit=vel_limit,
            label=f"Velocity Limit {vel_limit:.3f} m/s",
            symmetric=False,
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Velocity Error Norm (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title("Velocity Error Norm With Limit")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_attitude_tracking_quaternion(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_angular_tracking_plot,
            legacy_filename="attitude_tracking.png",
        )

    def _render_attitude_error_quaternion_with_limit(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        n = min(len(time), len(q_cur), len(q_ref))
        if n == 0:
            raise PlotSkippedError("Quaternion series not available")

        angle_err_deg = np.degrees(
            np.array(
                [quat_angle_error(q_ref[i], q_cur[i]) for i in range(n)], dtype=float
            )
        )
        limit_deg = self._thresholds()["attitude_tolerance_deg"]
        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.plot(
            time[:n],
            angle_err_deg,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Quaternion Geodesic Error",
        )
        add_limit_overlay(
            ax,
            time_s=time[:n],
            values=angle_err_deg,
            limit=limit_deg,
            label=f"Attitude Tolerance {limit_deg:.3f} deg",
            symmetric=False,
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Attitude Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title("Attitude Error Quaternion With Limit")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_angular_rate_error_with_limit(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        n = len(time)
        err_wx = self._col_float("Error_WX", n)
        err_wy = self._col_float("Error_WY", n)
        err_wz = self._col_float("Error_WZ", n)
        err_deg = np.degrees(np.sqrt(err_wx**2 + err_wy**2 + err_wz**2))
        limit_deg = self._thresholds()["angular_rate_limit_degps"]

        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.plot(
            time, err_deg, color=PlotStyle.COLOR_ERROR, label="Angular Rate Error Norm"
        )
        add_limit_overlay(
            ax,
            time_s=time,
            values=err_deg,
            limit=limit_deg,
            label=f"Angular Rate Limit {limit_deg:.3f} deg/s",
            symmetric=False,
        )
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Error (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title("Angular Rate Error With Limit")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_thruster_usage_summary(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_thruster_usage_plot,
            legacy_filename="thruster_usage.png",
        )

    def _render_thruster_valve_activity_aggregate(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_thruster_valve_activity_plot,
            legacy_filename="thruster_valve_activity.png",
        )

    def _render_command_vs_valve_tracking(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_command_vs_valve_tracking_plot,
            legacy_filename="command_vs_valve_tracking.png",
        )

    def _render_pwm_duty_cycles(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_pwm_quantization_plot,
            legacy_filename="pwm_duty_cycles.png",
        )

    def _render_control_effort(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_control_effort_plot,
            legacy_filename="control_effort.png",
        )

    def _render_reaction_wheel_output(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_reaction_wheel_output_plot,
            legacy_filename="reaction_wheel_output.png",
        )

    def _render_actuator_limits_with_overlays(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_actuator_limits_plot,
            legacy_filename="actuator_limits.png",
        )

    def _render_thruster_impulse_proxy(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_thruster_impulse_proxy_plot,
            legacy_filename="thruster_impulse_proxy.png",
        )

    def _render_cumulative_impulse_delta_v_proxy(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_cumulative_impulse_delta_v_proxy_plot,
            legacy_filename="cumulative_impulse_and_delta_v_proxy.png",
        )

    def _render_per_thruster_valve_activity(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        path_dir = output_path.parent
        per_thruster = sorted(path_dir.glob("thruster_valve_activity_thruster_*.png"))
        if not per_thruster:
            self.generate_thruster_valve_activity_plot(path_dir)
            per_thruster = sorted(
                path_dir.glob("thruster_valve_activity_thruster_*.png")
            )
        if not per_thruster:
            raise PlotSkippedError("Per-thruster valve activity unavailable")

        outputs: list[Path] = []
        for src in per_thruster:
            match = re.search(r"thruster_(\d+)\.png$", src.name)
            if match is None:
                continue
            tid = int(match.group(1))
            dst = path_dir / f"10_thruster_{tid:02d}_valve_activity.png"
            outputs.append(self._rename_output(src, dst))
        if not outputs:
            raise PlotSkippedError("No per-thruster valve activity files generated")
        return outputs

    def _render_mpc_solve_time_with_limit(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_mpc_performance_plot,
            legacy_filename="mpc_performance.png",
        )

    def _render_solver_health_timeline(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_solver_health_plot,
            legacy_filename="solver_health.png",
        )

    def _render_solver_iterations_and_status(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_solver_iterations_and_status_timeline_plot,
            legacy_filename="solver_iterations_and_status_timeline.png",
        )

    def _render_timing_intervals(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_timing_intervals_plot,
            legacy_filename="timing_intervals.png",
        )

    def _render_error_vs_solve_time_scatter(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_error_vs_solve_time_scatter_plot,
            legacy_filename="error_vs_solve_time_scatter.png",
        )

    def _render_fallback_and_breach_timeline(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        n = len(time)
        fallback = self._col("MPC_Fallback_Used")
        exceeded = self._col("MPC_Time_Limit_Exceeded")
        if len(fallback) == 0 and len(exceeded) == 0:
            raise PlotSkippedError("Fallback/breach columns unavailable")

        fallback_flags = np.zeros(n, dtype=float)
        exceeded_flags = np.zeros(n, dtype=float)
        for i in range(min(n, len(fallback))):
            fallback_flags[i] = (
                1.0
                if str(fallback[i]).strip().lower() in {"1", "true", "yes", "on"}
                else 0.0
            )
        for i in range(min(n, len(exceeded))):
            exceeded_flags[i] = (
                1.0
                if str(exceeded[i]).strip().lower() in {"1", "true", "yes", "on"}
                else 0.0
            )

        fig, ax = plt.subplots(1, 1, figsize=PlotStyle.FIGSIZE_SINGLE)
        ax.step(
            time,
            fallback_flags,
            where="post",
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
            label="Fallback Used",
        )
        ax.step(
            time,
            exceeded_flags,
            where="post",
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Solve Time Limit Exceeded",
        )
        ax.set_ylim(-0.1, 1.2)
        ax.set_yticks([0, 1])
        ax.set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_ylabel("Flag", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        ax.set_title("Fallback And Breach Timeline")
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_waypoint_progress(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_waypoint_progress_plot,
            legacy_filename="waypoint_progress.png",
        )

    def _render_mode_timeline(self, output_path: Path, *, spec: Any) -> list[Path]:
        rows = self._load_csv_artifact_rows("mode_timeline.csv")
        if not rows:
            raise PlotSkippedError("mode_timeline.csv not available")
        time = np.array(
            [float(row.get("time_s", 0.0) or 0.0) for row in rows], dtype=float
        )
        mode_labels: list[str] = []
        mode_codes: list[int] = []
        for row in rows:
            mode = str(row.get("mode", "") or "UNKNOWN")
            if mode not in mode_labels:
                mode_labels.append(mode)
            mode_codes.append(mode_labels.index(mode))
        time_in_mode = np.array(
            [float(row.get("time_in_mode_s", 0.0) or 0.0) for row in rows], dtype=float
        )

        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        axes[0].step(time, mode_codes, where="post", color=PlotStyle.COLOR_SIGNAL_POS)
        axes[0].set_yticks(range(len(mode_labels)))
        axes[0].set_yticklabels(mode_labels)
        axes[0].set_ylabel("Mode", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

        axes[1].plot(time, time_in_mode, color=PlotStyle.COLOR_SIGNAL_ANG)
        axes[1].set_ylabel("Time In Mode (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        fig.suptitle("Mode Timeline")
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_completion_gate_trace(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        rows = self._load_csv_artifact_rows("completion_gate_trace.csv")
        if not rows:
            raise PlotSkippedError("completion_gate_trace.csv not available")
        time = np.array(
            [float(row.get("time_s", 0.0) or 0.0) for row in rows], dtype=float
        )
        gate_ok = np.array(
            [float(row.get("gate_ok", 0) or 0) for row in rows], dtype=float
        )
        complete = np.array(
            [float(row.get("complete", 0) or 0) for row in rows], dtype=float
        )
        hold_elapsed = np.array(
            [float(row.get("hold_elapsed_s", 0.0) or 0.0) for row in rows], dtype=float
        )
        hold_required = np.array(
            [float(row.get("hold_required_s", 0.0) or 0.0) for row in rows], dtype=float
        )

        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        axes[0].step(
            time,
            gate_ok,
            where="post",
            label="Gate OK",
            color=PlotStyle.COLOR_SIGNAL_ANG,
        )
        axes[0].step(
            time,
            complete,
            where="post",
            label="Complete",
            color=PlotStyle.COLOR_SIGNAL_POS,
        )
        axes[0].set_ylim(-0.1, 1.1)
        axes[0].set_yticks([0, 1])
        axes[0].set_ylabel("Gate Flags", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1].plot(
            time, hold_elapsed, color=PlotStyle.COLOR_SIGNAL_POS, label="Hold Elapsed"
        )
        axes[1].plot(
            time,
            hold_required,
            color=PlotStyle.COLOR_THRESHOLD,
            linestyle="--",
            label="Hold Required",
        )
        axes[1].set_ylabel("Hold Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        fig.suptitle("Completion Gate Trace")
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_path_progress_remaining_distance(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        time = self._get_time_axis()
        n = len(time)
        progress = self._col_float("Path_Progress", n)
        remaining = self._col_float("Path_Remaining", n)
        if not np.any(progress) and not np.any(remaining):
            raise PlotSkippedError("Path progress columns unavailable")
        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        axes[0].plot(
            time, progress, color=PlotStyle.COLOR_SIGNAL_POS, label="Path Progress"
        )
        axes[0].set_ylabel("Progress", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1].plot(
            time,
            remaining,
            color=PlotStyle.COLOR_SIGNAL_ANG,
            label="Remaining Distance",
        )
        axes[1].set_ylabel("Remaining (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        fig.suptitle("Path Progress & Remaining Distance")
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_event_timeline_density(
        self, output_path: Path, *, spec: Any
    ) -> list[Path]:
        rows = self._load_jsonl_artifact_rows("event_timeline.jsonl")
        if not rows:
            raise PlotSkippedError("event_timeline.jsonl not available")
        times = np.array(
            [float(row.get("time_s", 0.0) or 0.0) for row in rows], dtype=float
        )
        if times.size == 0:
            raise PlotSkippedError("No event timestamps available")

        fig, axes = plt.subplots(2, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS, sharex=True)
        bins = min(max(len(times), 5), 40)
        axes[0].hist(times, bins=bins, color=PlotStyle.COLOR_SIGNAL_POS, alpha=0.8)
        axes[0].set_ylabel("Events / Bin", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].set_title("Event Density")
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)

        sorted_times = np.sort(times)
        axes[1].step(
            sorted_times,
            np.arange(1, len(sorted_times) + 1),
            where="post",
            color=PlotStyle.COLOR_SIGNAL_ANG,
            linewidth=PlotStyle.LINEWIDTH,
        )
        axes[1].set_ylabel("Cumulative Events", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        fig.suptitle("Event Timeline Density")
        PlotStyle.save_figure(fig, output_path)
        return [output_path]

    def _render_path_shaping_note(self, output_path: Path, *, spec: Any) -> list[Path]:
        return self._render_from_legacy(
            output_path,
            legacy_renderer=self.generate_path_shaping_note_plot,
            legacy_filename="path_shaping_note.png",
        )

    def generate_position_tracking_plot(self, plot_dir: Path) -> None:
        """Generate position tracking over time plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Position Tracking - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        # X position tracking
        axes[0].plot(
            time,
            self._col("Current_X"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current X",
        )
        axes[0].plot(
            time,
            self._col("Reference_X"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference X",
        )
        axes[0].set_ylabel("X Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("X Position Tracking")

        # Y position tracking
        axes[1].plot(
            time,
            self._col("Current_Y"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Y",
        )
        axes[1].plot(
            time,
            self._col("Reference_Y"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Y",
        )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("Y Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Y Position Tracking")

        # Z position tracking
        axes[2].plot(
            time,
            self._col("Current_Z"),
            color=PlotStyle.COLOR_SIGNAL_POS,
            linewidth=PlotStyle.LINEWIDTH,
            label="Current Z",
        )
        axes[2].plot(
            time,
            self._col("Reference_Z"),
            color=PlotStyle.COLOR_TARGET,
            linestyle="--",
            linewidth=PlotStyle.LINEWIDTH,
            label="Reference Z",
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Z Position (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Z Position Tracking")

        PlotStyle.save_figure(fig, plot_dir / "position_tracking.png")

    def generate_position_error_plot(self, plot_dir: Path) -> None:
        """Generate position error plot."""
        fig, axes = plt.subplots(3, 1, figsize=PlotStyle.FIGSIZE_SUBPLOTS)
        fig.suptitle(f"Position Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        # Calculate errors
        error_x = self._col("Current_X") - self._col("Reference_X")
        error_y = self._col("Current_Y") - self._col("Reference_Y")
        error_z = self._col("Current_Z") - self._col("Reference_Z")

        # X error
        axes[0].plot(
            time,
            error_x,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="X Error",
        )
        axes[0].axhline(
            y=0,
            color=PlotStyle.COLOR_MUTED,
            linestyle="-",
            linewidth=0.5,
            alpha=0.3,
        )
        axes[0].set_ylabel("X Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("X Position Error")

        # Y error
        axes[1].plot(
            time,
            error_y,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Y Error",
        )
        axes[1].axhline(
            y=0,
            color=PlotStyle.COLOR_MUTED,
            linestyle="-",
            linewidth=0.5,
            alpha=0.3,
        )
        axes[1].set_ylabel("Y Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[1].set_title("Y Position Error")

        # Z error
        axes[2].plot(
            time,
            error_z,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Z Error",
        )
        axes[2].axhline(
            y=0,
            color=PlotStyle.COLOR_MUTED,
            linestyle="-",
            linewidth=0.5,
            alpha=0.3,
        )
        axes[2].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].set_ylabel("Z Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[2].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[2].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[2].set_title("Z Position Error")

        PlotStyle.save_figure(fig, plot_dir / "position_error.png")

    def generate_angular_tracking_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-component tracking plot."""
        fig, axes = plt.subplots(4, 1, figsize=(10, 10))
        fig.suptitle(f"Quaternion Tracking - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_cur), len(q_ref))

        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion tracking data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
                ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            PlotStyle.save_figure(fig, plot_dir / "attitude_tracking.png")
            return

        comp_labels = ("w", "x", "y", "z")
        for i, comp in enumerate(comp_labels):
            axes[i].plot(
                time[:min_len],
                q_cur[:min_len, i],
                color=PlotStyle.COLOR_SIGNAL_ANG,
                linewidth=PlotStyle.LINEWIDTH,
                label=f"Current q{comp}",
            )
            axes[i].plot(
                time[:min_len],
                q_ref[:min_len, i],
                color=PlotStyle.COLOR_TARGET,
                linestyle="--",
                linewidth=PlotStyle.LINEWIDTH,
                label=f"Reference q{comp}",
            )
            axes[i].set_ylabel(f"q{comp}", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            axes[i].grid(True, alpha=PlotStyle.GRID_ALPHA)
            axes[i].legend(fontsize=PlotStyle.LEGEND_SIZE)
            axes[i].set_title(f"q{comp} Tracking")

        axes[3].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_tracking.png")

    def generate_angular_error_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-component error plot."""
        fig, axes = plt.subplots(4, 1, figsize=(10, 10))
        fig.suptitle(f"Quaternion Component Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_cur), len(q_ref))

        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion error data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
                ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
            PlotStyle.save_figure(fig, plot_dir / "attitude_error.png")
            return

        # Use shortest-sign representation per sample.
        dot = np.sum(q_cur[:min_len] * q_ref[:min_len], axis=1)
        sign = np.where(dot < 0.0, -1.0, 1.0).reshape(-1, 1)
        q_err = q_cur[:min_len] - sign * q_ref[:min_len]

        comp_labels = ("w", "x", "y", "z")
        for i, comp in enumerate(comp_labels):
            axes[i].plot(
                time[:min_len],
                q_err[:, i],
                color=PlotStyle.COLOR_ERROR,
                linewidth=PlotStyle.LINEWIDTH,
                label=f"q{comp} Error",
            )
            axes[i].axhline(
                y=0,
                color=PlotStyle.COLOR_MUTED,
                linestyle="-",
                linewidth=0.5,
                alpha=0.3,
            )
            axes[i].set_ylabel(f"q{comp} err", fontsize=PlotStyle.AXIS_LABEL_SIZE)
            axes[i].grid(True, alpha=PlotStyle.GRID_ALPHA)
            axes[i].legend(fontsize=PlotStyle.LEGEND_SIZE)
            axes[i].set_title(f"q{comp} Error")
        axes[3].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_error.png")

    def generate_quaternion_attitude_error_plot(self, plot_dir: Path) -> None:
        """Generate quaternion-geodesic attitude error (rotation-invariant)."""
        fig, axes = plt.subplots(2, 1, figsize=(10, 7))
        fig.suptitle(f"Quaternion Attitude Error - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)
        q_curr = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        min_len = min(len(time), len(q_curr), len(q_ref))
        if min_len == 0:
            for ax in axes:
                ax.text(
                    0.5,
                    0.5,
                    "Quaternion attitude data\nnot available",
                    ha="center",
                    va="center",
                    transform=ax.transAxes,
                    fontsize=PlotStyle.ANNOTATION_SIZE,
                )
            PlotStyle.save_figure(fig, plot_dir / "attitude_error_quaternion.png")
            return

        q_err_deg = np.degrees(
            np.array(
                [quat_angle_error(q_ref[i], q_curr[i]) for i in range(min_len)],
                dtype=float,
            )
        )
        q_err_deg = np.nan_to_num(q_err_deg, nan=0.0, posinf=0.0, neginf=0.0)
        q_err_rate = np.gradient(q_err_deg, max(float(self.dt), 1e-9))

        axes[0].plot(
            time[:min_len],
            q_err_deg,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="SO(3) Angle Error",
        )
        axes[0].set_ylabel("Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0].legend(fontsize=PlotStyle.LEGEND_SIZE)
        axes[0].set_title("Quaternion Geodesic Error")

        axes[1].plot(
            time[:min_len],
            q_err_rate,
            color=PlotStyle.COLOR_SECONDARY,
            linewidth=PlotStyle.LINEWIDTH,
            label="d(Error)/dt",
        )
        axes[1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].set_ylabel("deg/s", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "attitude_error_quaternion.png")

    def generate_error_norms_plot(self, plot_dir: Path) -> None:
        """Generate error norm summary plot."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle(f"Error Norms - {self.system_title}")

        time = np.arange(self._get_len()) * float(self.dt)

        def get_series(name: str) -> np.ndarray:
            vals = self._col(name)
            return vals if len(vals) else np.zeros_like(time)

        err_x = get_series("Error_X")
        err_y = get_series("Error_Y")
        err_z = get_series("Error_Z")
        err_vx = get_series("Error_VX")
        err_vy = get_series("Error_VY")
        err_vz = get_series("Error_VZ")
        err_wx = get_series("Error_WX")
        err_wy = get_series("Error_WY")
        err_wz = get_series("Error_WZ")

        pos_err_norm = np.sqrt(err_x**2 + err_y**2 + err_z**2)
        vel_err_norm = np.sqrt(err_vx**2 + err_vy**2 + err_vz**2)
        q_cur = self._get_quaternion_series("Current")
        q_ref = self._get_quaternion_series("Reference")
        q_len = min(len(time), len(q_cur), len(q_ref))
        if q_len > 0:
            ang_err_norm = np.degrees(
                np.array(
                    [quat_angle_error(q_ref[i], q_cur[i]) for i in range(q_len)],
                    dtype=float,
                )
            )
            if q_len < len(time):
                # Pad tail for mixed/legacy data lengths.
                pad = np.full(len(time) - q_len, ang_err_norm[-1], dtype=float)
                ang_err_norm = np.concatenate([ang_err_norm, pad])
        else:
            err_angle_rad = get_series("Error_Angle_Rad")
            if len(err_angle_rad) == len(time):
                ang_err_norm = np.degrees(err_angle_rad)
            else:
                ang_err_norm = np.zeros_like(time)
        angvel_err_norm = np.degrees(np.sqrt(err_wx**2 + err_wy**2 + err_wz**2))

        axes[0, 0].plot(
            time,
            pos_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Position Error Norm",
        )
        axes[0, 0].set_ylabel("Position Error (m)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[0, 0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0, 0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[0, 1].plot(
            time,
            vel_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Velocity Error Norm",
        )
        axes[0, 1].set_ylabel(
            "Velocity Error (m/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
        axes[0, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[0, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1, 0].plot(
            time,
            ang_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Attitude Error Norm",
        )
        axes[1, 0].set_ylabel(
            "Attitude Error (deg)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
        axes[1, 0].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 0].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1, 0].legend(fontsize=PlotStyle.LEGEND_SIZE)

        axes[1, 1].plot(
            time,
            angvel_err_norm,
            color=PlotStyle.COLOR_ERROR,
            linewidth=PlotStyle.LINEWIDTH,
            label="Angular Rate Error Norm",
        )
        axes[1, 1].set_ylabel(
            "Angular Rate Error (deg/s)", fontsize=PlotStyle.AXIS_LABEL_SIZE
        )
        axes[1, 1].set_xlabel("Time (s)", fontsize=PlotStyle.AXIS_LABEL_SIZE)
        axes[1, 1].grid(True, alpha=PlotStyle.GRID_ALPHA)
        axes[1, 1].legend(fontsize=PlotStyle.LEGEND_SIZE)

        PlotStyle.save_figure(fig, plot_dir / "error_norms.png")

    def generate_trajectory_plot(self, plot_dir: Path) -> None:
        """Generate trajectory plot."""
        generate_trajectory_plot(self, plot_dir)

    def generate_trajectory_3d_interactive_plot(self, plot_dir: Path) -> None:
        """Generate interactive 3D trajectory plot (HTML)."""
        generate_trajectory_3d_interactive_plot(self, plot_dir)

    def generate_trajectory_3d_orientation_plot(self, plot_dir: Path) -> None:
        """Generate 3D trajectory plot with orientation arrows."""
        generate_trajectory_3d_orientation_plot(self, plot_dir)

    def generate_thruster_usage_plot(self, plot_dir: Path) -> None:
        """Generate thruster usage plot using actual valve states."""
        generate_thruster_usage_plot(self, plot_dir)

    def generate_thruster_valve_activity_plot(self, plot_dir: Path) -> None:
        """Generate detailed valve activity plot for each thruster (0.0 to 1.0)."""
        generate_thruster_valve_activity_plot(self, plot_dir)

    def generate_command_vs_valve_tracking_plot(self, plot_dir: Path) -> None:
        """Generate commanded-vs-actual valve tracking summary."""
        generate_command_vs_valve_tracking_plot(self, plot_dir)

    def generate_pwm_quantization_plot(self, plot_dir: Path) -> None:
        """Generate PWM duty cycle plot showing MPC u-values vs time."""
        generate_pwm_quantization_plot(self, plot_dir)

    def generate_pwm_duty_cycles_from_physics(self, plot_dir: Path) -> None:
        """Fallback: Generate PWM plot from physics_data Thruster_X_Cmd columns."""
        # This shows binary valve states, not continuous duty cycles
        pass

    def _get_thruster_count(self) -> int:
        """Determine thruster count based on available data or config."""
        return infer_thruster_count(self.data_accessor, self.app_config)

    def generate_control_effort_plot(self, plot_dir: Path) -> None:
        """Generate control effort plot."""
        generate_control_effort_plot(self, plot_dir)

    def generate_reaction_wheel_output_plot(self, plot_dir: Path) -> None:
        """Generate reaction wheel torque output plot."""
        generate_reaction_wheel_output_plot(self, plot_dir)

    def generate_actuator_limits_plot(self, plot_dir: Path) -> None:
        """Generate actuator outputs with limit overlays."""
        generate_actuator_limits_plot(self, plot_dir)

    def generate_constraint_violations_plot(self, plot_dir: Path) -> None:
        """Generate constraint violation plot."""
        generate_constraint_violations_plot(self, plot_dir)

    def generate_translation_attitude_coupling_plot(self, plot_dir: Path) -> None:
        """Generate frame-agnostic translation-attitude coupling plot."""
        generate_translation_attitude_coupling_plot(self, plot_dir)

    def generate_z_tilt_coupling_plot(self, plot_dir: Path) -> None:
        """Backward-compatible wrapper for legacy callsites."""
        generate_translation_attitude_coupling_plot(self, plot_dir)

    def generate_thruster_impulse_proxy_plot(self, plot_dir: Path) -> None:
        """Generate thruster impulse proxy plot."""
        generate_thruster_impulse_proxy_plot(self, plot_dir)

    def generate_cumulative_impulse_delta_v_proxy_plot(self, plot_dir: Path) -> None:
        """Generate cumulative impulse and delta-v proxy plot."""
        generate_cumulative_impulse_delta_v_proxy_plot(self, plot_dir)

    def generate_phase_position_velocity_plot(self, plot_dir: Path) -> None:
        """Generate position vs velocity phase plots."""
        generate_phase_position_velocity_plot(self, plot_dir)

    def generate_phase_attitude_rate_plot(self, plot_dir: Path) -> None:
        """Generate attitude vs rate phase plots."""
        generate_phase_attitude_rate_plot(self, plot_dir)

    def generate_solver_health_plot(self, plot_dir: Path) -> None:
        """Generate solver health summary plot."""
        generate_solver_health_plot(self, plot_dir)

    def generate_waypoint_progress_plot(self, plot_dir: Path) -> None:
        """Generate waypoint/mission phase progress plot."""
        generate_waypoint_progress_plot(self, plot_dir)

    def generate_path_shaping_note_plot(self, plot_dir: Path) -> None:
        """Generate manual path shaping note plot."""
        generate_path_shaping_note_plot(self, plot_dir)

    def generate_velocity_tracking_plot(self, plot_dir: Path) -> None:
        """Generate velocity tracking over time plot."""
        generate_velocity_tracking_plot(self, plot_dir)

    def generate_velocity_magnitude_plot(self, plot_dir: Path) -> None:
        """Generate velocity magnitude over time plot (speed vs time)."""
        generate_velocity_magnitude_plot(self, plot_dir)

    def generate_mpc_performance_plot(self, plot_dir: Path) -> None:
        """Generate MPC performance plot."""
        generate_mpc_performance_plot(self, plot_dir)

    def generate_solver_iterations_and_status_timeline_plot(
        self, plot_dir: Path
    ) -> None:
        """Generate solver iterations and status timeline plot."""
        generate_solver_iterations_and_status_timeline_plot(self, plot_dir)

    def generate_error_vs_solve_time_scatter_plot(self, plot_dir: Path) -> None:
        """Generate error-vs-solve-time scatter plot."""
        generate_error_vs_solve_time_scatter_plot(self, plot_dir)

    def generate_timing_intervals_plot(self, plot_dir: Path) -> None:
        """Generate timing intervals plot."""
        generate_timing_intervals_plot(self, plot_dir)
