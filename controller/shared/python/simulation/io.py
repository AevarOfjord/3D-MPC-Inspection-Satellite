"""
Simulation IO Module

Handles data export, directory management, and file operations for simulations.
Extracted from SatelliteMPCLinearizedSimulation to reduce class size.
"""

import csv
import hashlib
import json
import logging
import math
import mimetypes
import os
import platform
import re
import subprocess
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from controller.configs.paths import SIMULATION_DATA_ROOT
from controller.shared.python.simulation.artifact_paths import (
    artifact_path,
    ensure_artifact_directories,
    resolve_existing_artifact_path,
)
from controller.shared.python.utils.orientation_utils import quat_angle_error

if TYPE_CHECKING:
    from controller.shared.python.simulation.engine import (
        SatelliteMPCLinearizedSimulation,
    )

logger = logging.getLogger(__name__)


class SimulationIO:
    """
    Data export and directory management for simulations.

    Handles:
    - Creating timestamped data directories
    - Saving mission summary reports
    - Coordinating with DataLogger and ReportGenerator
    """

    def __init__(self, simulation: "SatelliteMPCLinearizedSimulation"):
        """
        Initialize SimulationIO with reference to parent simulation.

        Args:
            simulation: Parent simulation instance
        """
        self.sim = simulation

    @staticmethod
    def _sanitize_run_token(raw: str, *, default: str = "unknown") -> str:
        """Normalize token strings for filesystem-safe run directory names."""
        token = re.sub(r"[^A-Za-z0-9._-]+", "_", str(raw or "").strip())
        token = re.sub(r"_+", "_", token).strip("._-")
        return token or default

    def _resolve_controller_profile_token(self) -> str:
        """Best-effort controller profile token for run directory naming."""
        profile_raw = str(
            getattr(self.sim, "controller_profile_mode", None)
            or getattr(
                getattr(self.sim, "mpc_controller", None), "controller_profile", ""
            )
            or "unknown"
        )
        # Directory labels do not need the stack prefix when all profiles are C++-backed.
        profile = profile_raw[4:] if profile_raw.startswith("cpp_") else profile_raw
        return self._sanitize_run_token(profile, default="unknown")

    def _resolve_mission_token(self) -> str:
        """Best-effort mission token for run directory naming."""
        env_name = str(os.environ.get("SATCTRL_RUNNER_MISSION_NAME", "")).strip()
        if env_name:
            return self._sanitize_run_token(env_name, default="auto")

        env_path = str(os.environ.get("SATCTRL_RUNNER_MISSION_PATH", "")).strip()
        if env_path:
            return self._sanitize_run_token(Path(env_path).stem, default="auto")

        sim_config = getattr(self.sim, "simulation_config", None)
        app_config = getattr(sim_config, "app_config", None)
        input_path = getattr(app_config, "input_file_path", None)
        if isinstance(input_path, str) and input_path.strip():
            return self._sanitize_run_token(Path(input_path).stem, default="auto")

        return "auto"

    def _build_run_dir_name(self) -> str:
        """Build deterministic run directory name: <timestamp>__<profile>__<mission>."""
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")
        profile_token = self._resolve_controller_profile_token()
        mission_token = self._resolve_mission_token()
        return f"{timestamp}__{profile_token}__{mission_token}"

    def _resolve_unique_run_dir(self, base_dir: Path, run_name: str) -> Path:
        """Ensure run directory name uniqueness when multiple runs start within a second."""
        candidate = base_dir / run_name
        if not candidate.exists():
            return candidate

        suffix = 2
        while True:
            alt = base_dir / f"{run_name}__{suffix:02d}"
            if not alt.exists():
                return alt
            suffix += 1

    def create_data_directories(self) -> Path:
        """
        Create the directory structure for saving data.

        Returns:
            Path to the run subdirectory
        """
        run_name = self._build_run_dir_name()
        run_path = self._resolve_unique_run_dir(SIMULATION_DATA_ROOT, run_name)

        # Create directories
        run_path.mkdir(parents=True, exist_ok=True)
        ensure_artifact_directories(run_path)
        self._write_run_status(
            run_dir=run_path,
            status="running",
            status_detail="Simulation initialized and data directory created",
            final=False,
        )

        logger.info(f"Created data directory: {run_path}")
        return run_path

    def _artifact_path(self, run_dir: Path, name: str) -> Path:
        """Resolve canonical output path for a run artifact."""
        return artifact_path(run_dir, name)

    def _artifact_existing_path(self, run_dir: Path, name: str) -> Path | None:
        """Resolve existing path for artifact (new layout first, then legacy)."""
        return resolve_existing_artifact_path(run_dir, name)

    def save_csv_data(self) -> None:
        """Save all logged data to CSV files (delegates to DataLoggers)."""
        self.sim.data_logger.save_csv_data()
        self.sim.physics_logger.save_csv_data()

    def save_mission_summary(self) -> None:
        """Generate and save mission summary report."""
        if not self.sim.data_save_path:
            logger.warning("Cannot save mission summary: No data save path set")
            return

        # Attempt to load state history from CSV if not in memory or trimmed
        history_for_report: list[np.ndarray] | np.ndarray = self.sim.state_history
        control_history_for_report: list[np.ndarray] = self.sim.control_history

        if not history_for_report or getattr(self.sim, "history_trimmed", False):
            loaded_history = self._load_history_from_csv()
            if loaded_history is not None:
                history_for_report = loaded_history

        if history_for_report is None or len(history_for_report) == 0:
            logger.warning("No state history available for full summary")
            return

        summary_path = self._artifact_path(
            self.sim.data_save_path, "mission_summary.txt"
        )

        # Use DataLogger stats for solve times
        solve_times = self.sim.data_logger.stats_solve_times

        if isinstance(history_for_report, np.ndarray):
            history_for_report_list = [row for row in history_for_report]
        else:
            history_for_report_list = history_for_report

        self.sim.report_generator.generate_report(
            output_path=summary_path,
            state_history=history_for_report_list,
            reference_state=self.sim.reference_state,
            control_time=self.sim.simulation_time,
            mpc_solve_times=solve_times,
            control_history=control_history_for_report,
            path_complete_time=self.sim.trajectory_endpoint_reached_time,
            position_tolerance=self.sim.position_tolerance,
            angle_tolerance=self.sim.angle_tolerance,
            control_update_interval=self.sim.control_update_interval,
            check_path_complete_func=self.sim.check_path_complete,
            test_mode="SIMULATION",
        )
        self._save_mission_metadata()
        self._save_reproducibility_manifest()

    def _save_mission_metadata(self) -> None:
        """Save mission metadata used by the web visualizer."""
        if not self.sim.data_save_path:
            return

        mission_state = getattr(self.sim, "mission_state", None)
        if mission_state is None and getattr(self.sim, "simulation_config", None):
            mission_state = getattr(self.sim.simulation_config, "mission_state", None)
        if mission_state is None:
            return

        metadata = {"mission_type": "path_following"}
        planned_path_frame_raw = str(
            getattr(mission_state, "path_frame", "LVLH")
        ).upper()
        metadata["planned_path_frame"] = (
            planned_path_frame_raw
            if planned_path_frame_raw in {"ECI", "LVLH"}
            else "LVLH"
        )
        frame_origin = getattr(mission_state, "frame_origin", (0.0, 0.0, 0.0))
        try:
            metadata["frame_origin"] = [
                float(frame_origin[0]),
                float(frame_origin[1]),
                float(frame_origin[2]),
            ]
        except Exception:
            metadata["frame_origin"] = [0.0, 0.0, 0.0]
        scan_object = getattr(mission_state, "visualization_scan_object", None)
        if isinstance(scan_object, dict):
            metadata["scan_object"] = scan_object

        planned_path = getattr(mission_state, "path_waypoints", None)
        if not planned_path:
            planned_path = getattr(self.sim, "planned_path", None)

        if planned_path:
            try:
                metadata["planned_path"] = [list(p) for p in planned_path]
            except Exception as exc:
                logger.warning(f"Failed to serialize planned_path for metadata: {exc}")

        metadata_path = self._artifact_path(
            self.sim.data_save_path, "mission_metadata.json"
        )
        try:
            metadata_path.write_text(json.dumps(metadata, indent=2))
        except Exception as exc:
            logger.warning(f"Failed to save mission metadata: {exc}")

    def _save_reproducibility_manifest(self) -> None:
        """Save run reproducibility metadata for traceability/debugging."""
        if not self.sim.data_save_path:
            return

        sim_config = getattr(self.sim, "simulation_config", None)
        app_config = getattr(sim_config, "app_config", None)
        if app_config is None:
            return

        try:
            app_config_dict = app_config.model_dump()
            config_json = json.dumps(
                app_config_dict, sort_keys=True, separators=(",", ":")
            )
            computed_config_hash = hashlib.sha256(
                config_json.encode("utf-8")
            ).hexdigest()[:12]

            solver_type = str(
                app_config_dict.get("mpc", {}).get("solver_type", "unknown")
            )
            manifest = {
                "schema_version": "run_reproducibility_manifest_v1",
                "generated_at": datetime.utcnow().isoformat() + "Z",
                "run_id": self.sim.data_save_path.name,
                "run_path": str(self.sim.data_save_path),
                "mission": {
                    "name": os.environ.get("SATCTRL_RUNNER_MISSION_NAME"),
                    "path": os.environ.get("SATCTRL_RUNNER_MISSION_PATH")
                    or app_config_dict.get("input_file_path"),
                },
                "preset": {
                    "name": os.environ.get("SATCTRL_RUNNER_PRESET_NAME"),
                },
                "configuration": {
                    "config_version": os.environ.get("SATCTRL_RUNNER_CONFIG_VERSION"),
                    "config_hash": os.environ.get("SATCTRL_RUNNER_CONFIG_HASH")
                    or computed_config_hash,
                    "computed_config_hash": computed_config_hash,
                    "overrides_active": os.environ.get(
                        "SATCTRL_RUNNER_OVERRIDES_ACTIVE"
                    )
                    == "1",
                    "app_config": app_config_dict,
                },
                "solver": {
                    "type": solver_type,
                    "backend_version": self._detect_solver_backend_version(solver_type),
                },
                "software": {
                    "satellite_control_version": self._get_package_version(),
                    "python_version": platform.python_version(),
                    "platform": platform.platform(),
                    "git": self._get_git_metadata(),
                },
            }

            manifest_path = self._artifact_path(
                self.sim.data_save_path, "reproducibility_manifest.json"
            )
            manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        except Exception as exc:
            logger.warning(f"Failed to save reproducibility manifest: {exc}")

    def finalize_run_artifacts(
        self,
        run_status: str = "completed",
        status_detail: str | None = None,
    ) -> None:
        """Generate post-run artifacts and finalize run status/index files."""
        if not self.sim.data_save_path:
            return

        run_dir = self.sim.data_save_path

        try:
            control_stats = self._generate_mpc_step_stats(run_dir)
            physics_stats = self._collect_physics_stats(run_dir)
            mission_stats = self._collect_mission_stats()

            kpi_summary = self._build_kpi_summary(
                run_dir=run_dir,
                control_stats=control_stats,
                physics_stats=physics_stats,
                mission_stats=mission_stats,
            )
            self._write_json(
                self._artifact_path(run_dir, "kpi_summary.json"), kpi_summary
            )

            constraints = self._build_constraint_violations(
                run_dir=run_dir,
                control_stats=control_stats,
                physics_stats=physics_stats,
                mission_stats=mission_stats,
            )
            self._write_json(
                self._artifact_path(run_dir, "constraint_violations.json"), constraints
            )

            self._write_event_timeline(
                run_dir=run_dir,
                control_stats=control_stats,
                physics_stats=physics_stats,
                run_status=run_status,
                status_detail=status_detail,
            )
            self._write_mode_timeline(run_dir)
            self._write_completion_gate_trace(run_dir)
            self._write_controller_health(run_dir)

            plots_index = self._build_plots_index(run_dir)
            self._write_json(
                self._artifact_path(run_dir, "plots_index.json"), plots_index
            )

            media_metadata = self._build_media_metadata(
                run_dir=run_dir, kpi_summary=kpi_summary
            )
            self._write_json(
                self._artifact_path(run_dir, "media_metadata.json"), media_metadata
            )

            compare_signature = self._build_compare_signature(
                run_dir=run_dir,
                kpi_summary=kpi_summary,
                control_stats=control_stats,
            )
            self._write_json(
                self._artifact_path(run_dir, "compare_signature.json"),
                compare_signature,
            )

            self._write_run_notes(
                run_dir=run_dir,
                kpi_summary=kpi_summary,
                constraints=constraints,
                compare_signature=compare_signature,
            )

            self._write_run_status(
                run_dir=run_dir,
                status=run_status,
                status_detail=status_detail,
                final=True,
                kpi_summary=kpi_summary,
                constraints=constraints,
            )

            # Regenerate summary after final status/constraints are written so
            # mission_summary.txt includes final lifecycle and artifact context.
            self.save_mission_summary()

            self._write_checksums_file(run_dir)
            self._write_artifacts_manifest(run_dir)
            self._update_global_run_indexes(run_dir)
        except Exception as exc:
            logger.warning(f"Failed while generating run artifacts: {exc}")

    def _write_run_status(
        self,
        run_dir: Path,
        status: str,
        status_detail: str | None = None,
        final: bool = False,
        kpi_summary: dict[str, Any] | None = None,
        constraints: dict[str, Any] | None = None,
    ) -> None:
        """Create/update run_status.json with lifecycle and summary metadata."""
        status_path = self._artifact_path(run_dir, "run_status.json")
        payload: dict[str, Any] = {}
        if status_path.exists():
            try:
                payload = json.loads(status_path.read_text(encoding="utf-8"))
            except Exception:
                payload = {}

        now_iso = datetime.utcnow().isoformat() + "Z"
        sim_config = getattr(self.sim, "simulation_config", None)
        app_config = getattr(sim_config, "app_config", None)
        app_config_dict = app_config.model_dump() if app_config is not None else {}
        cfg_json = json.dumps(app_config_dict, sort_keys=True, separators=(",", ":"))
        cfg_hash = hashlib.sha256(cfg_json.encode("utf-8")).hexdigest()[:12]
        config_meta = {
            "config_hash": os.environ.get("SATCTRL_RUNNER_CONFIG_HASH") or cfg_hash,
            "config_version": os.environ.get("SATCTRL_RUNNER_CONFIG_VERSION")
            or "app_config_v3",
            "overrides_active": os.environ.get("SATCTRL_RUNNER_OVERRIDES_ACTIVE")
            == "1",
            "response_mirrors_enabled": os.environ.get(
                "SATCTRL_RUNNER_RESPONSE_MIRRORS_ENABLED", "1"
            )
            in {"1", "true", "TRUE"},
        }
        mpc_controller = getattr(self.sim, "mpc_controller", None)
        config_meta["controller_core"] = str(
            getattr(self.sim, "controller_core_mode", None)
            or getattr(mpc_controller, "controller_core", "v6")
        )
        config_meta["controller_profile"] = str(
            getattr(self.sim, "controller_profile_mode", None)
            or getattr(mpc_controller, "controller_profile", "cpp_hybrid_rti_osqp")
        )
        config_meta["solver_backend"] = str(
            getattr(mpc_controller, "solver_backend", "OSQP")
        )
        config_meta["linearization_mode"] = str(
            getattr(self.sim, "linearization_mode", None)
            or getattr(mpc_controller, "linearization_mode", "hybrid_tolerant_stage")
        )
        config_meta["shared_params_hash"] = str(
            getattr(mpc_controller, "shared_params_hash", "unknown")
        )
        config_meta["effective_params_hash"] = str(
            getattr(mpc_controller, "effective_params_hash", "unknown")
        )
        config_meta["override_diff"] = dict(
            getattr(mpc_controller, "profile_override_diff", {})
        )

        payload.setdefault("schema_version", "run_status_v1")
        payload.setdefault("run_id", run_dir.name)
        payload.setdefault("run_path", str(run_dir))
        payload.setdefault("started_at", now_iso)
        payload["updated_at"] = now_iso
        payload["status"] = status
        payload["status_detail"] = status_detail
        payload["mission"] = {
            "name": os.environ.get("SATCTRL_RUNNER_MISSION_NAME"),
            "path": os.environ.get("SATCTRL_RUNNER_MISSION_PATH")
            or app_config_dict.get("input_file_path"),
        }
        payload["preset"] = {"name": os.environ.get("SATCTRL_RUNNER_PRESET_NAME")}
        payload["config"] = config_meta

        if final:
            payload["completed_at"] = now_iso
            payload["final_simulation_time_s"] = float(
                getattr(self.sim, "simulation_time", 0.0)
            )
            try:
                start_dt = datetime.fromisoformat(
                    str(payload.get("started_at", now_iso)).replace("Z", "+00:00")
                )
                end_dt = datetime.fromisoformat(now_iso.replace("Z", "+00:00"))
                payload["wall_clock_duration_s"] = max(
                    0.0, (end_dt - start_dt).total_seconds()
                )
            except Exception:
                payload["wall_clock_duration_s"] = None
            payload["success"] = bool(status == "completed")
            if kpi_summary:
                payload["kpi_snapshot"] = {
                    "final_position_error_m": kpi_summary.get("final_position_error_m"),
                    "final_angle_error_deg": kpi_summary.get("final_angle_error_deg"),
                    "mpc_mean_solve_time_ms": kpi_summary.get("mpc_mean_solve_time_ms"),
                    "mpc_max_solve_time_ms": kpi_summary.get("mpc_max_solve_time_ms"),
                    "hold_reset_count": kpi_summary.get("hold_reset_count"),
                    "dominant_terminal_gate_fail_reason_last_60s": kpi_summary.get(
                        "dominant_terminal_gate_fail_reason_last_60s"
                    ),
                    "terminal_gate_fail_reason_counts_last_60s": kpi_summary.get(
                        "terminal_gate_fail_reason_counts_last_60s"
                    ),
                }
            if constraints:
                payload["constraints_pass"] = constraints.get("pass")

        status_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    @staticmethod
    def _to_float(value: Any, default: float = 0.0) -> float:
        """Convert values from CSV/JSON to float safely."""
        try:
            return float(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _to_bool_flag(value: Any) -> bool:
        """Normalize various boolean-like string values."""
        if isinstance(value, bool):
            return value
        text = str(value or "").strip().lower()
        return text in {"1", "true", "yes", "y", "on"}

    @staticmethod
    def _vector_norm3(x: float, y: float, z: float) -> float:
        """Fast Euclidean norm for 3-vector."""
        return math.sqrt(x * x + y * y + z * z)

    def _generate_mpc_step_stats(self, run_dir: Path) -> dict[str, Any]:
        """Generate mpc_step_stats.csv and aggregate control-loop statistics."""
        control_csv = self._artifact_existing_path(
            run_dir, "control_data.csv"
        ) or self._artifact_path(run_dir, "control_data.csv")
        output_csv = self._artifact_path(run_dir, "mpc_step_stats.csv")
        if not control_csv.exists():
            return {
                "control_steps": 0,
                "solve_times_ms": [],
                "timing_violation_count": 0,
                "time_limit_exceeded_count": 0,
                "final_time_s": 0.0,
                "final_position_error_m": 0.0,
                "final_angle_error_deg": 0.0,
                "final_velocity_error_mps": 0.0,
                "final_angular_velocity_error_degps": 0.0,
                "terminal_gate_fail_reason_counts": {},
                "terminal_gate_fail_reason_counts_last_60s": {},
                "pointing_policy_counts": {},
                "pointing_context_source_counts": {},
                "hold_reset_count": 0,
            }

        def _normalize_terminal_fail_reason(raw: Any) -> str:
            token = str(raw or "").strip().lower()
            mapping = {
                "progress": "progress",
                "path_progress": "progress",
                "position": "position",
                "position_error": "position",
                "angle": "angle",
                "angle_error": "angle",
                "velocity": "velocity",
                "velocity_error": "velocity",
                "angular_velocity": "angular_velocity",
                "angular_velocity_error": "angular_velocity",
                "none": "none",
                "": "none",
            }
            return mapping.get(token, "none")

        solve_times_ms: list[float] = []
        control_steps = 0
        timing_violation_count = 0
        time_limit_exceeded_count = 0
        total_active_thrusters = 0.0
        total_thruster_switches = 0
        final_time_s = 0.0
        final_position_error_m = 0.0
        final_angle_error_deg = 0.0
        final_velocity_error_mps = 0.0
        final_angular_velocity_error_degps = 0.0
        final_path_progress = 0.0
        final_path_remaining_m = 0.0
        max_position_error = {"value": 0.0, "time": 0.0}
        max_angle_error = {"value": 0.0, "time": 0.0}
        first_time_limit_exceeded_time: float | None = None
        first_timing_violation_time: float | None = None
        hold_reset_count = 0
        terminal_gate_fail_reason_counts: dict[str, int] = {}
        pointing_policy_counts: dict[str, int] = {}
        pointing_context_source_counts: dict[str, int] = {}
        recent_fail_reasons: deque[tuple[float, str]] = deque()

        output_headers = [
            "Step",
            "Control_Time_s",
            "Solve_Time_s",
            "Solve_Time_ms",
            "Pos_Error_m",
            "Ang_Error_deg",
            "Linear_Speed_mps",
            "Angular_Rate_radps",
            "Active_Thrusters",
            "Thruster_Switches",
            "Path_S_m",
            "Path_Progress",
            "Path_Remaining_m",
            "Path_Error_m",
            "Path_Endpoint_Error_m",
            "Velocity_Error_mps",
            "Angular_Velocity_Error_degps",
            "Timing_Violation",
            "MPC_Time_Limit_Exceeded",
            "MPC_Status",
            "MPC_Iterations",
            "Terminal_Gate_Fail_Reason",
            "Hold_Timer_s",
            "Hold_Reset_Count",
            "Pointing_Policy_Active",
            "Pointing_Context_Source_Active",
        ]

        with (
            control_csv.open("r", encoding="utf-8", newline="") as src,
            output_csv.open("w", encoding="utf-8", newline="") as dst,
        ):
            reader = csv.DictReader(src)
            writer = csv.DictWriter(dst, fieldnames=output_headers)
            writer.writeheader()

            for row in reader:
                control_steps += 1
                t = self._to_float(
                    row.get("Control_Time"), self._to_float(row.get("Time"))
                )
                solve_time_s = self._to_float(
                    row.get("MPC_Solve_Time"), self._to_float(row.get("Solve_Time"))
                )
                solve_time_ms = solve_time_s * 1000.0
                pos_error_m = self._vector_norm3(
                    self._to_float(row.get("Error_X")),
                    self._to_float(row.get("Error_Y")),
                    self._to_float(row.get("Error_Z")),
                )
                if row.get("Error_Angle_Rad") not in (None, ""):
                    ang_error_rad = self._to_float(row.get("Error_Angle_Rad"))
                elif all(
                    row.get(col) not in (None, "")
                    for col in (
                        "Current_QW",
                        "Current_QX",
                        "Current_QY",
                        "Current_QZ",
                        "Reference_QW",
                        "Reference_QX",
                        "Reference_QY",
                        "Reference_QZ",
                    )
                ):
                    q_cur = np.array(
                        [
                            self._to_float(row.get("Current_QW"), 1.0),
                            self._to_float(row.get("Current_QX"), 0.0),
                            self._to_float(row.get("Current_QY"), 0.0),
                            self._to_float(row.get("Current_QZ"), 0.0),
                        ],
                        dtype=float,
                    )
                    q_ref = np.array(
                        [
                            self._to_float(row.get("Reference_QW"), 1.0),
                            self._to_float(row.get("Reference_QX"), 0.0),
                            self._to_float(row.get("Reference_QY"), 0.0),
                            self._to_float(row.get("Reference_QZ"), 0.0),
                        ],
                        dtype=float,
                    )
                    ang_error_rad = float(quat_angle_error(q_ref, q_cur))
                else:
                    ang_error_rad = 0.0
                ang_error_deg = math.degrees(ang_error_rad)
                speed_mps = self._vector_norm3(
                    self._to_float(row.get("Current_VX")),
                    self._to_float(row.get("Current_VY")),
                    self._to_float(row.get("Current_VZ")),
                )
                rate_radps = self._vector_norm3(
                    self._to_float(row.get("Current_WX")),
                    self._to_float(row.get("Current_WY")),
                    self._to_float(row.get("Current_WZ")),
                )
                velocity_error_mps = self._vector_norm3(
                    self._to_float(row.get("Error_VX")),
                    self._to_float(row.get("Error_VY")),
                    self._to_float(row.get("Error_VZ")),
                )
                angular_velocity_error_radps = self._vector_norm3(
                    self._to_float(row.get("Error_WX")),
                    self._to_float(row.get("Error_WY")),
                    self._to_float(row.get("Error_WZ")),
                )
                angular_velocity_error_degps = math.degrees(
                    angular_velocity_error_radps
                )
                active_thrusters = int(
                    self._to_float(row.get("Total_Active_Thrusters"))
                )
                thruster_switches = int(self._to_float(row.get("Thruster_Switches")))
                path_s = self._to_float(row.get("Path_S"))
                path_progress = self._to_float(row.get("Path_Progress"))
                path_remaining = self._to_float(row.get("Path_Remaining"))
                path_error = self._to_float(row.get("Path_Error"))
                endpoint_error = self._to_float(row.get("Path_Endpoint_Error"))
                timing_violation = self._to_bool_flag(row.get("Timing_Violation", ""))
                time_limit_exceeded = self._to_bool_flag(
                    row.get("MPC_Time_Limit_Exceeded", "")
                )
                terminal_gate_fail_reason = _normalize_terminal_fail_reason(
                    row.get("Terminal_Gate_Fail_Reason")
                    or row.get("Completion_Gate_Last_Breach_Reason")
                )
                hold_timer_s = self._to_float(row.get("Hold_Timer_s"))
                hold_reset_count = max(
                    hold_reset_count, int(self._to_float(row.get("Hold_Reset_Count")))
                )
                pointing_policy = str(row.get("Pointing_Policy_Active") or "").strip()
                if not pointing_policy:
                    pointing_policy = "unknown"
                pointing_context_source = str(
                    row.get("Pointing_Context_Source_Active")
                    or row.get("Pointing_Context_Source")
                    or ""
                ).strip()
                if not pointing_context_source:
                    pointing_context_source = "unknown"

                writer.writerow(
                    {
                        "Step": row.get("Step", control_steps),
                        "Control_Time_s": f"{t:.6f}",
                        "Solve_Time_s": f"{solve_time_s:.6f}",
                        "Solve_Time_ms": f"{solve_time_ms:.3f}",
                        "Pos_Error_m": f"{pos_error_m:.6f}",
                        "Ang_Error_deg": f"{ang_error_deg:.6f}",
                        "Linear_Speed_mps": f"{speed_mps:.6f}",
                        "Angular_Rate_radps": f"{rate_radps:.6f}",
                        "Active_Thrusters": active_thrusters,
                        "Thruster_Switches": thruster_switches,
                        "Path_S_m": f"{path_s:.6f}",
                        "Path_Progress": f"{path_progress:.6f}",
                        "Path_Remaining_m": f"{path_remaining:.6f}",
                        "Path_Error_m": f"{path_error:.6f}",
                        "Path_Endpoint_Error_m": f"{endpoint_error:.6f}",
                        "Velocity_Error_mps": f"{velocity_error_mps:.6f}",
                        "Angular_Velocity_Error_degps": (
                            f"{angular_velocity_error_degps:.6f}"
                        ),
                        "Timing_Violation": int(timing_violation),
                        "MPC_Time_Limit_Exceeded": int(time_limit_exceeded),
                        "MPC_Status": row.get("MPC_Status", ""),
                        "MPC_Iterations": row.get("MPC_Iterations", ""),
                        "Terminal_Gate_Fail_Reason": terminal_gate_fail_reason,
                        "Hold_Timer_s": f"{hold_timer_s:.6f}",
                        "Hold_Reset_Count": hold_reset_count,
                        "Pointing_Policy_Active": pointing_policy,
                        "Pointing_Context_Source_Active": pointing_context_source,
                    }
                )

                solve_times_ms.append(solve_time_ms)
                total_active_thrusters += active_thrusters
                total_thruster_switches += thruster_switches
                final_time_s = t
                final_position_error_m = pos_error_m
                final_angle_error_deg = ang_error_deg
                final_velocity_error_mps = velocity_error_mps
                final_angular_velocity_error_degps = angular_velocity_error_degps
                final_path_progress = path_progress
                final_path_remaining_m = path_remaining

                if pos_error_m >= max_position_error["value"]:
                    max_position_error = {"value": pos_error_m, "time": t}
                if ang_error_deg >= max_angle_error["value"]:
                    max_angle_error = {"value": ang_error_deg, "time": t}

                if timing_violation:
                    timing_violation_count += 1
                    if first_timing_violation_time is None:
                        first_timing_violation_time = t
                if time_limit_exceeded:
                    time_limit_exceeded_count += 1
                    if first_time_limit_exceeded_time is None:
                        first_time_limit_exceeded_time = t

                terminal_gate_fail_reason_counts[terminal_gate_fail_reason] = (
                    int(
                        terminal_gate_fail_reason_counts.get(
                            terminal_gate_fail_reason, 0
                        )
                    )
                    + 1
                )
                pointing_policy_counts[pointing_policy] = (
                    int(pointing_policy_counts.get(pointing_policy, 0)) + 1
                )
                pointing_context_source_counts[pointing_context_source] = (
                    int(pointing_context_source_counts.get(pointing_context_source, 0))
                    + 1
                )
                if terminal_gate_fail_reason != "none":
                    recent_fail_reasons.append((t, terminal_gate_fail_reason))
                while recent_fail_reasons and (t - recent_fail_reasons[0][0]) > 60.0:
                    recent_fail_reasons.popleft()

        mean_active_thrusters = (
            total_active_thrusters / control_steps if control_steps > 0 else 0.0
        )
        terminal_gate_fail_reason_counts_last_60s: dict[str, int] = {}
        for _, reason in recent_fail_reasons:
            terminal_gate_fail_reason_counts_last_60s[reason] = (
                int(terminal_gate_fail_reason_counts_last_60s.get(reason, 0)) + 1
            )

        return {
            "control_steps": control_steps,
            "solve_times_ms": solve_times_ms,
            "timing_violation_count": timing_violation_count,
            "time_limit_exceeded_count": time_limit_exceeded_count,
            "first_time_limit_exceeded_time_s": first_time_limit_exceeded_time,
            "first_timing_violation_time_s": first_timing_violation_time,
            "final_time_s": final_time_s,
            "final_position_error_m": final_position_error_m,
            "final_angle_error_deg": final_angle_error_deg,
            "final_velocity_error_mps": final_velocity_error_mps,
            "final_angular_velocity_error_degps": final_angular_velocity_error_degps,
            "final_path_progress": final_path_progress,
            "final_path_remaining_m": final_path_remaining_m,
            "max_position_error": max_position_error,
            "max_angle_error": max_angle_error,
            "mean_active_thrusters": mean_active_thrusters,
            "total_thruster_switches": total_thruster_switches,
            "terminal_gate_fail_reason_counts": terminal_gate_fail_reason_counts,
            "terminal_gate_fail_reason_counts_last_60s": (
                terminal_gate_fail_reason_counts_last_60s
            ),
            "pointing_policy_counts": pointing_policy_counts,
            "pointing_context_source_counts": pointing_context_source_counts,
            "hold_reset_count": int(hold_reset_count),
        }

    def _collect_physics_stats(self, run_dir: Path) -> dict[str, Any]:
        """Collect high-frequency stats from controller.shared.python.physics.data.csv."""
        physics_csv = self._artifact_existing_path(
            run_dir, "physics_data.csv"
        ) or self._artifact_path(run_dir, "physics_data.csv")
        if not physics_csv.exists():
            return {
                "physics_steps": 0,
                "max_linear_speed_mps": 0.0,
                "max_angular_rate_radps": 0.0,
                "max_linear_speed_time_s": 0.0,
                "max_angular_rate_time_s": 0.0,
                "linear_speed_violation_count": 0,
                "angular_rate_violation_count": 0,
            }

        sim_config = getattr(self.sim, "simulation_config", None)
        app_config = getattr(sim_config, "app_config", None)
        mpc_cfg = getattr(app_config, "mpc", None)
        linear_limit = (
            float(getattr(mpc_cfg, "max_linear_velocity", 0.0)) if mpc_cfg else 0.0
        )
        angular_limit = (
            float(getattr(mpc_cfg, "max_angular_velocity", 0.0)) if mpc_cfg else 0.0
        )

        physics_steps = 0
        max_linear_speed_mps = 0.0
        max_angular_rate_radps = 0.0
        max_linear_speed_time_s = 0.0
        max_angular_rate_time_s = 0.0
        linear_speed_violation_count = 0
        angular_rate_violation_count = 0

        with physics_csv.open("r", encoding="utf-8", newline="") as src:
            reader = csv.DictReader(src)
            for row in reader:
                physics_steps += 1
                t = self._to_float(row.get("Time"))
                vx = self._to_float(row.get("Current_VX"))
                vy = self._to_float(row.get("Current_VY"))
                vz = self._to_float(row.get("Current_VZ"))
                wx = self._to_float(row.get("Current_WX"))
                wy = self._to_float(row.get("Current_WY"))
                wz = self._to_float(row.get("Current_WZ"))

                linear_speed = self._vector_norm3(vx, vy, vz)
                angular_rate = self._vector_norm3(wx, wy, wz)
                if linear_speed >= max_linear_speed_mps:
                    max_linear_speed_mps = linear_speed
                    max_linear_speed_time_s = t
                if angular_rate >= max_angular_rate_radps:
                    max_angular_rate_radps = angular_rate
                    max_angular_rate_time_s = t

                if linear_limit > 0.0 and linear_speed > linear_limit:
                    linear_speed_violation_count += 1
                if angular_limit > 0.0 and angular_rate > angular_limit:
                    angular_rate_violation_count += 1

        return {
            "physics_steps": physics_steps,
            "max_linear_speed_mps": max_linear_speed_mps,
            "max_angular_rate_radps": max_angular_rate_radps,
            "max_linear_speed_time_s": max_linear_speed_time_s,
            "max_angular_rate_time_s": max_angular_rate_time_s,
            "linear_speed_violation_count": linear_speed_violation_count,
            "angular_rate_violation_count": angular_rate_violation_count,
        }

    def _collect_mission_stats(self) -> dict[str, Any]:
        """Collect mission-level values available from controller.shared.python.runtime.state."""
        mission_state = getattr(self.sim, "mission_state", None)
        if mission_state is None and getattr(self.sim, "simulation_config", None):
            mission_state = getattr(self.sim.simulation_config, "mission_state", None)

        path_length = 0.0
        path_points = 0
        path_completed = False
        if mission_state is not None:
            try:
                path_waypoints = mission_state.get_resolved_path_waypoints()
                path_points = len(path_waypoints or [])
            except Exception:
                path_points = len(getattr(mission_state, "path_waypoints", []) or [])
            try:
                path_length = float(
                    mission_state.get_resolved_path_length(compute_if_missing=True)
                )
            except Exception:
                path_length = float(getattr(mission_state, "path_length", 0.0) or 0.0)

        try:
            path_completed = bool(self.sim.check_path_complete())
        except Exception:
            path_completed = False

        return {
            "path_length_m": path_length,
            "path_waypoint_count": path_points,
            "path_completed": path_completed,
            "trajectory_endpoint_reached_time_s": getattr(
                self.sim, "trajectory_endpoint_reached_time", None
            ),
        }

    def _build_kpi_summary(
        self,
        run_dir: Path,
        control_stats: dict[str, Any],
        physics_stats: dict[str, Any],
        mission_stats: dict[str, Any],
    ) -> dict[str, Any]:
        """Build compact KPI summary JSON for quick run-level comparison."""
        solve_times = control_stats.get("solve_times_ms", [])
        solve_mean = sum(solve_times) / len(solve_times) if solve_times else 0.0
        solve_max = max(solve_times) if solve_times else 0.0

        def percentile(data: list[float], p: float) -> float:
            if not data:
                return 0.0
            sorted_vals = sorted(data)
            idx = int(round((len(sorted_vals) - 1) * p))
            return sorted_vals[max(0, min(idx, len(sorted_vals) - 1))]

        perf_metrics_path = self._artifact_existing_path(
            run_dir, "performance_metrics.json"
        ) or self._artifact_path(run_dir, "performance_metrics.json")
        perf_metrics = {}
        if perf_metrics_path.exists():
            try:
                perf_metrics = json.loads(perf_metrics_path.read_text(encoding="utf-8"))
            except Exception:
                perf_metrics = {}

        solver_health = getattr(self.sim, "solver_health", None)
        solver_fallback_count = int(getattr(solver_health, "fallback_count", 0))
        solver_hard_limit_breaches = int(
            getattr(solver_health, "hard_limit_breaches", 0)
        )
        solver_status = str(getattr(solver_health, "status", "ok"))
        solver_fallback_active = bool(getattr(solver_health, "fallback_active", False))
        solver_fallback_age_s = float(getattr(solver_health, "fallback_age_s", 0.0))
        solver_fallback_scale = float(getattr(solver_health, "fallback_scale", 0.0))
        terminal_fail_counts = dict(
            control_stats.get("terminal_gate_fail_reason_counts", {}) or {}
        )
        terminal_fail_counts_last_60s = dict(
            control_stats.get("terminal_gate_fail_reason_counts_last_60s", {}) or {}
        )
        dominant_terminal_fail_reason_last_60s = "none"
        if terminal_fail_counts_last_60s:
            dominant_terminal_fail_reason_last_60s = max(
                terminal_fail_counts_last_60s.items(),
                key=lambda item: int(item[1]),
            )[0]

        return {
            "schema_version": "kpi_summary_v1",
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "final_time_s": control_stats.get("final_time_s", 0.0),
            "final_position_error_m": control_stats.get("final_position_error_m", 0.0),
            "final_angle_error_deg": control_stats.get("final_angle_error_deg", 0.0),
            "final_velocity_error_mps": control_stats.get(
                "final_velocity_error_mps", 0.0
            ),
            "final_angular_velocity_error_degps": control_stats.get(
                "final_angular_velocity_error_degps", 0.0
            ),
            "max_position_error_m": control_stats.get("max_position_error", {}).get(
                "value", 0.0
            ),
            "max_angle_error_deg": control_stats.get("max_angle_error", {}).get(
                "value", 0.0
            ),
            "mpc_control_steps": control_stats.get("control_steps", 0),
            "physics_steps": physics_stats.get("physics_steps", 0),
            "mpc_mean_solve_time_ms": solve_mean,
            "mpc_p95_solve_time_ms": percentile(solve_times, 0.95),
            "mpc_max_solve_time_ms": solve_max,
            "timing_violation_count": control_stats.get("timing_violation_count", 0),
            "solver_time_limit_exceeded_count": control_stats.get(
                "time_limit_exceeded_count", 0
            ),
            "mean_active_thrusters": control_stats.get("mean_active_thrusters", 0.0),
            "total_thruster_switches": control_stats.get("total_thruster_switches", 0),
            "max_linear_speed_mps": physics_stats.get("max_linear_speed_mps", 0.0),
            "max_angular_rate_degps": math.degrees(
                physics_stats.get("max_angular_rate_radps", 0.0)
            ),
            "final_path_progress": control_stats.get("final_path_progress", 0.0),
            "final_path_remaining_m": control_stats.get("final_path_remaining_m", 0.0),
            "path_length_m": mission_stats.get("path_length_m", 0.0),
            "path_waypoint_count": mission_stats.get("path_waypoint_count", 0),
            "path_completed": mission_stats.get("path_completed", False),
            "solver_fallback_count": solver_fallback_count,
            "solver_hard_limit_breaches": solver_hard_limit_breaches,
            "solver_status": solver_status,
            "solver_fallback_active": solver_fallback_active,
            "solver_fallback_age_s": solver_fallback_age_s,
            "solver_fallback_scale": solver_fallback_scale,
            "terminal_gate_fail_reason_counts": terminal_fail_counts,
            "terminal_gate_fail_reason_counts_last_60s": terminal_fail_counts_last_60s,
            "dominant_terminal_gate_fail_reason_last_60s": (
                dominant_terminal_fail_reason_last_60s
            ),
            "hold_reset_count": int(control_stats.get("hold_reset_count", 0)),
            "pointing_policy_counts": dict(
                control_stats.get("pointing_policy_counts", {}) or {}
            ),
            "pointing_context_source_counts": dict(
                control_stats.get("pointing_context_source_counts", {}) or {}
            ),
            "performance_metrics_ref": perf_metrics.get("simulation", {}),
        }

    def _build_constraint_violations(
        self,
        run_dir: Path,
        control_stats: dict[str, Any],
        physics_stats: dict[str, Any],
        mission_stats: dict[str, Any],
    ) -> dict[str, Any]:
        """Build explicit list of threshold/constraint violations."""
        sim_config = getattr(self.sim, "simulation_config", None)
        app_config = getattr(sim_config, "app_config", None)
        mpc_cfg = getattr(app_config, "mpc", None)

        linear_limit = (
            float(getattr(mpc_cfg, "max_linear_velocity", 0.0)) if mpc_cfg else 0.0
        )
        angular_limit = (
            float(getattr(mpc_cfg, "max_angular_velocity", 0.0)) if mpc_cfg else 0.0
        )
        solver_time_limit_s = (
            float(getattr(mpc_cfg, "solver_time_limit", 0.0)) if mpc_cfg else 0.0
        )

        violations: list[dict[str, Any]] = []

        linear_count = int(physics_stats.get("linear_speed_violation_count", 0))
        if linear_count > 0 and linear_limit > 0.0:
            violations.append(
                {
                    "type": "max_linear_velocity",
                    "count": linear_count,
                    "limit_mps": linear_limit,
                    "max_observed_mps": physics_stats.get("max_linear_speed_mps"),
                    "max_observed_time_s": physics_stats.get("max_linear_speed_time_s"),
                }
            )

        angular_count = int(physics_stats.get("angular_rate_violation_count", 0))
        if angular_count > 0 and angular_limit > 0.0:
            violations.append(
                {
                    "type": "max_angular_velocity",
                    "count": angular_count,
                    "limit_radps": angular_limit,
                    "max_observed_radps": physics_stats.get("max_angular_rate_radps"),
                    "max_observed_time_s": physics_stats.get("max_angular_rate_time_s"),
                }
            )

        timing_count = int(control_stats.get("timing_violation_count", 0))
        if timing_count > 0:
            violations.append(
                {
                    "type": "control_timing_violation",
                    "count": timing_count,
                    "first_time_s": control_stats.get("first_timing_violation_time_s"),
                }
            )

        solver_limit_count = int(control_stats.get("time_limit_exceeded_count", 0))
        if solver_limit_count > 0:
            violations.append(
                {
                    "type": "mpc_solver_time_limit_exceeded",
                    "count": solver_limit_count,
                    "solver_time_limit_s": solver_time_limit_s,
                    "first_time_s": control_stats.get(
                        "first_time_limit_exceeded_time_s"
                    ),
                }
            )

        return {
            "schema_version": "constraint_violations_v1",
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "path_completed": mission_stats.get("path_completed", False),
            "limits": {
                "max_linear_velocity_mps": linear_limit,
                "max_angular_velocity_radps": angular_limit,
                "solver_time_limit_s": solver_time_limit_s,
            },
            "violations": violations,
            "pass": len(violations) == 0,
        }

    def _write_event_timeline(
        self,
        run_dir: Path,
        control_stats: dict[str, Any],
        physics_stats: dict[str, Any],
        run_status: str,
        status_detail: str | None,
    ) -> None:
        """Write coarse event timeline as JSON-lines for easy streaming/grep."""
        events: list[dict[str, Any]] = []
        events.append(
            {
                "time_s": 0.0,
                "event": "run_started",
                "details": {"run_id": run_dir.name},
            }
        )

        max_pos = control_stats.get("max_position_error", {})
        max_ang = control_stats.get("max_angle_error", {})
        if max_pos.get("value", 0.0) > 0.0:
            events.append(
                {
                    "time_s": max_pos.get("time", 0.0),
                    "event": "max_position_error",
                    "details": {"value_m": max_pos.get("value", 0.0)},
                }
            )
        if max_ang.get("value", 0.0) > 0.0:
            events.append(
                {
                    "time_s": max_ang.get("time", 0.0),
                    "event": "max_angle_error",
                    "details": {"value_deg": max_ang.get("value", 0.0)},
                }
            )

        first_timeout = control_stats.get("first_time_limit_exceeded_time_s")
        if first_timeout is not None:
            events.append(
                {
                    "time_s": first_timeout,
                    "event": "first_solver_time_limit_exceeded",
                    "details": {},
                }
            )

        first_timing_violation = control_stats.get("first_timing_violation_time_s")
        if first_timing_violation is not None:
            events.append(
                {
                    "time_s": first_timing_violation,
                    "event": "first_control_timing_violation",
                    "details": {},
                }
            )

        max_lin_t = physics_stats.get("max_linear_speed_time_s", 0.0)
        max_lin_v = physics_stats.get("max_linear_speed_mps", 0.0)
        if max_lin_v > 0.0:
            events.append(
                {
                    "time_s": max_lin_t,
                    "event": "max_linear_speed",
                    "details": {"value_mps": max_lin_v},
                }
            )

        final_time_s = control_stats.get("final_time_s", 0.0)
        events.append(
            {
                "time_s": final_time_s,
                "event": "run_finished",
                "details": {"status": run_status, "status_detail": status_detail},
            }
        )

        events.sort(key=lambda item: self._to_float(item.get("time_s")))
        timeline_path = self._artifact_path(run_dir, "event_timeline.jsonl")
        with timeline_path.open("w", encoding="utf-8") as handle:
            for event in events:
                handle.write(json.dumps(event) + "\n")

    def _write_mode_timeline(self, run_dir: Path) -> None:
        """Write mode timeline CSV when runtime mode telemetry is available."""
        entries = getattr(self.sim, "mode_timeline", None)
        if not isinstance(entries, list) or not entries:
            return

        output_path = self._artifact_path(run_dir, "mode_timeline.csv")
        headers = [
            "time_s",
            "mode",
            "time_in_mode_s",
            "path_s",
            "path_error_m",
            "endpoint_error_m",
        ]
        with output_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            for row in entries:
                if not isinstance(row, dict):
                    continue
                writer.writerow(
                    {
                        "time_s": f"{self._to_float(row.get('time_s')):.6f}",
                        "mode": str(row.get("mode", "")),
                        "time_in_mode_s": f"{self._to_float(row.get('time_in_mode_s')):.6f}",
                        "path_s": f"{self._to_float(row.get('path_s')):.6f}",
                        "path_error_m": f"{self._to_float(row.get('path_error_m')):.6f}",
                        "endpoint_error_m": (
                            ""
                            if row.get("endpoint_error_m") is None
                            else f"{self._to_float(row.get('endpoint_error_m')):.6f}"
                        ),
                    }
                )

    def _write_completion_gate_trace(self, run_dir: Path) -> None:
        """Write completion gate trace CSV for strict termination auditability."""
        entries = getattr(self.sim, "completion_gate_trace", None)
        if not isinstance(entries, list) or not entries:
            return

        output_path = self._artifact_path(run_dir, "completion_gate_trace.csv")
        headers = [
            "time_s",
            "progress_ok",
            "position_ok",
            "angle_ok",
            "velocity_ok",
            "angular_velocity_ok",
            "hold_elapsed_s",
            "hold_required_s",
            "gate_ok",
            "complete",
            "last_breach_reason",
            "fail_reason",
            "hold_reset_count",
            "path_s",
            "path_length",
            "path_error",
        ]
        with output_path.open("w", encoding="utf-8", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            for row in entries:
                if not isinstance(row, dict):
                    continue
                writer.writerow(
                    {
                        "time_s": f"{self._to_float(row.get('time_s')):.6f}",
                        "progress_ok": int(bool(row.get("progress_ok", False))),
                        "position_ok": int(bool(row.get("position_ok", False))),
                        "angle_ok": int(bool(row.get("angle_ok", False))),
                        "velocity_ok": int(bool(row.get("velocity_ok", False))),
                        "angular_velocity_ok": int(
                            bool(row.get("angular_velocity_ok", False))
                        ),
                        "hold_elapsed_s": f"{self._to_float(row.get('hold_elapsed_s')):.6f}",
                        "hold_required_s": f"{self._to_float(row.get('hold_required_s')):.6f}",
                        "gate_ok": int(bool(row.get("gate_ok", False))),
                        "complete": int(bool(row.get("complete", False))),
                        "last_breach_reason": str(
                            row.get("last_breach_reason", "") or ""
                        ),
                        "fail_reason": str(row.get("fail_reason", "") or ""),
                        "hold_reset_count": int(
                            self._to_float(row.get("hold_reset_count"), 0.0)
                        ),
                        "path_s": f"{self._to_float(row.get('path_s')):.6f}",
                        "path_length": f"{self._to_float(row.get('path_length')):.6f}",
                        "path_error": f"{self._to_float(row.get('path_error')):.6f}",
                    }
                )

    def _write_controller_health(self, run_dir: Path) -> None:
        """Write controller health summary JSON."""
        solver_health = getattr(self.sim, "solver_health", None)
        mode_state = getattr(self.sim, "mode_state", None)
        completion_gate = getattr(self.sim, "completion_gate", None)
        pointing_status = getattr(self.sim, "pointing_status", None)
        mpc_controller = getattr(self.sim, "mpc_controller", None)
        payload = {
            "schema_version": "controller_health",
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "controller_core": str(
                getattr(self.sim, "controller_core_mode", None)
                or getattr(mpc_controller, "controller_core", "v6")
            ),
            "controller_profile": str(
                getattr(self.sim, "controller_profile_mode", None)
                or getattr(mpc_controller, "controller_profile", "cpp_hybrid_rti_osqp")
            ),
            "solver_backend": str(getattr(mpc_controller, "solver_backend", "OSQP")),
            "linearization_mode": str(
                getattr(self.sim, "linearization_mode", None)
                or getattr(
                    mpc_controller, "linearization_mode", "hybrid_tolerant_stage"
                )
            ),
            "shared_params_hash": str(
                getattr(mpc_controller, "shared_params_hash", "unknown")
            ),
            "effective_params_hash": str(
                getattr(mpc_controller, "effective_params_hash", "unknown")
            ),
            "override_diff": dict(getattr(mpc_controller, "profile_override_diff", {})),
            "solver_health": {
                "status": str(getattr(solver_health, "status", "ok")),
                "fallback_count": int(getattr(solver_health, "fallback_count", 0)),
                "hard_limit_breaches": int(
                    getattr(solver_health, "hard_limit_breaches", 0)
                ),
                "last_fallback_reason": getattr(
                    solver_health, "last_fallback_reason", None
                ),
                "fallback_reasons": dict(
                    getattr(solver_health, "fallback_reasons", {}) or {}
                ),
                "fallback_active": bool(
                    getattr(solver_health, "fallback_active", False)
                ),
                "fallback_age_s": float(
                    self._to_float(getattr(solver_health, "fallback_age_s", 0.0))
                ),
                "fallback_scale": float(
                    self._to_float(getattr(solver_health, "fallback_scale", 0.0))
                ),
            },
            "mode_state": {
                "current_mode": str(getattr(mode_state, "current_mode", "TRACK")),
                "time_in_mode_s": float(
                    self._to_float(getattr(mode_state, "time_in_mode_s", 0.0))
                ),
            },
            "completion_gate": {
                "position_ok": bool(getattr(completion_gate, "position_ok", False)),
                "angle_ok": bool(getattr(completion_gate, "angle_ok", False)),
                "velocity_ok": bool(getattr(completion_gate, "velocity_ok", False)),
                "angular_velocity_ok": bool(
                    getattr(completion_gate, "angular_velocity_ok", False)
                ),
                "hold_elapsed_s": float(
                    self._to_float(getattr(completion_gate, "hold_elapsed_s", 0.0))
                ),
                "hold_required_s": float(
                    self._to_float(getattr(completion_gate, "hold_required_s", 0.0))
                ),
                "last_breach_reason": getattr(
                    completion_gate, "last_breach_reason", None
                ),
                "fail_reason": getattr(completion_gate, "fail_reason", "none"),
                "hold_reset_count": int(
                    self._to_float(getattr(completion_gate, "hold_reset_count", 0.0))
                ),
                "complete": bool(getattr(completion_gate, "complete", False)),
            },
            "pointing_status": {
                "pointing_context_source": (
                    pointing_status.get("pointing_context_source")
                    if isinstance(pointing_status, dict)
                    else None
                ),
                "pointing_policy": (
                    pointing_status.get("pointing_policy")
                    if isinstance(pointing_status, dict)
                    else None
                ),
                "pointing_axis_world": (
                    list(pointing_status.get("pointing_axis_world", [0.0, 0.0, 1.0]))
                    if isinstance(pointing_status, dict)
                    else [0.0, 0.0, 1.0]
                ),
                "z_axis_error_deg": float(
                    self._to_float(
                        pointing_status.get("z_axis_error_deg")
                        if isinstance(pointing_status, dict)
                        else 0.0
                    )
                ),
                "x_axis_error_deg": float(
                    self._to_float(
                        pointing_status.get("x_axis_error_deg")
                        if isinstance(pointing_status, dict)
                        else 0.0
                    )
                ),
                "pointing_guardrail_breached": bool(
                    pointing_status.get("pointing_guardrail_breached", False)
                    if isinstance(pointing_status, dict)
                    else False
                ),
                "pointing_guardrail_reason": (
                    pointing_status.get("pointing_guardrail_reason")
                    if isinstance(pointing_status, dict)
                    else None
                ),
                "object_visible_side": (
                    pointing_status.get("object_visible_side")
                    if isinstance(pointing_status, dict)
                    else None
                ),
            },
        }
        self._write_json(
            self._artifact_path(run_dir, "controller_health.json"), payload
        )

    def _build_plots_index(self, run_dir: Path) -> dict[str, Any]:
        """Index static/interactive plot outputs for quick UI consumption."""
        plots_dir = run_dir / "Plots"
        plot_files: list[dict[str, Any]] = []

        if plots_dir.exists():
            for path in sorted(plots_dir.rglob("*")):
                if not path.is_file():
                    continue
                rel = path.relative_to(run_dir)
                plot_files.append(
                    {
                        "path": str(rel),
                        "name": path.name,
                        "size_bytes": path.stat().st_size,
                        "format": path.suffix.lower().lstrip("."),
                        "interactive": path.suffix.lower() == ".html",
                    }
                )

        top_level_html: list[dict[str, Any]] = []
        for path in sorted(run_dir.glob("*.html")):
            top_level_html.append(
                {
                    "path": path.name,
                    "name": path.name,
                    "size_bytes": path.stat().st_size,
                    "format": "html",
                    "interactive": True,
                }
            )

        manifest_path = plots_dir / "plot_manifest.json"
        manifest: dict[str, Any] = {}
        if manifest_path.exists():
            try:
                payload = json.loads(manifest_path.read_text(encoding="utf-8"))
                if isinstance(payload, dict):
                    manifest = payload
            except Exception:
                manifest = {}

        files_v2: list[dict[str, Any]] = []
        groups_v2: list[dict[str, Any]] = []
        failures_v2: list[dict[str, Any]] = []

        if manifest:
            raw_groups = manifest.get("groups", [])
            if isinstance(raw_groups, list):
                groups_v2 = sorted(
                    [
                        {
                            "id": str(group.get("id", "unknown")),
                            "order": int(self._to_float(group.get("order"), 0)),
                            "title": str(group.get("title", "Unknown")),
                            "path": str(group.get("path", "")),
                            "file_count": int(
                                self._to_float(group.get("file_count"), default=0.0)
                            ),
                        }
                        for group in raw_groups
                        if isinstance(group, dict)
                    ],
                    key=lambda item: item["order"],
                )

            raw_files = manifest.get("files", [])
            if isinstance(raw_files, list):
                files_v2 = sorted(
                    [
                        {
                            "plot_id": str(item.get("plot_id", "")),
                            "title": str(item.get("title", "")),
                            "path": str(item.get("path", "")),
                            "order": int(self._to_float(item.get("order"), 0)),
                            "group_id": str(item.get("group_id", "")),
                            "format": str(
                                item.get(
                                    "format",
                                    Path(str(item.get("path", ""))).suffix.lstrip("."),
                                )
                            ),
                            "interactive": bool(item.get("interactive", False)),
                            "status": str(item.get("status", "ok")),
                        }
                        for item in raw_files
                        if isinstance(item, dict)
                    ],
                    key=lambda item: item["order"],
                )

            raw_failures = manifest.get("failures", [])
            if isinstance(raw_failures, list):
                failures_v2 = [
                    {
                        "plot_id": str(item.get("plot_id", "")),
                        "reason": str(item.get("reason", "")),
                    }
                    for item in raw_failures
                    if isinstance(item, dict)
                ]
        else:
            group_map: dict[str, dict[str, Any]] = {}
            for item in plot_files:
                rel = Path(item["path"])
                if len(rel.parts) < 3 or rel.parts[0] != "Plots":
                    continue
                folder = rel.parts[1]
                match = re.match(r"^(\d+)_?(.*)$", folder)
                if match:
                    group_order = int(match.group(1))
                    group_title = (
                        match.group(2).replace("_", " ").strip().title() or folder
                    )
                else:
                    group_order = 999
                    group_title = folder.replace("_", " ").title()
                group_id = folder
                if group_id not in group_map:
                    group_map[group_id] = {
                        "id": group_id,
                        "order": group_order,
                        "title": group_title,
                        "path": str(Path("Plots") / folder),
                        "file_count": 0,
                    }
                group_map[group_id]["file_count"] += 1

                file_match = re.match(r"^(\d+)_?(.*)$", item["name"])
                within_group_order = int(file_match.group(1)) if file_match else 999
                files_v2.append(
                    {
                        "plot_id": Path(item["name"]).stem,
                        "title": Path(item["name"]).stem.replace("_", " ").title(),
                        "path": item["path"],
                        "order": group_order * 1000 + within_group_order,
                        "group_id": group_id,
                        "format": item["format"],
                        "interactive": item["interactive"],
                        "status": "ok",
                    }
                )
            groups_v2 = sorted(group_map.values(), key=lambda item: item["order"])
            files_v2 = sorted(files_v2, key=lambda item: item["order"])

        return {
            "schema_version": "plots_index_v2",
            "suite_version": str(
                manifest.get(
                    "suite_version", "postrun_v2_full" if manifest else "legacy"
                )
            ),
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "groups": groups_v2,
            "files": files_v2,
            "failures": failures_v2,
            "plot_files": plot_files,
            "top_level_html": top_level_html,
            "plot_count": len(plot_files),
        }

    def _build_media_metadata(
        self, run_dir: Path, kpi_summary: dict[str, Any]
    ) -> dict[str, Any]:
        """Collect media output metadata (video/images) for player UX."""
        media_extensions = {".mp4", ".gif", ".webm", ".png", ".jpg", ".jpeg"}
        media_items: list[dict[str, Any]] = []

        for path in sorted(run_dir.rglob("*")):
            if not path.is_file():
                continue
            suffix = path.suffix.lower()
            if suffix not in media_extensions:
                continue
            rel = path.relative_to(run_dir)
            item: dict[str, Any] = {
                "path": str(rel),
                "name": path.name,
                "format": suffix.lstrip("."),
                "size_bytes": path.stat().st_size,
                "kind": "video" if suffix in {".mp4", ".gif", ".webm"} else "image",
            }
            if item["kind"] == "video":
                item["duration_s"] = self._probe_video_duration_seconds(path)
                if item["duration_s"] is None:
                    item["duration_s"] = self._to_float(
                        kpi_summary.get("final_time_s"), 0.0
                    )
            media_items.append(item)

        return {
            "schema_version": "media_metadata_v1",
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "items": media_items,
            "video_count": sum(1 for item in media_items if item["kind"] == "video"),
            "image_count": sum(1 for item in media_items if item["kind"] == "image"),
        }

    def _probe_video_duration_seconds(self, path: Path) -> float | None:
        """Best-effort ffprobe duration extraction for video files."""
        try:
            result = subprocess.run(
                [
                    "ffprobe",
                    "-v",
                    "error",
                    "-show_entries",
                    "format=duration",
                    "-of",
                    "default=noprint_wrappers=1:nokey=1",
                    str(path),
                ],
                capture_output=True,
                text=True,
                timeout=2.0,
                check=False,
            )
            if result.returncode != 0:
                return None
            value = self._to_float(result.stdout.strip(), default=-1.0)
            return value if value >= 0.0 else None
        except Exception:
            return None

    def _build_compare_signature(
        self,
        run_dir: Path,
        kpi_summary: dict[str, Any],
        control_stats: dict[str, Any],
    ) -> dict[str, Any]:
        """Build compact deterministic signature for run-to-run comparisons."""
        status_path = self._artifact_existing_path(
            run_dir, "run_status.json"
        ) or self._artifact_path(run_dir, "run_status.json")
        status_payload: dict[str, Any] = {}
        if status_path.exists():
            try:
                status_payload = json.loads(status_path.read_text(encoding="utf-8"))
            except Exception:
                status_payload = {}

        basis = {
            "mission_name": status_payload.get("mission", {}).get("name"),
            "preset_name": status_payload.get("preset", {}).get("name"),
            "config_hash": status_payload.get("config", {}).get("config_hash"),
            "path_length_m": kpi_summary.get("path_length_m"),
            "final_position_error_m": kpi_summary.get("final_position_error_m"),
            "final_angle_error_deg": kpi_summary.get("final_angle_error_deg"),
            "final_time_s": kpi_summary.get("final_time_s"),
            "mpc_mean_solve_time_ms": kpi_summary.get("mpc_mean_solve_time_ms"),
            "mpc_max_solve_time_ms": kpi_summary.get("mpc_max_solve_time_ms"),
            "timing_violation_count": control_stats.get("timing_violation_count"),
            "solver_time_limit_exceeded_count": control_stats.get(
                "time_limit_exceeded_count"
            ),
        }
        basis_json = json.dumps(basis, sort_keys=True, separators=(",", ":"))
        signature = hashlib.sha256(basis_json.encode("utf-8")).hexdigest()[:24]

        return {
            "schema_version": "compare_signature_v1",
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "signature": signature,
            "basis": basis,
        }

    def _write_run_notes(
        self,
        run_dir: Path,
        kpi_summary: dict[str, Any],
        constraints: dict[str, Any],
        compare_signature: dict[str, Any],
    ) -> None:
        """Write lightweight markdown notes for humans reviewing run outputs."""
        status_payload = {}
        status_path = self._artifact_existing_path(
            run_dir, "run_status.json"
        ) or self._artifact_path(run_dir, "run_status.json")
        if status_path.exists():
            try:
                status_payload = json.loads(status_path.read_text(encoding="utf-8"))
            except Exception:
                status_payload = {}

        lines = [
            f"# Run Notes: {run_dir.name}",
            "",
            f"- Generated: {datetime.utcnow().isoformat()}Z",
            f"- Status: {status_payload.get('status', 'unknown')}",
            f"- Mission: {status_payload.get('mission', {}).get('name') or 'n/a'}",
            f"- Preset: {status_payload.get('preset', {}).get('name') or 'n/a'}",
            f"- Config Hash: {status_payload.get('config', {}).get('config_hash') or 'n/a'}",
            "",
            "## KPI Snapshot",
            "",
            f"- Final Time: {self._to_float(kpi_summary.get('final_time_s')):.3f} s",
            f"- Final Position Error: {self._to_float(kpi_summary.get('final_position_error_m')):.6f} m",
            f"- Final Angle Error: {self._to_float(kpi_summary.get('final_angle_error_deg')):.6f} deg",
            f"- MPC Mean Solve: {self._to_float(kpi_summary.get('mpc_mean_solve_time_ms')):.3f} ms",
            f"- MPC Max Solve: {self._to_float(kpi_summary.get('mpc_max_solve_time_ms')):.3f} ms",
            f"- Timing Violations: {int(self._to_float(kpi_summary.get('timing_violation_count')))}",
            f"- Solver Time-Limit Exceeded: {int(self._to_float(kpi_summary.get('solver_time_limit_exceeded_count')))}",
            "",
            "## Constraints",
            "",
            f"- Pass: {bool(constraints.get('pass'))}",
            f"- Violations: {len(constraints.get('violations', []))}",
            "",
            "## Compare Signature",
            "",
            f"- Signature: `{compare_signature.get('signature', '')}`",
            "",
        ]
        self._artifact_path(run_dir, "run_notes.md").write_text(
            "\n".join(lines), encoding="utf-8"
        )

    def _write_checksums_file(self, run_dir: Path) -> None:
        """Write sha256 checksums for all run files (except this checksum file)."""
        checksum_path = self._artifact_path(run_dir, "checksums.sha256")
        lines: list[str] = []
        for path in sorted(run_dir.rglob("*")):
            if not path.is_file():
                continue
            if path == checksum_path:
                continue
            rel = path.relative_to(run_dir)
            digest = self._sha256_file(path)
            lines.append(f"{digest}  {rel}")
        checksum_path.write_text(
            "\n".join(lines) + ("\n" if lines else ""), encoding="utf-8"
        )

    def _write_artifacts_manifest(self, run_dir: Path) -> None:
        """Write structured manifest for every generated file in run directory."""
        checksums = self._load_checksums_map(
            self._artifact_path(run_dir, "checksums.sha256")
        )
        files: list[dict[str, Any]] = []
        total_size = 0
        for path in sorted(run_dir.rglob("*")):
            if not path.is_file():
                continue
            rel = path.relative_to(run_dir)
            if path == self._artifact_path(run_dir, "artifacts_manifest.json"):
                continue
            size = path.stat().st_size
            total_size += size
            mime_type, _ = mimetypes.guess_type(str(path))
            files.append(
                {
                    "path": str(rel),
                    "size_bytes": size,
                    "sha256": checksums.get(str(rel)),
                    "mime_type": mime_type,
                    "category": self._artifact_category(rel),
                }
            )

        payload = {
            "schema_version": "artifacts_manifest_v1",
            "run_id": run_dir.name,
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "file_count": len(files),
            "total_size_bytes": total_size,
            "files": files,
        }
        self._artifact_path(run_dir, "artifacts_manifest.json").write_text(
            json.dumps(payload, indent=2), encoding="utf-8"
        )

    def _update_global_run_indexes(self, run_dir: Path, max_runs: int = 500) -> None:
        """Update data/simulation_data/runs_index.json and latest_run.txt."""
        base_dir = run_dir.parent
        latest_run_path = base_dir / "latest_run.txt"
        latest_run_path.write_text(run_dir.name + "\n", encoding="utf-8")

        runs: list[dict[str, Any]] = []
        for candidate in sorted(
            base_dir.iterdir(),
            key=lambda path: path.stat().st_mtime if path.exists() else 0.0,
            reverse=True,
        ):
            if not candidate.is_dir():
                continue
            status_path = self._artifact_existing_path(
                candidate, "run_status.json"
            ) or self._artifact_path(candidate, "run_status.json")
            kpi_path = self._artifact_existing_path(
                candidate, "kpi_summary.json"
            ) or self._artifact_path(candidate, "kpi_summary.json")
            if (
                not status_path.exists()
                and self._artifact_existing_path(candidate, "physics_data.csv") is None
            ):
                continue

            status_payload: dict[str, Any] = {}
            kpi_payload: dict[str, Any] = {}
            try:
                if status_path.exists():
                    status_payload = json.loads(status_path.read_text(encoding="utf-8"))
            except Exception:
                status_payload = {}
            try:
                if kpi_path.exists():
                    kpi_payload = json.loads(kpi_path.read_text(encoding="utf-8"))
            except Exception:
                kpi_payload = {}

            runs.append(
                {
                    "id": candidate.name,
                    "modified": candidate.stat().st_mtime,
                    "status": status_payload.get("status", "unknown"),
                    "mission_name": status_payload.get("mission", {}).get("name"),
                    "preset_name": status_payload.get("preset", {}).get("name"),
                    "config_hash": status_payload.get("config", {}).get("config_hash"),
                    "final_time_s": kpi_payload.get("final_time_s"),
                    "final_position_error_m": kpi_payload.get("final_position_error_m"),
                    "final_angle_error_deg": kpi_payload.get("final_angle_error_deg"),
                    "path_completed": kpi_payload.get("path_completed"),
                }
            )

            if len(runs) >= max_runs:
                break

        index_payload = {
            "schema_version": "runs_index_v1",
            "generated_at": datetime.utcnow().isoformat() + "Z",
            "latest_run_id": run_dir.name,
            "run_count": len(runs),
            "runs": runs,
        }
        (base_dir / "runs_index.json").write_text(
            json.dumps(index_payload, indent=2), encoding="utf-8"
        )

    def _artifact_category(self, rel_path: Path) -> str:
        """Classify artifact type for manifest consumers."""
        suffix = rel_path.suffix.lower()
        if suffix in {".mp4", ".gif", ".webm"}:
            return "media"
        if suffix in {".png", ".jpg", ".jpeg", ".svg", ".html"}:
            return "plot"
        if suffix in {".csv"}:
            return "data"
        if suffix in {".json", ".jsonl", ".txt", ".md", ".sha256"}:
            return "metadata"
        return "other"

    def _load_checksums_map(self, checksum_path: Path) -> dict[str, str]:
        """Parse checksums.sha256 into a map for manifest assembly."""
        mapping: dict[str, str] = {}
        if not checksum_path.exists():
            return mapping
        try:
            for line in checksum_path.read_text(encoding="utf-8").splitlines():
                parts = line.split("  ", 1)
                if len(parts) != 2:
                    continue
                digest, rel = parts
                mapping[rel.strip()] = digest.strip()
        except Exception:
            return {}
        return mapping

    def _sha256_file(self, path: Path) -> str:
        """Compute SHA-256 digest for a file path."""
        hasher = hashlib.sha256()
        with path.open("rb") as handle:
            while True:
                chunk = handle.read(1024 * 1024)
                if not chunk:
                    break
                hasher.update(chunk)
        return hasher.hexdigest()

    def _write_json(self, path: Path, payload: dict[str, Any]) -> None:
        """Write JSON atomically via temporary file replacement."""
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        tmp.replace(path)

    def _get_package_version(self) -> str:
        try:
            from importlib.metadata import version

            return str(version("satellite-control"))
        except Exception:
            return "unknown"

    def _detect_solver_backend_version(self, solver_type: str) -> str | None:
        solver = str(solver_type or "").upper()
        if solver == "OSQP":
            try:
                import osqp

                return str(getattr(osqp, "__version__", "unknown"))
            except Exception:
                return None
        return None

    def _get_git_metadata(self) -> dict[str, Any]:
        repo_root = Path(__file__).resolve().parents[3]
        result = {
            "commit": None,
            "branch": None,
            "dirty": None,
        }

        try:
            commit = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
            )
            result["commit"] = commit.stdout.strip()
        except Exception:
            pass

        try:
            branch = subprocess.run(
                ["git", "rev-parse", "--abbrev-ref", "HEAD"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
            )
            result["branch"] = branch.stdout.strip()
        except Exception:
            pass

        try:
            dirty = subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=repo_root,
                check=True,
                capture_output=True,
                text=True,
            )
            result["dirty"] = bool(dirty.stdout.strip())
        except Exception:
            pass

        return result

    def _load_history_from_csv(self) -> np.ndarray | None:
        """
        Load state history from CSV file if not in memory.

        Returns:
            State history array or None if loading fails
        """
        try:
            import pandas as pd

            if self.sim.data_save_path is None:
                return None

            csv_path = self._artifact_path(self.sim.data_save_path, "control_data.csv")
            if csv_path.exists():
                df = pd.read_csv(csv_path)

                # Check if 3D columns exist
                if "Current_Z" in df.columns:
                    # Load 3D data
                    pos = df[["Current_X", "Current_Y", "Current_Z"]].values
                    vel = df[["Current_VX", "Current_VY", "Current_VZ"]].values
                    ang_vel = df[["Current_WX", "Current_WY", "Current_WZ"]].values

                    has_quat_cols = all(
                        col in df.columns
                        for col in (
                            "Current_QW",
                            "Current_QX",
                            "Current_QY",
                            "Current_QZ",
                        )
                    )
                    if not has_quat_cols:
                        logger.debug(
                            "control_data.csv missing quaternion columns; "
                            "skipping CSV history reload."
                        )
                        return None
                    quat_wxyz = df[
                        ["Current_QW", "Current_QX", "Current_QY", "Current_QZ"]
                    ].values.astype(float)
                    q_norm = np.linalg.norm(quat_wxyz, axis=1, keepdims=True)
                    q_norm[q_norm <= 1e-12] = 1.0
                    quat_wxyz = quat_wxyz / q_norm

                    # Construct 13-element state: [pos(3), quat(4), vel(3), ang_vel(3)]
                    # Shape (N, 13)
                    state_history = np.column_stack([pos, quat_wxyz, vel, ang_vel])
                    return state_history
                else:
                    logger.debug(
                        "control_data.csv does not contain required 3D state columns; "
                        "skipping CSV history reload."
                    )
                    return None
        except Exception as e:
            logger.debug(f"Could not load history from CSV: {e}")

        return None

    def save_animation_mp4(self, fig: Any, ani: Any) -> str | None:
        """
        Save the animation as MP4 file.

        Args:
            fig: Matplotlib figure object
            ani: Matplotlib animation object

        Returns:
            Path to saved MP4 file or None if save failed
        """
        self.sim.visualizer.sync_from_controller()
        result: str | None = self.sim.visualizer.save_animation_mp4(fig, ani)
        return result
