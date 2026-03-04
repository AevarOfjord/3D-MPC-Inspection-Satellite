"""
Manager for running background simulation processes and streaming logs.
"""

import asyncio
import hashlib
import json
import logging
import os
import re
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from fastapi import WebSocket

from controller.configs.models import MPCCoreParams, MPCParams
from controller.configs.paths import (
    DASHBOARD_DATA_ROOT,
    PROJECT_ROOT,
    SCRIPTS_DIR,
)
from controller.registry import (
    LEGACY_PROFILE_REWRITE_MAP,
    rewrite_legacy_controller_profile,
)
from controller.shared.python.simulation.artifact_paths import (
    artifact_path,
    resolve_existing_artifact_path,
)

logger = logging.getLogger("dashboard.runner")
_ANSI_ESCAPE_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")

# Constants
SIMULATION_SCRIPT = SCRIPTS_DIR / "run_simulation.py"
PRESETS_FILE = DASHBOARD_DATA_ROOT / "runner_presets.json"
APP_CONFIG_SCHEMA_VERSION = "app_config_v3"
APP_CONFIG_SCHEMA_VERSION_LEGACY = "app_config_v2"
COMPATIBILITY_WINDOW = "current"
DISABLE_CONFIG_MIRRORS_ENV = "SATCTRL_DISABLE_CONFIG_MIRRORS"
REMOVED_MPC_FIELDS = (
    "coast_pos_tolerance",
    "coast_vel_tolerance",
    "coast_min_speed",
    "progress_taper_distance",
    "progress_slowdown_distance",
)
_LEGACY_CONTROLLER_BACKEND_MAP = {
    "v2": "cpp_hybrid_rti_osqp",
    "v1": "cpp_linearized_rti_osqp",
}
_MPC_CORE_FIELDS = frozenset(MPCCoreParams.model_fields.keys())
_MPC_RUNTIME_FIELDS = frozenset(MPCParams.model_fields.keys())


class RunnerManager:
    """
    Manages the execution of the simulation command and streams output
    to connected WebSocket clients.
    """

    def __init__(self):
        self.process: asyncio.subprocess.Process | None = None
        self.active_websockets: list[WebSocket] = []
        self._log_history: list[str] = []
        self.max_history_lines = 1000
        self._custom_config: dict[str, Any] | None = None
        self._active_preset_name: str | None = None
        self._temp_config_path: str | None = None
        self._current_run_dir: Path | None = None
        self._presets_path = PRESETS_FILE
        self._presets: dict[str, dict[str, Any]] = self._load_presets()
        self._removed_mpc_fields_seen: set[str] = set()
        self._legacy_mpc_core_fields_seen: set[str] = set()
        self._legacy_controller_backend_warned = False

    def _track_removed_mpc_fields(self, payload: dict[str, Any] | None) -> None:
        """Track and strip removed MPC fields from a section payload in-place."""
        if not isinstance(payload, dict):
            return
        for field in REMOVED_MPC_FIELDS:
            if field in payload:
                payload.pop(field, None)
                self._removed_mpc_fields_seen.add(field)

    def _load_presets(self) -> dict[str, dict[str, Any]]:
        """Load persisted presets from disk."""
        try:
            if not self._presets_path.exists():
                return {}
            payload = json.loads(self._presets_path.read_text(encoding="utf-8"))
            presets = payload.get("presets") if isinstance(payload, dict) else None
            if not isinstance(presets, dict):
                return {}
            normalized: dict[str, dict[str, Any]] = {}
            for name, item in presets.items():
                if not isinstance(name, str) or not isinstance(item, dict):
                    continue
                config = item.get("config")
                if not isinstance(config, dict):
                    continue
                envelope = self._normalize_to_envelope(config)
                if not envelope:
                    continue
                normalized[name] = {
                    "config": envelope,
                    "updated_at": str(item.get("updated_at", "")),
                }
            return normalized
        except Exception as exc:
            logger.warning(
                "Failed to load presets from %s: %s", self._presets_path, exc
            )
            return {}

    def _persist_presets(self) -> None:
        """Persist in-memory presets to disk."""
        self._presets_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 3,
            "presets": self._presets,
        }
        self._presets_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    def _map_legacy_mpc_overrides(self, legacy_mpc: dict[str, Any]) -> dict[str, Any]:
        """Map legacy UI payload shape (control.mpc.*) to AppConfig.mpc fields."""
        mapped: dict[str, Any] = {}

        direct_fields = {
            "prediction_horizon",
            "control_horizon",
            "dt",
            "solver_time_limit",
            "solver_type",
            "verbose_mpc",
            "Q_contour",
            "Q_progress",
            "progress_reward",
            "Q_lag",
            "Q_lag_default",
            "Q_velocity_align",
            "Q_s_anchor",
            "Q_smooth",
            "Q_attitude",
            "Q_axis_align",
            "Q_terminal_pos",
            "Q_terminal_s",
            "q_angular_velocity",
            "r_thrust",
            "r_rw_torque",
            "thrust_l1_weight",
            "thrust_pair_weight",
            "thruster_type",
            "path_speed",
            "path_speed_min",
            "path_speed_max",
            "max_linear_velocity",
            "max_angular_velocity",
            "enable_delta_u_coupling",
            "enable_gyro_jacobian",
            "enable_auto_state_bounds",
            "enable_thruster_hysteresis",
            "thruster_hysteresis_on",
            "thruster_hysteresis_off",
            "controller_profile",
            "solver_backend",
        }

        for key, value in legacy_mpc.items():
            if key in REMOVED_MPC_FIELDS:
                self._removed_mpc_fields_seen.add(key)
                continue
            if key in direct_fields:
                mapped[key] = value
        legacy_backend = legacy_mpc.get("controller_backend")
        if "controller_profile" not in mapped and isinstance(legacy_backend, str):
            mapped_profile = _LEGACY_CONTROLLER_BACKEND_MAP.get(legacy_backend)
            if mapped_profile is not None:
                mapped["controller_profile"] = mapped_profile
                self._legacy_mpc_core_fields_seen.add("controller_backend")

        weights = legacy_mpc.get("weights")
        if isinstance(weights, dict):
            weight_map = {
                "Q_contour": "Q_contour",
                "Q_progress": "Q_progress",
                "Q_lag": "Q_lag",
                "Q_lag_default": "Q_lag_default",
                "Q_velocity_align": "Q_velocity_align",
                "Q_s_anchor": "Q_s_anchor",
                "Q_smooth": "Q_smooth",
                "Q_attitude": "Q_attitude",
                "Q_axis_align": "Q_axis_align",
                "Q_terminal_pos": "Q_terminal_pos",
                "Q_terminal_s": "Q_terminal_s",
                "angular_velocity": "q_angular_velocity",
                "thrust": "r_thrust",
                "rw_torque": "r_rw_torque",
                "progress_reward": "progress_reward",
                "thrust_l1_weight": "thrust_l1_weight",
                "thrust_pair_weight": "thrust_pair_weight",
            }
            for src, dst in weight_map.items():
                if src in weights:
                    mapped[dst] = weights[src]

        settings = legacy_mpc.get("settings")
        if isinstance(settings, dict):
            settings_map = {
                "dt": "dt",
                "thruster_type": "thruster_type",
                "max_linear_velocity": "max_linear_velocity",
                "max_angular_velocity": "max_angular_velocity",
                "enable_delta_u_coupling": "enable_delta_u_coupling",
                "enable_gyro_jacobian": "enable_gyro_jacobian",
                "enable_auto_state_bounds": "enable_auto_state_bounds",
                "verbose_mpc": "verbose_mpc",
            }
            for src, dst in settings_map.items():
                if src in settings:
                    mapped[dst] = settings[src]

        path_following = legacy_mpc.get("path_following")
        if isinstance(path_following, dict):
            path_map = {
                "path_speed": "path_speed",
                "path_speed_min": "path_speed_min",
                "path_speed_max": "path_speed_max",
                "enable_thruster_hysteresis": "enable_thruster_hysteresis",
                "thruster_hysteresis_on": "thruster_hysteresis_on",
                "thruster_hysteresis_off": "thruster_hysteresis_off",
            }
            for removed in REMOVED_MPC_FIELDS:
                if removed in path_following:
                    self._removed_mpc_fields_seen.add(removed)
            for src, dst in path_map.items():
                if src in path_following:
                    mapped[dst] = path_following[src]

        return mapped

    @staticmethod
    def _clone_section(value: Any) -> dict[str, Any] | None:
        if not isinstance(value, dict):
            return None
        return dict(value)

    @staticmethod
    def _extract_app_config_sections(payload: dict[str, Any] | None) -> dict[str, Any]:
        """
        Extract canonical app_config sections from legacy payloads.

        Canonical sections:
            physics, reference_scheduler, mpc_core, mpc_profile_overrides, actuator_policy,
            controller_contracts, simulation, input_file_path
        """
        if not isinstance(payload, dict):
            return {}

        source = payload
        app_config_candidate = payload.get("app_config")
        if isinstance(app_config_candidate, dict):
            source = app_config_candidate

        sections: dict[str, Any] = {}
        for section in (
            "physics",
            "reference_scheduler",
            "mpc_core",
            "mpc_profile_overrides",
            "actuator_policy",
            "controller_contracts",
            "simulation",
        ):
            section_payload = RunnerManager._clone_section(source.get(section))
            if section_payload is not None:
                sections[section] = section_payload

        # Legacy compatibility: map `mpc` to canonical `mpc_core`.
        if "mpc_core" not in sections:
            mpc_payload = RunnerManager._clone_section(source.get("mpc"))
            if mpc_payload is not None:
                sections["mpc_core"] = mpc_payload

        if "input_file_path" in source:
            sections["input_file_path"] = source.get("input_file_path")
        return sections

    @staticmethod
    def _actuator_policy_from_mpc_core(mpc_core: dict[str, Any]) -> dict[str, Any]:
        actuator: dict[str, Any] = {}
        for key in (
            "enable_thruster_hysteresis",
            "thruster_hysteresis_on",
            "thruster_hysteresis_off",
            "terminal_bypass_band_m",
        ):
            if key in mpc_core:
                actuator[key] = mpc_core[key]
        return actuator

    @staticmethod
    def _split_mpc_sections(sections: dict[str, Any]) -> None:
        """
        Split merged legacy mpc_core payload into canonical mpc + mpc_core sections.

        Legacy clients may send runtime MPC knobs under mpc_core. Canonical schema
        now forbids extra fields in mpc_core, so move runtime fields into mpc.
        """
        mpc_core = sections.get("mpc_core")
        if not isinstance(mpc_core, dict):
            return

        mpc_runtime = sections.get("mpc")
        if not isinstance(mpc_runtime, dict):
            mpc_runtime = {}
            sections["mpc"] = mpc_runtime

        for key in list(mpc_core.keys()):
            if key in _MPC_RUNTIME_FIELDS and key not in _MPC_CORE_FIELDS:
                mpc_runtime[key] = mpc_core.pop(key)

    def _extract_runtime_overrides(
        self,
        payload: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """
        Extract section dicts supported by SimulationConfig.create_with_overrides.

        Runtime keeps `mpc` as execution config while preserving metadata sections.
        """
        sections = self._normalize_overrides_to_sections(payload or {})
        self._split_mpc_sections(sections)
        runtime: dict[str, Any] = {}
        for section in (
            "physics",
            "mpc",
            "reference_scheduler",
            "mpc_core",
            "mpc_profile_overrides",
            "actuator_policy",
            "controller_contracts",
            "simulation",
        ):
            value = sections.get(section)
            if isinstance(value, dict):
                runtime[section] = value

        if "mpc" not in runtime:
            mpc_core = sections.get("mpc_core")
            if isinstance(mpc_core, dict):
                runtime["mpc"] = dict(mpc_core)

        actuator_policy = sections.get("actuator_policy")
        if isinstance(actuator_policy, dict):
            runtime.setdefault("mpc", {}).update(
                {
                    key: actuator_policy[key]
                    for key in (
                        "enable_thruster_hysteresis",
                        "thruster_hysteresis_on",
                        "thruster_hysteresis_off",
                    )
                    if key in actuator_policy
                }
            )

        if "input_file_path" in sections:
            runtime["input_file_path"] = sections.get("input_file_path")
        return runtime

    @staticmethod
    def _with_response_mirrors(payload: dict[str, Any]) -> dict[str, Any]:
        """Mirror canonical app_config sections at top-level for compatibility."""
        mirrors_disabled = str(
            os.environ.get(DISABLE_CONFIG_MIRRORS_ENV, "")
        ).strip().lower() in {"1", "true", "yes", "on"}
        if mirrors_disabled:
            return dict(payload)
        mirrored = dict(payload)
        app_config = payload.get("app_config")
        app_cfg = app_config if isinstance(app_config, dict) else {}
        mpc_core = app_cfg.get("mpc_core")
        if not isinstance(mpc_core, dict):
            mpc_core = app_cfg.get("mpc", {})
        mirrored["physics"] = app_cfg.get("physics", {})
        mirrored["mpc"] = mpc_core
        mirrored["mpc_profile_overrides"] = app_cfg.get("mpc_profile_overrides", {})
        mirrored["simulation"] = app_cfg.get("simulation", {})
        mirrored["input_file_path"] = app_cfg.get("input_file_path")
        return mirrored

    def _normalize_overrides_to_sections(
        self, overrides: dict[str, Any]
    ) -> dict[str, Any]:
        """Accept legacy, v1-flat, v2, and v3 payloads and return canonical sections."""
        normalized = self._extract_app_config_sections(overrides)

        # Legacy UI payload: { control: { mpc: ... }, sim: ... }
        control = overrides.get("control") if isinstance(overrides, dict) else None
        if isinstance(control, dict):
            legacy_mpc = control.get("mpc")
            if isinstance(legacy_mpc, dict):
                mapped_mpc = self._map_legacy_mpc_overrides(legacy_mpc)
                if mapped_mpc:
                    existing_mpc_core = normalized.get("mpc_core")
                    if isinstance(existing_mpc_core, dict):
                        existing_mpc_core.update(mapped_mpc)
                    else:
                        normalized["mpc_core"] = mapped_mpc

        legacy_sim = overrides.get("sim")
        if isinstance(legacy_sim, dict):
            sim_updates: dict[str, Any] = {}
            if "duration" in legacy_sim:
                sim_updates["max_duration"] = legacy_sim["duration"]
            if "dt" in legacy_sim:
                sim_updates["dt"] = legacy_sim["dt"]
            if "control_dt" in legacy_sim:
                sim_updates["control_dt"] = legacy_sim["control_dt"]
            if sim_updates:
                existing_sim = normalized.get("simulation")
                if isinstance(existing_sim, dict):
                    existing_sim.update(sim_updates)
                else:
                    normalized["simulation"] = sim_updates

        mpc_core_section = normalized.get("mpc_core")
        if isinstance(mpc_core_section, dict) and "dt" in mpc_core_section:
            simulation_section = normalized.get("simulation")
            if isinstance(simulation_section, dict):
                simulation_section.setdefault("control_dt", mpc_core_section["dt"])
            else:
                normalized["simulation"] = {"control_dt": mpc_core_section["dt"]}
        if isinstance(mpc_core_section, dict):
            self._migrate_legacy_mpc_core_section(mpc_core_section)
            profile_raw = mpc_core_section.get("controller_profile")
            rewritten = rewrite_legacy_controller_profile(profile_raw)
            if rewritten != profile_raw:
                mpc_core_section["controller_profile"] = rewritten
            self._track_removed_mpc_fields(mpc_core_section)

        profile_overrides = normalized.get("mpc_profile_overrides")
        if isinstance(profile_overrides, dict):
            for old_key, new_key in LEGACY_PROFILE_REWRITE_MAP.items():
                if old_key in profile_overrides and new_key not in profile_overrides:
                    profile_overrides[new_key] = profile_overrides.pop(old_key)

        if "actuator_policy" not in normalized and isinstance(mpc_core_section, dict):
            inferred_actuator = self._actuator_policy_from_mpc_core(mpc_core_section)
            if inferred_actuator:
                normalized["actuator_policy"] = inferred_actuator

        return normalized

    def _migrate_legacy_mpc_core_section(self, mpc_core: dict[str, Any]) -> None:
        """
        One-time migration for deprecated mpc_core.controller_backend.

        Mapping:
          v2 -> hybrid
          v1 -> linear
        """
        if "controller_backend" not in mpc_core:
            return

        backend = str(mpc_core.get("controller_backend"))
        mapped_profile = _LEGACY_CONTROLLER_BACKEND_MAP.get(backend)
        if mapped_profile is not None and (
            "controller_profile" not in mpc_core or not mpc_core["controller_profile"]
        ):
            mpc_core["controller_profile"] = mapped_profile

        mpc_core.pop("controller_backend", None)
        self._legacy_mpc_core_fields_seen.add("controller_backend")
        if not self._legacy_controller_backend_warned:
            logger.warning(
                "Deprecated mpc_core.controller_backend encountered; auto-migrated "
                "to mpc_core.controller_profile."
            )
            self._legacy_controller_backend_warned = True

    @staticmethod
    def _build_app_config_payload(
        app_config_payload: dict[str, Any],
    ) -> dict[str, Any]:
        app_config: dict[str, Any] = {}
        for section in (
            "physics",
            "reference_scheduler",
            "mpc_profile_overrides",
            "actuator_policy",
            "controller_contracts",
            "simulation",
        ):
            value = app_config_payload.get(section)
            if isinstance(value, dict):
                app_config[section] = dict(value)

        # Canonical v3 mpc_core payload is derived from controller.shared.python.runtime.`mpc` so legacy and
        # current UI consumers receive a complete MPC editable section.
        mpc_runtime = app_config_payload.get("mpc")
        mpc_core_profile = app_config_payload.get("mpc_core")
        mpc_core_payload: dict[str, Any] = {}
        if isinstance(mpc_runtime, dict):
            mpc_core_payload.update(dict(mpc_runtime))
        if isinstance(mpc_core_profile, dict):
            for key, value in mpc_core_profile.items():
                if key not in mpc_core_payload:
                    mpc_core_payload[key] = value
        mpc_core_payload.pop("controller_backend", None)
        if mpc_core_payload:
            app_config["mpc_core"] = mpc_core_payload

        if "actuator_policy" not in app_config and isinstance(
            app_config.get("mpc_core"), dict
        ):
            inferred = RunnerManager._actuator_policy_from_mpc_core(
                app_config["mpc_core"]
            )
            if inferred:
                app_config["actuator_policy"] = inferred

        if "input_file_path" in app_config_payload:
            app_config["input_file_path"] = app_config_payload.get("input_file_path")
        return app_config

    def _normalize_to_envelope(self, overrides: dict[str, Any]) -> dict[str, Any]:
        """Convert accepted payload shapes into canonical app_config envelope."""
        sections = self._normalize_overrides_to_sections(overrides)
        app_config: dict[str, Any] = {}
        for section in (
            "physics",
            "reference_scheduler",
            "mpc_core",
            "mpc_profile_overrides",
            "actuator_policy",
            "controller_contracts",
            "simulation",
        ):
            value = sections.get(section)
            if isinstance(value, dict):
                app_config[section] = dict(value)
        if "input_file_path" in sections:
            app_config["input_file_path"] = sections.get("input_file_path")

        if not app_config:
            return {}

        return {
            "schema_version": APP_CONFIG_SCHEMA_VERSION,
            "app_config": app_config,
        }

    def get_config(self) -> dict:
        """Get the current configuration (default + overrides)."""
        from controller.configs.models import MPCParams
        from controller.configs.simulation_config import SimulationConfig

        # Start with default
        config = SimulationConfig.create_default()

        # Apply overrides if present
        if self._custom_config:
            runtime_overrides = self._extract_runtime_overrides(self._custom_config)
            if runtime_overrides:
                config = SimulationConfig.create_with_overrides(
                    runtime_overrides,
                    base_config=config,
                )

        # Preserve legacy UI shape while also exposing full AppConfig sections.
        ui_config = config.to_dict()
        app_config_payload = self._build_app_config_payload(
            config.app_config.model_dump()
        )
        v2_payload = {
            "schema_version": APP_CONFIG_SCHEMA_VERSION,
            "app_config": app_config_payload,
        }
        ui_config.update(self._with_response_mirrors(v2_payload))
        config_json = json.dumps(
            app_config_payload, sort_keys=True, separators=(",", ":")
        )
        config_hash = hashlib.sha256(config_json.encode("utf-8")).hexdigest()[:12]
        ui_config["config_meta"] = {
            "config_hash": config_hash,
            "config_version": APP_CONFIG_SCHEMA_VERSION,
            "overrides_active": bool(self._custom_config),
            "active_preset_name": self._active_preset_name,
            "response_mirrors_enabled": (
                str(os.environ.get(DISABLE_CONFIG_MIRRORS_ENV, "")).strip().lower()
                not in {"1", "true", "yes", "on"}
            ),
            "compatibility_window": COMPATIBILITY_WINDOW,
            "deprecations": {
                "legacy_payload_dual_read": True,
                "response_mirrors": True,
                "removed_mpc_fields_seen": sorted(self._removed_mpc_fields_seen),
                "legacy_mpc_core_fields_seen": sorted(
                    self._legacy_mpc_core_fields_seen
                ),
                "removed_mpc_fields_policy": "warn_ignore",
                "removed_mpc_fields_sunset": "next_major",
                "sunset_note": (
                    "Legacy payload shapes and top-level mirrors are transitional and "
                    f"scheduled for removal after {COMPATIBILITY_WINDOW}."
                ),
            },
            "generated_at": datetime.now(UTC).isoformat(),
        }
        ui_config["mpc_parameter_groups"] = MPCParams.parameter_groups()
        return ui_config

    def update_config(self, overrides: dict, active_preset_name: str | None = None):
        """Update the custom configuration overrides."""
        normalized = self._normalize_to_envelope(overrides)
        self._custom_config = normalized if normalized else None
        self._active_preset_name = active_preset_name if self._custom_config else None
        sections = list(normalized.get("app_config", {}).keys()) if normalized else []
        logger.info(
            "Updated custom configuration overrides: sections=%s, preset=%s",
            sections,
            self._active_preset_name,
        )

    def reset_config(self):
        """Clear custom overrides and revert to default configuration."""
        self._custom_config = None
        self._active_preset_name = None
        logger.info("Reset custom configuration overrides to defaults")

    def list_presets(self) -> dict[str, dict[str, Any]]:
        """List available named presets."""
        listed: dict[str, dict[str, Any]] = {}
        for name, data in sorted(self._presets.items()):
            config_payload = data.get("config")
            if not isinstance(config_payload, dict):
                continue
            envelope = self._normalize_to_envelope(config_payload)
            if not envelope:
                continue
            listed[name] = {
                "config": self._with_response_mirrors(envelope),
                "updated_at": data.get("updated_at"),
            }
        return listed

    def get_preset(self, name: str) -> dict[str, Any] | None:
        """Fetch a single preset by name."""
        return self._presets.get(name)

    def save_preset(self, name: str, config: dict[str, Any]) -> dict[str, Any]:
        """Create or update a named preset."""
        trimmed = name.strip()
        if not trimmed:
            raise ValueError("Preset name cannot be empty")
        normalized = self._normalize_to_envelope(config)
        if not normalized:
            raise ValueError("Preset config is empty or invalid")
        try:
            from controller.configs.simulation_config import SimulationConfig

            base = SimulationConfig.create_default()
            runtime_overrides = self._extract_runtime_overrides(normalized)
            resolved = SimulationConfig.create_with_overrides(
                runtime_overrides,
                base_config=base,
            )
            resolved_config = self._build_app_config_payload(
                resolved.app_config.model_dump()
            )
            normalized = {
                "schema_version": APP_CONFIG_SCHEMA_VERSION,
                "app_config": resolved_config,
            }
        except Exception as exc:
            raise ValueError(str(exc)) from exc
        item = {
            "config": normalized,
            "updated_at": datetime.now(UTC).isoformat(),
        }
        self._presets[trimmed] = item
        self._persist_presets()
        return {
            "config": self._with_response_mirrors(normalized),
            "updated_at": item["updated_at"],
        }

    def delete_preset(self, name: str) -> bool:
        """Delete a named preset."""
        if name in self._presets:
            del self._presets[name]
            self._persist_presets()
            return True
        return False

    def clear_presets(self) -> None:
        """Delete all saved presets."""
        self._presets.clear()
        self._persist_presets()

    def apply_preset(self, name: str) -> dict[str, Any]:
        """Apply a preset as active run overrides."""
        preset = self.get_preset(name)
        if not preset:
            raise KeyError(name)
        config = preset.get("config")
        if not isinstance(config, dict):
            raise ValueError("Preset config is invalid")
        self.update_config(config, active_preset_name=name)
        return self.get_config()

    async def connect(self, websocket: WebSocket):
        """Accept a new WebSocket connection and send history."""
        await websocket.accept()
        self.active_websockets.append(websocket)
        logger.info(
            f"WebSocket connected. Total clients: {len(self.active_websockets)}"
        )

        # Send history upon connection
        if self._log_history:
            try:
                history_text = "".join(self._log_history)
                await websocket.send_text(history_text)
            except Exception as e:
                logger.error(f"Error sending history to websocket: {e}")

    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection."""
        if websocket in self.active_websockets:
            self.active_websockets.remove(websocket)
            logger.info(
                f"WebSocket disconnected. Remaining clients: {len(self.active_websockets)}"
            )

    async def _broadcast(self, message: str):
        """Send a message to all connected clients."""
        # Add to history
        self._log_history.append(message)
        if len(self._log_history) > self.max_history_lines:
            self._log_history.pop(0)

        to_remove = []
        for connection in self.active_websockets:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.warning(f"Error sending to websocket, removing client: {e}")
                to_remove.append(connection)

        for conn in to_remove:
            self.disconnect(conn)

    async def start_simulation(self, mission_name: str | None = None):
        """Start the simulation process."""
        if self.process and self.process.returncode is None:
            await self._broadcast("\n>>> Simulator is already running.\n")
            return

        self._log_history.clear()
        self._current_run_dir = None

        cmd_args = [str(SIMULATION_SCRIPT), "run"]
        resolved_mission_path: str | None = None
        if mission_name:
            try:
                # Resolve mission path
                from controller.shared.python.mission.repository import (
                    resolve_mission_file,
                )

                mission_path = resolve_mission_file(
                    mission_name, source_priority=("local",)
                )
                cmd_args.extend(["--mission", str(mission_path)])
                resolved_mission_path = str(mission_path)
                await self._broadcast(f">>> Selected mission: {mission_name}\n")
            except Exception as e:
                await self._broadcast(
                    f">>> Error resolving mission '{mission_name}': {e}\n"
                )
                return

        # Inject custom config if present
        if self._custom_config:
            import json
            import tempfile

            try:
                runtime_overrides = self._extract_runtime_overrides(self._custom_config)
                if not runtime_overrides:
                    runtime_overrides = {}
                # Create a temporary file to store the config overrides
                # We use a named temporary file that persists until we delete it
                # Note: On Windows, opening a temp file twice can be an issue, but we're passing path to subprocess
                with tempfile.NamedTemporaryFile(
                    mode="w", suffix=".json", delete=False
                ) as tmp:
                    json.dump(runtime_overrides, tmp)
                    config_path = tmp.name

                cmd_args.extend(["--config", config_path])
                self._temp_config_path = config_path  # Store to clean up later
                await self._broadcast(">>> Using custom configuration overrides\n")
            except Exception as e:
                logger.error(f"Failed to create config file: {e}")
                await self._broadcast(
                    f">>> Warning: Failed to apply custom config: {e}\n"
                )

        await self._broadcast(f">>> Starting simulation: python {' '.join(cmd_args)}\n")

        try:
            # Setup environment
            # Inherit current env but ensure PYTHONPATH includes project root.
            env = os.environ.copy()
            python_path = env.get("PYTHONPATH", "")
            project_root = str(PROJECT_ROOT)
            if project_root not in python_path:
                env["PYTHONPATH"] = (
                    f"{project_root}:{python_path}" if python_path else project_root
                )
            # Ensure terminal-color escape codes are disabled in streamed runner logs.
            env["NO_COLOR"] = "1"
            config_meta = self.get_config().get("config_meta", {})
            env["SATCTRL_RUNNER_CONFIG_HASH"] = str(config_meta.get("config_hash", ""))
            env["SATCTRL_RUNNER_CONFIG_VERSION"] = str(
                config_meta.get("config_version", "")
            )
            env["SATCTRL_RUNNER_OVERRIDES_ACTIVE"] = (
                "1" if bool(config_meta.get("overrides_active")) else "0"
            )
            env["SATCTRL_RUNNER_RESPONSE_MIRRORS_ENABLED"] = (
                "1" if bool(config_meta.get("response_mirrors_enabled", True)) else "0"
            )
            if self._active_preset_name:
                env["SATCTRL_RUNNER_PRESET_NAME"] = self._active_preset_name
            if mission_name:
                env["SATCTRL_RUNNER_MISSION_NAME"] = mission_name
            if resolved_mission_path:
                env["SATCTRL_RUNNER_MISSION_PATH"] = resolved_mission_path

            # Use sys.executable to ensure we use the same virtualenv python
            cmd = [sys.executable] + cmd_args

            # Start process
            self.process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                limit=10 * 1024 * 1024,  # 10MB limit per chunk
                env=env,
                cwd=str(PROJECT_ROOT),  # Run from project root
            )

            await self._broadcast(
                f">>> Process started with PID: {self.process.pid}\n\n"
            )

            asyncio.create_task(self._monitor_stream(self.process.stdout))
            asyncio.create_task(self._monitor_stream(self.process.stderr))

            # Background task to wait for completion
            asyncio.create_task(self._wait_for_completion())

        except Exception as e:
            logger.error(f"Failed to start simulation: {e}")
            await self._broadcast(f"\n>>> Error starting simulation: {e}\n")
            self.process = None
            if self._temp_config_path and os.path.exists(self._temp_config_path):
                try:
                    os.unlink(self._temp_config_path)
                except OSError:
                    pass
            self._temp_config_path = None

    async def stop_simulation(self):
        """Stop the currently running simulation."""
        if self.process and self.process.returncode is None:
            await self._broadcast("\n>>> Stopping simulation...\n")
            try:
                self.process.terminate()
                # Wait briefly
                try:
                    await asyncio.wait_for(self.process.wait(), timeout=3.0)
                except TimeoutError:
                    logger.warning("Process did not terminate gracefully, killing it.")
                    self.process.kill()
                    await self.process.wait()

                await self._broadcast(">>> Simulation stopped by user.\n")
            except Exception as e:
                logger.error(f"Error stopping process: {e}")
                await self._broadcast(f">>> Error stopping process: {e}\n")
        else:
            await self._broadcast("\n>>> No simulation is running to stop.\n")

        # Cleanup temp file if it exists
        if self._temp_config_path and os.path.exists(self._temp_config_path):
            try:
                os.unlink(self._temp_config_path)
            except OSError:
                pass
        self._temp_config_path = None

    async def _monitor_stream(self, stream: asyncio.StreamReader, stream_name: str):
        """Read lines from a stream and broadcast them."""
        # Note: We need to be careful not to block
        while True:
            # readline() yields bytes ending in \n usually
            line = await stream.readline()
            if line:
                decoded = line.decode("utf-8", errors="replace")
                clean = _ANSI_ESCAPE_RE.sub("", decoded)
                self._maybe_capture_run_dir(clean)
                # We broadcast the raw line including newline chars usually,
                # but let's ensure it handles buffering correctly on frontend.
                await self._broadcast(clean)
            else:
                break

    async def _wait_for_completion(self):
        """Wait for the process to finish and broadcast the result."""
        if self.process:
            return_code = await self.process.wait()
            self._finalize_run_status_from_process_exit(return_code)
            await self._broadcast(
                f"\n>>> Simulation finished with return code {return_code}.\n"
            )

    def _maybe_capture_run_dir(self, line: str) -> None:
        """Extract run directory from process logs when available."""
        if self._current_run_dir is not None:
            return
        match = re.search(r"Created data directory:\s*(.+?)\s*$", line.strip())
        if not match:
            return
        raw_path = match.group(1).strip().strip("'\"")
        candidate = Path(raw_path)
        if not candidate.is_absolute():
            candidate = (PROJECT_ROOT / candidate).resolve()
        else:
            candidate = candidate.resolve()
        if candidate.exists() and candidate.is_dir():
            self._current_run_dir = candidate

    def _finalize_run_status_from_process_exit(self, return_code: int) -> None:
        """
        Ensure run_status reflects subprocess exit state when process exits early.

        Normal successful runs update run_status in simulation code. This path
        only patches status when we detect non-zero exits or stale in-progress
        statuses.
        """
        run_dir = self._current_run_dir
        if run_dir is None:
            return
        status_path = resolve_existing_artifact_path(
            run_dir, "run_status.json"
        ) or artifact_path(run_dir, "run_status.json")
        if not status_path.exists():
            return

        try:
            payload = json.loads(status_path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}

        status = str(payload.get("status", "running")).lower()
        now_iso = datetime.now(UTC).isoformat()
        updated = False

        if return_code != 0:
            payload["status"] = "failed"
            payload["status_detail"] = (
                f"Simulation process exited with non-zero return code {return_code}"
            )
            payload["success"] = False
            payload["updated_at"] = now_iso
            payload.setdefault("completed_at", now_iso)
            updated = True
        elif status in {"running", "initializing"}:
            payload["status"] = "completed"
            payload["status_detail"] = "Simulation process exited with return code 0"
            payload["success"] = True
            payload["updated_at"] = now_iso
            payload.setdefault("completed_at", now_iso)
            updated = True

        if updated:
            status_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
            self._update_global_run_index_files(run_dir.parent, run_dir.name)

    def _update_global_run_index_files(
        self, base_dir: Path, latest_run_id: str
    ) -> None:
        """Best-effort refresh of runs_index/latest_run pointers."""
        try:
            (base_dir / "latest_run.txt").write_text(
                latest_run_id + "\n", encoding="utf-8"
            )
            runs: list[dict[str, Any]] = []
            for candidate in sorted(base_dir.iterdir(), reverse=True):
                if not candidate.is_dir():
                    continue
                status_path = resolve_existing_artifact_path(
                    candidate, "run_status.json"
                ) or artifact_path(candidate, "run_status.json")
                if (
                    not status_path.exists()
                    and resolve_existing_artifact_path(candidate, "physics_data.csv")
                    is None
                ):
                    continue
                status_payload: dict[str, Any] = {}
                try:
                    if status_path.exists():
                        status_payload = json.loads(
                            status_path.read_text(encoding="utf-8")
                        )
                except Exception:
                    status_payload = {}
                runs.append(
                    {
                        "id": candidate.name,
                        "modified": candidate.stat().st_mtime,
                        "status": status_payload.get("status", "unknown"),
                        "mission_name": status_payload.get("mission", {}).get("name"),
                        "preset_name": status_payload.get("preset", {}).get("name"),
                        "config_hash": status_payload.get("config", {}).get(
                            "config_hash"
                        ),
                    }
                )
                if len(runs) >= 500:
                    break
            payload = {
                "schema_version": "runs_index_v1",
                "generated_at": datetime.now(UTC).isoformat(),
                "latest_run_id": latest_run_id,
                "run_count": len(runs),
                "runs": runs,
            }
            (base_dir / "runs_index.json").write_text(
                json.dumps(payload, indent=2), encoding="utf-8"
            )
        except Exception:
            return
