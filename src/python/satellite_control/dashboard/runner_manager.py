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
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger("dashboard.runner")

# Constants
PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
SIMULATION_SCRIPT = SCRIPTS_DIR / "run_simulation.py"
PRESETS_FILE = PROJECT_ROOT / "Data" / "Dashboard" / "runner_presets.json"

class RunnerManager:
    """
    Manages the execution of the simulation command and streams output
    to connected WebSocket clients.
    """
    def __init__(self):
        self.process: Optional[asyncio.subprocess.Process] = None
        self.active_websockets: list[WebSocket] = []
        self._log_history: list[str] = []
        self.max_history_lines = 1000
        self._custom_config: dict | None = None
        self._active_preset_name: str | None = None
        self._temp_config_path: str | None = None
        self._current_run_dir: Path | None = None
        self._presets_path = PRESETS_FILE
        self._presets: dict[str, dict[str, Any]] = self._load_presets()

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
                normalized[name] = {
                    "config": config,
                    "updated_at": str(item.get("updated_at", "")),
                }
            return normalized
        except Exception as exc:
            logger.warning("Failed to load presets from %s: %s", self._presets_path, exc)
            return {}

    def _persist_presets(self) -> None:
        """Persist in-memory presets to disk."""
        self._presets_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": 1,
            "presets": self._presets,
        }
        self._presets_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    @staticmethod
    def _map_legacy_mpc_overrides(legacy_mpc: dict[str, Any]) -> dict[str, Any]:
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
            "Q_smooth",
            "Q_attitude",
            "Q_terminal_pos",
            "Q_terminal_s",
            "q_angular_velocity",
            "r_thrust",
            "r_rw_torque",
            "thrust_l1_weight",
            "thrust_pair_weight",
            "coast_pos_tolerance",
            "coast_vel_tolerance",
            "coast_min_speed",
            "thruster_type",
            "obstacle_margin",
            "enable_collision_avoidance",
            "path_speed",
            "path_speed_min",
            "path_speed_max",
            "progress_taper_distance",
            "progress_slowdown_distance",
            "max_linear_velocity",
            "max_angular_velocity",
            "enable_delta_u_coupling",
            "enable_gyro_jacobian",
            "enable_auto_state_bounds",
        }

        for key, value in legacy_mpc.items():
            if key in direct_fields:
                mapped[key] = value

        weights = legacy_mpc.get("weights")
        if isinstance(weights, dict):
            weight_map = {
                "Q_contour": "Q_contour",
                "Q_progress": "Q_progress",
                "Q_lag": "Q_lag",
                "Q_smooth": "Q_smooth",
                "Q_attitude": "Q_attitude",
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
                "enable_collision_avoidance": "enable_collision_avoidance",
                "obstacle_margin": "obstacle_margin",
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
                "progress_taper_distance": "progress_taper_distance",
                "progress_slowdown_distance": "progress_slowdown_distance",
                "coast_pos_tolerance": "coast_pos_tolerance",
                "coast_vel_tolerance": "coast_vel_tolerance",
                "coast_min_speed": "coast_min_speed",
            }
            for src, dst in path_map.items():
                if src in path_following:
                    mapped[dst] = path_following[src]

        return mapped

    def _normalize_overrides(self, overrides: dict[str, Any]) -> dict[str, Any]:
        """Accept both new AppConfig shape and legacy UI shape."""
        normalized: dict[str, Any] = {}

        for section in ("physics", "mpc", "simulation", "input_file_path"):
            if section in overrides:
                normalized[section] = overrides[section]

        # Legacy UI payload: { control: { mpc: ... }, sim: ... }
        control = overrides.get("control")
        if isinstance(control, dict):
            legacy_mpc = control.get("mpc")
            if isinstance(legacy_mpc, dict):
                mapped_mpc = self._map_legacy_mpc_overrides(legacy_mpc)
                if mapped_mpc:
                    existing_mpc = normalized.get("mpc")
                    if isinstance(existing_mpc, dict):
                        existing_mpc.update(mapped_mpc)
                    else:
                        normalized["mpc"] = mapped_mpc

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

        # Keep control_dt aligned with MPC dt when dt is explicitly overridden.
        mpc_section = normalized.get("mpc")
        if isinstance(mpc_section, dict) and "dt" in mpc_section:
            simulation_section = normalized.get("simulation")
            if isinstance(simulation_section, dict):
                simulation_section.setdefault("control_dt", mpc_section["dt"])
            else:
                normalized["simulation"] = {"control_dt": mpc_section["dt"]}

        return normalized

    def get_config(self) -> dict:
        """Get the current configuration (default + overrides)."""
        from satellite_control.config.simulation_config import SimulationConfig
        
        # Start with default
        config = SimulationConfig.create_default()
        
        # Apply overrides if present
        if self._custom_config:
            config = SimulationConfig.create_with_overrides(
                self._custom_config, base_config=config
            )

        # Preserve legacy UI shape while also exposing full AppConfig sections.
        ui_config = config.to_dict()
        app_config = config.app_config.model_dump()
        ui_config["physics"] = app_config.get("physics", {})
        ui_config["mpc"] = app_config.get("mpc", {})
        ui_config["simulation"] = app_config.get("simulation", {})
        ui_config["input_file_path"] = app_config.get("input_file_path")
        config_json = json.dumps(app_config, sort_keys=True, separators=(",", ":"))
        config_hash = hashlib.sha256(config_json.encode("utf-8")).hexdigest()[:12]
        ui_config["config_meta"] = {
            "config_hash": config_hash,
            "config_version": "app_config_v1",
            "overrides_active": bool(self._custom_config),
            "active_preset_name": self._active_preset_name,
            "generated_at": datetime.now(timezone.utc).isoformat(),
        }
        return ui_config

    def update_config(self, overrides: dict, active_preset_name: str | None = None):
        """Update the custom configuration overrides."""
        normalized = self._normalize_overrides(overrides)
        self._custom_config = normalized if normalized else None
        self._active_preset_name = active_preset_name if normalized else None
        logger.info(
            "Updated custom configuration overrides: sections=%s, preset=%s",
            list(normalized.keys()),
            self._active_preset_name,
        )

    def reset_config(self):
        """Clear custom overrides and revert to default configuration."""
        self._custom_config = None
        self._active_preset_name = None
        logger.info("Reset custom configuration overrides to defaults")

    def list_presets(self) -> dict[str, dict[str, Any]]:
        """List available named presets."""
        return {
            name: {
                "config": data.get("config", {}),
                "updated_at": data.get("updated_at"),
            }
            for name, data in sorted(self._presets.items())
        }

    def get_preset(self, name: str) -> dict[str, Any] | None:
        """Fetch a single preset by name."""
        return self._presets.get(name)

    def save_preset(self, name: str, config: dict[str, Any]) -> dict[str, Any]:
        """Create or update a named preset."""
        trimmed = name.strip()
        if not trimmed:
            raise ValueError("Preset name cannot be empty")
        normalized = self._normalize_overrides(config)
        if not normalized:
            raise ValueError("Preset config is empty or invalid")
        try:
            from satellite_control.config.simulation_config import SimulationConfig

            base = SimulationConfig.create_default()
            resolved = SimulationConfig.create_with_overrides(normalized, base_config=base)
            resolved_config = resolved.app_config.model_dump()
            normalized = {
                section: resolved_config[section]
                for section in ("physics", "mpc", "simulation")
                if isinstance(resolved_config.get(section), dict)
            }
            input_path = resolved_config.get("input_file_path")
            if input_path:
                normalized["input_file_path"] = input_path
        except Exception as exc:
            raise ValueError(str(exc)) from exc
        item = {
            "config": normalized,
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }
        self._presets[trimmed] = item
        self._persist_presets()
        return item

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
        logger.info(f"WebSocket connected. Total clients: {len(self.active_websockets)}")
        
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
            logger.info(f"WebSocket disconnected. Remaining clients: {len(self.active_websockets)}")

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

    async def start_simulation(self, mission_name: Optional[str] = None):
        """Start the simulation process."""
        if self.process and self.process.returncode is None:
            await self._broadcast("\n>>> Simulator is already running.\n")
            return

        self._log_history.clear()
        self._current_run_dir = None
        
        cmd_args = [str(SIMULATION_SCRIPT)]
        resolved_mission_path: str | None = None
        if mission_name:
            try:
                # Resolve mission path
                from satellite_control.mission.repository import (
                    resolve_mission_file,
                )
                mission_path = resolve_mission_file(mission_name, source_priority=("local",))
                cmd_args.extend(["--mission", str(mission_path)])
                resolved_mission_path = str(mission_path)
                await self._broadcast(f">>> Selected mission: {mission_name}\n")
            except Exception as e:
                await self._broadcast(f">>> Error resolving mission '{mission_name}': {e}\n")
                return

        # Inject custom config if present
        if self._custom_config:
            import json
            import tempfile
            
            try:
                # Create a temporary file to store the config overrides
                # We use a named temporary file that persists until we delete it
                # Note: On Windows, opening a temp file twice can be an issue, but we're passing path to subprocess
                with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tmp:
                    json.dump(self._custom_config, tmp)
                    config_path = tmp.name
                
                cmd_args.extend(["--config", config_path])
                self._temp_config_path = config_path # Store to clean up later
                await self._broadcast(f">>> Using custom configuration overrides\n")
            except Exception as e:
                 logger.error(f"Failed to create config file: {e}")
                 await self._broadcast(f">>> Warning: Failed to apply custom config: {e}\n")

        await self._broadcast(f">>> Starting simulation: python {' '.join(cmd_args)}\n")

        try:
            # Setup environment
            # Inherit current env but ensure PYTHONPATH includes src/python
            env = os.environ.copy()
            python_path = env.get("PYTHONPATH", "")
            src_python = str(PROJECT_ROOT / "src" / "python")
            if src_python not in python_path:
                env["PYTHONPATH"] = f"{src_python}:{python_path}" if python_path else src_python
            config_meta = self.get_config().get("config_meta", {})
            env["SATCTRL_RUNNER_CONFIG_HASH"] = str(config_meta.get("config_hash", ""))
            env["SATCTRL_RUNNER_CONFIG_VERSION"] = str(config_meta.get("config_version", ""))
            env["SATCTRL_RUNNER_OVERRIDES_ACTIVE"] = (
                "1" if bool(config_meta.get("overrides_active")) else "0"
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
                limit=10*1024*1024, # 10MB limit per chunk
                env=env,
                cwd=str(PROJECT_ROOT) # Run from project root
            )
            
            await self._broadcast(f">>> Process started with PID: {self.process.pid}\n\n")

            # Background tasks for reading streams
            asyncio.create_task(self._monitor_stream(self.process.stdout, "STDOUT"))
            asyncio.create_task(self._monitor_stream(self.process.stderr, "STDERR"))
            
            # Background task to wait for completion
            asyncio.create_task(self._wait_for_completion())

        except Exception as e:
            logger.error(f"Failed to start simulation: {e}")
            await self._broadcast(f"\n>>> Error starting simulation: {e}\n")
            self.process = None
            if self._temp_config_path and os.path.exists(self._temp_config_path):
                try:
                    os.unlink(self._temp_config_path)
                except:
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
                except asyncio.TimeoutError:
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
             except:
                 pass
        self._temp_config_path = None

    async def _monitor_stream(self, stream: asyncio.StreamReader, stream_name: str):
        """Read lines from a stream and broadcast them."""
        # Note: We need to be careful not to block
        while True:
            # readline() yields bytes ending in \n usually
            line = await stream.readline()
            if line:
                decoded = line.decode('utf-8', errors='replace')
                self._maybe_capture_run_dir(decoded)
                # We broadcast the raw line including newline chars usually, 
                # but let's ensure it handles buffering correctly on frontend.
                await self._broadcast(decoded)
            else:
                break

    async def _wait_for_completion(self):
        """Wait for the process to finish and broadcast the result."""
        if self.process:
            return_code = await self.process.wait()
            self._finalize_run_status_from_process_exit(return_code)
            await self._broadcast(f"\n>>> Simulation finished with return code {return_code}.\n")

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
        status_path = run_dir / "run_status.json"
        if not status_path.exists():
            return

        try:
            payload = json.loads(status_path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}

        status = str(payload.get("status", "running")).lower()
        now_iso = datetime.now(timezone.utc).isoformat()
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

    def _update_global_run_index_files(self, base_dir: Path, latest_run_id: str) -> None:
        """Best-effort refresh of runs_index/latest_run pointers."""
        try:
            (base_dir / "latest_run.txt").write_text(latest_run_id + "\n", encoding="utf-8")
            runs: list[dict[str, Any]] = []
            for candidate in sorted(base_dir.iterdir(), reverse=True):
                if not candidate.is_dir():
                    continue
                status_path = candidate / "run_status.json"
                if not status_path.exists() and not (candidate / "physics_data.csv").exists():
                    continue
                status_payload: dict[str, Any] = {}
                try:
                    if status_path.exists():
                        status_payload = json.loads(status_path.read_text(encoding="utf-8"))
                except Exception:
                    status_payload = {}
                runs.append(
                    {
                        "id": candidate.name,
                        "modified": candidate.stat().st_mtime,
                        "status": status_payload.get("status", "unknown"),
                        "mission_name": status_payload.get("mission", {}).get("name"),
                        "preset_name": status_payload.get("preset", {}).get("name"),
                        "config_hash": status_payload.get("config", {}).get("config_hash"),
                    }
                )
                if len(runs) >= 500:
                    break
            payload = {
                "schema_version": "runs_index_v1",
                "generated_at": datetime.now(timezone.utc).isoformat(),
                "latest_run_id": latest_run_id,
                "run_count": len(runs),
                "runs": runs,
            }
            (base_dir / "runs_index.json").write_text(
                json.dumps(payload, indent=2), encoding="utf-8"
            )
        except Exception:
            return
