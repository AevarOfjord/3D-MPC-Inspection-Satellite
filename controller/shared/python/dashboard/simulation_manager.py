"""Simulation manager for dashboard runtime control and websocket telemetry."""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any

import numpy as np
from fastapi import WebSocket

from controller.configs.simulation_config import SimulationConfig
from controller.configs.timing import SIMULATION_DT
from controller.shared.python.mission.runtime_loader import (
    compile_unified_mission_runtime,
)
from controller.shared.python.mission.unified_mission import MissionDefinition
from controller.shared.python.simulation.engine import SatelliteMPCLinearizedSimulation
from controller.shared.python.utils.orientation_utils import quat_wxyz_to_euler_xyz

logger = logging.getLogger("dashboard")


class ConnectionManager:
    """Manages WebSocket connections for real-time updates."""

    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        try:
            self.active_connections.remove(websocket)
        except ValueError:
            pass  # Already disconnected

    async def broadcast(self, message: dict[str, Any]):
        # Convert numpy types to native types for JSON serialization
        json_str = json.dumps(
            message,
            default=lambda x: x.tolist() if isinstance(x, np.ndarray) else str(x),
        )
        for connection in self.active_connections:
            try:
                await connection.send_text(json_str)
            except Exception as e:
                logger.error(f"Error sending message: {e}")
                # Keep connection handling centralized in websocket disconnect flow.


class SimulationManager:
    """
    Encapsulates the simulation state, configuration, and execution loop.
    """

    def __init__(self):
        self.sim_instance: SatelliteMPCLinearizedSimulation | None = None
        self.current_unified_mission: MissionDefinition | None = None
        self.simulation_task: asyncio.Task | None = None

        # State flags
        self.paused: bool = True
        self.simulation_speed: float = 1.0
        self.pending_steps: int = 0

        self.connection_manager = ConnectionManager()

    async def start(self):
        """Start the background simulation loop."""
        if self.simulation_task is None or self.simulation_task.done():
            self.simulation_task = asyncio.create_task(self._run_loop())
            logger.info("Simulation loop started.")

    async def stop(self):
        """Stop the background simulation loop."""
        if self.simulation_task:
            self.simulation_task.cancel()
            try:
                await self.simulation_task
            except asyncio.CancelledError:
                pass
            self.simulation_task = None
            logger.info("Simulation loop stopped.")

    async def update_unified_mission(self, mission: MissionDefinition):
        """Update unified mission config and restart simulation."""
        logger.info("Updating unified mission config (v2).")

        await self.stop()

        self.current_unified_mission = mission
        self.sim_instance = None

        mission_runtime = compile_unified_mission_runtime(
            mission,
            simulation_config=SimulationConfig.create_default(),
        )
        sim_config = mission_runtime.simulation_config
        start_pos = mission_runtime.start_pos
        end_pos = mission_runtime.end_pos

        self.sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=start_pos,
            end_pos=end_pos,
            start_angle=(0.0, 0.0, 0.0),
            end_angle=(0.0, 0.0, 0.0),
        )

        await self.start()
        self.paused = False
        self.pending_steps = 0

    def set_unified_mission(self, mission: MissionDefinition) -> None:
        """Store unified mission definition (v2) without altering simulation yet."""
        self.current_unified_mission = mission

    def control(self, command: Any) -> dict[str, Any]:
        """Handle control commands (pause, resume, step)."""
        action = getattr(command, "action", None)
        if action == "pause":
            self.paused = True
        elif action == "resume":
            self.paused = False
        elif action == "step":
            self.paused = True
            self.pending_steps += max(1, int(getattr(command, "steps", 1)))

        return {
            "status": "success",
            "paused": self.paused,
            "pending_steps": self.pending_steps,
        }

    def set_speed(self, speed: float) -> float:
        """Set simulation speed multiplier."""
        self.simulation_speed = max(0.1, min(speed, 10.0))
        return self.simulation_speed

    async def reset(self):
        """Reset simulation to initial state."""
        await self.stop()
        self.sim_instance = None
        self.current_unified_mission = None
        self.paused = True
        self.simulation_speed = 1.0
        self.pending_steps = 0
        await self.start()

    def _initialize_simulation(self):
        """Initialize the simulation instance based on current config."""
        sim_config = SimulationConfig.create_default()
        self.sim_instance = SatelliteMPCLinearizedSimulation(
            simulation_config=sim_config,
            start_pos=(10.0, 0.0, 0.0),
            end_pos=(0.0, 0.0, 0.0),
            start_angle=(0.0, 0.0, 0.0),
            end_angle=(0.0, 0.0, 0.0),
        )

    def _get_telemetry_dict(self) -> dict[str, Any]:
        """Construct telemetry dictionary from current simulation state."""
        if not self.sim_instance:
            return {}

        state = self.sim_instance.get_current_state()

        # Fallback values if sim_instance properties aren't ready
        ref_state = getattr(self.sim_instance, "reference_state", None)
        ref_pos = (0.0, 0.0, 0.0)
        ref_quat = None
        ref_ori = (0.0, 0.0, 0.0)

        if ref_state is not None and len(ref_state) >= 7:
            ref_pos = tuple(float(v) for v in ref_state[0:3])
            ref_quat = tuple(float(v) for v in ref_state[3:7])
            try:
                ref_ori = tuple(quat_wxyz_to_euler_xyz(ref_quat).tolist())
            except Exception:
                ref_ori = (0.0, 0.0, 0.0)
        num_thrusters = getattr(self.sim_instance.mpc_controller, "num_thrusters", 12)
        last_output = getattr(
            self.sim_instance, "last_control_output", np.zeros(num_thrusters + 3)
        )

        frame = "LVLH"
        frame_origin: list[float] | None = None
        scan_object: dict[str, Any] | None = None
        mission_state = getattr(
            getattr(self.sim_instance, "simulation_config", None), "mission_state", None
        )
        if mission_state is not None:
            frame_raw = str(getattr(mission_state, "path_frame", "LVLH")).upper()
            frame = frame_raw if frame_raw in {"LVLH", "ECI"} else "LVLH"
            origin_raw = getattr(mission_state, "frame_origin", None)
            if origin_raw is not None:
                try:
                    frame_origin = [
                        float(origin_raw[0]),
                        float(origin_raw[1]),
                        float(origin_raw[2]),
                    ]
                except Exception:
                    frame_origin = None
            scan_object_raw = getattr(mission_state, "visualization_scan_object", None)
            if isinstance(scan_object_raw, dict):
                scan_object = dict(scan_object_raw)
        if frame == "LVLH" and frame_origin is None:
            frame_origin = [0.0, 0.0, 0.0]
        elif frame != "LVLH":
            frame_origin = None

        mode_state_obj = getattr(self.sim_instance, "mode_state", None)
        completion_gate_obj = getattr(self.sim_instance, "completion_gate", None)
        solver_health_obj = getattr(self.sim_instance, "solver_health", None)
        pointing_status_obj = getattr(self.sim_instance, "pointing_status", None)

        mode_state = None
        if mode_state_obj is not None:
            mode_state = {
                "current_mode": str(getattr(mode_state_obj, "current_mode", "TRACK")),
                "time_in_mode_s": float(getattr(mode_state_obj, "time_in_mode_s", 0.0)),
            }

        completion_gate = None
        if completion_gate_obj is not None:
            completion_gate = {
                "position_ok": bool(getattr(completion_gate_obj, "position_ok", False)),
                "angle_ok": bool(getattr(completion_gate_obj, "angle_ok", False)),
                "velocity_ok": bool(getattr(completion_gate_obj, "velocity_ok", False)),
                "angular_velocity_ok": bool(
                    getattr(completion_gate_obj, "angular_velocity_ok", False)
                ),
                "hold_elapsed_s": float(
                    getattr(completion_gate_obj, "hold_elapsed_s", 0.0)
                ),
                "hold_required_s": float(
                    getattr(completion_gate_obj, "hold_required_s", 0.0)
                ),
                "last_breach_reason": getattr(
                    completion_gate_obj, "last_breach_reason", None
                ),
                "fail_reason": getattr(completion_gate_obj, "fail_reason", "none"),
                "hold_reset_count": int(
                    getattr(completion_gate_obj, "hold_reset_count", 0)
                ),
            }

        solver_health = None
        if solver_health_obj is not None:
            solver_health = {
                "status": str(getattr(solver_health_obj, "status", "ok")),
                "fallback_count": int(getattr(solver_health_obj, "fallback_count", 0)),
                "hard_limit_breaches": int(
                    getattr(solver_health_obj, "hard_limit_breaches", 0)
                ),
                "fallback_active": bool(
                    getattr(solver_health_obj, "fallback_active", False)
                ),
                "fallback_age_s": float(
                    getattr(solver_health_obj, "fallback_age_s", 0.0)
                ),
                "fallback_scale": float(
                    getattr(solver_health_obj, "fallback_scale", 0.0)
                ),
                "last_fallback_reason": (
                    getattr(solver_health_obj, "last_fallback_reason", None)
                ),
                "fallback_reasons": dict(
                    getattr(solver_health_obj, "fallback_reasons", {}) or {}
                ),
            }

        pointing_status = None
        if isinstance(pointing_status_obj, dict):
            pointing_status = {
                "pointing_context_source": pointing_status_obj.get(
                    "pointing_context_source"
                ),
                "pointing_policy": pointing_status_obj.get("pointing_policy"),
                "pointing_axis_world": list(
                    pointing_status_obj.get("pointing_axis_world", [0.0, 0.0, 1.0])
                    or [0.0, 0.0, 1.0]
                ),
                "z_axis_error_deg": float(
                    pointing_status_obj.get("z_axis_error_deg", 0.0) or 0.0
                ),
                "x_axis_error_deg": float(
                    pointing_status_obj.get("x_axis_error_deg", 0.0) or 0.0
                ),
                "pointing_guardrail_breached": bool(
                    pointing_status_obj.get("pointing_guardrail_breached", False)
                ),
                "object_visible_side": pointing_status_obj.get("object_visible_side"),
                "pointing_guardrail_reason": pointing_status_obj.get(
                    "pointing_guardrail_reason"
                ),
            }

        return {
            "time": self.sim_instance.simulation_time,
            "position": state[0:3],
            "quaternion": state[3:7],
            "velocity": state[7:10],
            "angular_velocity": state[10:13],
            "reference_position": ref_pos,
            "reference_orientation": ref_ori,
            "reference_quaternion": ref_quat,
            "thrusters": last_output[:num_thrusters],
            "rw_torque": last_output[num_thrusters:],
            "planned_path": getattr(self.sim_instance, "planned_path", []),
            "paused": self.paused,
            "sim_speed": self.simulation_speed,
            "solve_time": getattr(self.sim_instance, "last_solve_time", 0.0),
            "pos_error": getattr(self.sim_instance, "last_pos_error", 0.0),
            "ang_error": getattr(self.sim_instance, "last_ang_error", 0.0),
            "controller_core": str(
                getattr(
                    self.sim_instance,
                    "controller_core_mode",
                    getattr(
                        getattr(self.sim_instance, "mpc_controller", None),
                        "controller_core",
                        "v6",
                    ),
                )
            ),
            "controller_profile": str(
                getattr(
                    self.sim_instance,
                    "controller_profile_mode",
                    getattr(
                        getattr(self.sim_instance, "mpc_controller", None),
                        "controller_profile",
                        "hybrid",
                    ),
                )
            ),
            "linearization_mode": str(
                getattr(
                    self.sim_instance,
                    "linearization_mode",
                    getattr(
                        getattr(self.sim_instance, "mpc_controller", None),
                        "linearization_mode",
                        "hybrid_tolerant_stage",
                    ),
                )
            ),
            "scan_object": scan_object,
            "frame": frame,
            "frame_origin": frame_origin,
            "mode_state": mode_state,
            "completion_gate": completion_gate,
            "solver_health": solver_health,
            "pointing_status": pointing_status,
        }

    async def _run_loop(self):
        """Main simulation execution loop."""
        logger.info("Starting internal simulation loop...")

        frame_dt = 0.016

        try:
            while True:
                # 1. Handle Pause / Idle
                if self.paused and self.pending_steps <= 0:
                    await asyncio.sleep(0.1)
                    if self.sim_instance:
                        await self.connection_manager.broadcast(
                            self._get_telemetry_dict()
                        )
                    continue

                # 2. Calculate Steps
                if not self.sim_instance:
                    self._initialize_simulation()

                if self.simulation_speed >= 1.0:
                    steps_per_frame = max(
                        1, int((frame_dt * self.simulation_speed) / SIMULATION_DT)
                    )
                    sleep_time = frame_dt
                else:
                    steps_per_frame = 1
                    sleep_time = frame_dt / max(self.simulation_speed, 0.1)

                # 3. Step Simulation
                if self.paused and self.pending_steps > 0:
                    step_count = max(1, self.pending_steps)
                    self.pending_steps = 0
                    for _ in range(step_count):
                        self.sim_instance.step()
                else:
                    for _ in range(steps_per_frame):
                        self.sim_instance.step()

                # 4. Broadcast State
                telemetry = self._get_telemetry_dict()
                await self.connection_manager.broadcast(telemetry)

                await asyncio.sleep(sleep_time)

        except asyncio.CancelledError:
            logger.info("Simulation loop task cancelled.")
        except Exception as e:
            logger.error(f"Error in simulation loop: {e}", exc_info=True)
