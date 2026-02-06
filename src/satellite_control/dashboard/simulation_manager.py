"""Simulation manager for dashboard runtime control and websocket telemetry."""

from __future__ import annotations

import asyncio
import json
import logging
from typing import Any, Dict, List, Optional

import numpy as np
from fastapi import WebSocket

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.timing import SIMULATION_DT
from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.mission.runtime_loader import compile_unified_mission_runtime
from src.satellite_control.mission.unified_mission import MissionDefinition, SegmentType
from src.satellite_control.utils.orientation_utils import quat_wxyz_to_euler_xyz

logger = logging.getLogger("dashboard")


class ConnectionManager:
    """Manages WebSocket connections for real-time updates."""

    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
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
        self.sim_instance: Optional[SatelliteMPCLinearizedSimulation] = None
        self.current_unified_mission: Optional[MissionDefinition] = None
        self.simulation_task: Optional[asyncio.Task] = None

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

    def control(self, command: Any) -> Dict[str, Any]:
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

    def _get_telemetry_dict(self) -> Dict[str, Any]:
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

        obstacles = []
        if (
            hasattr(self.sim_instance, "simulation_config")
            and hasattr(self.sim_instance.simulation_config, "mission_state")
            and self.sim_instance.simulation_config.mission_state.obstacles
        ):
            obstacles = [
                {"position": list(o.position), "radius": o.radius}
                for o in self.sim_instance.simulation_config.mission_state.obstacles
            ]

        frame = "ECI"
        frame_origin = None
        if self.current_unified_mission:

            def norm_frame(val: Any) -> str:
                if hasattr(val, "value"):
                    return str(val.value).upper()
                return str(val).upper()

            uses_lvlh = False
            if norm_frame(self.current_unified_mission.start_pose.frame) == "LVLH":
                uses_lvlh = True
            for seg in self.current_unified_mission.segments:
                if seg.type == SegmentType.TRANSFER:
                    if norm_frame(seg.end_pose.frame) == "LVLH":
                        uses_lvlh = True
                        break
                if seg.type == SegmentType.SCAN:
                    if norm_frame(seg.scan.frame) == "LVLH":
                        uses_lvlh = True
                        break

            if uses_lvlh:
                frame = "LVLH"
                origin = None
                start_target_id = getattr(
                    self.current_unified_mission, "start_target_id", None
                )
                if start_target_id:
                    for seg in self.current_unified_mission.segments:
                        if (
                            seg.type == SegmentType.SCAN
                            and seg.target_id == start_target_id
                            and seg.target_pose
                        ):
                            origin = list(seg.target_pose.position)
                            break
                if origin is None:
                    for seg in self.current_unified_mission.segments:
                        if seg.type == SegmentType.SCAN and seg.target_pose:
                            origin = list(seg.target_pose.position)
                            break
                if origin is not None:
                    frame_origin = origin

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
            "obstacles": obstacles,
            "planned_path": getattr(self.sim_instance, "planned_path", []),
            "paused": self.paused,
            "sim_speed": self.simulation_speed,
            "solve_time": getattr(self.sim_instance, "last_solve_time", 0.0),
            "pos_error": getattr(self.sim_instance, "last_pos_error", 0.0),
            "ang_error": getattr(self.sim_instance, "last_ang_error", 0.0),
            "frame": frame,
            "frame_origin": frame_origin,
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
