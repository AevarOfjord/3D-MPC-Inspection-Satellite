"""
Mission Sequencer
=================

Manages the execution of multi-stage missions defined in YAML configuration.
Acts as a high-level state machine that coordinates the `MissionStateManager`.

Supported Stage Types:
- goto: Navigate to a specific 3D point.
- orbit: Execute a circular orbit around a point (using DXF shape mode backend).
- return: Return to the initial home position.
"""

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np
from omegaconf import DictConfig, OmegaConf

from src.satellite_control.config.mission_state import MissionState

logger = logging.getLogger(__name__)


@dataclass
class MissionStage:
    """Represents a single stage in a mission sequence."""

    type: str
    params: Dict[str, Any]
    status: str = "PENDING"  # PENDING, ACTIVE, COMPLETE


class MissionSequencer:
    """
    Executes a sequence of mission stages.

    updates the MissionState based on the current active stage.
    """

    def __init__(self, mission_state: MissionState):
        self.mission_state = mission_state
        self.stages: List[MissionStage] = []
        self.current_stage_idx: int = -1
        self.active_stage: Optional[MissionStage] = None

        # State tracking
        self.stage_start_time: Optional[float] = None
        self.last_update_time: float = 0.0

    def load_mission(self, mission_config: DictConfig) -> None:
        """
        Load a mission sequence from a configuration object.

        Args:
            mission_config: OmegaConf DictConfig containing 'stages' list.
        """
        logger.info(f"Loading mission: {mission_config.get('name', 'Unnamed')}")
        self.stages = []

        if "stages" not in mission_config:
            logger.warning("No stages found in mission config!")
            return

        for stage_conf in mission_config.stages:
            stage_type = stage_conf.get("type", "unknown")
            # Convert config to dict, excluding 'type'
            params = OmegaConf.to_container(stage_conf, resolve=True)
            if "type" in params:
                del params["type"]

            self.stages.append(MissionStage(type=stage_type, params=params))

        logger.info(f"Loaded {len(self.stages)} stages.")
        self.reset()

    def reset(self) -> None:
        """Reset sequencer to initial state."""
        self.current_stage_idx = -1
        self.active_stage = None
        self.stage_start_time = None
        for stage in self.stages:
            stage.status = "PENDING"

    def start(self) -> None:
        """Start mission execution."""
        if not self.stages:
            logger.warning("Cannot start: No stages loaded.")
            return

        self._advance_stage()

    def update(self, current_time: float, current_state: np.ndarray) -> None:
        """
        Update sequencer logic. Should be called every simulation step.

        Args:
            current_time: Current simulation time (s).
            current_state: Current satellite state vector (13-element).
        """
        if self.active_stage is None:
            return

        self.last_update_time = current_time
        if self.stage_start_time is None:
            self.stage_start_time = current_time

        # Check completion conditions
        if self._check_stage_completion(current_time, current_state):
            logger.info(
                f"Stage {self.current_stage_idx} ({self.active_stage.type}) COMPLETED."
            )
            self.active_stage.status = "COMPLETE"
            self._advance_stage()

    def _advance_stage(self) -> None:
        """Move to the next stage in the sequence."""
        self.current_stage_idx += 1

        if self.current_stage_idx >= len(self.stages):
            logger.info("Mission Sequence COMPLETED.")
            self.active_stage = None
            return

        self.active_stage = self.stages[self.current_stage_idx]
        self.active_stage.status = "ACTIVE"
        self.stage_start_time = None  # Will be set on next update

        logger.info(
            f"Starting Stage {self.current_stage_idx}: {self.active_stage.type}"
        )
        self._apply_stage_config(self.active_stage)

    def _apply_stage_config(self, stage: MissionStage) -> None:
        """
        Apply configuration to MissionState for the new stage.
        """
        # Reset relevant flags
        self.mission_state.enable_waypoint_mode = False
        self.mission_state.dxf_shape_mode_active = False

        if stage.type == "goto":
            self._setup_goto_stage(stage)
        elif stage.type == "orbit":
            self._setup_orbit_stage(stage)
        elif stage.type == "return":
            self._setup_return_stage(stage)
        else:
            logger.warning(f"Unknown stage type: {stage.type}")

    def _setup_goto_stage(self, stage: MissionStage) -> None:
        """Configure Waypoint Mode for 'goto' stage."""
        target = stage.params.get("target", [0, 0, 0])
        hold_time = stage.params.get("hold_time", 5.0)

        # Set up single-point "multi-point" mission
        self.mission_state.enable_multi_point_mode = True
        self.mission_state.waypoint_targets = [tuple(target)]
        self.mission_state.waypoint_angles = [(0.0, 0.0, 0.0)]  # Default orientation
        self.mission_state.current_target_index = 0

        # We misuse 'target_hold_time' via AppConfig usually, but here we might need
        # to rely on the Sequencer to wait.
        # For now, let's just let the mission manager handle reaching it.

    def _setup_orbit_stage(self, stage: MissionStage) -> None:
        """Configure DXF Shape Mode for 'orbit' stage."""
        radius = stage.params.get("radius", 2.0)
        center = np.array(stage.params.get("center", [0, 0, 0]))
        speed = stage.params.get("speed", 0.1)

        # Generate circle path
        num_points = 36
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        path = []
        for theta in angles:
            x = center[0] + radius * np.cos(theta)
            y = center[1] + radius * np.sin(theta)
            z = center[2]
            path.append((x, y, z))

        self.mission_state.dxf_shape_mode_active = True
        self.mission_state.dxf_shape_path = path
        self.mission_state.dxf_target_speed = speed
        self.mission_state.dxf_path_length = 2 * np.pi * radius
        self.mission_state.dxf_shape_phase = "POSITIONING"  # Start by going to start

        # Reset internal DXF state
        self.mission_state.dxf_mission_start_time = None
        self.mission_state.dxf_tracking_start_time = None

    def _setup_return_stage(self, stage: MissionStage) -> None:
        """Configure 'goto' stage for return (0,0,0)."""
        # Ideally return to initial state, often 0,0,0 or defined home
        self.mission_state.enable_multi_point_mode = True
        self.mission_state.waypoint_targets = [(0.0, 0.0, 0.0)]
        self.mission_state.waypoint_angles = [(0.0, 0.0, 0.0)]
        self.mission_state.current_target_index = 0

    def _check_stage_completion(
        self, current_time: float, current_state: np.ndarray
    ) -> bool:
        """Determine if current stage is finished."""
        if self.stage_start_time is None:
            return False

        elapsed = current_time - self.stage_start_time

        if self.active_stage.type == "goto" or self.active_stage.type == "return":
            # Check if MissionStateManager thinks we reached the target
            # Note: MissionStateManager handles "reach + hold" logic internally for multi-point.
            # We can check strict completion:

            # If we are in "COMPLETE" phase of multi-point
            if self.mission_state.multi_point_phase == "COMPLETE":
                return True

            # Or if we just want to impose our own timeout/logic?
            # Let's trust MissionStateManager's "COMPLETE" flag for now.
            # But we need to ensure we reset it when starting the stage.

            # Hack check for now:
            # If current_target_index >= len(targets), we are done.
            targets = self.mission_state.waypoint_targets
            if targets and self.mission_state.current_target_index >= len(targets):
                return True

        elif self.active_stage.type == "orbit":
            # Orbit indefinitely? Or for N loops?
            # For this MVP, let's say run for 'duration' seconds if provided, else 1 loop
            duration = self.active_stage.params.get("duration", None)
            loops = self.active_stage.params.get("loops", 1)

            if duration:
                return elapsed >= duration

            # Check logic for loops (approximate based on path length and speed)
            path_len = self.mission_state.dxf_path_length
            speed = self.mission_state.dxf_target_speed
            if speed > 0 and path_len > 0:
                loop_time = path_len / speed
                # Add some buffer for positioning phase
                if elapsed > (loop_time * loops + 10.0):
                    # This is a weak check. Real logic should track progress.
                    # Ideally check if 'dxf_shape_phase' somehow indicates completion?
                    # The current MissionStateManager loops forever on DXF.
                    return True

        return False
