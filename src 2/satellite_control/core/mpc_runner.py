"""
MPC Runner Module

Encapsulates the high-level control loop logic:
1. State measurement (with noise)
2. MPC solver invocation
3. Control action processing (handling limits, fallbacks)
"""

import logging
import time
from typing import Any, Dict, Optional, Tuple

import numpy as np

from typing import TYPE_CHECKING

from src.satellite_control.control.mpc_controller import MPCController

if TYPE_CHECKING:
    from src.satellite_control.config.models import AppConfig

logger = logging.getLogger(__name__)


class MPCRunner:
    """
    Executes the MPC control strategy.

    Decoupled from simulation physics loop. Manages its own state including
    previous thruster commands and command history for warm-start.
    """

    def __init__(
        self,
        mpc_controller: MPCController,
        config: Optional["AppConfig"] = None,
        state_validator=None,
    ):
        """
        Initialize runner with configuration.

        Args:
            mpc_controller: Initialized MPC Controller instance (MPCController)
            config: Optional AppConfig (v3.0.0). Not currently used but kept for future use.
            state_validator: Optional validator for sensor noise
        """
        self.mpc: MPCController = mpc_controller
        self.config = config  # Stored for potential future use
        self.state_validator = state_validator

        # Internal state management
        self.previous_thrusters = np.zeros(12, dtype=np.float64)
        self.command_history: list = []
        self.call_count = 0

    def compute_control_action(
        self,
        true_state: np.ndarray,
        target_state: np.ndarray,
        previous_thrusters: np.ndarray,
        target_trajectory: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, Dict[str, Any], float, float]:
        """
        Compute the next control action based on current state.

        Returns:
            Tuple of:
            - thruster_action (np.ndarray): The computed command [8,]
            - mpc_info (dict): Solver metadata
            - mpc_computation_time (float): Time taken in seconds
            - command_sent_time (float): Timestamp when command was finalized
        """

        # 1. Apply Sensor Noise (Simulate OptiTrack/IMU imperfections)
        if self.state_validator:
            # Note: validator logic expects lists or arrays?
            # Simulation.get_noisy_state does:
            # return self.state_validator.apply_sensor_noise(true_state)
            measured_state = self.state_validator.apply_sensor_noise(true_state)
        else:
            measured_state = true_state

        # 2. Run Controller
        start_compute_time = time.time()
        mpc_info: Dict[str, Any] = {}

        try:
            thruster_action, mpc_info = self.mpc.get_control_action(
                x_current=measured_state,
                x_target=target_state,
                previous_thrusters=previous_thrusters,
                x_target_trajectory=target_trajectory,
            )
        except TypeError:
            # Fallback for controllers that don't accept trajectory
            # yet
            logger.warning("Controller does not support trajectory! " "Falling back to setpoint.")
            thruster_action, mpc_info = self.mpc.get_control_action(
                measured_state, target_state, previous_thrusters
            )

        end_compute_time = time.time()
        mpc_computation_time = end_compute_time - start_compute_time
        command_sent_time = end_compute_time

        # 3. Post-process Action
        # 3. Post-process Action
        if thruster_action is not None:
            # Ensure shape is 1D
            if thruster_action.ndim == 2:
                thruster_action = thruster_action[0, :]

            # Get expected size from controller if available
            expected_size = getattr(self.mpc, "nu", 12)

            # Validate size if possible
            if len(thruster_action) != expected_size:
                logger.warning(
                    f"Control action size mismatch! Got {len(thruster_action)}, "
                    f"expected {expected_size}. Using as is."
                )

            # Only clip if we are in pure thruster mode (legacy)
            # If we are in RW mode, values can be negative (torque)
            # We assume the controller handles its own clipping for mixed modes
            # Legacy MPCController implies nu=12 implies all thrusters [0,1]
            if expected_size == 12:
                thruster_action = np.clip(thruster_action, 0.0, 1.0).astype(np.float64)
            else:
                # Trust the controller's output (already clipped in get_control_action)
                thruster_action = thruster_action.astype(np.float64)

        else:
            # Fallback if controller failed completely
            expected_size = getattr(self.mpc, "nu", 12)
            thruster_action = np.zeros(expected_size, dtype=np.float64)
            logger.error("Controller returned None! Defaulting to zero control.")

        # Update internal state (resize previous_thrusters if needed)
        if len(self.previous_thrusters) != len(thruster_action):
            self.previous_thrusters = np.zeros_like(thruster_action)

        self.previous_thrusters = thruster_action.copy()
        self.command_history.append(thruster_action.copy())
        self.call_count += 1

        return (
            thruster_action,
            mpc_info,
            mpc_computation_time,
            command_sent_time,
        )

    def reset(self) -> None:
        """Reset runner state for a new simulation run."""
        self.previous_thrusters = np.zeros(12, dtype=np.float64)
        self.command_history.clear()
        self.call_count = 0

    def get_previous_thrusters(self) -> np.ndarray:
        """Get last commanded thruster pattern for MPC warm-start."""
        return self.previous_thrusters.copy()
