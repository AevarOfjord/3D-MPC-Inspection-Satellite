"""
MPC Runner Module

Encapsulates the high-level control loop logic:
1. State measurement (with noise)
2. MPC solver invocation
3. Control action processing (handling limits, fallbacks)
"""

import logging
import time
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from satellite_control.config.physics import THRUSTER_COUNT
from satellite_control.control.mpc_controller import MPCController

if TYPE_CHECKING:
    from satellite_control.config.models import AppConfig

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
        default_thruster_count = THRUSTER_COUNT
        if self.config is not None and hasattr(self.config, "physics"):
            try:
                default_thruster_count = len(self.config.physics.thruster_positions)
            except Exception:
                logger.debug("Failed to read thruster count from config")
                default_thruster_count = THRUSTER_COUNT
        self.thruster_count = getattr(self.mpc, "num_thrusters", default_thruster_count)
        self.rw_axes = getattr(self.mpc, "num_rw_axes", 0)
        self.previous_thrusters = np.zeros(self.thruster_count, dtype=np.float64)
        cfg_mpc = getattr(self.config, "mpc", None) if self.config is not None else None
        self.enable_thruster_hysteresis = bool(
            getattr(
                cfg_mpc,
                "enable_thruster_hysteresis",
                getattr(self.mpc, "enable_thruster_hysteresis", True),
            )
        )
        self.thruster_hysteresis_on = float(
            getattr(
                cfg_mpc,
                "thruster_hysteresis_on",
                getattr(self.mpc, "thruster_hysteresis_on", 0.015),
            )
        )
        self.thruster_hysteresis_off = float(
            getattr(
                cfg_mpc,
                "thruster_hysteresis_off",
                getattr(self.mpc, "thruster_hysteresis_off", 0.007),
            )
        )
        if self.thruster_hysteresis_on <= self.thruster_hysteresis_off:
            logger.warning(
                "Invalid thruster hysteresis thresholds (on=%.6f, off=%.6f); disabling hysteresis.",
                self.thruster_hysteresis_on,
                self.thruster_hysteresis_off,
            )
            self.enable_thruster_hysteresis = False

    def _apply_thruster_hysteresis(
        self,
        thruster_action: np.ndarray,
        previous_thrusters: np.ndarray,
    ) -> np.ndarray:
        """Apply on/off hysteresis to reduce chatter in normalized thruster commands."""
        if not self.enable_thruster_hysteresis:
            return thruster_action

        prev = np.array(previous_thrusters, dtype=np.float64, copy=False).reshape(-1)
        if prev.size != self.thruster_count:
            prev = self.previous_thrusters

        prev_active = prev >= self.thruster_hysteresis_off
        turn_on = (~prev_active) & (thruster_action >= self.thruster_hysteresis_on)
        stay_on = prev_active & (thruster_action >= self.thruster_hysteresis_off)
        active_mask = turn_on | stay_on

        # Inactive channels are forced to exactly 0.0.
        return np.where(active_mask, thruster_action, 0.0).astype(np.float64, copy=False)

    def compute_control_action(
        self,
        true_state: np.ndarray,
        previous_thrusters: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, dict[str, Any], float, float]:
        """
        Compute the next control action based on current state.

        Returns:
            Tuple of:
            - thruster_action (np.ndarray): Thruster commands [N]
            - rw_torque (np.ndarray): Reaction wheel torques (normalized) [3]
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
        start_compute_time = time.perf_counter()
        mpc_info: dict[str, Any] = {}

        try:
            control_action, mpc_info = self.mpc.get_control_action(
                x_current=measured_state,
                previous_thrusters=previous_thrusters,
            )
        except TypeError:
            logger.warning("Controller signature mismatch; falling back.")
            control_action, mpc_info = self.mpc.get_control_action(
                measured_state, previous_thrusters
            )

        end_compute_time = time.perf_counter()
        mpc_computation_time = end_compute_time - start_compute_time
        command_sent_time = end_compute_time

        # 3. Post-process Action
        rw_torque = np.zeros(self.rw_axes, dtype=np.float64)
        thruster_action = None
        if control_action is not None:
            # Ensure shape is 1D
            if control_action.ndim == 2:
                control_action = control_action[0, :]

            if hasattr(self.mpc, "split_control"):
                rw_torque, thruster_action = self.mpc.split_control(control_action)
            elif self.rw_axes and control_action.size >= self.rw_axes:
                rw_torque = control_action[: self.rw_axes]
                thruster_action = control_action[self.rw_axes :]
            else:
                thruster_action = control_action

            # Validate size - must match configured thruster count
            if len(thruster_action) != self.thruster_count:
                logger.error(
                    f"Invalid thruster array size {len(thruster_action)}, "
                    f"expected {self.thruster_count}. Defaulting to zero thrust."
                )
                rw_torque = np.zeros(self.rw_axes, dtype=np.float64)
                thruster_action = np.zeros(self.thruster_count, dtype=np.float64)
            else:
                # Enforce bounds (in-place clip avoids extra allocation)
                np.clip(thruster_action, 0.0, 1.0, out=thruster_action)
                thruster_action = thruster_action.astype(np.float64, copy=False)
                thruster_action = self._apply_thruster_hysteresis(
                    thruster_action=thruster_action,
                    previous_thrusters=previous_thrusters,
                )
                if self.rw_axes:
                    np.clip(rw_torque, -1.0, 1.0, out=rw_torque)
                    rw_torque = rw_torque.astype(np.float64, copy=False)
        else:
            # Fallback if controller failed completely (should return
            # fallback though)
            thruster_action = np.zeros(self.thruster_count, dtype=np.float64)
            logger.error("Controller returned None! Defaulting to zero thrust.")

        # Update internal state (share one copy for both)
        thruster_copy = thruster_action.copy()
        self.previous_thrusters = thruster_copy

        return (
            thruster_action,
            rw_torque,
            mpc_info,
            mpc_computation_time,
            command_sent_time,
        )

    def reset(self) -> None:
        """Reset runner state for a new simulation run."""
        self.previous_thrusters = np.zeros(self.thruster_count, dtype=np.float64)

    def get_previous_thrusters(self) -> np.ndarray:
        """Get last commanded thruster pattern for MPC warm-start."""
        return self.previous_thrusters.copy()
