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
from config.physics import THRUSTER_COUNT
from control.mpc_controller import MPCController
from core.v6_controller_runtime import ActuatorPolicyV6

if TYPE_CHECKING:
    from config.models import AppConfig

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
        actuator_policy: Any | None = None,
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
        self.actuator_policy = actuator_policy
        self.mode_state: Any | None = None

        # Internal state management
        default_thruster_count = THRUSTER_COUNT
        if self.config is not None:
            default_thruster_count = len(self.config.physics.thruster_positions)
        self.thruster_count = getattr(self.mpc, "num_thrusters", default_thruster_count)
        self.rw_axes = getattr(self.mpc, "num_rw_axes", 0)
        self.previous_thrusters = np.zeros(self.thruster_count, dtype=np.float64)
        if self.actuator_policy is None:
            cfg_mpc = self.config.mpc if self.config is not None else None
            cfg_actuator = (
                self.config.actuator_policy if self.config is not None else None
            )
            self.actuator_policy = ActuatorPolicyV6(
                enable_thruster_hysteresis=bool(
                    getattr(
                        cfg_mpc,
                        "enable_thruster_hysteresis",
                        getattr(
                            cfg_actuator,
                            "enable_thruster_hysteresis",
                            getattr(self.mpc, "enable_thruster_hysteresis", True),
                        ),
                    )
                ),
                thruster_hysteresis_on=float(
                    getattr(
                        cfg_mpc,
                        "thruster_hysteresis_on",
                        getattr(
                            cfg_actuator,
                            "thruster_hysteresis_on",
                            getattr(self.mpc, "thruster_hysteresis_on", 0.015),
                        ),
                    )
                ),
                thruster_hysteresis_off=float(
                    getattr(
                        cfg_mpc,
                        "thruster_hysteresis_off",
                        getattr(
                            cfg_actuator,
                            "thruster_hysteresis_off",
                            getattr(self.mpc, "thruster_hysteresis_off", 0.007),
                        ),
                    )
                ),
                terminal_bypass_band_m=float(
                    getattr(
                        cfg_actuator,
                        "terminal_bypass_band_m",
                        0.20,
                    )
                ),
            )

    def set_mode_state(self, mode_state: Any | None) -> None:
        """Set current V6 mode state (TRACK/RECOVER/SETTLE/HOLD/COMPLETE)."""
        self.mode_state = mode_state

    def _current_mode_name(self) -> str:
        mode = getattr(self.mode_state, "current_mode", None)
        if isinstance(mode, str) and mode:
            return mode
        return "TRACK"

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
        mode_name = self._current_mode_name()
        try:
            self.mpc.set_runtime_mode(mode_name)
        except Exception:
            logger.debug(
                "Failed to forward runtime mode to MPC controller", exc_info=True
            )

        try:
            control_action, mpc_info = self.mpc.get_control_action(
                x_current=measured_state,
                previous_thrusters=previous_thrusters,
            )
        except Exception:
            logger.exception(
                "Controller execution failed; applying zero-command fallback."
            )
            control_action = np.zeros(
                self.rw_axes + self.thruster_count,
                dtype=np.float64,
            )
            mpc_info = {
                "status": -1,
                "status_name": "FAILED",
                "solver_fallback": True,
                "solver_fallback_reason": "controller_exception",
                "solver_success": False,
                "solve_time": 0.0,
                "iterations": None,
                "objective_value": None,
                "timeout": False,
                "time_limit_exceeded": False,
                "controller_core": str(getattr(self.mpc, "controller_core", "unknown")),
                "solver_backend": str(getattr(self.mpc, "solver_backend", "OSQP")),
                "solver_type": str(getattr(self.mpc, "solver_type", "OSQP")),
                "solver_time_limit": float(getattr(self.mpc, "solver_time_limit", 0.0)),
            }

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

            rw_torque, thruster_action = self.mpc.split_control(control_action)

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
                endpoint_error = mpc_info.get("path_endpoint_error")
                try:
                    thruster_action = self.actuator_policy.apply(
                        thruster_action,
                        previous_thrusters,
                        mode=mode_name,
                        endpoint_error_m=endpoint_error,
                    )
                except Exception:
                    logger.warning(
                        "ActuatorPolicyV6 apply failed; using unclipped controller command.",
                        exc_info=True,
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
