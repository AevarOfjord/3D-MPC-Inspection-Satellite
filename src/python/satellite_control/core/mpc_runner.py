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

    def set_mode_state(self, mode_state: Any | None) -> None:
        """Set current V6 mode state (TRACK/RECOVER/SETTLE/HOLD/COMPLETE)."""
        self.mode_state = mode_state

    def _current_mode_name(self) -> str:
        mode = getattr(self.mode_state, "current_mode", None)
        if isinstance(mode, str) and mode:
            return mode
        return "TRACK"

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

    def _should_bypass_hysteresis_for_terminal_settling(
        self,
        mpc_info: dict[str, Any],
    ) -> bool:
        """Allow fine thrust corrections near endpoint to satisfy terminal hold tolerances."""
        if not self.enable_thruster_hysteresis:
            return False

        endpoint_error = mpc_info.get("path_endpoint_error")
        if endpoint_error is None:
            return False

        try:
            endpoint_error_val = float(endpoint_error)
        except (TypeError, ValueError):
            return False

        if not np.isfinite(endpoint_error_val) or endpoint_error_val < 0.0:
            return False

        pos_tol = 0.1
        if self.state_validator is not None:
            try:
                pos_tol = float(
                    getattr(self.state_validator, "position_tolerance", pos_tol)
                )
            except Exception:
                pos_tol = 0.1

        # Keep hysteresis for most of the trajectory, but disable it close to
        # the endpoint where tiny commands are needed to satisfy strict
        # position/velocity/omega completion thresholds.
        terminal_settle_band = max(0.25, 3.0 * pos_tol)
        return endpoint_error_val <= terminal_settle_band

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
        if hasattr(self.mpc, "set_runtime_mode"):
            try:
                self.mpc.set_runtime_mode(mode_name)
            except Exception:
                logger.debug("Failed to forward runtime mode to MPC controller", exc_info=True)

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
        except Exception:
            logger.exception("Controller execution failed; applying zero-command fallback.")
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
                endpoint_error = mpc_info.get("path_endpoint_error")
                if self.actuator_policy is not None:
                    try:
                        thruster_action = self.actuator_policy.apply(
                            thruster_action,
                            previous_thrusters,
                            mode=mode_name,
                            endpoint_error_m=endpoint_error,
                        )
                    except Exception:
                        logger.debug(
                            "ActuatorPolicyV6 apply failed, using legacy hysteresis",
                            exc_info=True,
                        )
                        if not self._should_bypass_hysteresis_for_terminal_settling(
                            mpc_info
                        ):
                            thruster_action = self._apply_thruster_hysteresis(
                                thruster_action=thruster_action,
                                previous_thrusters=previous_thrusters,
                            )
                else:
                    if not self._should_bypass_hysteresis_for_terminal_settling(
                        mpc_info
                    ):
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
