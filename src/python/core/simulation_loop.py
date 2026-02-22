"""
Simulation Loop Module

Handles the main simulation loop execution.
Extracted from simulation.py to improve modularity.

This module handles:
- Main loop setup and initialization
- Animation mode (matplotlib) vs batch mode (headless)
- Step-by-step simulation execution
- Loop termination conditions
- Data saving and cleanup
"""

import logging
import sys
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from core.simulation import SatelliteMPCLinearizedSimulation

# matplotlib is imported lazily inside _run_matplotlib_animation()

logger = logging.getLogger(__name__)


class SimulationLoop:
    """
    Handles the main simulation loop execution.

    This class encapsulates all the loop logic that was previously
    in SatelliteMPCLinearizedSimulation._run_simulation_with_globals
    and update_simulation.
    """

    def __init__(self, simulation: "SatelliteMPCLinearizedSimulation"):
        """
        Initialize the simulation loop.

        Args:
            simulation: The SatelliteMPCLinearizedSimulation instance
        """
        self.simulation = simulation

    def _get_mission_state(self):
        """Get mission_state from simulation_config.."""
        if (
            not hasattr(self.simulation, "simulation_config")
            or not self.simulation.simulation_config
        ):
            raise ValueError("simulation_config is required.")
        return self.simulation.simulation_config.mission_state

    def _get_app_config(self):
        """Get app_config from simulation_config.."""
        if (
            not hasattr(self.simulation, "simulation_config")
            or not self.simulation.simulation_config
        ):
            raise ValueError("simulation_config is required.")
        return self.simulation.simulation_config.app_config

    def run(
        self,
        show_animation: bool = True,
        structured_config: Any = None,  # Deprecated
    ) -> Path | None:
        """
        Run the simulation loop.

        Args:
            show_animation: Whether to display animation during simulation
            structured_config: Structured config to use (for context manager)

        Returns:
            Path to data save directory, or None
        """
        return self._run_with_globals(show_animation=show_animation)

    def _run_with_globals(self, show_animation: bool = True) -> Path | None:
        """
        Run linearized MPC simulation.

        Args:
            show_animation: Whether to display animation during simulation
        """
        logger.info("Starting Linearized MPC Simulation...")
        logger.info("Press 'q' to quit early, Space to pause/resume")
        self.simulation.is_running = True

        # Clear any previous data from the logger
        self.simulation.data_logger.clear_logs()
        self.simulation.physics_logger.clear_logs()

        self.simulation.data_save_path = self.simulation.create_data_directories()
        if self.simulation.data_save_path:
            self.simulation.data_logger.set_save_path(self.simulation.data_save_path)
            self.simulation.physics_logger.set_save_path(self.simulation.data_save_path)
            logger.info("Created data directory: %s", self.simulation.data_save_path)

        # Simulation Context
        from core.simulation_context import (
            SimulationContext,
        )

        if not hasattr(self.simulation, "context"):
            self.simulation.context = SimulationContext()
            self.simulation.context.dt = self.simulation.satellite.dt
            self.simulation.context.control_dt = self.simulation.control_update_interval

        # Initialize MPC Controller (Linearized Model)
        try:
            has_fig = (
                hasattr(self.simulation.satellite, "fig")
                and self.simulation.satellite.fig is not None
            )
            if show_animation and has_fig:
                # Matplotlib animation mode (legacy)
                return self._run_matplotlib_animation()
            else:
                # Run headless batch mode (no visualization)
                return self._run_batch_mode()

        except KeyboardInterrupt:
            logger.info("Simulation cancelled by user")
            self.simulation.is_running = False

            # Save data when interrupted
            if self.simulation.data_save_path is not None:
                if self.simulation.data_logger.get_log_count() > 0:
                    logger.info("Saving simulation data...")
                    self.simulation.save_csv_data()
                    self.simulation.visualizer.sync_from_controller()
                    self.simulation.save_mission_summary()
                    logger.info("Data saved to: %s", self.simulation.data_save_path)

                    # Try to generate visualizations if we have enough data
                    if self.simulation.data_logger.get_log_count() > 10:
                        try:
                            logger.info("Auto-generating visualizations...")
                            self.simulation.auto_generate_visualizations()
                            logger.info("All visualizations complete!")
                        except Exception as e:
                            logger.warning("Could not generate visualizations: %s", e)
                try:
                    self.simulation.finalize_run_artifacts(
                        run_status="interrupted",
                        status_detail="Simulation interrupted by user",
                    )
                except Exception as exc:
                    logger.warning(
                        "Failed to finalize run artifacts after interruption: %s", exc
                    )
        except Exception as exc:
            logger.exception("Unhandled simulation loop failure: %s", exc)
            self.simulation.is_running = False
            if self.simulation.data_save_path is not None:
                try:
                    if self.simulation.data_logger.get_log_count() > 0:
                        self.simulation.save_csv_data()
                        self.simulation.visualizer.sync_from_controller()
                        self.simulation.save_mission_summary()
                except Exception as save_exc:
                    logger.warning(
                        "Failed to save partial data after failure: %s", save_exc
                    )
                try:
                    self.simulation.finalize_run_artifacts(
                        run_status="failed",
                        status_detail=f"{type(exc).__name__}: {exc}",
                    )
                except Exception as finalize_exc:
                    logger.warning(
                        "Failed to finalize run artifacts after failure: %s",
                        finalize_exc,
                    )
            raise

        finally:
            # Cleanup
            pass
        return self.simulation.data_save_path

    def _run_matplotlib_animation(self) -> Path | None:
        """Run simulation with matplotlib animation."""
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        fig = self.simulation.satellite.fig
        ani = FuncAnimation(
            fig,
            self.update_step,
            interval=int(self.simulation.satellite.dt * 1000),
            blit=False,
            repeat=False,
            cache_frame_data=False,
        )
        plt.show()  # Show the animation window live

        # After animation is complete, save files
        if self.simulation.data_save_path is not None:
            logger.info("Simulation finished. Data saving in progress...")
            self.simulation.save_simulation_data()  # Consolidated save call

            # Use the new prompt for MP4 only
            if self._prompt_for_mp4():
                self.simulation.save_animation_mp4(fig, ani)
            else:
                logger.info("Skipping MP4 animation generation.")

            # Always generate static visualizations
            logger.info("Auto-generating visualizations...")
            self.simulation.auto_generate_visualizations(generate_animation=False)
            logger.info("All visualizations complete!")
            logger.info("Data saved to: %s", self.simulation.data_save_path)

            self.simulation.finalize_run_artifacts(
                run_status="completed",
                status_detail="Simulation completed in animation mode",
            )

        self._enforce_timing_contract_if_needed()

        return self.simulation.data_save_path

    def _run_batch_mode(self) -> Path | None:
        """Run simulation in batch mode (headless)."""
        # Performance Optimization: Batch physics steps
        # Calculate how many physics steps fit in one control update
        steps_per_batch = int(
            self.simulation.control_update_interval / self.simulation.satellite.dt
        )
        if steps_per_batch < 1:
            steps_per_batch = 1

        batch_mode = steps_per_batch > 1
        logger.info(
            f"Running optimized simulation loop. "
            f"Batch: {steps_per_batch} (dt={self.simulation.satellite.dt:.4f}s)"
        )

        fast_batch_steps = steps_per_batch - 1
        can_batch_physics = (
            batch_mode
            and hasattr(self.simulation.satellite, "update_physics_batch")
            and getattr(self.simulation.thruster_manager, "thruster_type", "") == "CON"
            and not getattr(
                self.simulation.thruster_manager, "use_realistic_physics", True
            )
            and float(getattr(self.simulation, "VALVE_DELAY", 0.0) or 0.0) == 0.0
            and float(getattr(self.simulation, "THRUST_RAMPUP_TIME", 0.0) or 0.0) == 0.0
        )

        while self.simulation.is_running:
            # Optimized Batch: Run physics steps without control logic
            # overhead
            if batch_mode:
                if can_batch_physics and fast_batch_steps > 0:
                    # Constant inputs in idealized continuous mode allow C++ batching.
                    self.simulation.process_command_queue()
                    self.simulation.satellite.update_physics_batch(
                        fast_batch_steps, self.simulation.satellite.dt
                    )
                    self.simulation.simulation_time = (
                        self.simulation.satellite.simulation_time
                    )
                    self.simulation.log_physics_step()
                else:
                    for _ in range(fast_batch_steps):
                        # Inline logic for speed
                        self.simulation.process_command_queue()
                        self.simulation.satellite.update_physics(
                            self.simulation.satellite.dt
                        )
                        self.simulation.simulation_time = (
                            self.simulation.satellite.simulation_time
                        )
                        self.simulation.log_physics_step()

            # Full Update (run MPC check, Mission Check, Logging, 1
            # Physics Step)
            self.update_step(None)  # type: ignore[arg-type]

            if not self.simulation.is_running:
                break

        if self.simulation.data_save_path is not None:
            logger.info("Simulation finished. Data saving in progress...")
            self.simulation.save_simulation_data()  # Consolidated save call

            # Always generate visualizations in batch mode contexts.
            # We prompt for MP4 animation because it is slow to render.
            logger.info("Auto-generating visualizations...")

            generate_mp4 = self._prompt_for_mp4()
            if not generate_mp4:
                logger.info(
                    "Skipping MP4 animation generation (user request or default)."
                )

            self.simulation.auto_generate_visualizations(
                generate_animation=generate_mp4
            )
            logger.info("All visualizations complete!")
            logger.info("Data saved to: %s", self.simulation.data_save_path)

            self.simulation.finalize_run_artifacts(
                run_status="completed",
                status_detail="Simulation completed in batch mode",
            )

        self._enforce_timing_contract_if_needed()

        return self.simulation.data_save_path

    def _prompt_for_mp4(self) -> bool:
        """
        Prompt the user whether to generate an MP4 video file.

        Returns:
            True if user inputs 'y' or 'yes' (case insensitive), False otherwise.
            Default is False (on simple Enter).
        """
        # Ensure input is visible and cursor is at the end
        if not sys.stdin.isatty():
            # Non-interactive mode (e.g. CI/CD), default to False
            return False

        try:
            print("\nGenerate MP4 animation? [y/N]: ", end="", flush=True)
            choice = sys.stdin.readline().strip().lower()
            return choice in ("y", "yes")
        except (KeyboardInterrupt, EOFError):
            return False

    def _enforce_timing_contract_if_needed(self) -> None:
        """Raise if strict timing contract enforcement is enabled and violated."""
        monitor = getattr(self.simulation, "performance_monitor", None)
        if monitor is None or not hasattr(monitor, "should_fail_on_timing_contract"):
            return
        if monitor.should_fail_on_timing_contract():
            summary = monitor.get_summary()
            raise RuntimeError(
                "MPC timing contract violated: "
                f"mean={summary.get('mpc_mean_ms', 0.0):.2f}ms, "
                f"max={summary.get('mpc_max_ms', 0.0):.2f}ms, "
                f"hard_limit_breaches={summary.get('mpc_hard_limit_breaches', 0)}"
            )

    def update_step(self, frame: int | None) -> list[Any]:
        """
        Update simulation step (called by matplotlib animation or batch loop).

        Args:
            frame: Current frame number (None for batch mode)

        Returns:
            List of artists for matplotlib animation
        """
        if not self.simulation.is_running:
            return []

        # Compute control first, then refresh reference so it reflects
        # what the controller expects at this moment.
        self.simulation.update_mpc_control()
        current_state = self.simulation.get_current_state()
        self.simulation.update_path_reference_state(current_state)

        # Process command queue to apply delayed commands (sets
        # active_thrusters)
        self.simulation.process_command_queue()

        # Advance physics: keep time bases aligned for valve timing
        dt = self.simulation.satellite.dt
        self.simulation.satellite.simulation_time = self.simulation.simulation_time

        # Record physics step time
        physics_start = time.perf_counter()
        self.simulation.satellite.update_physics(dt)
        physics_time = time.perf_counter() - physics_start
        self.simulation.performance_monitor.record_physics_step(physics_time)
        self.simulation.performance_monitor.increment_step()
        self.simulation.simulation_time = self.simulation.satellite.simulation_time

        # Log High-Frequency Physics Data
        self.simulation.log_physics_step()

        # Check termination conditions
        if self._check_termination_conditions():
            return []

        # Redraw
        self.simulation.draw_simulation()
        self.simulation.update_mpc_info_panel()  # Use custom MPC info panel instead

        return []

    def _check_termination_conditions(self) -> bool:
        """
        Check if simulation should terminate.

        Returns:
            True if simulation should stop, False otherwise
        """
        if getattr(self.simulation, "continuous_mode", False):
            return False

        if self._check_path_following_completion():
            return True

        # Only stop simulation when max time is reached
        max_time = self.simulation.max_simulation_time
        if max_time and max_time > 0 and self.simulation.simulation_time >= max_time:
            logger.info("SIMULATION COMPLETE at %.1fs", self.simulation.simulation_time)
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        return False

    def _check_path_following_completion(self) -> bool:
        """Terminate when path progress reaches the end of the path."""
        from core.path_completion import get_path_completion_status
        from core.v6_controller_runtime import TerminalSupervisorV6

        mission_state = self._get_mission_state()
        terminal_supervisor = getattr(self.simulation, "v6_terminal_supervisor", None)
        if terminal_supervisor is None:
            hold_required = float(getattr(mission_state, "path_hold_end", 10.0) or 10.0)
            terminal_supervisor = TerminalSupervisorV6(hold_required_s=hold_required)
            self.simulation.v6_terminal_supervisor = terminal_supervisor

        status = get_path_completion_status(self.simulation)
        hold_required = float(getattr(mission_state, "path_hold_end", 10.0) or 10.0)
        terminal_supervisor.hold_required_s = max(0.0, hold_required)
        gate = terminal_supervisor.evaluate(
            sim_time_s=float(self.simulation.simulation_time),
            progress_ok=bool(status.get("progress_ok", False)),
            position_ok=bool(status.get("position_ok", False)),
            angle_ok=bool(status.get("angle_ok", False)),
            velocity_ok=bool(status.get("velocity_ok", False)),
            angular_velocity_ok=bool(status.get("angular_velocity_ok", False)),
        )
        self.simulation.v6_completion_gate = gate
        self.simulation.v6_completion_reached = bool(gate.complete)
        self.simulation.trajectory_endpoint_reached_time = (
            terminal_supervisor.hold_started_at_s
            if terminal_supervisor.hold_started_at_s is not None
            else None
        )

        if hasattr(self.simulation, "v6_completion_gate_trace"):
            self.simulation._append_capped_history(
                self.simulation.v6_completion_gate_trace,
                {
                    "time_s": float(self.simulation.simulation_time),
                    "progress_ok": bool(gate.progress_ok),
                    "position_ok": bool(gate.position_ok),
                    "angle_ok": bool(gate.angle_ok),
                    "velocity_ok": bool(gate.velocity_ok),
                    "angular_velocity_ok": bool(gate.angular_velocity_ok),
                    "hold_elapsed_s": float(gate.hold_elapsed_s),
                    "hold_required_s": float(gate.hold_required_s),
                    "gate_ok": bool(gate.gate_ok),
                    "complete": bool(gate.complete),
                    "last_breach_reason": gate.last_breach_reason,
                    "path_s": float(status.get("path_s", 0.0)),
                    "path_length": float(status.get("path_length", 0.0)),
                    "path_error": float(status.get("path_error", 0.0)),
                },
            )

        if gate.complete:
            logger.info(
                "PATH FOLLOWING COMPLETE! Held for %.1f seconds.",
                gate.hold_required_s,
            )
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        return False
