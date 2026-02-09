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
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from satellite_control.core.simulation import SatelliteMPCLinearizedSimulation

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
        from satellite_control.core.simulation_context import (
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
            if (
                self.simulation.data_save_path is not None
                and self.simulation.data_logger.get_log_count() > 0
            ):
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
            logger.info("Saving simulation data...")
            self.simulation.save_csv_data()
            self.simulation.visualizer.sync_from_controller()
            self.simulation.save_mission_summary()
            self.simulation.save_animation_mp4(fig, ani)
            logger.info("Data saved to: %s", self.simulation.data_save_path)

            logger.info("Auto-generating performance plots...")
            self.simulation.auto_generate_visualizations()
            logger.info("All visualizations complete!")

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
            logger.info("Saving simulation data...")
            self.simulation.save_csv_data()
            self.simulation.visualizer.sync_from_controller()
            self.simulation.save_mission_summary()
            logger.info("CSV data saved to: %s", self.simulation.data_save_path)

            # Auto-generate all visualizations
            logger.info("Auto-generating visualizations...")
            self.simulation.auto_generate_visualizations()
            logger.info("All visualizations complete!")

        return self.simulation.data_save_path

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

        # Update reference state from path (path-only mode)
        current_state = self.simulation.get_current_state()
        self.simulation.update_path_reference_state(current_state)

        self.simulation.update_mpc_control()

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
        mission_state = self._get_mission_state()
        if not self.simulation.check_path_complete():
            self.simulation.trajectory_endpoint_reached_time = None
            return False

        if self.simulation.trajectory_endpoint_reached_time is None:
            self.simulation.trajectory_endpoint_reached_time = (
                self.simulation.simulation_time
            )

        hold_time_required = float(getattr(mission_state, "path_hold_end", 0.0) or 0.0)
        if hold_time_required <= 0.0:
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        hold_time = (
            self.simulation.simulation_time
            - self.simulation.trajectory_endpoint_reached_time
        )
        if hold_time >= hold_time_required:
            logger.info(
                "PATH FOLLOWING COMPLETE! Held for %.1f seconds.",
                hold_time_required,
            )
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        return False
