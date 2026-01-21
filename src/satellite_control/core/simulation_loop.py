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
from typing import Any, List, Optional, TYPE_CHECKING

if TYPE_CHECKING:
    from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

# V4.0.0: Legacy imports removed - using dependency injection

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
        """Get mission_state from simulation_config (V3.0.0: required)."""
        if (
            not hasattr(self.simulation, "simulation_config")
            or not self.simulation.simulation_config
        ):
            raise ValueError(
                "simulation_config is required (V3.0.0: no SatelliteConfig fallback)"
            )
        return self.simulation.simulation_config.mission_state

    def _get_app_config(self):
        """Get app_config from simulation_config (V3.0.0: required)."""
        if (
            not hasattr(self.simulation, "simulation_config")
            or not self.simulation.simulation_config
        ):
            raise ValueError(
                "simulation_config is required (V3.0.0: no SatelliteConfig fallback)"
            )
        return self.simulation.simulation_config.app_config

    def run(
        self,
        show_animation: bool = True,
        structured_config: Any = None,  # Deprecated
    ) -> Optional[Path]:
        """
        Run the simulation loop.

        Args:
            show_animation: Whether to display animation during simulation
            structured_config: Structured config to use (for context manager)

        Returns:
            Path to data save directory, or None
        """
        # V4.0.0: Removed legacy use_structured_config context manager
        return self._run_with_globals(show_animation=show_animation)

    def _run_with_globals(self, show_animation: bool = True) -> Optional[Path]:
        """
        Run linearized MPC simulation.

        Args:
            show_animation: Whether to display animation during simulation
        """
        print("\nStarting Linearized MPC Simulation...")
        print("Press 'q' to quit early, Space to pause/resume")
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
        from src.satellite_control.core.simulation_context import (
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
            print("\n\nSimulation cancelled by user")
            self.simulation.is_running = False

            # Save data when interrupted
            if (
                self.simulation.data_save_path is not None
                and self.simulation.data_logger.get_log_count() > 0
            ):
                print("\nSaving simulation data...")
                self.simulation.save_csv_data()
                self.simulation.visualizer.sync_from_controller()
                self.simulation.save_mission_summary()
                print(f" Data saved to: {self.simulation.data_save_path}")

                # Try to generate visualizations if we have enough data
                if self.simulation.data_logger.get_log_count() > 10:
                    try:
                        print("\n Auto-generating visualizations...")
                        self.simulation.auto_generate_visualizations()
                        print(" All visualizations complete!")
                    except Exception as e:
                        logger.warning(
                            f"WARNING: Could not generate visualizations: {e}"
                        )

        finally:
            # Cleanup
            pass
        return self.simulation.data_save_path

    def _run_matplotlib_animation(self) -> Optional[Path]:
        """Run simulation with matplotlib animation."""
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
            print("\nSaving simulation data...")
            self.simulation.save_csv_data()
            self.simulation.visualizer.sync_from_controller()
            self.simulation.save_mission_summary()
            self.simulation.save_animation_mp4(fig, ani)
            print(f" Data saved to: {self.simulation.data_save_path}")

            print("\n Auto-generating performance plots...")
            self.simulation.auto_generate_visualizations()
            print(" All visualizations complete!")

        return self.simulation.data_save_path

    def _run_batch_mode(self) -> Optional[Path]:
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

        while self.simulation.is_running:
            # Optimized Batch: Run physics steps without control logic
            # overhead
            if batch_mode:
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
            print("\nSaving simulation data...")
            self.simulation.save_csv_data()
            self.simulation.visualizer.sync_from_controller()
            self.simulation.save_mission_summary()
            print(f" CSV data saved to: {self.simulation.data_save_path}")

            # Auto-generate all visualizations
            print("\n Auto-generating visualizations...")
            self.simulation.auto_generate_visualizations()
            print(" All visualizations complete!")

        return self.simulation.data_save_path

    def update_step(self, frame: Optional[int]) -> List[Any]:
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
        # V4.0.0: Continuous Mode override
        if getattr(self.simulation, "continuous_mode", False):
            return False

        if self._check_path_following_completion():
            return True

        # Only stop simulation when max time is reached
        max_time = self.simulation.max_simulation_time
        if max_time and max_time > 0 and self.simulation.simulation_time >= max_time:
            print(f"\nSIMULATION COMPLETE at {self.simulation.simulation_time:.1f}s")
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        return False

    def _check_path_following_completion(self) -> bool:
        """Terminate when path progress reaches the end of the path."""
        mission_state = self._get_mission_state()
        path_length = float(getattr(mission_state, "dxf_path_length", 0.0) or 0.0)
        if path_length <= 0.0:
            path = getattr(mission_state, "mpcc_path_waypoints", None)
            if path and len(path) > 1:
                path_arr = np.array(path, dtype=float)
                path_length = float(
                    np.sum(np.linalg.norm(path_arr[1:] - path_arr[:-1], axis=1))
                )
        if path_length <= 0.0:
            return False

        path_s = getattr(self.simulation.mpc_controller, "s", None)
        if path_s is None:
            return False

        if path_s < path_length:
            self.simulation.trajectory_endpoint_reached_time = None
            return False

        if self.simulation.trajectory_endpoint_reached_time is None:
            self.simulation.trajectory_endpoint_reached_time = (
                self.simulation.simulation_time
            )

        hold_time_required = float(getattr(mission_state, "trajectory_hold_end", 0.0) or 0.0)
        if hold_time_required <= 0.0:
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        hold_time = (
            self.simulation.simulation_time
            - self.simulation.trajectory_endpoint_reached_time
        )
        if hold_time >= hold_time_required:
            print(
                f"\nPATH FOLLOWING COMPLETE! "
                f"Held for {hold_time_required:.1f} seconds."
            )
            self.simulation.is_running = False
            self.simulation.print_performance_summary()
            return True

        return False
