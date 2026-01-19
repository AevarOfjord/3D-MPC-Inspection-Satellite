"""
Simulation IO Module

Handles data export, directory management, and file operations for simulations.
Extracted from SatelliteMPCLinearizedSimulation to reduce class size.
"""

import logging
import json
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, List, Optional, Union

import numpy as np
from scipy.spatial.transform import Rotation

if TYPE_CHECKING:
    from src.satellite_control.core.simulation import (
        SatelliteMPCLinearizedSimulation,
    )

logger = logging.getLogger(__name__)


class SimulationIO:
    """
    Data export and directory management for simulations.

    Handles:
    - Creating timestamped data directories
    - Saving mission summary reports
    - Coordinating with DataLogger and ReportGenerator
    """

    def __init__(self, simulation: "SatelliteMPCLinearizedSimulation"):
        """
        Initialize SimulationIO with reference to parent simulation.

        Args:
            simulation: Parent simulation instance
        """
        self.sim = simulation

    def create_data_directories(self) -> Path:
        """
        Create the directory structure for saving data.

        Returns:
            Path to the timestamped subdirectory
        """
        timestamp = datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

        # Create directory path: Data/Simulation/timestamp
        base_path = Path("Data")
        sim_path = base_path / "Simulation"
        timestamped_path = sim_path / timestamp

        # Create directories
        timestamped_path.mkdir(parents=True, exist_ok=True)

        logger.info(f"Created data directory: {timestamped_path}")
        return timestamped_path

    def save_csv_data(self) -> None:
        """Save all logged data to CSV files (delegates to DataLoggers)."""
        self.sim.data_logger.save_csv_data()
        self.sim.physics_logger.save_csv_data()

    def save_mission_summary(self) -> None:
        """Generate and save mission summary report."""
        if not self.sim.data_save_path:
            logger.warning("Cannot save mission summary: No data save path set")
            return

        # Attempt to load state history from CSV if not in memory
        history_for_report: Union[List[np.ndarray], np.ndarray] = self.sim.state_history
        control_history_for_report: List[np.ndarray] = self.sim.control_history

        if not history_for_report:
            loaded_history = self._load_history_from_csv()
            if loaded_history is not None:
                history_for_report = loaded_history

        if history_for_report is None or len(history_for_report) == 0:
            logger.warning("No state history available for full summary")
            return

        summary_path = self.sim.data_save_path / "mission_summary.txt"

        # Use DataLogger stats for solve times
        solve_times = self.sim.data_logger.stats_solve_times

        if isinstance(history_for_report, np.ndarray):
            history_for_report_list = [row for row in history_for_report]
        else:
            history_for_report_list = history_for_report

        self.sim.report_generator.generate_report(
            output_path=summary_path,
            state_history=history_for_report_list,
            target_state=self.sim.target_state,
            control_time=self.sim.simulation_time,
            mpc_solve_times=solve_times,
            control_history=control_history_for_report,
            target_reached_time=self.sim.target_reached_time,
            target_maintenance_time=self.sim.target_maintenance_time,
            times_lost_target=self.sim.times_lost_target,
            maintenance_position_errors=self.sim.maintenance_position_errors,
            maintenance_angle_errors=self.sim.maintenance_angle_errors,
            position_tolerance=self.sim.position_tolerance,
            angle_tolerance=self.sim.angle_tolerance,
            control_update_interval=self.sim.control_update_interval,
            angle_difference_func=self.sim.angle_difference,
            check_target_reached_func=self.sim.check_target_reached,
            test_mode="SIMULATION",
        )
        self._save_mission_metadata()

    def _save_mission_metadata(self) -> None:
        """Save mission metadata used by the web visualizer."""
        if not self.sim.data_save_path:
            return

        mission_manager = getattr(self.sim, "mission_manager", None)
        mission_state = getattr(mission_manager, "mission_state", None)
        if mission_state is None:
            return

        trajectory_type = getattr(mission_state, "trajectory_type", "")
        is_scan = bool(
            getattr(mission_state, "mesh_scan_mode_active", False)
            or trajectory_type in ("scan", "starlink_orbit")
        )
        if not is_scan:
            return

        pose = getattr(mission_state, "mesh_scan_object_pose", None)
        if pose and len(pose) >= 6:
            pos = [float(pose[0]), float(pose[1]), float(pose[2])]
            ori = [float(pose[3]), float(pose[4]), float(pose[5])]
        else:
            center = getattr(mission_state, "trajectory_object_center", None)
            pos = [float(val) for val in (center or (0.0, 0.0, 0.0))]
            ori = [0.0, 0.0, 0.0]

        # Determine scan object type based on trajectory type
        if trajectory_type == "starlink_orbit":
            obj_path = getattr(mission_state, "mesh_scan_obj_path", "")
            scan_object = {
                "type": "starlink",
                "position": pos,
                "orientation": ori,
                "radius": 1.5,  # Approximate Starlink max radius
                "height": 0.3,  # Approximate Starlink height
                "obj_path": obj_path,
            }
        else:
            # Default cylinder for scan missions
            scan_object = {
                "type": "cylinder",
                "position": pos,
                "orientation": ori,
                "radius": 0.25,
                "height": 3.0,
            }

        metadata = {
            "mission_type": trajectory_type if trajectory_type else "scan_object",
            "scan_object": scan_object,
        }

        metadata_path = self.sim.data_save_path / "mission_metadata.json"
        try:
            metadata_path.write_text(json.dumps(metadata, indent=2))
        except Exception as exc:
            logger.warning(f"Failed to save mission metadata: {exc}")

    def _load_history_from_csv(self) -> Optional[np.ndarray]:
        """
        Load state history from CSV file if not in memory.

        Returns:
            State history array or None if loading fails
        """
        try:
            import pandas as pd

            if self.sim.data_save_path is None:
                return None

            csv_path = self.sim.data_save_path / "control_data.csv"
            if csv_path.exists():
                df = pd.read_csv(csv_path)

                # Check if 3D columns exist
                if "Current_Z" in df.columns:
                    # Load 3D data
                    pos = df[["Current_X", "Current_Y", "Current_Z"]].values
                    vel = df[["Current_VX", "Current_VY", "Current_VZ"]].values
                    ang_vel = df[["Current_WX", "Current_WY", "Current_WZ"]].values

                    # Euler to Quat
                    # Yaw, Roll, Pitch might be logged.
                    # simulation_logger logs: Current_Roll, Current_Pitch, Current_Yaw
                    r = df["Current_Roll"].values
                    p = df["Current_Pitch"].values
                    y = df["Current_Yaw"].values

                    # Stack and convert
                    euler = np.column_stack([r, p, y])
                    quat = Rotation.from_euler("xyz", euler, degrees=False).as_quat()
                    # Scipy output is xyzw, we want wxyz for internal state usually?
                    # simulation.py usually expects [w, x, y, z] or [q0, q1, q2, q3]
                    # Let's check logic: mpc_controller says [qw, qx, qy, qz]
                    # Scipy is [x, y, z, w]
                    quat_wxyz = np.column_stack(
                        [quat[:, 3], quat[:, 0], quat[:, 1], quat[:, 2]]
                    )

                    # Construct 13-element state: [pos(3), quat(4), vel(3), ang_vel(3)]
                    # Shape (N, 13)
                    state_history = np.column_stack([pos, quat_wxyz, vel, ang_vel])
                    return state_history
                else:
                    # Legacy 2D fallback
                    values: np.ndarray = df[
                        [
                            "Current_X",
                            "Current_Y",
                            "Current_VX",
                            "Current_VY",
                            "Current_Yaw",
                            "Current_Angular_Vel",
                        ]
                    ].values.astype(np.float64)
                    return values
        except Exception as e:
            logger.debug(f"Could not load history from CSV: {e}")

        return None

    def save_animation_mp4(self, fig: Any, ani: Any) -> Optional[str]:
        """
        Save the animation as MP4 file.

        Args:
            fig: Matplotlib figure object
            ani: Matplotlib animation object

        Returns:
            Path to saved MP4 file or None if save failed
        """
        self.sim.visualizer.sync_from_controller()
        result: Optional[str] = self.sim.visualizer.save_animation_mp4(fig, ani)
        return result
