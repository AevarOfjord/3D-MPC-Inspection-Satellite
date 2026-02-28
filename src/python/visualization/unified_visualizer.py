#!/usr/bin/env python3
"""
Unified Visualization Module for MPC Satellite Control

Unified visualization system for simulation systems.
Automatically generates MP4 animations and performance plots from CSV data.

Features:
- Unified interface for both simulation and real test data
- Automatic data detection and loading
- High-quality MP4 animation generation
- Comprehensive performance analysis plots
- Configurable titles and labels based on data source
- Real-time animation with proper timing
"""

import csv
import os
import platform
import shutil
import sys
import warnings
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from config.constants import Constants
from config.models import AppConfig
from config.paths import (
    LEGACY_SIMULATION_DATA_ROOT,
    SIMULATION_DATA_ROOT,
    resolve_repo_path,
)
from config.physics import THRUSTER_COUNT
from config.simulation_config import SimulationConfig
from cycler import cycler
from matplotlib.axes import Axes
from matplotlib.figure import Figure
from mission.state import MissionState
from simulation.artifact_paths import resolve_existing_artifact_path
from visualization.plot_style import PlotStyle

matplotlib.use("Agg")  # Use non-interactive backend


# Configure ffmpeg path with safe fallbacks
try:
    # Try to get from constants first
    from config.constants import Constants

    ffmpeg_path = Constants.FFMPEG_PATH
    if ffmpeg_path and os.path.exists(ffmpeg_path):
        plt.rcParams["animation.ffmpeg_path"] = ffmpeg_path
    else:
        # If ffmpeg already in PATH, do nothing
        if shutil.which("ffmpeg") is None:
            if platform.system() == "Darwin":
                brew_ffmpeg = "/opt/homebrew/bin/ffmpeg"
                if os.path.exists(brew_ffmpeg):
                    plt.rcParams["animation.ffmpeg_path"] = brew_ffmpeg
except Exception:
    pass


warnings.filterwarnings("ignore")

# Worker process cache for parallel frame rendering
_worker_gen_cache: Any | None = None

# Import cycler and Circle from correct modules

# Use Seaborn for prettier plot styling (falls back to matplotlib if not installed)

_SEABORN_AVAILABLE = False

# Fallback to matplotlib color scheme
plt.rcParams["axes.prop_cycle"] = cycler(
    color=[
        "#000000",  # Black
        "#CC0000",  # Dark Red
        "#1f77b4",  # Blue
        "#2ca02c",  # Green
        "#9467bd",  # Purple
        "#8c564b",  # Brown
        "#e377c2",  # Pink
        "#7f7f7f",  # Gray
    ]
)

# Consistent plot styling
plt.rcParams.update(
    {
        "axes.titlesize": 16,
        "axes.titleweight": "bold",
        "axes.labelsize": 14,
        "legend.fontsize": 12,
        "font.family": "serif",  # Serif for academic look
        "lines.linewidth": 2,
        "grid.alpha": 0.2,
    }
)


class UnifiedVisualizationGenerator:
    """Unified visualization generator for simulation data."""

    def __init__(
        self,
        data_directory: str,
        interactive: bool = False,
        load_data: bool = True,
        prefer_pandas: bool = True,
        app_config: AppConfig | None = None,
        mission_state: MissionState | None = None,
    ):
        """Initialize the visualization generator.

        Args:
            data_directory: Base directory containing the data
            interactive: If True, allow user to select which data to visualize
            load_data: If True, automatically locate and load the newest CSV
            prefer_pandas: If True, try pandas; otherwise use csv module backend
            app_config: Optional AppConfig for accessing physical and MPC parameters (v4.0.0: uses defaults if None)
            mission_state: Optional MissionState for accessing mission-specific data (v4.0.0: no fallback)
        """
        self.data_directory = Path(data_directory)
        self.mode = "simulation"
        self.csv_path: Path | None = None
        self.output_dir: Path | None = None
        self.data: Any | None = None  # DataFrame when pandas, self when csv backend
        self.control_data: Any | None = None
        self.fig: Figure | None = None
        self.ax_main: Axes | None = None
        self.ax_info: Axes | None = None
        # Data backend management
        self._data_backend: str | None = None  # 'pandas' or 'csv'
        self._rows: list[dict[str, Any]] | None = None  # list of dicts when csv backend
        self._col_data: dict[str, np.ndarray] | None = (
            None  # dict of column -> array when csv backend
        )
        self.use_pandas = prefer_pandas

        # Set titles and labels
        self._setup_labels()

        self.fps: float | None = None  # Will be calculated from simulation timestep
        self.speedup_factor = 1.0  # Real-time playback (no speedup)
        self.real_time = True  # Enable real-time animation
        self.dt: float | None = None  # Simulation timestep (will be auto-detected)

        # Store config references (v3.0.0)
        self.app_config = app_config
        self.mission_state = mission_state

        if app_config and app_config.physics:
            self.satellite_size = app_config.physics.satellite_size
        else:
            default_config = SimulationConfig.create_default()
            self.satellite_size = default_config.app_config.physics.satellite_size

        self.satellite_color = "blue"
        self.reference_color = "red"
        self.trajectory_color = "cyan"

        # Get thruster positions and forces from app_config if available
        self.thrusters = {}
        if app_config and app_config.physics:
            for thruster_id, pos in app_config.physics.thruster_positions.items():
                self.thrusters[thruster_id] = pos
            self.thruster_forces = app_config.physics.thruster_forces.copy()
        else:
            default_config = SimulationConfig.create_default()
            for (
                thruster_id,
                pos,
            ) in default_config.app_config.physics.thruster_positions.items():
                self.thrusters[thruster_id] = pos
            self.thruster_forces = (
                default_config.app_config.physics.thruster_forces.copy()
            )

        # Initialize component generators (lazy initialization)
        self._plot_generator: Any | None = None
        self._video_renderer: Any | None = None

        if load_data:
            if interactive:
                self.select_data_interactively()
            else:
                self.find_newest_data()

    def _setup_labels(self) -> None:
        """Setup titles and labels for simulation."""
        self.system_name = "Simulation"
        self.system_title = "MPC Simulation"
        self.data_source = "Simulation Data"
        self.plot_prefix = "Simulation"
        self.animation_title = "MPC Satellite Simulation Visualization"
        self.trajectory_title = "MPC - Satellite Trajectory"
        self.frame_title_template = "MPC - Frame {frame}"

    def _find_csv_files_for_folder(self, folder: Path) -> list[Path]:
        """Find CSV artifacts in canonical locations with legacy fallback."""
        preferred = [
            resolve_existing_artifact_path(folder, "physics_data.csv"),
            resolve_existing_artifact_path(folder, "control_data.csv"),
            resolve_existing_artifact_path(folder, "simulation_data.csv"),
        ]
        csv_files = [path for path in preferred if path is not None]
        if csv_files:
            return csv_files

        data_dir = folder / "Data"
        if data_dir.exists():
            nested = sorted(data_dir.rglob("*.csv"))
            if nested:
                return nested

        return sorted(folder.glob("*.csv"))

    def find_newest_data(self) -> None:
        """Find the newest data folder and CSV file."""
        print(f"Searching for {self.system_name} data in: {self.data_directory}")

        if not self.data_directory.exists():
            raise FileNotFoundError(f"Data directory not found: {self.data_directory}")

        # First, check if the current directory itself contains CSV files
        csv_files = self._find_csv_files_for_folder(self.data_directory)
        if csv_files:
            self.csv_path = csv_files[0]
            self.output_dir = self.data_directory
            print(f"Using CSV file: {self.csv_path.name} in current directory")
            self.load_csv_data()
            return

        # If no CSV in current directory, search subdirectories
        data_folders = [d for d in self.data_directory.iterdir() if d.is_dir()]

        if not data_folders:
            raise FileNotFoundError(f"No data folders found in: {self.data_directory}")

        data_folders.sort(key=lambda x: x.stat().st_mtime, reverse=True)

        print(f"Found {len(data_folders)} data folders")

        # Try folders until we find one with CSV data
        newest_folder = None
        csv_files = []

        for folder in data_folders:
            print(f"Checking folder: {folder.name}")

            csv_files = self._find_csv_files_for_folder(folder)

            if csv_files:
                newest_folder = folder
                print(f"Using folder: {newest_folder.name}")
                break
            else:
                print("  No CSV files found, trying next folder...")

        if not csv_files or newest_folder is None:
            raise FileNotFoundError("No CSV files found in any data folder")

        self.csv_path = csv_files[0]
        self.output_dir = newest_folder

        print(f"Using CSV file: {self.csv_path.name}")
        print(f"Output directory: {self.output_dir}")

        # Load and validate data
        self.load_csv_data()

    def select_data_interactively(self) -> None:
        """Allow user to interactively select which data to visualize."""
        print(f"\n{'=' * 60}")
        print(f"INTERACTIVE {self.system_name.upper()} DATA SELECTION")
        print(f"{'=' * 60}")

        if not self.data_directory.exists():
            raise FileNotFoundError(f"Data directory not found: {self.data_directory}")

        # Find all subdirectories with CSV files
        data_folders = []
        for folder in self.data_directory.iterdir():
            if folder.is_dir():
                csv_files = self._find_csv_files_for_folder(folder)
                if csv_files:
                    data_folders.append((folder, csv_files))

        if not data_folders:
            raise FileNotFoundError(
                f"No folders with CSV data found in: {self.data_directory}"
            )

        data_folders.sort(key=lambda x: x[0].stat().st_mtime, reverse=True)

        print(f"Found {len(data_folders)} folders with data:")
        print()

        for i, (folder, csv_files) in enumerate(data_folders, 1):
            # Get folder timestamp
            timestamp = datetime.fromtimestamp(folder.stat().st_mtime)
            print(f"{i:2}. {folder.name}")
            print(f"    Modified: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"    CSV files: {len(csv_files)}")
            print()

        # Get user selection
        while True:
            try:
                choice = input(
                    f"Select folder (1-{len(data_folders)}) or 'q' to quit: "
                ).strip()
                if choice.lower() == "q":
                    print("Visualization cancelled.")
                    sys.exit(0)
                folder_idx = int(choice) - 1
                if 0 <= folder_idx < len(data_folders):
                    selected_folder, csv_files = data_folders[folder_idx]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(data_folders)}")
            except ValueError:
                print("Please enter a valid number or 'q' to quit")

        if len(csv_files) > 1:
            print(f"\nMultiple CSV files found in {selected_folder.name}:")
            for i, csv_file in enumerate(csv_files, 1):
                print(f"{i}. {csv_file.name}")

            while True:
                try:
                    csv_choice = input(
                        f"Select CSV file (1-{len(csv_files)}): "
                    ).strip()
                    csv_idx = int(csv_choice) - 1
                    if 0 <= csv_idx < len(csv_files):
                        selected_csv = csv_files[csv_idx]
                        break
                    else:
                        print(f"Please enter a number between 1 and {len(csv_files)}")
                except ValueError:
                    print("Please enter a valid number")
        else:
            selected_csv = csv_files[0]

        self.csv_path = selected_csv
        self.output_dir = selected_folder

        print(f"\n Selected: {selected_folder.name}/{selected_csv.name}")

        # Load and validate data
        self.load_csv_data()

    def load_csv_data(self) -> None:
        """Load and validate CSV data."""
        assert self.csv_path is not None, "CSV path must be set before loading data"

        try:
            if self.use_pandas:
                import pandas as pd

                print(f"Loading CSV data from: {self.csv_path}")
                self.data = pd.read_csv(self.csv_path)
                self._data_backend = "pandas"
                print(f"Loaded {len(self.data)} data points")

                # If loading physics_data.csv, try to load control_data.csv for
                # MPC metrics
                if self.csv_path.name == "physics_data.csv":
                    control_path = self.csv_path.with_name("control_data.csv")
                    if control_path.exists():
                        try:
                            self.control_data = pd.read_csv(control_path)
                            print(
                                f"Loaded sibling control data: {len(self.control_data)} points"
                            )
                        except Exception as e:
                            print(f"Failed to load sibling control data: {e}")
                            self.control_data = None
                    else:
                        print("Sibling control_data.csv not found.")
                        self.control_data = None

                # Pre-calculate derived metrics for animation
                self._detect_timestep()  # Ensure dt is set
                if self.dt is not None:
                    # Linear Speed
                    if (
                        "Current_VX" in self.data.columns
                        and "Current_VY" in self.data.columns
                    ):
                        self.data["Linear_Speed"] = np.sqrt(
                            self.data["Current_VX"] ** 2 + self.data["Current_VY"] ** 2
                        )
                    else:
                        self.data["Linear_Speed"] = 0.0

                    # Accumulated Thruster Usage (s) - Sum of all valve states
                    # Use physics_data Thruster_X_Val columns for accurate PWM
                    # tracking. This matches the thruster_usage.png calculation.
                    has_valve_data = "Thruster_1_Val" in self.data.columns
                    if has_valve_data:
                        thruster_ids = sorted(
                            {
                                int(col.split("_")[1])
                                for col in self.data.columns
                                if col.startswith("Thruster_") and col.endswith("_Val")
                            }
                        )
                        valve_sum_per_step = np.zeros(len(self.data))
                        for tid in thruster_ids:
                            col_name = f"Thruster_{tid}_Val"
                            if col_name in self.data.columns:
                                vals = self.data[col_name].values
                                try:
                                    vals = np.array([float(x) for x in vals])
                                except (ValueError, TypeError):
                                    vals = np.zeros(len(vals), dtype=float)
                                valve_sum_per_step += vals

                        # Cumulative sum of valve usage * physics dt
                        physics_dt = float(self.dt)
                        cumulative_usage = np.cumsum(valve_sum_per_step) * physics_dt
                        self.data["Accumulated_Usage_S"] = cumulative_usage

                        # Also store on control_data by time-matching
                        if (
                            self.control_data is not None
                            and "Control_Time" in self.control_data.columns
                        ):
                            # For each control row, find closest physics row
                            ctrl_times = self.control_data["Control_Time"].values
                            phys_times = self.data["Time"].values
                            ctrl_usage = []
                            for ct in ctrl_times:
                                idx = np.searchsorted(phys_times, ct, side="right") - 1
                                idx = max(0, min(idx, len(cumulative_usage) - 1))
                                ctrl_usage.append(cumulative_usage[idx])
                            self.control_data["Accumulated_Usage_S"] = ctrl_usage

                        print(
                            f"Calculated Max Usage from Control Data: "
                            f"{cumulative_usage[-1]:.2f} s"
                        )
                    else:
                        print(
                            "Could not calculate usage: Thruster_X_Val columns missing"
                        )

            else:
                print(f"Loading CSV data (csv backend) from: {self.csv_path}")
                self._load_csv_data_csvmodule()
                self._data_backend = "csv"
                print(f"Loaded {self._get_len()} data points")

            # Validate required columns
            required_cols = [
                "Current_X",
                "Current_Y",
                "Current_Yaw",
                "Reference_X",
                "Reference_Y",
                "Reference_Yaw",
                "Command_Vector",
            ]
            if self._data_backend == "pandas" and self.data is not None:
                cols = self.data.columns
            elif self._col_data is not None:
                cols = self._col_data.keys()
            else:
                cols = []
            missing_cols = [col for col in required_cols if col not in cols]
            if missing_cols:
                raise ValueError(f"Missing required columns: {missing_cols}")

            self._detect_timestep()

            # Determine dt again if not set (redundant but safe)
            if self.dt is None:
                self._detect_timestep()

            print("CSV data validation successful")
            if self.dt is not None:
                actual_time = None
                if self._data_backend == "pandas" and self.data is not None:
                    if "Time" in self.data.columns and len(self.data) > 0:
                        try:
                            actual_time = float(self.data["Time"].iloc[-1])
                        except Exception:
                            actual_time = None
                elif self._col_data is not None and "Time" in self._col_data:
                    try:
                        actual_time = float(self._col("Time")[-1])
                    except Exception:
                        actual_time = None

                if actual_time is not None:
                    print(f"Time range: 0.0s to {actual_time:.1f}s")
                else:
                    print(
                        f"Time range: 0.0s to {self._get_len() * float(self.dt):.1f}s"
                    )
                print(f"Detected timestep (dt): {self.dt:.3f}s")
            if self.fps is not None:
                print(f"Calculated frame rate: {self.fps:.1f} FPS")

        except Exception as e:
            print(f"Error loading CSV data: {e}")
            raise

    def _detect_timestep(self) -> None:
        """Auto-detect timestep from data and calculate frame rate."""
        if self._data_backend == "pandas" and self.data is not None:
            cols = self.data.columns
        elif self._col_data is not None:
            cols = self._col_data.keys()
        else:
            cols = []
        if "CONTROL_DT" in cols:
            if self._data_backend == "pandas" and self.data is not None:
                val = self.data["CONTROL_DT"].iloc[0]
            else:
                val = self._col("CONTROL_DT")[0]
            self.dt = float(val)
            print(f" Detected timestep from CONTROL_DT: {self.dt}s")

        elif "Time" in cols and self._get_len() > 1:
            try:
                # Calculate average time difference
                time_diffs = []
                for i in range(1, min(10, self._get_len())):
                    curr = float(self._row(i)["Time"])
                    prev = float(self._row(i - 1)["Time"])
                    time_diffs.append(curr - prev)

                if time_diffs:
                    self.dt = float(np.mean(time_diffs))
                    print(f" Detected timestep from Time column: {self.dt:.4f}s")
                else:
                    self.dt = 0.005  # Fallback
            except Exception:
                self.dt = 0.005

        elif "Actual_Time_Interval" in cols and self._get_len() > 1:
            # Use average of actual time intervals
            intervals = self._col("Actual_Time_Interval")
            valid_intervals = intervals[intervals > 0]  # Filter out zero values
            if len(valid_intervals) > 0:
                self.dt = float(np.mean(valid_intervals))
                print(f" Detected timestep from Actual_Time_Interval: {self.dt:.3f}s")
            else:
                self.dt = 0.25  # Fallback
                print(f"Using fallback timestep: {self.dt}s")

        elif "Step" in cols and self._get_len() > 1:
            if "Control_Time" in cols:
                control_time_col = self._col("Control_Time")
                total_time = (
                    float(control_time_col[-1]) if len(control_time_col) > 0 else 0.0
                )
                total_steps = self._get_len() - 1
                if total_steps > 0:
                    self.dt = total_time / total_steps
                    print(f" Calculated timestep from time: {self.dt:.3f}s")
                else:
                    self.dt = 0.25
                    print(f"Using fallback timestep: {self.dt}s")
            else:
                self.dt = 0.25
                print(f"Using default timestep assumption: {self.dt}s")

        elif "MPC_Start_Time" in cols and self._get_len() > 1:
            try:
                # Calculate average time difference between consecutive rows
                time_diffs = []
                for i in range(1, min(10, self._get_len())):  # Sample first 10 rows
                    curr_time_raw = self._row(i)["MPC_Start_Time"]
                    prev_time_raw = self._row(i - 1)["MPC_Start_Time"]
                    if curr_time_raw is not None and prev_time_raw is not None:
                        curr_time = float(curr_time_raw)
                        prev_time = float(prev_time_raw)
                        time_diff = curr_time - prev_time
                        if 0.01 <= time_diff <= 10.0:  # Reasonable timestep range
                            time_diffs.append(time_diff)

                if time_diffs:
                    self.dt = float(np.mean(time_diffs))
                else:
                    self.dt = 0.25  # Default fallback
            except Exception:
                self.dt = 0.25  # Default fallback
        else:
            # Default timestep
            self.dt = 0.25

        # This ensures each frame represents one timestep
        assert self.dt is not None and self.dt > 0, "dt must be positive"

        ideal_fps = 1.0 / self.dt
        TARGET_PLAYBACK_FPS = 60.0

        if ideal_fps > TARGET_PLAYBACK_FPS:
            # Calculate integer step size (frame skipping)
            step_size = max(1, int(round(ideal_fps / TARGET_PLAYBACK_FPS)))

            self.speedup_factor = float(step_size)

            # adjust video FPS to match the effective sampling rate
            # effective_fps = ideal_fps / step_size
            self.fps = ideal_fps / step_size

            print(
                f"Optimization: Simulation runs at {ideal_fps:.1f}Hz. "
                f"Rendering every {step_size}th frame. "
                f"Video FPS set to {self.fps:.2f} for Real-Time playback."
            )
        else:
            self.speedup_factor = 1.0
            self.fps = ideal_fps

    @staticmethod
    def parse_command_vector(command_str: Any) -> np.ndarray:
        """Parse command vector string to numpy array.

        Args:
            command_str: String representation of command vector (or any type)

        Returns:
            numpy array of thruster commands
        """
        try:
            # Handle None or empty
            if command_str is None or command_str == "":
                return np.zeros(THRUSTER_COUNT)

            # Convert to string if not already
            command_str = str(command_str)

            # Remove brackets and split
            command_str = command_str.strip("[]")
            values = [float(x.strip()) for x in command_str.split(",")]
            return np.array(values)
        except Exception:
            return np.zeros(THRUSTER_COUNT)

    def get_active_thrusters(self, command_vector: np.ndarray) -> list:
        """Get list of active thruster IDs from command vector.

        Args:
            command_vector: Array of thruster commands

        Returns:
            List of active thruster IDs (1..N)
        """
        return [i + 1 for i, cmd in enumerate(command_vector) if cmd > 0.5]

    def setup_plot(self) -> None:
        """Initialize matplotlib figure and axes."""
        # Create figure with subplots
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.suptitle(self.animation_title, fontsize=16)

        # 3D Main Axis
        self.ax_main = self.fig.add_subplot(1, 3, (1, 2), projection="3d")
        self.ax_main.set_xlim(-3, 3)
        self.ax_main.set_ylim(-3, 3)
        self.ax_main.set_zlim(-3, 3)
        self.ax_main.set_xlabel("X Position (m)")
        self.ax_main.set_ylabel("Y Position (m)")
        self.ax_main.set_zlabel("Z Position (m)")
        self.ax_main.set_title(self.trajectory_title)

        self.ax_info = plt.subplot2grid((1, 3), (0, 2))
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")
        self.ax_info.set_title("System Info", fontsize=12)

        plt.tight_layout()

    def draw_satellite(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        active_thrusters: list,
    ) -> None:
        """Draw satellite at given position and orientation (3D)."""
        assert self.ax_main is not None, "ax_main must be initialized"

        # Prepare rotation matrix (Rz * Ry * Rx)
        cos_r = np.cos(roll)
        sin_r = np.sin(roll)
        cos_p = np.cos(pitch)
        sin_p = np.sin(pitch)
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        rotation_matrix = np.array(
            [
                [
                    cos_y * cos_p,
                    cos_y * sin_p * sin_r - sin_y * cos_r,
                    cos_y * sin_p * cos_r + sin_y * sin_r,
                ],
                [
                    sin_y * cos_p,
                    sin_y * sin_p * sin_r + cos_y * cos_r,
                    sin_y * sin_p * cos_r - cos_y * sin_r,
                ],
                [-sin_p, cos_p * sin_r, cos_p * cos_r],
            ]
        )

        radius = self.satellite_size / 2

        # Check shape from config.
        shape = "sphere"
        if self.app_config and self.app_config.physics:
            if hasattr(self.app_config.physics, "satellite_shape"):
                shape = self.app_config.physics.satellite_shape

        if shape == "cube":
            # Draw Cube
            # Create a cube of points
            r = [-radius, radius]
            X, Y = np.meshgrid(r, r)
            # Define 6 faces
            # Top/Bottom
            faces = []
            faces.append((X, Y, np.full_like(X, radius)))  # Top (+Z)
            faces.append((X, Y, np.full_like(X, -radius)))  # Bottom (-Z)
            # Left/Right
            faces.append((X, np.full_like(X, radius), Y))  # Right (+Y)
            faces.append((X, np.full_like(X, -radius), Y))  # Left (-Y)
            # Front/Back
            faces.append((np.full_like(X, radius), X, Y))  # Front (+X)
            faces.append((np.full_like(X, -radius), X, Y))  # Back (-X)

            for xx, yy, zz in faces:
                # Rotate
                points = np.stack([xx.ravel(), yy.ravel(), zz.ravel()], axis=1)
                rotated = points @ rotation_matrix.T
                xs = rotated[:, 0].reshape(xx.shape) + x
                ys = rotated[:, 1].reshape(yy.shape) + y
                zs = rotated[:, 2].reshape(zz.shape) + z

                self.ax_main.plot_surface(
                    xs,
                    ys,
                    zs,
                    color=self.satellite_color,
                    alpha=0.6,
                    linewidth=0.5,
                    edgecolor="k",
                    antialiased=True,
                    shade=True,
                )
        else:
            # Draw Sphere (Default)
            u = np.linspace(0, 2 * np.pi, 24)
            v = np.linspace(0, np.pi, 12)
            uu, vv = np.meshgrid(u, v)
            xs = radius * np.cos(uu) * np.sin(vv)
            ys = radius * np.sin(uu) * np.sin(vv)
            zs = radius * np.cos(vv)

            points = np.stack([xs.ravel(), ys.ravel(), zs.ravel()], axis=1)
            rotated = points @ rotation_matrix.T
            xs = rotated[:, 0].reshape(xs.shape) + x
            ys = rotated[:, 1].reshape(ys.shape) + y
            zs = rotated[:, 2].reshape(zs.shape) + z

            self.ax_main.plot_surface(
                xs,
                ys,
                zs,
                color=self.satellite_color,
                alpha=0.5,
                linewidth=0,
                antialiased=True,
            )

        # Draw thrusters
        for thruster_id, pos in self.thrusters.items():
            tx, ty, tz = pos[0], pos[1], pos[2] if len(pos) > 2 else 0.0

            # Rotate thruster position
            thruster_pos = np.array([tx, ty, tz]) @ rotation_matrix.T
            thruster_x = x + thruster_pos[0]
            thruster_y = y + thruster_pos[1]
            thruster_z = z + thruster_pos[2]

            # Color and size based on activity
            if thruster_id in active_thrusters:
                color = "red"
                size = 80
                marker = "o"
                alpha = 1.0
            else:
                color = "gray"
                size = 20
                marker = "o"
                alpha = 0.3

            self.ax_main.scatter(
                thruster_x,
                thruster_y,
                thruster_z,
                c=color,
                s=size,
                marker=marker,
                alpha=alpha,
                edgecolors="black",
                linewidth=0.5,
                depthshade=True,
            )

        # Draw orientation arrow (body X-axis)
        arrow_length = self.satellite_size * 1.5
        body_x = rotation_matrix @ np.array([1.0, 0.0, 0.0])
        arrow_end_x = x + arrow_length * body_x[0]
        arrow_end_y = y + arrow_length * body_x[1]
        arrow_end_z = z + arrow_length * body_x[2]

        self.ax_main.plot(
            [x, arrow_end_x],
            [y, arrow_end_y],
            [z, arrow_end_z],
            color="green",
            linewidth=2,
        )

    def draw_reference(
        self,
        reference_x: float,
        reference_y: float,
        reference_z: float,
        reference_roll: float,
        reference_pitch: float,
        reference_yaw: float,
    ) -> None:
        """Draw reference position and orientation (3D)."""
        assert self.ax_main is not None, "ax_main must be initialized"

        self.ax_main.scatter(
            reference_x,
            reference_y,
            reference_z,
            c=self.reference_color,
            s=200,
            marker="x",
            linewidth=4,
            label="Reference",
        )

        # Reference sphere (wireframe visual)
        # Simple point for now, maybe add small circle in XY plane at Z
        # Matplotlib 3D doesn't support 'Circle' patch easily in 3D space, need plot_surface or plot(xs, ys, zs)
        theta = np.linspace(0, 2 * np.pi, 20)
        cx = reference_x + 0.1 * np.cos(theta)
        cy = reference_y + 0.1 * np.sin(theta)
        cz = np.full_like(cx, reference_z)
        self.ax_main.plot(
            cx, cy, cz, color=self.reference_color, alpha=0.5, linestyle="--"
        )

        # Reference orientation arrow (body X-axis)
        arrow_length = self.satellite_size * 0.6
        cos_r = np.cos(reference_roll)
        sin_r = np.sin(reference_roll)
        cos_p = np.cos(reference_pitch)
        sin_p = np.sin(reference_pitch)
        cos_y = np.cos(reference_yaw)
        sin_y = np.sin(reference_yaw)
        rotation_matrix = np.array(
            [
                [
                    cos_y * cos_p,
                    cos_y * sin_p * sin_r - sin_y * cos_r,
                    cos_y * sin_p * cos_r + sin_y * sin_r,
                ],
                [
                    sin_y * cos_p,
                    sin_y * sin_p * sin_r + cos_y * cos_r,
                    sin_y * sin_p * cos_r - cos_y * sin_r,
                ],
                [-sin_p, cos_p * sin_r, cos_p * cos_r],
            ]
        )
        body_x = rotation_matrix @ np.array([1.0, 0.0, 0.0])
        arrow_end_x = reference_x + arrow_length * body_x[0]
        arrow_end_y = reference_y + arrow_length * body_x[1]
        arrow_end_z = reference_z + arrow_length * body_x[2]
        self.ax_main.plot(
            [reference_x, arrow_end_x],
            [reference_y, arrow_end_y],
            [reference_z, arrow_end_z],
            color=self.reference_color,
            alpha=0.8,
            linewidth=2,
        )

    def draw_trajectory(
        self, trajectory_x: list, trajectory_y: list, trajectory_z: list
    ) -> None:
        """Draw satellite trajectory (3D)."""
        assert self.ax_main is not None, "ax_main must be initialized"

        # Ensure lengths match
        min_len = min(len(trajectory_x), len(trajectory_y), len(trajectory_z))
        if min_len > 1:
            self.ax_main.plot(
                trajectory_x[:min_len],
                trajectory_y[:min_len],
                trajectory_z[:min_len],
                color=self.trajectory_color,
                linewidth=2,
                alpha=0.8,
                linestyle="-",
                label="Trajectory",
            )

    def update_info_panel(self, step: int, current_data: Any) -> None:
        """Update information panel with current data using professional styling.

        Args:
            step: Current step
            current_data: Current row of data
        """
        assert self.ax_info is not None, "ax_info must be initialized"
        assert self.dt is not None, "dt must be set"

        self.ax_info.clear()
        self.ax_info.set_xlim(0, 1)
        self.ax_info.set_ylim(0, 1)
        self.ax_info.axis("off")

        # Professional Title
        self.ax_info.text(
            0.05,
            0.95,
            "System Telemetry",
            fontsize=14,
            weight="bold",
            color=PlotStyle.COLOR_PRIMARY,
            fontfamily=getattr(PlotStyle, "FONT_FAMILY", "serif"),
        )

        time = step * float(self.dt)

        # Determine Mission Phase and Metrics from Control Data
        mission_phase = ""
        mpc_solve_time = 0.0
        accumulated_usage_s = 0.0
        if hasattr(self, "control_data") and self.control_data is not None:
            import pandas as pd

            # Find closest control timestamp
            if (
                "Control_Time" in self.control_data.columns
                and "Mission_Phase" in self.control_data.columns
            ):
                # Ensure sorted
                idx = (
                    self.control_data["Control_Time"].searchsorted(time, side="right")
                    - 1
                )
                idx = max(0, min(idx, len(self.control_data) - 1))
                row = self.control_data.iloc[idx]

                # Phase
                phase_val = row["Mission_Phase"]
                if phase_val is not None and str(phase_val).lower() != "nan":
                    mission_phase = str(phase_val)

                # Solver Time
                if "MPC_Solve_Time" in row and pd.notna(row["MPC_Solve_Time"]):
                    mpc_solve_time = float(row["MPC_Solve_Time"]) * 1000.0
                elif "MPC_Computation_Time" in row:
                    mpc_solve_time = (
                        float(row["MPC_Computation_Time"]) * 1000.0
                    )  # to ms

                # Accumulated Thruster Usage (calculated on control_data)
                if "Accumulated_Usage_S" in row:
                    accumulated_usage_s = float(row["Accumulated_Usage_S"])

        # Display Logic: Refined Content

        # Group 1: Time & Phase (High Level)
        header_lines = [f"Time: {time:.3f} s"]
        if mission_phase:
            header_lines.append(f"Phase: {mission_phase}")

        # Group 2: STATE (Blue)
        state_lines = [
            "STATE",
            f"  X:   {current_data['Current_X']:>7.3f} m",
            f"  Y:   {current_data['Current_Y']:>7.3f} m",
            f"  Z:   {current_data.get('Current_Z', 0.0):>7.3f} m",
            f"  Roll:  {np.degrees(current_data.get('Current_Roll', 0.0)):>6.1f}°",
            f"  Pitch: {np.degrees(current_data.get('Current_Pitch', 0.0)):>6.1f}°",
            f"  Yaw:   {np.degrees(current_data.get('Current_Yaw', 0.0)):>6.1f}°",
        ]

        # Group 3: DYNAMICS (Blue/Black) - Speed info
        lin_speed = current_data.get("Linear_Speed", 0.0)
        ang_speed = abs(np.degrees(current_data.get("Current_Angular_Vel", 0.0)))
        dynamics_lines = [
            "DYNAMICS",
            f"  Speed: {lin_speed:.3f} m/s",
            f"  Ang V: {ang_speed:.1f} °/s",
        ]

        # Group 4: ERRORS (Red)
        err_x = abs(current_data["Error_X"])
        err_y = abs(current_data["Error_Y"])
        err_z = abs(current_data.get("Error_Z", 0.0))
        err_ang = abs(np.degrees(current_data.get("Error_Angle_Rad", 0.0)))

        error_lines = [
            "ERRORS",
            f"  Err X:   {err_x:.3f} m",
            f"  Err Y:   {err_y:.3f} m",
            f"  Err Z:   {err_z:.3f} m",
            f"  Att Err:  {err_ang:.1f}°",
        ]

        # Group 5: SYSTEM (Black) - Usage & Perf
        system_lines = [
            "SYSTEM",
            f"  Total Thruster Usage: {accumulated_usage_s:.1f} s",
            f"  Solver: {mpc_solve_time:.1f} ms",
        ]

        # Define data groups with semantic colors
        # Format: (List of strings, Color, bold_header_index)
        groups = [
            (header_lines, PlotStyle.COLOR_PRIMARY, None),
            (state_lines, PlotStyle.COLOR_SIGNAL_POS, 0),  # Blue
            (
                dynamics_lines,
                PlotStyle.COLOR_SIGNAL_POS,
                0,
            ),  # Blue (Grouped by association)
            (error_lines, PlotStyle.COLOR_REFERENCE, 0),  # Red
            (system_lines, PlotStyle.COLOR_PRIMARY, 0),  # Black
        ]
        y_pos = 0.85
        line_height = 0.035
        group_spacing = 0.02

        font_family = getattr(PlotStyle, "FONT_FAMILY", "serif")

        for lines, color, bold_idx in groups:
            for i, line in enumerate(lines):
                weight = (
                    "bold" if (bold_idx is not None and i == bold_idx) else "normal"
                )
                size = 11 if weight == "bold" else 10

                # Special handling for Phase to make it pop?
                # Keeping simple for consistency first.

                current_color = color
                # Hack: semantic coloring within the group for "Error" lines?
                # The prompt asked for "Green Block" for Angle & Errors.
                # If we want errors red, we'd need line-by-line control.
                # For now, sticking to the section color logic to avoid complex spaghetti.

                self.ax_info.text(
                    0.05,
                    y_pos,
                    line,
                    fontsize=size,
                    weight=weight,
                    color=current_color,
                    verticalalignment="top",
                    fontfamily=font_family,
                )
                y_pos -= line_height
            y_pos -= group_spacing

    def animate_frame(self, frame: int) -> list[Any]:
        """Animation update function for each frame (3D)."""
        assert self.ax_main is not None, "ax_main must be initialized"
        assert self.dt is not None, "dt must be set"

        # Clear main plot
        self.ax_main.clear()
        self.ax_main.set_xlim(-3, 3)
        self.ax_main.set_ylim(-3, 3)
        self.ax_main.set_zlim(-3, 3)
        self.ax_main.set_xlabel("X (m)")
        self.ax_main.set_ylabel("Y (m)")
        self.ax_main.set_zlabel("Z (m)")
        self.ax_main.set_title(self.frame_title_template.format(frame))

        # Consistent viewing angle
        self.ax_main.view_init(elev=30.0, azim=45)

        # Get current data
        step = min(int(frame * self.speedup_factor), self._get_len() - 1)
        current_data = self._row(step)

        # Parse command vector and get active thrusters
        command_vector = self.parse_command_vector(current_data["Command_Vector"])
        active_thrusters = self.get_active_thrusters(command_vector)

        # Type-safe extraction
        reference_x = float(current_data.get("Reference_X", 0.0) or 0.0)
        reference_y = float(current_data.get("Reference_Y", 0.0) or 0.0)
        reference_z = float(
            current_data.get("Reference_Z", 0.0) or 0.0
        )  # Assume Reference_Z exists or default 0
        reference_roll = float(current_data.get("Reference_Roll", 0.0) or 0.0)
        reference_pitch = float(current_data.get("Reference_Pitch", 0.0) or 0.0)
        reference_yaw = float(current_data.get("Reference_Yaw", 0.0) or 0.0)

        self.draw_reference(
            reference_x,
            reference_y,
            reference_z,
            reference_roll,
            reference_pitch,
            reference_yaw,
        )

        # Draw trajectory
        traj_x = self._col("Current_X")[: step + 1].tolist()
        traj_y = self._col("Current_Y")[: step + 1].tolist()
        # Try to get Current_Z col, else zeros
        if "Current_Z" in current_data:
            traj_z = self._col("Current_Z")[: step + 1].tolist()
        else:
            traj_z = [0.0] * len(traj_x)
        self.draw_trajectory(traj_x, traj_y, traj_z)

        # Draw satellite
        curr_x = float(current_data.get("Current_X", 0.0) or 0.0)
        curr_y = float(current_data.get("Current_Y", 0.0) or 0.0)
        curr_z = float(current_data.get("Current_Z", 0.0) or 0.0)
        curr_roll = float(current_data.get("Current_Roll", 0.0) or 0.0)
        curr_pitch = float(current_data.get("Current_Pitch", 0.0) or 0.0)
        curr_yaw = float(current_data.get("Current_Yaw", 0.0) or 0.0)

        self.draw_satellite(
            curr_x,
            curr_y,
            curr_z,
            curr_roll,
            curr_pitch,
            curr_yaw,
            active_thrusters,
        )

        # Add legend
        self.ax_main.legend(loc="upper right", fontsize=9)

        # Update info panel
        self.update_info_panel(step, current_data)

        return []

    def generate_animation(self, output_filename: str | None = None) -> None:
        """Generate and save the MP4 animation using VideoRenderer.

        Args:
            output_filename: Name of output MP4 file (optional)
        """
        assert self.fps is not None, (
            "FPS must be calculated before generating animation"
        )
        assert self.dt is not None, "dt must be set before generating animation"
        assert self.output_dir is not None, "Output directory must be set"

        if output_filename is None:
            output_filename = f"{self.plot_prefix}_animation.mp4"

        video_renderer = self._get_video_renderer()
        video_renderer.generate_animation(output_filename)

    def _get_plot_generator(self):
        """Get or create PlotGenerator instance (lazy initialization)."""
        if self._plot_generator is None:
            from visualization.plot_generator import PlotGenerator

            assert self.dt is not None, "dt must be set before generating plots"
            self._plot_generator = PlotGenerator(
                data_accessor=self,
                dt=self.dt,
                system_title=self.system_title,
                app_config=self.app_config,
            )
        return self._plot_generator

    def _get_video_renderer(self):
        """Get or create VideoRenderer instance (lazy initialization)."""
        if self._video_renderer is None:
            from visualization.video_renderer import VideoRenderer

            assert self.dt is not None, "dt must be set before generating animation"
            assert self.fps is not None, "fps must be set before generating animation"
            assert self.output_dir is not None, (
                "output_dir must be set before generating animation"
            )

            self._video_renderer = VideoRenderer(
                data_accessor=self,
                dt=self.dt,
                fps=self.fps,
                output_dir=self.output_dir,
                system_title=self.system_title,
                speedup_factor=self.speedup_factor,
                satellite_size=self.satellite_size,
                satellite_color=self.satellite_color,
                reference_color=self.reference_color,
                trajectory_color=self.trajectory_color,
                thrusters=self.thrusters,
                frame_title_template=self.frame_title_template,
            )
        return self._video_renderer

    def generate_performance_plots(self) -> None:
        """Generate performance analysis plots."""
        assert self.dt is not None, "dt must be set before generating plots"
        assert self.output_dir is not None, (
            "output_dir must be set before generating plots"
        )

        plots_dir = self.output_dir / "Plots"
        plot_generator = self._get_plot_generator()
        plot_generator.generate_all_plots(plots_dir)

    # --- CSV backend helpers ---
    def _load_csv_data_csvmodule(self) -> None:
        """Load CSV data using csv module backend."""
        assert self.csv_path is not None, "CSV path must be set"

        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            rows = []
            for r in reader:
                rows.append(r)
        # Build column-wise data with type conversion
        cols = reader.fieldnames or []
        col_data: dict[str, list[Any]] = {c: [] for c in cols}
        for r in rows:
            for c in cols:
                v = r.get(c, "")
                if v is None:
                    col_data[c].append("")
                    continue
                # Convert booleans and numbers where applicable
                lv = v.strip()
                if lv.lower() in ("true", "false"):
                    col_data[c].append(lv.lower() == "true")
                else:
                    try:
                        col_data[c].append(float(lv))
                    except Exception:
                        col_data[c].append(v)
        # Store
        self._rows = rows
        self._col_data = {k: np.array(v) for k, v in col_data.items()}
        self.data = self  # allow attribute access in unchanged code paths

    def _get_len(self) -> int:
        """Get length of data."""
        if self._data_backend == "pandas":
            return len(self.data) if self.data is not None else 0
        if self._rows is not None:
            return len(self._rows)
        if self._col_data is not None and len(self._col_data) > 0:
            # Get length of first column
            first_key = next(iter(self._col_data))
            return len(self._col_data[first_key])
        return 0

    def _col(self, name: str) -> np.ndarray:
        """Get column data."""
        if self._data_backend == "pandas" and self.data is not None:
            try:
                return (
                    self.data[name].values
                    if hasattr(self.data[name], "values")
                    else np.array(self.data[name])
                )
            except KeyError:
                return np.array([])
        return (
            self._col_data.get(name, np.array([]))
            if self._col_data is not None
            else np.array([])
        )

    def _row(self, idx: int) -> dict[str, Any]:
        """Get row data."""
        if self._data_backend == "pandas" and self.data is not None:
            row_data: dict[str, Any] = dict(self.data.iloc[idx])
            return row_data
        # Build a dict using typed column arrays so consumers see floats/bools
        if self._col_data is not None:
            return {
                k: (self._col_data[k][idx] if k in self._col_data else None)
                for k in self._col_data.keys()
            }
        return {}


def select_data_file_interactive() -> tuple:
    """Interactive file browser to select a CSV data file."""
    print("\n" + "=" * 60)
    print("   VISUALIZATION DATA FILE SELECTOR")
    print("=" * 60)

    search_roots = []
    for root in (SIMULATION_DATA_ROOT, LEGACY_SIMULATION_DATA_ROOT):
        if root.exists() and root.resolve() not in search_roots:
            search_roots.append(root.resolve())

    if not search_roots:
        print(
            " Error: data directory not found at "
            f"{SIMULATION_DATA_ROOT.resolve()} (legacy {LEGACY_SIMULATION_DATA_ROOT.resolve()})"
        )
        return None, None

    sim_csvs: list[Path] = []
    for root in search_roots:
        sim_csvs.extend(list(root.rglob("simulation_data.csv")))

    all_csvs = []

    # Add simulation data
    for csv_path in sorted(sim_csvs, key=lambda p: p.stat().st_mtime, reverse=True):
        timestamp_dir = csv_path.parent.name
        all_csvs.append(("Simulation", timestamp_dir, csv_path))

    if not all_csvs:
        print(" No data files found in data/simulation_data")
        return None, None

    # Display available files
    print(f"\nFound {len(all_csvs)} data file(s):\n")
    for idx, (mode, timestamp, csv_path) in enumerate(all_csvs, 1):
        file_size = csv_path.stat().st_size / 1024  # KB
        mod_time = datetime.fromtimestamp(csv_path.stat().st_mtime).strftime(
            "%Y-%m-%d %H:%M:%S"
        )
        print(f"{idx:2d}. [{mode:10s}] {timestamp} ({file_size:.1f} KB) - {mod_time}")

    # Get user selection
    print("\n" + "-" * 60)
    while True:
        try:
            choice = input(f"Select file (1-{len(all_csvs)}) or 'q' to quit: ").strip()
            if choice.lower() == "q":
                print("Cancelled.")
                return None, None

            idx = int(choice) - 1
            if 0 <= idx < len(all_csvs):
                mode, timestamp, csv_path = all_csvs[idx]
                print(f"\n Selected: {csv_path}")
                return str(csv_path.parent), mode.lower()
            else:
                print(f"Invalid selection. Please enter 1-{len(all_csvs)}")
        except ValueError:
            print("Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            print("\nCancelled.")
            return None, None


def main() -> int:
    """Main function for standalone usage."""
    import argparse

    parser = argparse.ArgumentParser(description="Generate MPC visualization")
    parser.add_argument(
        "--data-dir",
        type=str,
        help="Data directory (optional, uses defaults based on mode)",
    )
    parser.add_argument(
        "--interactive", action="store_true", help="Interactive data selection"
    )
    parser.add_argument(
        "--plots-only",
        action="store_true",
        help="Generate only plots, skip animation",
    )

    args = parser.parse_args()

    # If no arguments provided, use interactive mode by default
    if args.data_dir is None and not args.interactive:
        print("No arguments provided - using interactive file selector")
        args.interactive = True

    # Interactive file selection
    if args.interactive or args.data_dir is None:
        data_dir, mode = select_data_file_interactive()
        if data_dir is None:
            return 1
        args.data_dir = data_dir

    # Default to canonical simulation data root if still not set.
    if args.data_dir is None:
        args.data_dir = str(SIMULATION_DATA_ROOT)
    else:
        args.data_dir = str(resolve_repo_path(args.data_dir))

    try:
        # Create visualizer
        viz = UnifiedVisualizationGenerator(
            data_directory=args.data_dir,
            interactive=False,
        )

        # Generate plots
        viz.generate_performance_plots()

        # Generate animation unless plots-only
        if not args.plots_only:
            viz.generate_animation()

    except Exception as e:
        print(f" Error: {e}")
        import traceback

        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
