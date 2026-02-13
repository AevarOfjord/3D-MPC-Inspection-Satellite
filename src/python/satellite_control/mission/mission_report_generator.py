"""
Mission Report Generator for Satellite Control System

Generates comprehensive post-mission analysis reports with detailed metrics
and statistics.
Provides formatted text reports for documentation and performance review.

Report sections:
- Mission configuration and parameters
- Performance metrics and analysis
- Path tracking results with error statistics
- Control system performance (thruster usage, control effort)
- MPC timing statistics and computational performance
- Collision avoidance and safety events

Output formats:
- Console display with formatted text
- Text file export for archival
- Summary statistics for comparison
"""

import csv
import json
import logging
from collections.abc import Callable
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from satellite_control.config.constants import Constants
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.utils.orientation_utils import quat_wxyz_to_euler_xyz

logger = logging.getLogger(__name__)


class MissionReportGenerator:
    """
    Generates mission summary reports with configuration and metrics.

    Creates detailed text reports including:
    - Mission type and configuration
    - Controller parameters
    - Physical parameters
    - Performance metrics (position, orientation, control effort)
    - MPC timing analysis
    - Path completion statistics
    """

    def __init__(self, config: SimulationConfig):
        """
        Initialize report generator.

        Args:
            config: SimulationConfig object.
        """
        self.config = config
        self.app_config = config.app_config
        self.mission_state = config.mission_state

    @staticmethod
    def _format_euler_deg(angle: tuple[float, float, float]) -> str:
        roll, pitch, yaw = np.degrees(angle)
        return f"roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°"

    def generate_report(
        self,
        output_path: Path,
        state_history: list[np.ndarray],
        reference_state: np.ndarray,
        control_time: float,
        mpc_solve_times: list[float],
        control_history: list[np.ndarray],
        path_complete_time: float | None,
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        check_path_complete_func: Callable[..., Any],
        test_mode: str = "SIMULATION",
    ) -> None:
        """
        Generate comprehensive mission summary report.

        Args:
            output_path: Path to save the report
            state_history: List of state vectors [x,y,z, qw,qx,qy,qz, vx,vy,vz, wx,wy,wz]
            reference_state: Path reference state vector
            control_time: Total mission duration in seconds
            mpc_solve_times: List of MPC solve times
            control_history: List of control vectors (thruster commands)
            path_complete_time: Time when path was first completed (None if never)
            position_tolerance: Position tolerance threshold
            angle_tolerance: Angle tolerance threshold
            control_update_interval: Control loop update interval
            check_path_complete_func: Function to check if path was completed
            test_mode: Test mode description
        """
        if state_history is None or len(state_history) == 0:
            logger.warning(
                "WARNING: Cannot generate report: No state history available"
            )
            return

        try:
            with open(output_path, "w") as f:
                # Header
                self._write_header(f, output_path, test_mode)

                # Mission Configuration
                self._write_mission_configuration(f, state_history, reference_state)

                # Controller Configuration
                self._write_controller_configuration(f)

                # Physical Parameters
                self._write_physical_parameters(f)

                # All Configuration Parameters (for test comparison)
                self._write_all_config_parameters(f)

                # Performance Results
                self._write_performance_results(
                    f,
                    state_history,
                    reference_state,
                    control_time,
                    mpc_solve_times,
                    control_history,
                    path_complete_time,
                    position_tolerance,
                    angle_tolerance,
                    control_update_interval,
                    check_path_complete_func,
                    output_path.parent,
                )

                # Footer
                f.write("\n" + "=" * 80 + "\n")
                f.write("END OF MISSION SUMMARY\n")
                f.write("=" * 80 + "\n")

            logger.info(f"Mission summary saved to: {output_path}")
            print(f" Mission summary saved: {output_path}")

        except Exception as e:
            logger.error(f"ERROR: Error generating mission report: {e}")

    def _write_header(self, f, output_path: Path, test_mode: str) -> None:
        """Write report header."""
        f.write("=" * 80 + "\n")
        f.write("SATELLITE CONTROL SYSTEM - MISSION SUMMARY & CONFIGURATION\n")
        f.write("=" * 80 + "\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Data Directory: {output_path.parent}\n")
        f.write(f"Test Mode: {test_mode}\n")
        f.write("=" * 80 + "\n\n")

    def _write_mission_configuration(
        self, f, state_history: list[np.ndarray], reference_state: np.ndarray
    ) -> None:
        """Write mission configuration section."""
        f.write("=" * 80 + "\n")
        f.write("MISSION CONFIGURATION - RECREATION DATA\n")
        f.write("=" * 80 + "\n")
        f.write(
            "This section contains all information needed to recreate this exact mission.\n\n"
        )

        initial_state = state_history[0]
        self._write_path_config(f, initial_state, reference_state)

        # Obstacle configuration
        self._write_obstacle_configuration(f)

    def _write_path_config(
        self, f, initial_state: np.ndarray, reference_state: np.ndarray
    ) -> None:
        """Write path-following configuration."""
        f.write("MISSION TYPE: PATH FOLLOWING (MPCC)\n")
        f.write("-" * 50 + "\n")
        f.write("Description: Satellite follows a generated path between endpoints\n\n")

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting Z position:     {initial_state[2]:.3f} m\n")

        q = initial_state[3:7]
        f.write(
            f"  Starting orientation:    {self._format_euler_deg(quat_wxyz_to_euler_xyz(q))}\n\n"
        )

        path = self.mission_state.get_resolved_path_waypoints()
        if path:
            start_pt = path[0]
            end_pt = path[-1]
        else:
            start_pt = initial_state[:3]
            end_pt = reference_state[:3]

        path_length = float(
            self.mission_state.get_resolved_path_length(compute_if_missing=True)
        )

        path_speed = float(
            self.mission_state.path_speed or self.app_config.mpc.path_speed
        )
        hold_end = float(self.mission_state.path_hold_end or 0.0)

        f.write("PATH CONFIGURATION:\n")
        f.write(
            f"  Path Start:              ({start_pt[0]:.3f}, {start_pt[1]:.3f}, {start_pt[2]:.3f}) m\n"
        )
        f.write(
            f"  Path End:                ({end_pt[0]:.3f}, {end_pt[1]:.3f}, {end_pt[2]:.3f}) m\n"
        )
        f.write(f"  Path Length:             {path_length:.3f} m\n")
        f.write(f"  Path Speed:              {path_speed:.3f} m/s\n")
        if hold_end > 0.0:
            f.write(f"  End Hold Time:           {hold_end:.1f} s\n")
        f.write("\n")

    def _write_obstacle_configuration(self, f) -> None:
        """Write obstacle configuration."""
        if self.mission_state.obstacles_enabled and self.mission_state.obstacles:
            f.write("OBSTACLE CONFIGURATION:\n")
            f.write("  Obstacle Avoidance:      ENABLED\n")
            f.write(f"  Number of Obstacles:     {len(self.mission_state.obstacles)}\n")
            for i, obs in enumerate(self.mission_state.obstacles, 1):
                if hasattr(obs, "position"):
                    ox, oy, oz = obs.position
                    orad = obs.radius
                else:
                    ox, oy, oz, orad = obs
                f.write(
                    f"  Obstacle {i}:              ({ox:.3f}, {oy:.3f}, {oz:.3f}) m, radius {orad:.3f} m\n"
                )
            f.write("\n")
        else:
            f.write("OBSTACLE CONFIGURATION:\n")
            f.write("  Obstacle Avoidance:      DISABLED\n\n")

    def _write_controller_configuration(self, f) -> None:
        """Write controller configuration section."""
        f.write("=" * 80 + "\n")
        f.write("CONTROLLER CONFIGURATION\n")
        f.write("=" * 80 + "\n")
        f.write("These parameters affect mission performance and control behavior.\n\n")

        f.write("MPC CONTROLLER PARAMETERS (MPCC):\n")
        f.write("-" * 50 + "\n")
        f.write("  Controller Type:         Linearized MPCC (Path Following)\n")
        f.write(f"  Solver:                  {self.app_config.mpc.solver_type}\n")
        f.write(
            f"  Prediction Horizon:      {self.app_config.mpc.prediction_horizon} steps\n"
        )
        f.write(
            f"  Control Horizon:         {self.app_config.mpc.control_horizon} steps\n"
        )
        f.write(f"  Simulation Timestep:     {self.app_config.simulation.dt:.3f} s\n")
        f.write(
            f"  Control Timestep:        {self.app_config.simulation.control_dt:.3f} s\n"
        )
        f.write(
            f"  Solver Time Limit:       {self.app_config.mpc.solver_time_limit:.3f} s\n\n"
        )

        f.write("COST FUNCTION WEIGHTS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Contour Weight (Q):      {self.app_config.mpc.Q_contour:.1f}\n")
        f.write(f"  Progress Weight (Q):     {self.app_config.mpc.Q_progress:.1f}\n")
        f.write(f"  Smooth Weight (Q):       {self.app_config.mpc.Q_smooth:.1f}\n")
        f.write(
            f"  Axis Align Weight (Q):   {self.app_config.mpc.Q_axis_align:.1f}\n"
        )
        f.write(
            f"  Angular Vel Weight (Q):  {self.app_config.mpc.q_angular_velocity:.1f}\n"
        )
        f.write(f"  Thrust Penalty (R):      {self.app_config.mpc.r_thrust:.3f}\n")
        f.write(f"  RW Torque Penalty (R):   {self.app_config.mpc.r_rw_torque:.3f}\n\n")

        f.write("PATH FOLLOWING SETTINGS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  Path Speed (m/s):        {self.app_config.mpc.path_speed:.3f} m/s\n"
        )

        f.write("\n")

    def _write_physical_parameters(self, f) -> None:
        """Write physical parameters section."""
        f.write("PHYSICAL PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  Total Mass:              {self.app_config.physics.total_mass:.3f} kg\n"
        )
        f.write(
            f"  Moment of Inertia:       {self.app_config.physics.moment_of_inertia:.6f} kg·m²\n"
        )
        f.write(
            f"  Satellite Size:          {self.app_config.physics.satellite_size:.3f} m\n"
        )
        com_x, com_y, com_z = (
            self.app_config.physics.com_offset[0],
            self.app_config.physics.com_offset[1],
            self.app_config.physics.com_offset[2],
        )
        f.write(
            f"  COM Offset:              ({com_x:.6f}, {com_y:.6f}, {com_z:.6f}) m\n\n"
        )

        f.write("THRUSTER FORCES:\n")
        for tid in sorted(self.app_config.physics.thruster_forces.keys()):
            f.write(
                f"  Thruster {tid}:             {self.app_config.physics.thruster_forces[tid]:.6f} N\n"
            )
        f.write("\n")

    def _write_all_config_parameters(self, f) -> None:
        """Write comprehensive all configuration parameters section."""
        f.write("=" * 80 + "\n")
        f.write("ALL CONFIGURATION PARAMETERS (FOR TEST COMPARISON)\n")
        f.write("=" * 80 + "\n")
        f.write(
            "Complete listing of all system parameters for easy test-to-test comparison.\n\n"
        )

        # MPC Parameters
        f.write("MPC PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  MPC_PREDICTION_HORIZON:        {self.app_config.mpc.prediction_horizon}\n"
        )
        f.write(
            f"  MPC_CONTROL_HORIZON:           {self.app_config.mpc.control_horizon}\n"
        )
        f.write(
            f"  MPC_SOLVER_TIME_LIMIT:         {self.app_config.mpc.solver_time_limit:.3f} s\n"
        )
        f.write(f"  MPC_SOLVER_TYPE:               {self.app_config.mpc.solver_type}\n")
        f.write(
            f"  Q_CONTOUR:                     {self.app_config.mpc.Q_contour:.1f}\n"
        )
        f.write(
            f"  Q_PROGRESS:                    {self.app_config.mpc.Q_progress:.1f}\n"
        )
        f.write(f"  Q_LAG:                         {self.app_config.mpc.Q_lag:.1f}\n")
        f.write(
            f"  Q_SMOOTH:                      {self.app_config.mpc.Q_smooth:.1f}\n"
        )
        f.write(
            f"  Q_ATTITUDE:                    {self.app_config.mpc.Q_attitude:.1f}\n"
        )
        f.write(
            f"  Q_AXIS_ALIGN:                  {self.app_config.mpc.Q_axis_align:.1f}\n"
        )
        f.write(
            f"  Q_TERMINAL_POS:                {self.app_config.mpc.Q_terminal_pos:.1f}\n"
        )
        f.write(
            f"  Q_TERMINAL_S:                  {self.app_config.mpc.Q_terminal_s:.1f}\n"
        )
        f.write(
            f"  Q_ANGULAR_VELOCITY:            {self.app_config.mpc.q_angular_velocity:.1f}\n"
        )
        f.write(
            f"  R_THRUST:                      {self.app_config.mpc.r_thrust:.3f}\n"
        )
        f.write(
            f"  R_RW_TORQUE:                   {self.app_config.mpc.r_rw_torque:.3f}\n"
        )
        f.write(
            f"  PATH_SPEED:                   {self.app_config.mpc.path_speed:.3f} m/s\n"
        )
        f.write(
            f"  PROGRESS_TAPER_DISTANCE:      {self.app_config.mpc.progress_taper_distance:.3f} m\n"
        )
        f.write(
            f"  PROGRESS_SLOWDOWN_DISTANCE:   {self.app_config.mpc.progress_slowdown_distance:.3f} m\n"
        )

        f.write(
            f"  ANGLE_TOLERANCE:               {np.degrees(Constants.ANGLE_TOLERANCE):.1f}°\n"
        )
        f.write(
            f"  VELOCITY_TOLERANCE:            {Constants.VELOCITY_TOLERANCE:.3f} m/s\n"
        )
        ang_vel_tol = np.degrees(Constants.ANGULAR_VELOCITY_TOLERANCE)
        f.write(f"  ANGULAR_VELOCITY_TOLERANCE:    {ang_vel_tol:.1f}°/s\n\n")

        # Timing Parameters
        f.write("TIMING PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  SIMULATION_DT:                 {self.app_config.simulation.dt:.3f} s\n"
        )
        f.write(
            f"  CONTROL_DT:                    {self.app_config.simulation.control_dt:.3f} s\n"
        )
        f.write(
            f"  MAX_SIMULATION_TIME:           {self.app_config.simulation.max_duration:.1f} s\n"
        )
        path_hold_end = float(getattr(self.mission_state, "path_hold_end", 0.0) or 0.0)
        f.write(f"  PATH_HOLD_END:                {path_hold_end:.1f} s\n")

        # Physics Parameters
        f.write("PHYSICS PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  TOTAL_MASS:                    {self.app_config.physics.total_mass:.3f} kg\n"
        )
        f.write(
            f"  MOMENT_OF_INERTIA:             {self.app_config.physics.moment_of_inertia:.6f} kg·m²\n"
        )
        f.write(
            f"  SATELLITE_SIZE:                {self.app_config.physics.satellite_size:.3f} m\n"
        )
        lin_damp = self.app_config.physics.damping_linear
        f.write(f"  LINEAR_DAMPING_COEFF:          {lin_damp:.3f} N/(m/s)\n")
        rot_damp = self.app_config.physics.damping_angular
        f.write(f"  ROTATIONAL_DAMPING_COEFF:      {rot_damp:.4f} N·m/(rad/s)\n")
        f.write(
            f"  THRUSTER_VALVE_DELAY:          {self.app_config.physics.thruster_valve_delay * 1000:.1f} ms\n"
        )
        f.write(
            f"  THRUSTER_RAMPUP_TIME:          {self.app_config.physics.thruster_rampup_time * 1000:.1f} ms\n"
        )
        thrust_noise = self.app_config.physics.thrust_force_noise_percent
        f.write(f"  THRUST_FORCE_NOISE_PERCENT:    {thrust_noise:.1f}%\n\n")

        # Sensor Noise Parameters
        f.write("SENSOR NOISE PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  POSITION_NOISE_STD:            {self.app_config.physics.position_noise_std * 1000:.2f} mm\n"
        )
        f.write(
            f"  ANGLE_NOISE_STD:               {np.degrees(self.app_config.physics.angle_noise_std):.2f}°\n"
        )
        f.write(
            f"  VELOCITY_NOISE_STD:            {self.app_config.physics.velocity_noise_std * 1000:.2f} mm/s\n"
        )
        ang_vel_std = np.degrees(self.app_config.physics.angular_velocity_noise_std)
        f.write(f"  ANGULAR_VEL_NOISE_STD:         {ang_vel_std:.2f}°/s\n\n")

        # Disturbance Parameters
        f.write("DISTURBANCE PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        rand_dist = self.app_config.physics.random_disturbances_enabled
        f.write(f"  RANDOM_DISTURBANCES_ENABLED:   {rand_dist}\n")
        f.write(
            f"  DISTURBANCE_FORCE_STD:         {self.app_config.physics.disturbance_force_std:.3f} N\n"
        )
        f.write(
            f"  DISTURBANCE_TORQUE_STD:        {self.app_config.physics.disturbance_torque_std:.4f} N·m\n\n"
        )

        # Optional/Feature Flags
        f.write("FEATURE FLAGS:\n")
        f.write("-" * 50 + "\n")
        real_phys = self.app_config.physics.use_realistic_physics
        f.write(f"  REALISTIC_PHYSICS_ENABLED:     {real_phys}\n")
        use_final_stab = self.app_config.simulation.use_final_stabilization
        f.write(f"  USE_FINAL_STABILIZATION:       {use_final_stab}\n")
        f.write(
            f"  HEADLESS_MODE:                 {self.app_config.simulation.headless}\n\n"
        )

    def _read_json_file(self, path: Path) -> dict[str, Any]:
        """Best-effort JSON reader that returns an empty dict on failure."""
        if not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            return payload if isinstance(payload, dict) else {}
        except Exception:
            return {}

    def _extract_obstacles(self) -> list[tuple[np.ndarray, float]]:
        """Extract configured obstacles as (center, radius)."""
        if not getattr(self.mission_state, "obstacles_enabled", False):
            return []
        obstacles = getattr(self.mission_state, "obstacles", None) or []
        normalized: list[tuple[np.ndarray, float]] = []
        for obs in obstacles:
            try:
                if hasattr(obs, "position"):
                    center = np.array(obs.position, dtype=float)
                    radius = float(obs.radius)
                else:
                    ox, oy, oz, radius = obs
                    center = np.array([ox, oy, oz], dtype=float)
                if center.size >= 3 and radius >= 0.0:
                    normalized.append((center[:3], radius))
            except Exception:
                continue
        return normalized

    def _compute_obstacle_clearance(
        self,
        state_history: list[np.ndarray],
        control_time: float,
    ) -> dict[str, Any]:
        """Compute obstacle clearance metrics from sampled state history."""
        obstacles = self._extract_obstacles()
        if not obstacles or not state_history:
            return {}

        min_clearance = float("inf")
        min_clearance_idx = 0
        min_clearance_obs = -1
        min_clearance_per_step: list[float] = []

        for idx, state in enumerate(state_history):
            pos = np.array(state[:3], dtype=float)
            step_min = float("inf")
            step_obs_idx = -1
            for obs_idx, (center, radius) in enumerate(obstacles):
                clearance = float(np.linalg.norm(pos - center) - radius)
                if clearance < step_min:
                    step_min = clearance
                    step_obs_idx = obs_idx
            min_clearance_per_step.append(step_min)
            if step_min < min_clearance:
                min_clearance = step_min
                min_clearance_idx = idx
                min_clearance_obs = step_obs_idx

        step_dt = 0.0
        if control_time > 0.0 and len(state_history) > 1:
            step_dt = control_time / float(len(state_history) - 1)

        obstacle_margin = float(getattr(self.app_config.mpc, "obstacle_margin", 0.0))
        margin_breach_count = int(
            sum(1 for clearance in min_clearance_per_step if clearance < obstacle_margin)
        )
        collision_count = int(sum(1 for clearance in min_clearance_per_step if clearance < 0.0))

        return {
            "obstacle_count": len(obstacles),
            "minimum_clearance_m": min_clearance,
            "minimum_clearance_time_s": min_clearance_idx * step_dt,
            "minimum_clearance_obstacle_index": min_clearance_obs + 1
            if min_clearance_obs >= 0
            else None,
            "required_margin_m": obstacle_margin,
            "minimum_margin_delta_m": min_clearance - obstacle_margin,
            "margin_breach_count": margin_breach_count,
            "collision_count": collision_count,
        }

    def _compute_actuator_tracking_summary(self, run_dir: Path) -> dict[str, Any]:
        """Estimate command-vs-valve tracking quality from physics CSV logs."""
        physics_csv = run_dir / "physics_data.csv"
        if not physics_csv.exists():
            return {}

        try:
            with physics_csv.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                if not reader.fieldnames:
                    return {}

                channel_pairs: list[tuple[str, str, str]] = []
                for name in reader.fieldnames:
                    if not name.startswith("Thruster_") or not name.endswith("_Cmd"):
                        continue
                    thruster_label = name[: -len("_Cmd")]
                    val_name = f"{thruster_label}_Val"
                    if val_name in reader.fieldnames:
                        channel_pairs.append((name, val_name, thruster_label))

                if not channel_pairs:
                    return {}

                sample_count = 0
                channel_sample_count = 0
                total_abs_error = 0.0
                max_abs_error = 0.0
                max_abs_error_time = 0.0
                max_abs_error_channel = ""
                per_thruster_totals: dict[str, float] = {
                    label: 0.0 for _, _, label in channel_pairs
                }
                per_thruster_samples: dict[str, int] = {
                    label: 0 for _, _, label in channel_pairs
                }

                for row in reader:
                    sample_count += 1
                    time_s = float(row.get("Time", 0.0) or 0.0)
                    for cmd_key, val_key, thruster in channel_pairs:
                        cmd = float(row.get(cmd_key, 0.0) or 0.0)
                        val = float(row.get(val_key, 0.0) or 0.0)
                        err = abs(val - cmd)
                        total_abs_error += err
                        channel_sample_count += 1
                        per_thruster_totals[thruster] += err
                        per_thruster_samples[thruster] += 1
                        if err > max_abs_error:
                            max_abs_error = err
                            max_abs_error_time = time_s
                            max_abs_error_channel = thruster

                if channel_sample_count == 0:
                    return {}

                thruster_mean_errors = {
                    thruster: per_thruster_totals[thruster]
                    / float(max(1, per_thruster_samples[thruster]))
                    for thruster in per_thruster_totals
                }
                worst_thruster = max(
                    thruster_mean_errors, key=lambda key: thruster_mean_errors[key]
                )

                return {
                    "samples": sample_count,
                    "thruster_count": len(channel_pairs),
                    "mean_abs_error": total_abs_error / float(channel_sample_count),
                    "max_abs_error": max_abs_error,
                    "max_abs_error_time_s": max_abs_error_time,
                    "max_abs_error_channel": max_abs_error_channel,
                    "worst_thruster": worst_thruster,
                    "worst_thruster_mean_abs_error": thruster_mean_errors[worst_thruster],
                }
        except Exception:
            return {}

    def _scan_artifact_index(self, run_dir: Path) -> dict[str, Any]:
        """Build compact artifact inventory directly from files on disk."""
        if not run_dir.exists():
            return {}

        category_counts = {
            "plot": 0,
            "media": 0,
            "data": 0,
            "metadata": 0,
            "other": 0,
        }
        total_size_bytes = 0
        file_count = 0
        plots_by_group: dict[str, int] = {}

        for artifact in sorted(run_dir.rglob("*")):
            if not artifact.is_file():
                continue
            rel = artifact.relative_to(run_dir)
            rel_posix = rel.as_posix()
            file_count += 1
            total_size_bytes += artifact.stat().st_size

            suffix = artifact.suffix.lower()
            if suffix in {".mp4", ".gif", ".webm"}:
                category = "media"
            elif suffix in {".png", ".jpg", ".jpeg", ".svg", ".html"}:
                category = "plot"
            elif suffix in {".csv"}:
                category = "data"
            elif suffix in {".json", ".jsonl", ".txt", ".md", ".sha256"}:
                category = "metadata"
            else:
                category = "other"
            category_counts[category] += 1

            if rel_posix.startswith("Plots/"):
                parts = rel.parts
                plot_group = parts[1] if len(parts) > 2 else "root"
                plots_by_group[plot_group] = plots_by_group.get(plot_group, 0) + 1

        return {
            "file_count": file_count,
            "total_size_bytes": total_size_bytes,
            "category_counts": category_counts,
            "plots_by_group": plots_by_group,
        }

    def _write_run_identity_and_termination(self, f, run_dir: Path) -> None:
        """Write run identity block and termination reason details."""
        status_payload = self._read_json_file(run_dir / "run_status.json")
        mission = status_payload.get("mission", {}) or {}
        preset = status_payload.get("preset", {}) or {}
        config = status_payload.get("config", {}) or {}
        started_at = status_payload.get("started_at")
        completed_at = status_payload.get("completed_at")
        wall_clock_s = status_payload.get("wall_clock_duration_s")

        f.write("\nRUN IDENTITY & TERMINATION\n")
        f.write("-" * 50 + "\n")
        f.write(f"Run ID:                    {run_dir.name}\n")
        f.write(f"Run Directory:             {run_dir}\n")
        f.write(f"Mission Name:              {mission.get('name') or 'n/a'}\n")
        f.write(f"Mission Path:              {mission.get('path') or 'n/a'}\n")
        f.write(f"Preset Name:               {preset.get('name') or 'n/a'}\n")
        f.write(f"Config Version:            {config.get('config_version') or 'n/a'}\n")
        f.write(f"Config Hash:               {config.get('config_hash') or 'n/a'}\n")
        f.write(
            f"Overrides Active:          {'YES' if config.get('overrides_active') else 'NO'}\n"
        )
        f.write(f"Started At:                {started_at or 'n/a'}\n")
        f.write(f"Completed At:              {completed_at or 'n/a'}\n")
        if isinstance(wall_clock_s, (int, float)):
            f.write(f"Wall Clock Duration:       {float(wall_clock_s):.2f} s\n")
        else:
            f.write("Wall Clock Duration:       n/a\n")
        f.write(f"Termination Status:        {status_payload.get('status') or 'unknown'}\n")
        f.write(
            f"Termination Detail:        {status_payload.get('status_detail') or 'n/a'}\n"
        )

    def _write_constraints_summary(
        self,
        f,
        run_dir: Path,
        timing_violations: int,
        solver_limit_exceeded: int,
    ) -> None:
        """Write explicit constraints pass/fail summary."""
        payload = self._read_json_file(run_dir / "constraint_violations.json")
        f.write("\nCONSTRAINTS SUMMARY\n")
        f.write("-" * 50 + "\n")

        if payload:
            violations = payload.get("violations", []) or []
            f.write(f"Constraints Pass:          {'YES' if payload.get('pass') else 'NO'}\n")
            f.write(f"Violation Types:           {len(violations)}\n")
            limits = payload.get("limits", {}) or {}
            if limits:
                lin_lim = float(limits.get("max_linear_velocity_mps", 0.0) or 0.0)
                ang_lim = float(limits.get("max_angular_velocity_radps", 0.0) or 0.0)
                margin = float(limits.get("obstacle_margin_m", 0.0) or 0.0)
                solve_lim = float(limits.get("solver_time_limit_s", 0.0) or 0.0)
                f.write(f"Max Linear Velocity:       {lin_lim:.3f} m/s\n")
                f.write(f"Max Angular Velocity:      {np.degrees(ang_lim):.2f}°/s\n")
                f.write(f"Obstacle Margin:           {margin:.3f} m\n")
                f.write(f"Solver Time Limit:         {solve_lim * 1000.0:.1f} ms\n")
            for item in violations:
                vtype = str(item.get("type", "unknown"))
                count = int(item.get("count", 0) or 0)
                f.write(f"- {vtype}: {count}\n")
            if not violations:
                f.write("- No violations detected\n")
            return

        f.write("Constraints Pass:          n/a (artifact not available yet)\n")
        f.write(f"Timing Violations:         {timing_violations}\n")
        f.write(f"Solver Limit Exceeded:     {solver_limit_exceeded}\n")

    def _write_obstacle_clearance_summary(
        self,
        f,
        state_history: list[np.ndarray],
        control_time: float,
    ) -> None:
        """Write minimum clearance and margin-breach summary."""
        f.write("\nOBSTACLE CLEARANCE SUMMARY\n")
        f.write("-" * 50 + "\n")
        clearance = self._compute_obstacle_clearance(
            state_history=state_history,
            control_time=control_time,
        )
        if not clearance:
            f.write("Obstacle Data:             n/a (obstacles disabled or unavailable)\n")
            return

        f.write(f"Obstacle Count:            {int(clearance['obstacle_count'])}\n")
        f.write(
            f"Min Clearance:             {float(clearance['minimum_clearance_m']):.4f} m\n"
        )
        f.write(
            f"Min Clearance Time:        {float(clearance['minimum_clearance_time_s']):.3f} s\n"
        )
        obs_idx = clearance.get("minimum_clearance_obstacle_index")
        f.write(f"Closest Obstacle Index:    {obs_idx if obs_idx is not None else 'n/a'}\n")
        f.write(
            f"Required Margin:           {float(clearance['required_margin_m']):.4f} m\n"
        )
        f.write(
            f"Clearance-Margin Delta:    {float(clearance['minimum_margin_delta_m']):.4f} m\n"
        )
        f.write(
            f"Margin Breach Samples:     {int(clearance['margin_breach_count'])}\n"
        )
        f.write(f"Collision Samples:         {int(clearance['collision_count'])}\n")

    def _write_actuator_tracking_summary(self, f, run_dir: Path) -> None:
        """Write command-vs-actual actuator tracking quality metrics."""
        f.write("\nACTUATOR TRACKING SUMMARY\n")
        f.write("-" * 50 + "\n")
        tracking = self._compute_actuator_tracking_summary(run_dir)
        if not tracking:
            f.write("Tracking Data:             n/a (physics_data.csv unavailable)\n")
            return

        f.write(f"Samples:                   {int(tracking['samples'])}\n")
        f.write(f"Thrusters Tracked:         {int(tracking['thruster_count'])}\n")
        f.write(
            f"Mean |Valve-Cmd|:          {float(tracking['mean_abs_error']):.4f} "
            f"({100.0 * float(tracking['mean_abs_error']):.2f}%)\n"
        )
        f.write(
            f"Max |Valve-Cmd|:           {float(tracking['max_abs_error']):.4f} "
            f"({100.0 * float(tracking['max_abs_error']):.2f}%)\n"
        )
        f.write(
            f"Max Error Channel:         {tracking.get('max_abs_error_channel') or 'n/a'}\n"
        )
        f.write(
            f"Max Error Time:            {float(tracking['max_abs_error_time_s']):.3f} s\n"
        )
        f.write(
            f"Worst Mean Tracking:       {tracking.get('worst_thruster') or 'n/a'} "
            f"({float(tracking['worst_thruster_mean_abs_error']):.4f})\n"
        )

    def _write_artifact_index(self, f, run_dir: Path) -> None:
        """Write compact artifact inventory and grouped plot counts."""
        f.write("\nARTIFACT INDEX\n")
        f.write("-" * 50 + "\n")
        index = self._scan_artifact_index(run_dir)
        if not index:
            f.write("Artifacts:                 n/a\n")
            return

        file_count = int(index["file_count"])
        total_size_mb = float(index["total_size_bytes"]) / (1024.0 * 1024.0)
        counts = index["category_counts"]
        f.write(f"Total Files:               {file_count}\n")
        f.write(f"Total Size:                {total_size_mb:.2f} MB\n")
        f.write(f"Plot Files:                {int(counts['plot'])}\n")
        f.write(f"Data Files:                {int(counts['data'])}\n")
        f.write(f"Media Files:               {int(counts['media'])}\n")
        f.write(f"Metadata Files:            {int(counts['metadata'])}\n")
        f.write(f"Other Files:               {int(counts['other'])}\n")

        plots_by_group = index.get("plots_by_group", {}) or {}
        if plots_by_group:
            f.write("Plot Groups:\n")
            for group in sorted(plots_by_group.keys()):
                f.write(f"- {group}: {int(plots_by_group[group])}\n")

    def _write_performance_results(
        self,
        f,
        state_history: list[np.ndarray],
        reference_state: np.ndarray,
        control_time: float,
        mpc_solve_times: list[float],
        control_history: list[np.ndarray],
        path_complete_time: float | None,
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        check_path_complete_func: Callable[..., Any],
        run_dir: Path,
    ) -> None:
        """Write performance results section."""
        f.write("=" * 80 + "\n")
        f.write("MISSION PERFORMANCE RESULTS\n")
        f.write("=" * 80 + "\n\n")

        # Calculate metrics
        initial_state = state_history[0]
        final_state = state_history[-1]
        initial_pos = initial_state[:3]
        final_pos = final_state[:3]
        path = self.mission_state.get_resolved_path_waypoints()
        if path:
            path_end = np.array(path[-1], dtype=float)
        else:
            path_end = np.array(reference_state[:3], dtype=float)

        pos_error_initial = np.linalg.norm(initial_pos - path_end)
        pos_error_final = np.linalg.norm(final_pos - path_end)

        # 3D Angle Errors (Quaternion)
        def get_ang_err(s1, s2):
            q1, q2 = s1[3:7], s2[3:7]
            dot = np.abs(np.dot(q1, q2))
            dot = min(1.0, max(-1.0, dot))
            return 2.0 * np.arccos(dot)

        ang_error_initial = get_ang_err(initial_state, reference_state)
        ang_error_final = get_ang_err(final_state, reference_state)

        trajectory_distance = sum(
            np.linalg.norm(state_history[i][:3] - state_history[i - 1][:3])
            for i in range(1, len(state_history))
        )

        mpc_convergence_times = (
            np.array(mpc_solve_times) if mpc_solve_times else np.array([])
        )

        total_thrust_activations = (
            sum(np.sum(control) for control in control_history)
            if control_history
            else 0
        )
        total_thrust_magnitude = (
            sum(np.linalg.norm(control) for control in control_history)
            if control_history
            else 0
        )

        switching_events = 0
        if len(control_history) > 1:
            for i in range(1, len(control_history)):
                curr_control = control_history[i]
                prev_control = control_history[i - 1]
                if len(curr_control) < 12:
                    curr_control = np.pad(
                        curr_control, (0, 12 - len(curr_control)), "constant"
                    )
                if len(prev_control) < 12:
                    prev_control = np.pad(
                        prev_control, (0, 12 - len(prev_control)), "constant"
                    )
                switching_events += np.sum(np.abs(curr_control - prev_control))

        success = check_path_complete_func()
        vel_magnitude_final = np.linalg.norm(final_state[7:10])
        timing_violations = 0
        solver_limit_exceeded = 0

        # Position & Trajectory Analysis
        f.write("[POSITION] POSITION & TRAJECTORY ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"Initial Position:          ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, "
            f"{initial_pos[2]:.3f}) m\n"
        )
        f.write(
            f"Final Position:            ({final_pos[0]:.3f}, {final_pos[1]:.3f}, "
            f"{final_pos[2]:.3f}) m\n"
        )
        f.write(
            f"Path End Position:         ({path_end[0]:.3f}, {path_end[1]:.3f}, "
            f"{path_end[2]:.3f}) m\n"
        )
        f.write(f"Initial Position Error:    {pos_error_initial:.4f} m\n")
        f.write(f"Final Position Error:      {pos_error_final:.4f} m\n")
        if pos_error_initial > 0:
            pos_improv = (pos_error_initial - pos_error_final) / pos_error_initial * 100
            f.write(f"Position Improvement:      {pos_improv:.1f}%\n")
        f.write(f"Total Distance Traveled:   {trajectory_distance:.3f} m\n")
        if trajectory_distance > 0:
            direct_dist = np.linalg.norm(path_end - initial_pos)
            traj_eff = direct_dist / trajectory_distance * 100
            f.write(f"Trajectory Efficiency:     {traj_eff:.1f}%\n")
        f.write("\n")

        # Orientation Analysis
        f.write("ORIENTATION ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Initial Angle Error:       {np.degrees(ang_error_initial):.2f}°\n")
        final_ang_deg = np.degrees(ang_error_final)
        tol_deg = np.degrees(angle_tolerance)
        f.write(
            f"Final Angle Error:         {final_ang_deg:.2f}° "
            f"(threshold: <{tol_deg:.1f}°)\n"
        )
        if ang_error_initial > 0:
            ang_improv = (ang_error_initial - ang_error_final) / ang_error_initial * 100
            f.write(f"Angle Improvement:         {ang_improv:.1f}%\n")
        f.write(f"Final Velocity Magnitude:  {vel_magnitude_final:.4f} m/s\n")
        f.write(
            f"Final Angular Velocity:    {np.degrees(np.linalg.norm(final_state[10:13])):.2f}"
            "°/s\n\n"
        )

        # MPC Performance
        f.write("LINEARIZED MPC CONTROLLER PERFORMANCE\n")
        f.write("-" * 50 + "\n")
        f.write(f"Total Test Time:           {control_time:.1f} s\n")
        f.write(f"MPC Updates:               {len(mpc_convergence_times)} cycles\n")
        if control_time > 0:
            f.write(
                f"MPC Update Rate:           {len(mpc_convergence_times) / control_time:.1f} Hz\n"
            )

        if len(mpc_convergence_times) > 0:
            mean_solve = float(np.mean(mpc_convergence_times))
            max_solve = float(np.max(mpc_convergence_times))
            f.write(
                f"Fastest MPC Solve:         {np.min(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"Slowest MPC Solve:         {max_solve:.3f} s\n"
            )
            f.write(f"Average MPC Solve:         {mean_solve:.3f} s\n")
            f.write(
                f"MPC Solve Std Dev:         {np.std(mpc_convergence_times):.3f} s\n"
            )
            timing_violations = sum(
                1 for t in mpc_convergence_times if t > (control_update_interval - 0.02)
            )
            n_times = len(mpc_convergence_times)
            pct = 100 * timing_violations / n_times
            f.write(
                f"Timing Violations:         {timing_violations}/{n_times} ({pct:.1f}%)\n"
            )
            rt_pct = np.mean(mpc_convergence_times) / control_update_interval
            f.write(
                f"Real-time Performance:     {rt_pct * 100:.1f}% of available time\n"
            )
            mean_target_ms = float(
                self.app_config.simulation.mpc_target_mean_solve_time_ms
            )
            hard_max_ms = float(self.app_config.simulation.mpc_hard_max_solve_time_ms)
            mean_ms = mean_solve * 1000.0
            max_ms = max_solve * 1000.0
            contract_pass = (mean_ms <= mean_target_ms) and (max_ms <= hard_max_ms)
            f.write(
                "Timing Contract:           "
                f"{'PASS' if contract_pass else 'FAIL'} "
                f"(mean {mean_ms:.2f}/{mean_target_ms:.2f} ms, "
                f"max {max_ms:.2f}/{hard_max_ms:.2f} ms)\n"
            )
            solver_limit_exceeded = int(
                sum(
                    1
                    for t in mpc_convergence_times
                    if t > float(self.app_config.mpc.solver_time_limit)
                )
            )
        f.write("\n")

        # Control Effort Analysis
        if control_history:
            f.write("CONTROL EFFORT & FUEL ANALYSIS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Total Thruster Activations: {total_thrust_activations:.0f}\n")
            f.write(f"Total Control Magnitude:    {total_thrust_magnitude:.2f} N·s\n")
            avg_ctrl = total_thrust_magnitude / len(control_history)
            f.write(f"Average Control per Step:   {avg_ctrl:.3f} N\n")
            f.write(f"Thruster Switching Events:  {switching_events:.0f}\n")
            if len(control_history) > 0:
                n_hist = len(control_history)
                smoothness = (1 - switching_events / (n_hist * 12)) * 100
                f.write(f"Control Smoothness:         {smoothness:.1f}%\n")
            if trajectory_distance > 0:
                fuel_eff = total_thrust_magnitude / trajectory_distance
                f.write(f"Fuel Efficiency:            {fuel_eff:.3f} N·s/m\n")
            f.write("\n")

        # Mission Success Analysis
        f.write("MISSION SUCCESS & PATH COMPLETION ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Position Tolerance:        <{position_tolerance:.3f} m\n")
        f.write(f"Angle Tolerance:           <{np.degrees(angle_tolerance):.1f}°\n")
        pos_met = "YES" if pos_error_final < position_tolerance else "NO"
        f.write(f"Position Threshold Met:    {pos_met}\n")
        f.write(
            f"Angle Threshold Met:       {'YES' if ang_error_final < angle_tolerance else 'NO'}\n"
        )
        f.write(
            f"Overall Mission Status:    {'SUCCESS' if success else 'INCOMPLETE'}\n\n"
        )

        # Precision Analysis
        f.write(f"Achieved Precision:        {pos_error_final:.4f}m\n")
        precision_ratio = (
            pos_error_final / position_tolerance if position_tolerance > 0 else float("inf")
        )
        if precision_ratio <= 1.0:
            f.write(f"Precision Ratio:           {precision_ratio:.2f} (PASSED)\n")
        else:
            over_pct = (precision_ratio - 1) * 100
            f.write(
                f"Precision Ratio:           {precision_ratio:.2f} "
                f"(FAILED - {over_pct:.1f}% over threshold)\n"
            )
        f.write("\n")

        # Path Completion Analysis
        if path_complete_time is not None:
            f.write("PATH COMPLETION STATUS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Path First Completed:      {path_complete_time:.1f} s\n")
        else:
            f.write("PATH COMPLETION STATUS\n")
            f.write("-" * 50 + "\n")
            f.write("Path Not Completed:        Mission incomplete\n")
            remaining_pos_error = max(0.0, float(pos_error_final - position_tolerance))
            remaining_ang_error = max(0.0, float(ang_error_final - angle_tolerance))
            f.write(f"Remaining Position Error:  {remaining_pos_error:.4f} m\n")
            f.write(
                f"Remaining Angle Error:     {np.degrees(remaining_ang_error):.2f}°\n"
            )

        self._write_run_identity_and_termination(f, run_dir=run_dir)
        self._write_constraints_summary(
            f,
            run_dir=run_dir,
            timing_violations=timing_violations,
            solver_limit_exceeded=solver_limit_exceeded,
        )
        self._write_obstacle_clearance_summary(
            f,
            state_history=state_history,
            control_time=control_time,
        )
        self._write_actuator_tracking_summary(f, run_dir=run_dir)
        self._write_artifact_index(f, run_dir=run_dir)


def create_mission_report_generator(config: SimulationConfig) -> MissionReportGenerator:
    """
    Factory function to create a mission report generator.

    Args:
        config: SimulationConfig object.

    Returns:
        Configured MissionReportGenerator instance
    """
    return MissionReportGenerator(config)
