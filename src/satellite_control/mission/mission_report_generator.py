"""
Mission Report Generator for Satellite Control System

Generates comprehensive post-mission analysis reports with detailed metrics
and statistics.
Provides formatted text reports for documentation and performance review.

Report sections:
- Mission configuration and parameters
- Performance metrics and analysis
- Target tracking results with error statistics
- Control system performance (thruster usage, control effort)
- MPC timing statistics and computational performance
- Collision avoidance and safety events

Output formats:
- Console display with formatted text
- Text file export for archival
- Summary statistics for comparison
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, List, Optional

import numpy as np

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.config.models import AppConfig
from src.satellite_control.config.constants import Constants
from src.satellite_control.utils.orientation_utils import quat_wxyz_to_euler_xyz

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
    - Target maintenance statistics
    """

    def __init__(self, config: SimulationConfig):
        """
        Initialize report generator.

        Args:
            config: SimulationConfig object (V4.0.0 Architecture)
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
        state_history: List[np.ndarray],
        target_state: np.ndarray,
        control_time: float,
        mpc_solve_times: List[float],
        control_history: List[np.ndarray],
        target_reached_time: Optional[float],
        target_maintenance_time: float,
        times_lost_target: int,
        maintenance_position_errors: List[float],
        maintenance_angle_errors: List[float],
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        angle_difference_func: Callable[..., Any],
        check_target_reached_func: Callable[..., Any],
        test_mode: str = "SIMULATION",
    ) -> None:
        """
        Generate comprehensive mission summary report.

        Args:
            output_path: Path to save the report
            state_history: List of state vectors [x,y,z, qw,qx,qy,qz, vx,vy,vz, wx,wy,wz]
            target_state: Target state vector
            control_time: Total mission duration in seconds
            mpc_solve_times: List of MPC solve times
            control_history: List of control vectors (thruster commands)
            target_reached_time: Time when target was first reached (None if never)
            target_maintenance_time: Time spent maintaining target
            times_lost_target: Number of times target was lost
            maintenance_position_errors: Position errors during maintenance
            maintenance_angle_errors: Angle errors during maintenance
            position_tolerance: Position tolerance threshold
            angle_tolerance: Angle tolerance threshold
            control_update_interval: Control loop update interval
            angle_difference_func: Function to calculate angle difference
            check_target_reached_func: Function to check if target was reached
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
                self._write_mission_configuration(f, state_history, target_state)

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
                    target_state,
                    control_time,
                    mpc_solve_times,
                    control_history,
                    target_reached_time,
                    target_maintenance_time,
                    times_lost_target,
                    maintenance_position_errors,
                    maintenance_angle_errors,
                    position_tolerance,
                    angle_tolerance,
                    control_update_interval,
                    angle_difference_func,
                    check_target_reached_func,
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
        self, f, state_history: List[np.ndarray], target_state: np.ndarray
    ) -> None:
        """Write mission configuration section."""
        f.write("=" * 80 + "\n")
        f.write("MISSION CONFIGURATION - RECREATION DATA\n")
        f.write("=" * 80 + "\n")
        f.write(
            "This section contains all information needed to recreate this exact mission.\n\n"
        )

        initial_state = state_history[0]

        # Determine mission type
        # Determine mission type
        if self.mission_state.dxf_shape_mode_active:
            self._write_profile_following_config(f, initial_state)
        elif self.mission_state.enable_waypoint_mode:
            self._write_waypoint_config(f, initial_state)
        else:
            self._write_point_to_point_config(f, initial_state, target_state)

        # Obstacle configuration
        self._write_obstacle_configuration(f)

    def _write_profile_following_config(self, f, initial_state: np.ndarray) -> None:
        """Write profile following (Mode 3) configuration."""
        f.write("MISSION TYPE: PROFILE FOLLOWING (MODE 3)\n")
        f.write("-" * 50 + "\n")
        f.write(
            "Description: Satellite follows a moving target along a custom path profile\n"
        )
        f.write("Mission Phases:\n")
        f.write(
            "  Phase 1: Move to closest point on profile and stabilize (3 seconds)\n"
        )
        f.write("  Phase 2: Track moving target along the path profile\n")
        f.write("  Phase 3: Stabilize at completion point (5 seconds)\n\n")

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting Z position:     {initial_state[2]:.3f} m\n")

        q = initial_state[3:7]
        f.write(
            f"  Starting orientation:    {self._format_euler_deg(quat_wxyz_to_euler_xyz(q))}\n\n"
        )

        f.write("SHAPE CONFIGURATION:\n")
        if self.mission_state.dxf_shape_center:
            center = self.mission_state.dxf_shape_center
            f.write(f"  Shape Center X:          {center[0]:.3f} m\n")
            f.write(f"  Shape Center Y:          {center[1]:.3f} m\n")
            if len(center) > 2:
                f.write(f"  Shape Center Z:          {center[2]:.3f} m\n")
        if self.mission_state.dxf_shape_rotation:
            f.write(
                f"  Shape Rotation:          {np.degrees(self.mission_state.dxf_shape_rotation):.1f}°\n"
            )
        f.write(
            f"  Offset Distance:         {self.mission_state.dxf_offset_distance:.3f} m\n"
        )
        f.write(
            f"  Path Length:             {self.mission_state.dxf_path_length:.3f} m\n"
        )
        if self.mission_state.dxf_base_shape:
            path_len = len(self.mission_state.dxf_shape_path)
            f.write(f"  Number of Path Points:   {path_len}\n\n")

        f.write("MOVING TARGET CONFIGURATION:\n")
        f.write(
            f"  Target Speed:            {self.mission_state.dxf_target_speed:.3f} m/s\n"
        )
        f.write(
            f"  Estimated Duration:      {self.mission_state.dxf_estimated_duration:.1f} s\n\n"
        )

    def _write_waypoint_config(self, f, initial_state: np.ndarray) -> None:
        """Write waypoint (Mode 1) configuration."""
        f.write("MISSION TYPE: WAYPOINT NAVIGATION (MODE 1)\n")
        f.write("-" * 50 + "\n")
        f.write("Description: Satellite visits multiple waypoints in sequence\n\n")

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting Z position:     {initial_state[2]:.3f} m\n")

        q = initial_state[3:7]
        f.write(
            f"  Starting orientation:    {self._format_euler_deg(quat_wxyz_to_euler_xyz(q))}\n\n"
        )

        f.write("WAYPOINTS:\n")
        f.write(
            f"  Number of Waypoints:     {len(self.mission_state.waypoint_targets)}\n"
        )
        for i, (target, angle) in enumerate(
            zip(
                self.mission_state.waypoint_targets, self.mission_state.waypoint_angles
            ),
            1,
        ):
            tx, ty, tz = target
            f.write(
                f"  Waypoint {i}:              ({tx:.3f}, {ty:.3f}, {tz:.3f}) m, "
                f"{self._format_euler_deg(angle)}\n"
            )
        f.write(
            f"\n  Hold Time per Waypoint:  {self.app_config.simulation.target_hold_time:.1f} s\n\n"
        )

    def _write_point_to_point_config(
        self, f, initial_state: np.ndarray, target_state: np.ndarray
    ) -> None:
        """Write point-to-point (Mode 1) configuration."""
        f.write("MISSION TYPE: POINT-TO-POINT (MODE 1)\n")
        f.write("-" * 50 + "\n")
        f.write(
            "Description: Satellite navigates to target position and orientation\n\n"
        )

        f.write("STARTING CONFIGURATION:\n")
        f.write(f"  Starting X position:     {initial_state[0]:.3f} m\n")
        f.write(f"  Starting Y position:     {initial_state[1]:.3f} m\n")
        f.write(f"  Starting Z position:     {initial_state[2]:.3f} m\n")

        q = initial_state[3:7]
        f.write(
            f"  Starting orientation:    {self._format_euler_deg(quat_wxyz_to_euler_xyz(q))}\n\n"
        )

        target_pos = target_state[:3]
        qt = target_state[3:7]
        target_euler = quat_wxyz_to_euler_xyz(qt)
        f.write("TARGET CONFIGURATION:\n")
        f.write(f"  Target X position:       {target_pos[0]:.3f} m\n")
        f.write(f"  Target Y position:       {target_pos[1]:.3f} m\n")
        f.write(f"  Target Z position:       {target_pos[2]:.3f} m\n")
        f.write(
            f"  Target orientation:      {self._format_euler_deg(target_euler)}\n\n"
        )

    def _write_obstacle_configuration(self, f) -> None:
        """Write obstacle configuration."""
        if self.mission_state.obstacles_enabled and self.mission_state.obstacles:
            f.write("OBSTACLE CONFIGURATION:\n")
            f.write("  Obstacle Avoidance:      ENABLED\n")
            f.write(f"  Number of Obstacles:     {len(self.mission_state.obstacles)}\n")
            for i, (ox, oy, oz, orad) in enumerate(self.mission_state.obstacles, 1):
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

        f.write("MPC CONTROLLER PARAMETERS:\n")
        f.write("-" * 50 + "\n")
        f.write("  Controller Type:         Linearized MPC (A*x[k] + B*u[k])\n")
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
        f.write(f"  Position Weight (Q):     {self.app_config.mpc.q_position:.1f}\n")
        f.write(f"  Velocity Weight (Q):     {self.app_config.mpc.q_velocity:.1f}\n")
        f.write(f"  Angle Weight (Q):        {self.app_config.mpc.q_angle:.1f}\n")
        f.write(
            f"  Angular Vel Weight (Q):  {self.app_config.mpc.q_angular_velocity:.1f}\n"
        )
        f.write(f"  Thrust Penalty (R):      {self.app_config.mpc.r_thrust:.3f}\n\n")

        f.write("CONTROL CONSTRAINTS:\n")
        f.write("-" * 50 + "\n")
        f.write(
            f"  Max Velocity:            {self.app_config.mpc.max_velocity:.3f} m/s\n"
        )
        f.write(
            f"  Max Angular Velocity:    {np.degrees(self.app_config.mpc.max_angular_velocity):.1f}°/s\n"
        )
        f.write(
            f"  Position Bounds:         ±{self.app_config.mpc.position_bounds:.1f} m\n\n"
        )

        f.write("TOLERANCE THRESHOLDS:\n")
        f.write("-" * 50 + "\n")
        f.write(f"  Position Tolerance:      {Constants.POSITION_TOLERANCE:.3f} m\n")
        f.write(
            f"  Angle Tolerance:         <{np.degrees(Constants.ANGLE_TOLERANCE):.1f}°\n"
        )
        f.write(f"  Velocity Tolerance:      {Constants.VELOCITY_TOLERANCE:.3f} m/s\n")
        ang_vel_tol = np.degrees(Constants.ANGULAR_VELOCITY_TOLERANCE)
        f.write(f"  Angular Vel Tolerance:   <{ang_vel_tol:.1f}°/s\n\n")

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
            f"  Q_POSITION:                    {self.app_config.mpc.q_position:.1f}\n"
        )
        f.write(
            f"  Q_VELOCITY:                    {self.app_config.mpc.q_velocity:.1f}\n"
        )
        f.write(f"  Q_ANGLE:                       {self.app_config.mpc.q_angle:.1f}\n")
        f.write(
            f"  Q_ANGULAR_VELOCITY:            {self.app_config.mpc.q_angular_velocity:.1f}\n"
        )
        f.write(
            f"  R_THRUST:                      {self.app_config.mpc.r_thrust:.3f}\n"
        )

        f.write(
            f"  MAX_VELOCITY:                  {self.app_config.mpc.max_velocity:.3f} m/s\n"
        )
        max_ang_vel = np.degrees(self.app_config.mpc.max_angular_velocity)
        f.write(f"  MAX_ANGULAR_VELOCITY:          {max_ang_vel:.1f}°/s\n")
        f.write(
            f"  POSITION_BOUNDS:               ±{self.app_config.mpc.position_bounds:.1f} m\n"
        )
        f.write(
            f"  DAMPING_ZONE:                  {self.app_config.mpc.damping_zone:.3f} m\n"
        )
        f.write(
            f"  VELOCITY_THRESHOLD:            {self.app_config.mpc.velocity_threshold:.3f} m/s\n"
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
        f.write(
            f"  TARGET_HOLD_TIME:              {self.app_config.simulation.target_hold_time:.1f} s\n"
        )
        # Use WAYPOINT_FINAL_STABILIZATION_TIME as fallback for
        # mission-specific stabilization times
        waypoint_final_stab = (
            self.app_config.simulation.waypoint_final_stabilization_time
        )
        f.write(f"  WAYPOINT_FINAL_STABILIZATION_TIME: {waypoint_final_stab:.1f} s\n")

        # Access optional config values using defaults if not present
        # These might be in app_config.simulation or just locals
        p2p_stab = getattr(
            self.app_config.simulation,
            "point_to_point_final_stabilization_time",
            waypoint_final_stab,
        )
        f.write(f"  POINT_TO_POINT_FINAL_STAB:    {p2p_stab:.1f} s\n")

        mp_stab = getattr(
            self.app_config.simulation,
            "multi_point_final_stabilization_time",
            waypoint_final_stab,
        )
        f.write(f"  MULTI_POINT_FINAL_STAB:       {mp_stab:.1f} s\n")

        dxf_stab = self.app_config.simulation.shape_final_stabilization_time
        f.write(f"  DXF_FINAL_STAB:                {dxf_stab:.1f} s\n\n")

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

    def _write_performance_results(
        self,
        f,
        state_history: List[np.ndarray],
        target_state: np.ndarray,
        control_time: float,
        mpc_solve_times: List[float],
        control_history: List[np.ndarray],
        target_reached_time: Optional[float],
        target_maintenance_time: float,
        times_lost_target: int,
        maintenance_position_errors: List[float],
        maintenance_angle_errors: List[float],
        position_tolerance: float,
        angle_tolerance: float,
        control_update_interval: float,
        angle_difference_func: Callable[..., Any],
        check_target_reached_func: Callable[..., Any],
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
        target_pos = target_state[:3]

        pos_error_initial = np.linalg.norm(initial_pos - target_pos)
        pos_error_final = np.linalg.norm(final_pos - target_pos)

        # 3D Angle Errors (Quaternion)
        def get_ang_err(s1, s2):
            q1, q2 = s1[3:7], s2[3:7]
            dot = np.abs(np.dot(q1, q2))
            dot = min(1.0, max(-1.0, dot))
            return 2.0 * np.arccos(dot)

        ang_error_initial = get_ang_err(initial_state, target_state)
        ang_error_final = get_ang_err(final_state, target_state)

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

        success = check_target_reached_func()
        vel_magnitude_final = np.linalg.norm(final_state[7:10])

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
            f"Target Position:           ({target_pos[0]:.3f}, {target_pos[1]:.3f}, "
            f"{target_pos[2]:.3f}) m\n"
        )
        f.write(f"Initial Position Error:    {pos_error_initial:.4f} m\n")
        f.write(f"Final Position Error:      {pos_error_final:.4f} m\n")
        if pos_error_initial > 0:
            pos_improv = (pos_error_initial - pos_error_final) / pos_error_initial * 100
            f.write(f"Position Improvement:      {pos_improv:.1f}%\n")
        f.write(f"Total Distance Traveled:   {trajectory_distance:.3f} m\n")
        if trajectory_distance > 0:
            direct_dist = np.linalg.norm(target_pos - initial_pos)
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
            f"(target: <{tol_deg:.1f}°)\n"
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
            f.write(
                f"Fastest MPC Solve:         {np.min(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"Slowest MPC Solve:         {np.max(mpc_convergence_times):.3f} s\n"
            )
            f.write(
                f"Average MPC Solve:         {np.mean(mpc_convergence_times):.3f} s\n"
            )
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
        f.write("MISSION SUCCESS & TARGET MAINTENANCE ANALYSIS\n")
        f.write("-" * 50 + "\n")
        f.write(f"Position Tolerance:        <{position_tolerance:.3f} m\n")
        f.write(f"Angle Tolerance:           <{np.degrees(angle_tolerance):.1f}°\n")
        pos_met = "YES" if pos_error_final < position_tolerance else "NO"
        f.write(f"Position Target Met:       {pos_met}\n")
        f.write(
            f"Angle Target Met:          {'YES' if ang_error_final < angle_tolerance else 'NO'}\n"
        )
        f.write(
            f"Overall Mission Status:    {'SUCCESS' if success else 'INCOMPLETE'}\n\n"
        )

        # Precision Analysis
        f.write(f"Achieved Precision:        {pos_error_final:.4f}m\n")
        precision_ratio = pos_error_final / 0.050
        if precision_ratio <= 1.0:
            f.write(f"Precision Ratio:           {precision_ratio:.2f} (PASSED)\n")
        else:
            over_pct = (precision_ratio - 1) * 100
            f.write(
                f"Precision Ratio:           {precision_ratio:.2f} "
                f"(FAILED - {over_pct:.1f}% over target)\n"
            )
        f.write("\n")

        # Target Maintenance Analysis
        if target_reached_time is not None:
            f.write("TARGET MAINTENANCE ANALYSIS\n")
            f.write("-" * 50 + "\n")
            f.write(f"Target First Reached:      {target_reached_time:.1f} s\n")
            maint_pct = target_maintenance_time / control_time * 100
            f.write(
                f"Target Maintenance Time:   {target_maintenance_time:.1f} s "
                f"({maint_pct:.1f}% of total)\n"
            )
            f.write(f"Times Lost Target:         {times_lost_target}\n")

            if maintenance_position_errors:
                avg_maintenance_pos_err = np.mean(maintenance_position_errors)
                max_maintenance_pos_err = np.max(maintenance_position_errors)
                avg_maintenance_ang_err = np.mean(maintenance_angle_errors)
                max_maintenance_ang_err = np.max(maintenance_angle_errors)

                f.write(f"Avg Maintenance Pos Error: {avg_maintenance_pos_err:.4f} m\n")
                f.write(f"Max Maintenance Pos Error: {max_maintenance_pos_err:.4f} m\n")
                f.write(
                    f"Avg Maintenance Ang Error: {np.degrees(avg_maintenance_ang_err):.2f}°\n"
                )
                f.write(
                    f"Max Maintenance Ang Error: {np.degrees(max_maintenance_ang_err):.2f}°\n"
                )

                pos_stability = 100 * (1 - avg_maintenance_pos_err / position_tolerance)
                ang_stability = 100 * (1 - avg_maintenance_ang_err / angle_tolerance)
                f.write(
                    f"Position Stability:        {pos_stability:.1f}% (within tolerance)\n"
                )
                f.write(
                    f"Angle Stability:           {ang_stability:.1f}% (within tolerance)\n"
                )
        else:
            f.write("TARGET STATUS\n")
            f.write("-" * 50 + "\n")
            f.write("Target Never Reached:      Mission incomplete\n")
            remaining_pos_error = max(0.0, float(pos_error_final - position_tolerance))
            remaining_ang_error = max(0.0, float(ang_error_final - angle_tolerance))
            f.write(f"Remaining Position Error:  {remaining_pos_error:.4f} m\n")
            f.write(
                f"Remaining Angle Error:     {np.degrees(remaining_ang_error):.2f}°\n"
            )


def create_mission_report_generator(config: SimulationConfig) -> MissionReportGenerator:
    """
    Factory function to create a mission report generator.

    Args:
        config: SimulationConfig object (V4.0.0 Architecture)

    Returns:
        Configured MissionReportGenerator instance
    """
    return MissionReportGenerator(config)
