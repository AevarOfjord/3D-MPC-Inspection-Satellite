#!/usr/bin/env python3
"""
Comprehensive post-processor for C++ simulation output.
Generates legacy-style output: reports, metrics, and full plot suite.

Usage:
    python3 scripts/post_processor.py [simulation_directory]
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import json
import sys
from datetime import datetime
from mpl_toolkits.mplot3d import Axes3D


class SimulationPostProcessor:
    """Process simulation CSV data and generate comprehensive outputs."""
    
    def __init__(self, sim_dir: Path):
        self.sim_dir = sim_dir
        self.plots_dir = sim_dir / "Plots"
        self.plots_dir.mkdir(exist_ok=True)
        
        # Load data
        self.df = pd.read_csv(sim_dir / "control_data.csv")
        self.t = self.df['Control_Time'].values
        
    def run_all(self):
        """Run all post-processing steps."""
        print(f"Post-processing: {self.sim_dir.name}")
        
        self.generate_mission_summary()
        self.generate_performance_metrics()
        self.generate_all_plots()
        
        print(f"✓ Generated mission_summary.txt")
        print(f"✓ Generated performance_metrics.json")
        print(f"✓ Generated {len(list(self.plots_dir.glob('*.png')))} plots in Plots/")
    
    def generate_mission_summary(self):
        """Generate mission_summary.txt."""
        df = self.df
        
        # Calculate metrics
        final_error = np.sqrt(
            df['Error_X'].iloc[-1]**2 + 
            df['Error_Y'].iloc[-1]**2 + 
            df['Error_Z'].iloc[-1]**2
        )
        initial_error = np.sqrt(
            df['Error_X'].iloc[0]**2 + 
            df['Error_Y'].iloc[0]**2 + 
            df['Error_Z'].iloc[0]**2
        )
        
        # Calculate total distance traveled
        dx = np.diff(df['Current_X'])
        dy = np.diff(df['Current_Y'])
        dz = np.diff(df['Current_Z'])
        total_distance = np.sum(np.sqrt(dx**2 + dy**2 + dz**2))
        
        summary = f"""================================================================================
SATELLITE CONTROL SYSTEM - MISSION SUMMARY & CONFIGURATION
================================================================================
Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
Data Directory: {self.sim_dir}
Test Mode: SIMULATION (C++ Backend)
================================================================================

================================================================================
MISSION CONFIGURATION
================================================================================

STARTING CONFIGURATION:
  Starting X position:     {df['Current_X'].iloc[0]:.3f} m
  Starting Y position:     {df['Current_Y'].iloc[0]:.3f} m
  Starting Z position:     {df['Current_Z'].iloc[0]:.3f} m
  Starting orientation:    roll={np.degrees(df['Current_Roll'].iloc[0]):.1f}°, pitch={np.degrees(df['Current_Pitch'].iloc[0]):.1f}°, yaw={np.degrees(df['Current_Yaw'].iloc[0]):.1f}°

TARGET CONFIGURATION:
  Target X position:       {df['Target_X'].iloc[-1]:.3f} m
  Target Y position:       {df['Target_Y'].iloc[-1]:.3f} m
  Target Z position:       {df['Target_Z'].iloc[-1]:.3f} m

================================================================================
CONTROLLER CONFIGURATION
================================================================================

MPC CONTROLLER PARAMETERS:
--------------------------------------------------
  Controller Type:         Linearized MPC (C++)
  Solver:                  OSQP
  Control Timestep:        0.050 s

================================================================================
MISSION PERFORMANCE RESULTS
================================================================================

POSITION & TRAJECTORY ANALYSIS
--------------------------------------------------
Initial Position:          ({df['Current_X'].iloc[0]:.3f}, {df['Current_Y'].iloc[0]:.3f}, {df['Current_Z'].iloc[0]:.3f}) m
Final Position:            ({df['Current_X'].iloc[-1]:.3f}, {df['Current_Y'].iloc[-1]:.3f}, {df['Current_Z'].iloc[-1]:.3f}) m
Target Position:           ({df['Target_X'].iloc[-1]:.3f}, {df['Target_Y'].iloc[-1]:.3f}, {df['Target_Z'].iloc[-1]:.3f}) m
Initial Position Error:    {initial_error:.4f} m
Final Position Error:      {final_error:.4f} m
Position Improvement:      {(1 - final_error/initial_error) * 100:.1f}%
Total Distance Traveled:   {total_distance:.3f} m

TIMING ANALYSIS
--------------------------------------------------
Total Test Time:           {df['Control_Time'].iloc[-1]:.2f} s
Control Steps:             {len(df)}
Control Rate:              {len(df) / df['Control_Time'].iloc[-1]:.1f} Hz

MISSION STATUS
--------------------------------------------------
Position Tolerance:        0.100 m
Position Target Met:       {"YES" if final_error < 0.1 else "NO"}
Achieved Precision:        {final_error:.4f} m

================================================================================
END OF MISSION SUMMARY
================================================================================
"""
        
        with open(self.sim_dir / "mission_summary.txt", 'w') as f:
            f.write(summary)
    
    def generate_performance_metrics(self):
        """Generate/Update performance_metrics.json."""
        df = self.df
        metrics_path = self.sim_dir / "performance_metrics.json"
        
        # Load existing C++ metrics if available
        metrics = {}
        if metrics_path.exists():
            try:
                with open(metrics_path, 'r') as f:
                    metrics = json.load(f)
            except json.JSONDecodeError:
                print("⚠ Warning: Could not parse existing metrics JSON")
        
        # Add Python-calculated metrics (Position/Control stats)
        # We preserve 'mpc' and 'physics' from C++ if they exist
        
        # Ensure 'simulation' block exists and has basics
        if 'simulation' not in metrics:
            metrics['simulation'] = {}
            
        metrics['simulation'].update({
            "total_time_s": float(df['Control_Time'].iloc[-1]),
            "total_steps": len(df),
            "control_rate_hz": float(len(df) / df['Control_Time'].iloc[-1])
        })
            
        metrics["position"] = {
            "initial_error_m": float(np.sqrt(df['Error_X'].iloc[0]**2 + df['Error_Y'].iloc[0]**2 + df['Error_Z'].iloc[0]**2)),
            "final_error_m": float(np.sqrt(df['Error_X'].iloc[-1]**2 + df['Error_Y'].iloc[-1]**2 + df['Error_Z'].iloc[-1]**2)),
            "min_error_m": float(np.sqrt(df['Error_X']**2 + df['Error_Y']**2 + df['Error_Z']**2).min()),
            "max_error_m": float(np.sqrt(df['Error_X']**2 + df['Error_Y']**2 + df['Error_Z']**2).max())
        }
        
        metrics["control"] = {
            "num_thrusters": 6,
            "total_thrust_ns": float(sum(
                df[f'Thruster_{i}_Val'].sum() * 0.05 
                for i in range(1, 7) if f'Thruster_{i}_Val' in df.columns
            ))
        }
        
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=2)
    
    def generate_all_plots(self):
        """Generate the full suite of plots (16 total as requested)."""
        plt.style.use('seaborn-v0_8-whitegrid')
        
        print("  • Generating Position Plots...")
        self._plot_error_norms()             # 1. Position Error (Mag)
        self._plot_position_error()          # 2. Components Position Error
        self._plot_position_tracking()       # 3. Components Position
        
        print("  • Generating Angular/Velocity Plots...")
        self._plot_angular_error_norm()      # 4. Angular Error (Mag) - NEW
        self._plot_angular_error()           # 5. Components Angular Error
        self._plot_angular_tracking()        # 6. Components Angle
        self._plot_velocity_magnitude()      # 7. Velocity (Mag)
        self._plot_velocity_tracking()       # 8. Components Velocity
        
        print("  • Generating Actuator/Control Plots...")
        self._plot_thruster_gantt()          # 9. Thruster usage (Bar/Gantt) - NEW
        self._plot_duty_cycles()             # 10. Thruster Duty Cycle
        self._plot_thruster_usage()          # 11. Thruster Usage
        self._plot_control_effort()          # 12. Control Effort
        self._plot_rw_torques()              # 13. Components Reaction Wheel Torque
        self._plot_mpc_solve_time()          # 13. MPC Solver Time (Numbering shared in request)
        
        print("  • Generating Trajectory Plots...")
        self._plot_trajectory_2d()           # 14. Trajectory (2D XY/XZ) - NEW
        self._plot_trajectory_3d_orientation() # 15. Trajectory Orientation
        self._plot_trajectory_3d()           # 16. 3D Trajectory
        
        plt.close('all')

    # =================================================================
    # NEW METHODS FOR REQUESTED PLOTS
    # =================================================================

    def _plot_angular_error_norm(self):
        """4. Angular Error - Show the magnitude of error (deg vs time)."""
        # Error_Roll, Error_Pitch, Error_Yaw are in radians usually? 
        # Actually in DataLogger they are likely radians.
        # Let's assume radians and convert to degrees.
        err_rad_sq = self.df['Error_Roll']**2 + self.df['Error_Pitch']**2 + self.df['Error_Yaw']**2
        err_deg = np.degrees(np.sqrt(err_rad_sq))
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(self.t, err_deg, 'r-', linewidth=2)
        ax.fill_between(self.t, 0, err_deg, color='red', alpha=0.1)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Angular Error Magnitude [°]')
        ax.set_title('Angular Error Magnitude', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'angular_error_magnitude.png', dpi=150)
        plt.close()

    def _plot_thruster_gantt(self):
        """9. Thruster usage - Bar chart - one bar for each thruster (thruster id vs time)."""
        # This implies a Gantt chart or "event plot" where y-axis is Thruster ID and x-axis is time.
        # We can color based on firing magnitude (if continuous) or just binary on/off.
        # Since it's PWM/Continuous, magnitude matters.
        # We can use pcolorcmd or broken_barh, or just scatter/plot with fixed y.
        
        fig, ax = plt.subplots(figsize=(12, 6))
        
        # Plot each thruster as a horizontal strip or line
        # Offset them on Y axis
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628']
        
        for i in range(6):
            col = f'Thruster_{i+1}_Val'
            if col in self.df.columns:
                thrust = self.df[col]
                # Filter noise
                mask = thrust > 0.01
                if mask.any():
                    # Plot as scatter points or line segments
                    # Use scatter for variable thrust
                    ax.scatter(self.t[mask], np.full(mask.sum(), i+1), 
                               s=thrust[mask]*50, c=colors[i], alpha=0.6, label=f'T{i+1}')
                    
        ax.set_yticks(range(1, 7))
        ax.set_yticklabels([f'T{i}' for i in range(1, 7)])
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Thruster ID')
        ax.set_title('Thruster Usage Activity (Gantt)', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'thruster_gantt.png', dpi=150)
        plt.close()

    def _plot_trajectory_2d(self):
        """14. Trajectory - two 2d plots of the trajectory: x-y plane and x-z plane."""
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        
        # XY Plane
        ax = axes[0]
        ax.plot(self.df['Current_X'], self.df['Current_Y'], 'b-', label='Path')
        ax.scatter(self.df['Current_X'].iloc[0], self.df['Current_Y'].iloc[0], c='g', marker='o', label='Start')
        ax.scatter(self.df['Target_X'].iloc[-1], self.df['Target_Y'].iloc[-1], c='gold', marker='*', s=150, label='Target')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_title('Trajectory (X-Y Plane)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        # XZ Plane
        ax = axes[1]
        ax.plot(self.df['Current_X'], self.df['Current_Z'], 'r-', label='Path')
        ax.scatter(self.df['Current_X'].iloc[0], self.df['Current_Z'].iloc[0], c='g', marker='o', label='Start')
        ax.scatter(self.df['Target_X'].iloc[-1], self.df['Target_Z'].iloc[-1], c='gold', marker='*', s=150, label='Target')
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Z [m]')
        ax.set_title('Trajectory (X-Z Plane)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.axis('equal')
        
        fig.suptitle('2D Trajectory Projections', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'trajectory_2d.png', dpi=150)
        plt.close()
    
    def _plot_position_tracking(self):
        """Plot position vs time."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        for i, (ax, dim, color) in enumerate(zip(axes, ['X', 'Y', 'Z'], ['r', 'g', 'b'])):
            ax.plot(self.t, self.df[f'Current_{dim}'], color=color, label=f'Actual {dim}')
            ax.axhline(y=self.df[f'Target_{dim}'].iloc[-1], color=color, linestyle='--', alpha=0.5, label=f'Target {dim}')
            ax.set_ylabel(f'{dim} Position [m]')
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Position Tracking', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'position_tracking.png', dpi=150)
        plt.close()
    
    def _plot_position_error(self):
        """Plot position error components."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        for i, (ax, dim, color) in enumerate(zip(axes, ['X', 'Y', 'Z'], ['r', 'g', 'b'])):
            ax.plot(self.t, self.df[f'Error_{dim}'], color=color)
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax.set_ylabel(f'{dim} Error [m]')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Position Error Components', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'position_error.png', dpi=150)
        plt.close()
    
    def _plot_error_norms(self):
        """Plot error magnitude over time."""
        error_norm = np.sqrt(
            self.df['Error_X']**2 + 
            self.df['Error_Y']**2 + 
            self.df['Error_Z']**2
        )
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(self.t, error_norm, 'b-', linewidth=2)
        ax.axhline(y=0.1, color='g', linestyle='--', label='Tolerance (10cm)')
        ax.axhline(y=0.05, color='orange', linestyle='--', label='Tolerance (5cm)')
        ax.fill_between(self.t, 0, error_norm, alpha=0.3)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Position Error [m]')
        ax.set_title('Position Error Magnitude', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'error_norms.png', dpi=150)
        plt.close()
    
    def _plot_velocity_tracking(self):
        """Plot velocity (estimated from position derivative)."""
        dt = np.diff(self.t)
        vx = np.diff(self.df['Current_X']) / dt
        vy = np.diff(self.df['Current_Y']) / dt
        vz = np.diff(self.df['Current_Z']) / dt
        t_vel = self.t[:-1]
        
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        for ax, v, dim, color in zip(axes, [vx, vy, vz], ['X', 'Y', 'Z'], ['r', 'g', 'b']):
            ax.plot(t_vel, v, color=color)
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax.set_ylabel(f'{dim} Velocity [m/s]')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Velocity Tracking', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'velocity_tracking.png', dpi=150)
        plt.close()
    
    def _plot_trajectory_3d(self):
        """Plot 3D trajectory."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        ax.plot(self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z'], 
                'b-', linewidth=2, label='Trajectory')
        ax.scatter([self.df['Current_X'].iloc[0]], [self.df['Current_Y'].iloc[0]], 
                   [self.df['Current_Z'].iloc[0]], c='green', s=100, marker='o', label='Start')
        ax.scatter([self.df['Current_X'].iloc[-1]], [self.df['Current_Y'].iloc[-1]], 
                   [self.df['Current_Z'].iloc[-1]], c='red', s=100, marker='x', label='End')
        ax.scatter([self.df['Target_X'].iloc[-1]], [self.df['Target_Y'].iloc[-1]], 
                   [self.df['Target_Z'].iloc[-1]], c='gold', s=150, marker='*', label='Target')
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('3D Trajectory', fontsize=14, fontweight='bold')
        ax.legend()
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'trajectory.png', dpi=150)
        plt.close()
    
    def _plot_thruster_usage(self):
        """Plot thruster commands."""
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True, sharey=True)
        axes = axes.flatten()
        
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#a65628']
        
        for i in range(6):
            col = f'Thruster_{i+1}_Val'
            if col in self.df.columns:
                axes[i].fill_between(self.t, 0, self.df[col], color=colors[i], alpha=0.7)
                axes[i].plot(self.t, self.df[col], color=colors[i], linewidth=0.5)
                axes[i].set_title(f'Thruster {i+1}', fontsize=11)
                axes[i].set_ylim(0, 1.1)
                axes[i].grid(True, alpha=0.3)
        
        fig.suptitle('Thruster Usage', fontsize=14, fontweight='bold')
        for ax in axes[-2:]:
            ax.set_xlabel('Time [s]')
        for ax in axes[::2]:
            ax.set_ylabel('Thrust [N]')
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'thruster_usage.png', dpi=150)
        plt.close()
    
    def _plot_control_effort(self):
        """Plot total control effort."""
        total_thrust = sum(
            self.df[f'Thruster_{i}_Val'] 
            for i in range(1, 7) if f'Thruster_{i}_Val' in self.df.columns
        )
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.fill_between(self.t, 0, total_thrust, alpha=0.7, color='steelblue')
        ax.plot(self.t, total_thrust, 'b-', linewidth=1)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Total Thrust [N]')
        ax.set_title('Control Effort (Total Thrust)', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'control_effort.png', dpi=150)
        plt.close()
    
    def _plot_angular_tracking(self):
        """Plot attitude angles."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        for ax, angle, name in zip(axes, ['Roll', 'Pitch', 'Yaw'], ['Roll', 'Pitch', 'Yaw']):
            ax.plot(self.t, np.degrees(self.df[f'Current_{angle}']), 'b-')
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)
            ax.set_ylabel(f'{name} [°]')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Attitude (Euler Angles)', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'angular_tracking.png', dpi=150)
        plt.close()


    def _plot_trajectory_3d_orientation(self):
        """Plot 3D trajectory with orientation vectors."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot Path
        ax.plot(self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z'], 
                'b-', linewidth=1, alpha=0.5, label='Path')
        
        # Quivers (Every Nth step to avoid clutter)
        step = max(1, len(self.df) // 20)
        
        # We need rotation matrix from Roll/Pitch/Yaw for quiver direction (Body X axis usually)
        # Or just use the Euler angles simplistically if we assume small angles, but better to computation.
        # Let's compute a simple "Forward" vector from RPY.
        # X-axis body vector in Inertial frame
        d_df = self.df.iloc[::step]
        
        # Simple RPY to BodyX conversion
        cr = np.cos(d_df['Current_Roll'])
        cp = np.cos(d_df['Current_Pitch'])
        cy = np.cos(d_df['Current_Yaw'])
        sr = np.sin(d_df['Current_Roll'])
        sp = np.sin(d_df['Current_Pitch'])
        sy = np.sin(d_df['Current_Yaw'])
        
        # Body X in Inertial: [ cy*cp, sy*cp, -sp ]
        u = cy * cp
        v = sy * cp
        w = -sp
        
        ax.quiver(d_df['Current_X'], d_df['Current_Y'], d_df['Current_Z'], 
                  u, v, w, length=0.2, normalize=True, color='red', alpha=0.6, label='Body X')
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('3D Trajectory w/ Orientation', fontsize=14, fontweight='bold')
        ax.legend()
        plt.savefig(self.plots_dir / 'trajectory_orientation.png', dpi=150)
        plt.close()

    def _plot_waypoint_progress(self):
        """Plot distance to current target."""
        dist = np.sqrt(
            (self.df['Target_X'] - self.df['Current_X'])**2 +
            (self.df['Target_Y'] - self.df['Current_Y'])**2 +
            (self.df['Target_Z'] - self.df['Current_Z'])**2
        )
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(self.t, dist, 'purple', linewidth=2)
        ax.fill_between(self.t, 0, dist, color='purple', alpha=0.1)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Distance to Target [m]')
        ax.set_title('Waypoint Progress', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'waypoint_progress.png', dpi=150)
        plt.close()

    def _plot_angular_error(self):
        """Plot angular error components."""
        fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        
        for ax, angle, color in zip(axes, ['Roll', 'Pitch', 'Yaw'], ['r', 'g', 'b']):
            # Convert error (radians) to degrees
            err_deg = np.degrees(self.df[f'Error_{angle}'])
            ax.plot(self.t, err_deg, color=color)
            ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            ax.set_ylabel(f'{angle} Error [°]')
            ax.grid(True, alpha=0.3)
        
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Angular Error Components', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'angular_error.png', dpi=150)
        plt.close()

    def _plot_rw_speeds(self):
        """Plot RW speeds."""
        fig, ax = plt.subplots(figsize=(12, 6))
        
        for dim, color in zip(['X', 'Y', 'Z'], ['r', 'g', 'b']):
            ax.plot(self.t, self.df[f'RW_Speed_{dim}'], color=color, label=f'RW {dim}')
            
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [rad/s]')
        ax.set_title('Reaction Wheel Speeds', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'rw_speeds.png', dpi=150)
        plt.close()

    def _plot_rw_torques(self):
        """Plot RW torques."""
        fig, ax = plt.subplots(figsize=(12, 6))
        
        for dim, color in zip(['X', 'Y', 'Z'], ['r', 'g', 'b']):
            ax.plot(self.t, self.df[f'RW_Torque_{dim}'], color=color, label=f'RW {dim}')
            
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Torque [Nm]')
        ax.set_title('Reaction Wheel Torques', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'rw_torques.png', dpi=150)
        plt.close()

    def _plot_momentum_management(self):
        """Plot total RW momentum norm."""
        # Assume generic inertia if not known, or just plot speed norm for now
        # Momentum H = I * w. Let's just plot Speed Norm as proxy for Saturation
        speed_norm = np.sqrt(
            self.df['RW_Speed_X']**2 +
            self.df['RW_Speed_Y']**2 +
            self.df['RW_Speed_Z']**2
        )
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(self.t, speed_norm, 'k-', linewidth=2)
        ax.fill_between(self.t, 0, speed_norm, color='gray', alpha=0.3)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed Norm [rad/s]')
        ax.set_title('RW Momentum (Speed Norm)', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'rw_momentum.png', dpi=150)
        plt.close()

    def _plot_valve_activity(self):
        """Plot cumulative thruster activity."""
        fig, ax = plt.subplots(figsize=(12, 6))
        
        for i in range(6):
            col = f'Thruster_{i+1}_Val'
            if col in self.df.columns:
                # Cumulative sum scaled by dt (0.05 approx)
                activity = np.cumsum(self.df[col]) * 0.05
                ax.plot(self.t, activity, label=f'Thruster {i+1}')
                
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Total On-Time [s]')
        ax.set_title('Valve Activity (Cumulative)', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'valve_activity.png', dpi=150)
        plt.close()

    def _plot_duty_cycles(self):
        """Plot moving average usage."""
        fig, ax = plt.subplots(figsize=(12, 6))
        window = 20 # 1 second window at 20Hz
        
        for i in range(6):
            col = f'Thruster_{i+1}_Val'
            if col in self.df.columns:
                duty = self.df[col].rolling(window=window).mean()
                ax.plot(self.t, duty, label=f'Thruster {i+1}')
                
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Duty Cycle (1s Avg)')
        ax.set_title('Thruster Duty Cycles', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'duty_cycles.png', dpi=150)
        plt.close()

    def _plot_force_distribution(self):
        pass # Todo: Requires body force logging

    def _plot_torque_distribution(self):
        pass # Todo: Requires body torque logging

    def _plot_phase_position(self):
        """Plot Phase Plane: Position vs Velocity."""
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        # Calculate velocity if not present (we assume implicit logging or diff)
        # We have Current_VX/VY/VZ from logging?
        # Check columns
        has_vel = 'Current_VX' in self.df.columns
        
        dims = ['X', 'Y', 'Z']
        colors = ['r', 'g', 'b']
        
        for i, (ax, dim, color) in enumerate(zip(axes, dims, colors)):
            pos = self.df[f'Current_{dim}']
            if has_vel:
                vel = self.df[f'Current_V{dim}'] # Assuming standard naming Current_VX
            else:
                # Diff it
                vel = np.gradient(pos, self.t)
            
            ax.plot(pos, vel, color=color, alpha=0.6)
            ax.set_xlabel(f'{dim} [m]')
            ax.set_ylabel(f'V{dim} [m/s]')
            ax.set_title(f'{dim} Phase Plane')
            ax.grid(True, alpha=0.3)
            
            # Start/End
            ax.plot(pos.iloc[0], vel.iloc[0] if isinstance(vel, pd.Series) else vel[0], 'go')
            ax.plot(pos.iloc[-1], vel.iloc[-1] if isinstance(vel, pd.Series) else vel[-1], 'rx')
            
        fig.suptitle('Position Phase Planes', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'phase_position.png', dpi=150)
        plt.close()

    def _plot_phase_attitude(self):
        """Plot Phase Plane: Angle vs Rate."""
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        modes = ['Roll', 'Pitch', 'Yaw'] # Euler
        rates = ['WX', 'WY', 'WZ'] # Body rates approximation (close enough for small angles)
        colors = ['r', 'g', 'b']
        
        has_rate = 'Current_WX' in self.df.columns # Need to check if I logged WX?
        # DataLogger logs "Current_WX,Current_WY,Current_WZ" - YES.
        
        for i, (ax, mode, rate, color) in enumerate(zip(axes, modes, rates, colors)):
            angle = np.degrees(self.df[f'Current_{mode}'])
            
            if has_rate:
                ang_rate = np.degrees(self.df[f'Current_{rate}'])
            else:
                ang_rate = np.degrees(np.gradient(self.df[f'Current_{mode}'], self.t))
                
            ax.plot(angle, ang_rate, color=color, alpha=0.6)
            ax.set_xlabel(f'{mode} [°]')
            ax.set_ylabel(f'Rate [°/s]')
            ax.set_title(f'{mode} Phase Plane')
            ax.grid(True, alpha=0.3)
            
        fig.suptitle('Attitude Phase Planes', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'phase_attitude.png', dpi=150)
        plt.close()

    def _plot_z_tilt_coupling(self):
        """Plot Tilt vs Z-Error."""
        tilt = np.sqrt(self.df['Current_Roll']**2 + self.df['Current_Pitch']**2)
        z_err = self.df['Error_Z']
        
        fig, ax = plt.subplots(figsize=(8, 6))
        sc = ax.scatter(np.degrees(tilt), z_err, c=self.t, cmap='viridis', alpha=0.5)
        plt.colorbar(sc, label='Time [s]')
        
        ax.set_xlabel('Tilt Angle (Roll+Pitch Norm) [°]')
        ax.set_ylabel('Z Error [m]')
        ax.set_title('Z-Tilt Coupling', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'z_tilt_coupling.png', dpi=150)
        plt.close()

    def _plot_velocity_magnitude(self):
        """Plot velocity norm."""
        dt = np.diff(self.t)
        vx = np.diff(self.df['Current_X']) / dt
        vy = np.diff(self.df['Current_Y']) / dt
        vz = np.diff(self.df['Current_Z']) / dt
        v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
        t_vel = self.t[:-1]
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(t_vel, v_norm, 'b-', linewidth=2)
        ax.fill_between(t_vel, 0, v_norm, color='blue', alpha=0.1)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Speed [m/s]')
        ax.set_title('Velocity Magnitude', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'velocity_magnitude.png', dpi=150)
        plt.close()

    def _plot_mpc_solve_time(self):
        """Plot MPC solve time per step."""
        fig, ax = plt.subplots(figsize=(12, 5))
        
        solve_times = self.df['MPC_Time_ms']
        ax.plot(self.t, solve_times, 'k-', linewidth=1)
        # Highlight spikes
        # mean + 3std?
        limit = solve_times.mean() + 3*solve_times.std()
        ax.axhline(y=limit, color='r', linestyle='--', alpha=0.5, label='3-Sigma')
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Solve Time [ms]')
        ax.set_title('MPC Solver Performance', fontsize=14, fontweight='bold')
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'mpc_solve_time.png', dpi=150)
        plt.close()

    def _plot_timing_intervals(self):
        """Plot control loop jitter."""
        dt = np.diff(self.t)
        target_dt = 0.05
        jitter = dt - target_dt
        
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(self.t[:-1], jitter * 1000, 'b.', markersize=2)
        
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Jitter (dt - 0.05) [ms]')
        ax.set_title('Control Loop Jitter', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        plt.savefig(self.plots_dir / 'timing_intervals.png', dpi=150)
        plt.close()

    def _plot_sim_performance(self):
        """Plot physics vs real time ratio if available? 
           For now just placeholder or use MPC time stats."""
        # Using MPC Time histogram
        fig, ax = plt.subplots(figsize=(10, 6))
        
        if 'MPC_Time_ms' in self.df.columns:
            solve_times = self.df['MPC_Time_ms']
            ax.hist(solve_times, bins=30, color='green', alpha=0.7)
            ax.set_xlabel('MPC Solve Time [ms]')
            ax.set_ylabel('Count')
            ax.set_title('Solver Distribution', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            plt.savefig(self.plots_dir / 'solver_dist.png', dpi=150)
            plt.close()

def find_latest_sim_dir(base_path: Path) -> Path:
    """Find the most recent simulation directory."""
    sim_dirs = sorted(base_path.glob("*"), key=lambda x: x.stat().st_mtime, reverse=True)
    if not sim_dirs:
        raise FileNotFoundError(f"No simulation directories found in {base_path}")
    return sim_dirs[0]


def main():
    base_path = Path("Data/Simulation")
    
    if len(sys.argv) > 1:
        sim_dir = Path(sys.argv[1])
    else:
        sim_dir = find_latest_sim_dir(base_path)
    
    processor = SimulationPostProcessor(sim_dir)
    processor.run_all()
    
    print(f"\n✓ Post-processing complete: {sim_dir}")


if __name__ == "__main__":
    main()
