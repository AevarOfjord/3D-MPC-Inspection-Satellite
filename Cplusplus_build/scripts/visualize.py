#!/usr/bin/env python3
"""
3D Visualization of Satellite Simulation using Matplotlib Animation.
Shows real-time playback of position and attitude from control_data.csv.
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import numpy as np
import sys
from pathlib import Path

class SatelliteVisualizer:
    def __init__(self, sim_dir: Path):
        self.sim_dir = sim_dir
        self.csv_path = sim_dir / "control_data.csv"
        
        if not self.csv_path.exists():
            raise FileNotFoundError(f"control_data.csv not found in {sim_dir}")
            
        print(f"Loading data from {self.csv_path}...")
        self.df = pd.read_csv(self.csv_path)
        
        # Check for required columns
        req_cols = ['Current_X', 'Current_Y', 'Current_Z', 
                    'Current_Roll', 'Current_Pitch', 'Current_Yaw']
        for c in req_cols:
            if c not in self.df.columns:
                print(f"Error: Missing column {c}")
                sys.exit(1)
                
        # Downsample for smooth animation if too large
        # Aim for ~30-60fps playback
        data_len = len(self.df)
        if data_len > 1000:
            step = data_len // 1000
            self.df = self.df.iloc[::step].reset_index(drop=True)
            
        self.n_frames = len(self.df)
        
        # Cube vertices (centered at origin, side length 0.3)
        r = 0.15
        self.cube_v = np.array([
            [-r, -r, -r], [r, -r, -r], [r, r, -r], [-r, r, -r], # Bottom
            [-r, -r, r], [r, -r, r], [r, r, r], [-r, r, r]      # Top
        ])
        # Edges by vertex indices
        self.cube_edges = [
            (0,1), (1,2), (2,3), (3,0), # Bottom
            (4,5), (5,6), (6,7), (7,4), # Top
            (0,4), (1,5), (2,6), (3,7)  # Verticals
        ]
        
        self.setup_plot()

    def setup_plot(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.fig.suptitle(f"Satellite Simulation: {self.sim_dir.name}", fontsize=14)
        
        # Plot full trajectory (static background)
        self.ax.plot(self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z'], 
                     'k--', alpha=0.3, label='Path')
        
        # Target
        tx, ty, tz = self.df['Target_X'].iloc[0], self.df['Target_Y'].iloc[0], self.df['Target_Z'].iloc[0]
        self.ax.scatter([tx], [ty], [tz], c='gold', marker='*', s=200, label='Target')
        
        # Dynamic Elements
        # Satellite Position Marker
        self.sat_dot, = self.ax.plot([], [], [], 'bo', markersize=8, label='Satellite')
        
        # Attitude Quivers (Body Axes)
        # We will update these segments in animate
        # Store quivers in a list [X_axis, Y_axis, Z_axis]
        self.quivers = []
        colors = ['r', 'g', 'b'] # X=Red, Y=Green, Z=Blue
        for c in colors:
            self.quivers.append(self.ax.quiver(0,0,0, 0,0,0, color=c, length=0.2, normalize=True))
            
        # Text Info
        self.time_text = self.fig.text(0.05, 0.9, "", fontsize=10, fontfamily='monospace')
        self.pos_text = self.fig.text(0.05, 0.85, "", fontsize=10, fontfamily='monospace')
        self.att_text = self.fig.text(0.05, 0.80, "", fontsize=10, fontfamily='monospace')

        # Set limits
        pad = 0.2
        self.ax.set_xlim(self.df['Current_X'].min()-pad, self.df['Current_X'].max()+pad)
        self.ax.set_ylim(self.df['Current_Y'].min()-pad, self.df['Current_Y'].max()+pad)
        self.ax.set_zlim(self.df['Current_Z'].min()-pad, self.df['Current_Z'].max()+pad)
        
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.legend()
        self.ax.grid(True)

    def get_rotation_matrix(self, r, p, y):
        """Convert Euler angles to Rotation Matrix (ZYX convention)."""
        # Roll (X), Pitch (Y), Yaw (Z)
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    def update(self, frame):
        row = self.df.iloc[frame]
        
        # Position
        x, y, z = row['Current_X'], row['Current_Y'], row['Current_Z']
        self.sat_dot.set_data([x], [y])
        self.sat_dot.set_3d_properties([z])
        
        # Attitude
        # Calculate body axes in inertial frame
        # Body X (1,0,0), Y(0,1,0), Z(0,0,1) rotated
        R = self.get_rotation_matrix(row['Current_Roll'], row['Current_Pitch'], row['Current_Yaw'])
        
        origin = np.array([x, y, z])
        axes = np.eye(3) # [X, Y, Z] vectors
        
        # Remove old quivers and create new ones (Matplotlib 3D quiver update is tricky/broken in older versions)
        # Simplest way is remove and replot, but that's slow.
        # Efficient way: set_segments (for Line3DCollection)? No, quiver returns Quiver3D object.
        # Let's try removing and clearing.
        
        # Actually, for 3D animation, 'set_segments' works if using Line3D.
        # Quiver is hard to update. Let's use simple lines for axes.
        
        return self.sat_dot,

    def run(self):
        # Using a simpler update loop because managing Quiver3D artist updates is complex across MPL versions
        # Restarting fig logic
        plt.close(self.fig)
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Static
        self.ax.plot(self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z'], 'k--', alpha=0.2)
        tx = self.df['Target_X'].iloc[-1] # Use last target
        ty = self.df['Target_Y'].iloc[-1]
        tz = self.df['Target_Z'].iloc[-1]
        self.ax.scatter([tx], [ty], [tz], c='gold', marker='*', s=150)
        
        # Dynamic objects
        self.cube_lines = [self.ax.plot([], [], [], 'k-', linewidth=1)[0] for _ in range(12)]
        self.lines = [self.ax.plot([],[],[], color=c, linewidth=2)[0] for c in ['r','g','b']]
        
        # Limits
        all_vals = np.concatenate([self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z']])
        vmin, vmax = all_vals.min(), all_vals.max()
        pad = 0.5
        self.ax.set_xlim(vmin-pad, vmax+pad)
        self.ax.set_ylim(vmin-pad, vmax+pad)
        self.ax.set_zlim(vmin-pad, vmax+pad)
        
        time_template = 'Time = %.1fs'
        self.time_text = self.fig.text(0.05, 0.95, '')

        # Cube vertices (centered at origin, side length 0.3)
        r = 0.15
        self.cube_v = np.array([
            [-r, -r, -r], [r, -r, -r], [r, r, -r], [-r, r, -r], # Bottom
            [-r, -r, r], [r, -r, r], [r, r, r], [-r, r, r]      # Top
        ])
        # Edges by vertex indices
        self.cube_edges = [
            (0,1), (1,2), (2,3), (3,0), # Bottom
            (4,5), (5,6), (6,7), (7,4), # Top
            (0,4), (1,5), (2,6), (3,7)  # Verticals
        ]

        def animate(i):
            row = self.df.iloc[i]
            x, y, z = row['Current_X'], row['Current_Y'], row['Current_Z']
            
            # Attitude Matrix
            R = self.get_rotation_matrix(row['Current_Roll'], row['Current_Pitch'], row['Current_Yaw'])
            
            # Update Cube
            rot_v = (R @ self.cube_v.T).T + np.array([x, y, z])
            
            for line, (i1, i2) in zip(self.cube_lines, self.cube_edges):
                p1 = rot_v[i1]
                p2 = rot_v[i2]
                line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
                line.set_3d_properties([p1[2], p2[2]])
            
            self.time_text.set_text(time_template % row['Control_Time'])
            
            # Attitude Axes
            length = 0.3
            for idx in range(3):
                vec = R[:, idx] * length
                self.lines[idx].set_data([x, x+vec[0]], [y, y+vec[1]])
                self.lines[idx].set_3d_properties([z, z+vec[2]])
            
            return self.time_text, *self.cube_lines, *self.lines

    def save(self, output_path: Path):
        """Save animation to MP4 file."""
        print(f"Saving animation to {output_path}...")
        
        # Set up writer
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=30, metadata=dict(artist='SatelliteSim'), bitrate=1800)
        
        # Create fresh figure for saving (no display)
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Setup static elements again
        self.ax.plot(self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z'], 'k--', alpha=0.3)
        tx, ty, tz = self.df['Target_X'].iloc[0], self.df['Target_Y'].iloc[0], self.df['Target_Z'].iloc[0]
        self.ax.scatter([tx], [ty], [tz], c='gold', marker='*', s=200)
        
        self.cube_lines = [self.ax.plot([], [], [], 'k-', linewidth=1)[0] for _ in range(12)]
        self.lines = [self.ax.plot([],[],[], color=c, linewidth=2)[0] for c in ['r','g','b']]
        
        # Limits
        all_vals = np.concatenate([self.df['Current_X'], self.df['Current_Y'], self.df['Current_Z']])
        vmin, vmax = all_vals.min(), all_vals.max()
        pad = 0.5
        self.ax.set_xlim(vmin-pad, vmax+pad)
        self.ax.set_ylim(vmin-pad, vmax+pad)
        self.ax.set_zlim(vmin-pad, vmax+pad)
        
        time_template = 'Time = %.1fs'
        self.time_text = self.fig.text(0.05, 0.95, '')

        # Animation function (same as run)
        def animate(i):
            row = self.df.iloc[i]
            x, y, z = row['Current_X'], row['Current_Y'], row['Current_Z']
            R = self.get_rotation_matrix(row['Current_Roll'], row['Current_Pitch'], row['Current_Yaw'])
            
            rot_v = (R @ self.cube_v.T).T + np.array([x, y, z])
            for line, (i1, i2) in zip(self.cube_lines, self.cube_edges):
                p1 = rot_v[i1]
                p2 = rot_v[i2]
                line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
                line.set_3d_properties([p1[2], p2[2]])
            
            length = 0.3
            for idx in range(3):
                vec = R[:, idx] * length
                self.lines[idx].set_data([x, x+vec[0]], [y, y+vec[1]])
                self.lines[idx].set_3d_properties([z, z+vec[2]])
                
            self.time_text.set_text(time_template % row['Control_Time'])
            return self.time_text, *self.cube_lines, *self.lines

        ani = animation.FuncAnimation(self.fig, animate, frames=len(self.df), blit=False)
        ani.save(output_path, writer=writer)
        print("Save complete.")

def find_latest_sim(base_path):
    sims = sorted(base_path.glob("*"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not sims:
        return None
    return sims[0]

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Visualize Satellite Simulation")
    parser.add_argument("sim_dir", nargs="?", type=Path, help="Path to simulation directory")
    parser.add_argument("--save", action="store_true", help="Save animation to MP4")
    args = parser.parse_args()

    if args.sim_dir:
        path = args.sim_dir
    else:
        path = find_latest_sim(Path("Data/Simulation"))
        
    if not path or not path.exists():
        print("No simulation data found.")
        sys.exit(1)
        
    print(f"Visualizing: {path}")
    viz = SatelliteVisualizer(path)
    
    if args.save:
        viz.save(path / "animation.mp4")
    else:
        viz.run()
