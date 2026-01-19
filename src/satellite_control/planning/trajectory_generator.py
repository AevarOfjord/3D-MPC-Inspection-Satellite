"""
Trajectory Generator/Smoother

Converts a list of waypoints (from RRT*) into a time-parameterized trajectory
that the MPC controller can track.
"""

import numpy as np
from typing import List, Tuple, Optional


class TrajectoryGenerator:
    def __init__(self, avg_velocity=0.2):
        self.avg_velocity = avg_velocity

    def generate_trajectory(self, waypoints: List[np.ndarray], dt=0.1) -> np.ndarray:
        """
        Generate a dense trajectory from sparse waypoints.
        Returns array of shape (N, 7): [time, x, y, z, vx, vy, vz]
        """
        if len(waypoints) < 2:
            return np.array([])

        trajectory = []
        current_time = 0.0

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]

            dist = np.linalg.norm(end - start)

            # Time to travel this segment
            duration = dist / self.avg_velocity
            if duration < dt:
                duration = dt  # Avoid div by zero or tiny steps

            steps = int(duration / dt)

            # Constant velocity for this segment
            velocity = (end - start) / duration

            for s in range(steps):
                t = s * dt
                # Linear Interpolation
                pos = start + velocity * t

                point = [
                    current_time + t,
                    pos[0],
                    pos[1],
                    pos[2],
                    velocity[0],
                    velocity[1],
                    velocity[2],
                ]
                trajectory.append(point)

            current_time += duration

        # Add final point
        last = waypoints[-1]
        trajectory.append([current_time, last[0], last[1], last[2], 0.0, 0.0, 0.0])

        return np.array(trajectory)
