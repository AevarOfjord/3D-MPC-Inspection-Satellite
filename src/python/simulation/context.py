"""
Simulation Context Module

Defines the SimulationContext data class which holds the instantaneous state
of the simulation. This facilitates passing state between components without
long argument lists.
"""

from dataclasses import dataclass, field

import numpy as np


@dataclass
class SimulationContext:
    """
    Holds the state of the simulation at a given timestep.
    """

    # Time
    simulation_time: float = 0.0
    dt: float = 0.005  # Physics timestep
    control_dt: float = 0.1  # Control update interval
    step_number: int = 0
    # State [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
    current_state: np.ndarray = field(default_factory=lambda: np.zeros(13))
    reference_state: np.ndarray = field(default_factory=lambda: np.zeros(13))
    # Frame Origin (ECI)
    frame_origin: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Mission
    mission_phase: str = "IDLE"
    waypoint_number: int = 0
    # Control
    last_control_update_time: float = 0.0
    active_thrusters: list[int] = field(default_factory=list)
    previous_thruster_command: np.ndarray | None = None
    rw_torque_command: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Performance / Internals
    computation_time_last_step: float = 0.0

    def update_state(
        self,
        time: float,
        state: np.ndarray,
        reference_state: np.ndarray,
        frame_origin: np.ndarray | None = None,
    ) -> None:
        """Update dynamic state variables."""
        self.simulation_time = float(time)
        self.current_state = np.array(state, dtype=np.float64, copy=True)
        self.reference_state = np.array(reference_state, dtype=np.float64, copy=True)
        if frame_origin is not None:
            self.frame_origin = np.array(frame_origin, dtype=np.float64, copy=True)
