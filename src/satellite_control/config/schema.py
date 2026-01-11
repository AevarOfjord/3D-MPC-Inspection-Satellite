from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class ThrusterConfig:
    position: List[float]  # [x, y, z]
    direction: List[float]  # [x, y, z] normal vector
    max_thrust: float
    min_impulse_bit: float
    group: str = "main"  # e.g., "main", "rcs"


@dataclass
class ReactionWheelConfig:
    axis: List[float]  # [x, y, z] axis of rotation
    max_torque: float
    max_momentum: float
    max_speed_rad_s: float
    inertia: float


@dataclass
class VehicleConfig:
    mass: float
    inertia: List[
        float
    ]  # [Ixx, Iyy, Izz] (diagonal assumption for simplicity initially)
    center_of_mass: List[float]
    thrusters: List[ThrusterConfig] = field(default_factory=list)
    reaction_wheels: List[ReactionWheelConfig] = field(default_factory=list)
    geometry_file: Optional[str] = None  # Path to .obj/.stl for visualizer
    mesh_scale: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


@dataclass
class MPCWeights:
    position: float
    velocity: float
    angle: float
    angular_velocity: float
    thrust: float  # R matrix diagonal
    rw_torque: float  # R matrix diagonal


@dataclass
class MPCConfig:
    prediction_horizon: int
    control_horizon: int
    weights: MPCWeights
    solver_time_limit: float = 0.01


@dataclass
class SimulationConfig:
    dt: float = 0.01
    duration: float = 10.0


@dataclass
class Config:
    vehicle: VehicleConfig
    mission: str  # Placeholder for full mission config
    control: MPCConfig
    env: str  # Placeholder
    sim: SimulationConfig
