"""
Mission Types and Data Structures

Defines the core mission types for orbital inspection:
- Flyby: Single pass by target
- Circumnavigation: Orbit around target
- Station-Keeping: Hold position relative to target
- Inspection: Multi-point detailed inspection
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional
import numpy as np
import json
from pathlib import Path


class MissionType(Enum):
    """Types of inspection missions."""

    FLYBY = "flyby"
    CIRCUMNAVIGATION = "circumnavigation"
    STATION_KEEPING = "station_keeping"
    INSPECTION = "inspection"
    CUSTOM = "custom"


class MissionStatus(Enum):
    """Status of mission execution."""

    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    FAILED = "failed"


@dataclass
class Waypoint:
    """A single waypoint in the mission trajectory."""

    position: np.ndarray  # [x, y, z] meters
    hold_time: float = 0.0  # Time to hold at waypoint (seconds)
    approach_speed: float = 0.05  # m/s
    name: str = ""

    def to_dict(self) -> dict:
        return {
            "position": self.position.tolist(),
            "hold_time": self.hold_time,
            "approach_speed": self.approach_speed,
            "name": self.name,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Waypoint":
        return cls(
            position=np.array(data["position"]),
            hold_time=data.get("hold_time", 0.0),
            approach_speed=data.get("approach_speed", 0.05),
            name=data.get("name", ""),
        )


@dataclass
class MissionPhase:
    """A phase of the mission with multiple waypoints."""

    name: str
    waypoints: List[Waypoint] = field(default_factory=list)
    description: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "waypoints": [wp.to_dict() for wp in self.waypoints],
            "description": self.description,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "MissionPhase":
        return cls(
            name=data["name"],
            waypoints=[Waypoint.from_dict(wp) for wp in data.get("waypoints", [])],
            description=data.get("description", ""),
        )


class ObstacleType(Enum):
    """Types of obstacles for collision avoidance."""

    SPHERE = "sphere"
    CYLINDER = "cylinder"
    BOX = "box"


@dataclass
class Obstacle:
    """
    An obstacle for collision avoidance (V3.0.0).

    Defines keep-out zones the satellite must avoid.
    """

    type: ObstacleType = ObstacleType.SPHERE
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    radius: float = 0.5  # For sphere/cylinder [m]
    size: np.ndarray = field(default_factory=lambda: np.ones(3))  # For box [m]
    name: str = "obstacle"

    def to_dict(self) -> dict:
        return {
            "type": self.type.value,
            "position": self.position.tolist(),
            "radius": self.radius,
            "size": self.size.tolist(),
            "name": self.name,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Obstacle":
        return cls(
            type=ObstacleType(data.get("type", "sphere")),
            position=np.array(data.get("position", [0, 0, 0])),
            radius=data.get("radius", 0.5),
            size=np.array(data.get("size", [1, 1, 1])),
            name=data.get("name", "obstacle"),
        )


@dataclass
class Mission:
    """
    Complete mission definition.

    A mission consists of:
    - Metadata (name, type, description)
    - Start position
    - One or more phases with waypoints
    - Safety parameters
    """

    name: str
    mission_type: MissionType
    phases: List[MissionPhase] = field(default_factory=list)
    start_position: np.ndarray = field(
        default_factory=lambda: np.array([5.0, 0.0, 0.0])
    )
    description: str = ""

    # Safety parameters
    keep_out_radius: float = 2.0  # Minimum distance from target (m)
    max_speed: float = 0.1  # Maximum approach speed (m/s)

    # Collision avoidance obstacles (V3.0.0)
    obstacles: List[Obstacle] = field(default_factory=list)

    # Execution state
    status: MissionStatus = MissionStatus.PENDING
    current_phase_idx: int = 0
    current_waypoint_idx: int = 0

    @property
    def total_waypoints(self) -> int:
        return sum(len(phase.waypoints) for phase in self.phases)

    @property
    def current_phase(self) -> Optional[MissionPhase]:
        if 0 <= self.current_phase_idx < len(self.phases):
            return self.phases[self.current_phase_idx]
        return None

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        phase = self.current_phase
        if phase and 0 <= self.current_waypoint_idx < len(phase.waypoints):
            return phase.waypoints[self.current_waypoint_idx]
        return None

    def get_all_waypoints(self) -> List[Waypoint]:
        """Get all waypoints from all phases in order."""
        waypoints = []
        for phase in self.phases:
            waypoints.extend(phase.waypoints)
        return waypoints

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "mission_type": self.mission_type.value,
            "description": self.description,
            "start_position": self.start_position.tolist(),
            "keep_out_radius": self.keep_out_radius,
            "max_speed": self.max_speed,
            "phases": [phase.to_dict() for phase in self.phases],
            "obstacles": [obs.to_dict() for obs in self.obstacles],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Mission":
        return cls(
            name=data["name"],
            mission_type=MissionType(data["mission_type"]),
            description=data.get("description", ""),
            start_position=np.array(data.get("start_position", [5.0, 0.0, 0.0])),
            keep_out_radius=data.get("keep_out_radius", 2.0),
            max_speed=data.get("max_speed", 0.1),
            phases=[MissionPhase.from_dict(p) for p in data.get("phases", [])],
            obstacles=[Obstacle.from_dict(o) for o in data.get("obstacles", [])],
        )

    def save(self, filepath: Path):
        """Save mission to JSON file."""
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, filepath: Path) -> "Mission":
        """Load mission from JSON file."""
        with open(filepath, "r") as f:
            return cls.from_dict(json.load(f))
