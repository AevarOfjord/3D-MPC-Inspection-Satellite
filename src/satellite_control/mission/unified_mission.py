"""
Unified mission schema for the single-builder workflow.

This module defines the data model for missions that mix transfer and scan
segments into a single optimized trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Union


class Frame(str, Enum):
    ECI = "ECI"
    LVLH = "LVLH"


class SegmentType(str, Enum):
    TRANSFER = "transfer"
    SCAN = "scan"
    HOLD = "hold"


class SpiralDirection(str, Enum):
    CW = "CW"
    CCW = "CCW"


class SensorAxis(str, Enum):
    PLUS_Y = "+Y"
    MINUS_Y = "-Y"


class SpiralAxis(str, Enum):
    PLUS_X = "+X"
    MINUS_X = "-X"
    PLUS_Y = "+Y"
    MINUS_Y = "-Y"
    PLUS_Z = "+Z"
    MINUS_Z = "-Z"
    CUSTOM = "custom"


@dataclass
class MissionObstacle:
    position: List[float]
    radius: float

    def to_dict(self) -> Dict[str, Any]:
        return {"position": list(self.position), "radius": float(self.radius)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionObstacle":
        return cls(
            position=list(data.get("position", [0.0, 0.0, 0.0])),
            radius=float(data.get("radius", 0.5)),
        )


@dataclass
class Pose:
    frame: Frame
    position: List[float]
    orientation: Optional[List[float]] = None  # quaternion [w, x, y, z]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame": self.frame.value,
            "position": list(self.position),
            "orientation": list(self.orientation) if self.orientation is not None else None,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Pose":
        return cls(
            frame=Frame(data["frame"]),
            position=list(data["position"]),
            orientation=list(data["orientation"]) if data.get("orientation") is not None else None,
        )


@dataclass
class Constraints:
    speed_max: Optional[float] = None
    accel_max: Optional[float] = None
    angular_rate_max: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "speed_max": self.speed_max,
            "accel_max": self.accel_max,
            "angular_rate_max": self.angular_rate_max,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Constraints":
        if not data:
            data = {}
        return cls(
            speed_max=data.get("speed_max"),
            accel_max=data.get("accel_max"),
            angular_rate_max=data.get("angular_rate_max"),
        )


@dataclass
class SplineControl:
    position: List[float]
    weight: float = 1.0

    def to_dict(self) -> Dict[str, Any]:
        return {"position": list(self.position), "weight": float(self.weight)}

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "SplineControl":
        return cls(position=list(data["position"]), weight=float(data.get("weight", 1.0)))


@dataclass
class SegmentBase:
    type: SegmentType
    constraints: Constraints = field(default_factory=Constraints)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "constraints": self.constraints.to_dict(),
        }


@dataclass
class TransferSegment(SegmentBase):
    end_pose: Pose = field(default_factory=lambda: Pose(Frame.ECI, [0.0, 0.0, 0.0]))

    def __post_init__(self) -> None:
        self.type = SegmentType.TRANSFER

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data["end_pose"] = self.end_pose.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TransferSegment":
        constraints = Constraints.from_dict(data.get("constraints", {}))
        end_pose = Pose.from_dict(data["end_pose"])
        return cls(type=SegmentType.TRANSFER, constraints=constraints, end_pose=end_pose)


@dataclass
class ScanConfig:
    frame: Frame = Frame.LVLH
    axis: SpiralAxis = SpiralAxis.PLUS_Z
    standoff: float = 10.0
    overlap: float = 0.25
    fov_deg: float = 60.0
    pitch: Optional[float] = None  # None means "auto"
    revolutions: int = 4
    direction: SpiralDirection = SpiralDirection.CW
    sensor_axis: SensorAxis = SensorAxis.PLUS_Y

    def to_dict(self) -> Dict[str, Any]:
        return {
            "frame": self.frame.value,
            "axis": self.axis.value,
            "standoff": self.standoff,
            "overlap": self.overlap,
            "fov_deg": self.fov_deg,
            "pitch": self.pitch,
            "revolutions": self.revolutions,
            "direction": self.direction.value,
            "sensor_axis": self.sensor_axis.value,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ScanConfig":
        if not data:
            data = {}
        return cls(
            frame=Frame(data.get("frame", Frame.LVLH.value)),
            axis=SpiralAxis(data.get("axis", SpiralAxis.PLUS_Z.value)),
            standoff=float(data.get("standoff", 10.0)),
            overlap=float(data.get("overlap", 0.25)),
            fov_deg=float(data.get("fov_deg", 60.0)),
            pitch=data.get("pitch"),
            revolutions=int(data.get("revolutions", 4)),
            direction=SpiralDirection(data.get("direction", SpiralDirection.CW.value)),
            sensor_axis=SensorAxis(data.get("sensor_axis", SensorAxis.PLUS_Y.value)),
        )


@dataclass
class ScanSegment(SegmentBase):
    target_id: str = ""
    target_pose: Optional[Pose] = None
    scan: ScanConfig = field(default_factory=ScanConfig)

    def __post_init__(self) -> None:
        self.type = SegmentType.SCAN

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data["target_id"] = self.target_id
        data["target_pose"] = self.target_pose.to_dict() if self.target_pose else None
        data["scan"] = self.scan.to_dict()
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "ScanSegment":
        constraints = Constraints.from_dict(data.get("constraints") or {})
        scan = ScanConfig.from_dict(data.get("scan") or {})
        target_pose = None
        if data.get("target_pose"):
            target_pose = Pose.from_dict(data["target_pose"])
        return cls(
            type=SegmentType.SCAN,
            constraints=constraints,
            target_id=data.get("target_id", ""),
            target_pose=target_pose,
            scan=scan,
        )


@dataclass
class HoldSegment(SegmentBase):
    duration: float = 0.0

    def __post_init__(self) -> None:
        self.type = SegmentType.HOLD

    def to_dict(self) -> Dict[str, Any]:
        data = super().to_dict()
        data["duration"] = self.duration
        return data

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "HoldSegment":
        constraints = Constraints.from_dict(data.get("constraints") or {})
        return cls(type=SegmentType.HOLD, constraints=constraints, duration=float(data.get("duration", 0.0)))


Segment = Union[TransferSegment, ScanSegment, HoldSegment]


@dataclass
class MissionOverrides:
    spline_controls: List[SplineControl] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "spline_controls": [c.to_dict() for c in self.spline_controls],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionOverrides":
        if not data:
            data = {}
        controls = [SplineControl.from_dict(c) for c in data.get("spline_controls", [])]
        return cls(spline_controls=controls)


@dataclass
class MissionDefinition:
    epoch: str
    start_pose: Pose
    segments: List[Segment] = field(default_factory=list)
    obstacles: List[MissionObstacle] = field(default_factory=list)
    overrides: MissionOverrides = field(default_factory=MissionOverrides)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "epoch": self.epoch,
            "start_pose": self.start_pose.to_dict(),
            "segments": [segment.to_dict() for segment in self.segments],
            "obstacles": [o.to_dict() for o in self.obstacles],
            "overrides": self.overrides.to_dict(),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MissionDefinition":
        segments: List[Segment] = []
        for segment in data.get("segments") or []:
            seg_type = segment.get("type")
            if seg_type == SegmentType.TRANSFER.value:
                segments.append(TransferSegment.from_dict(segment))
            elif seg_type == SegmentType.SCAN.value:
                segments.append(ScanSegment.from_dict(segment))
            elif seg_type == SegmentType.HOLD.value:
                segments.append(HoldSegment.from_dict(segment))
            else:
                raise ValueError(f"Unknown segment type: {seg_type}")

        overrides = MissionOverrides.from_dict(data.get("overrides") or {})
        obstacles = [MissionObstacle.from_dict(o) for o in (data.get("obstacles") or [])]
        return cls(
            epoch=str(data["epoch"]),
            start_pose=Pose.from_dict(data["start_pose"]),
            segments=segments,
            obstacles=obstacles,
            overrides=overrides,
        )
