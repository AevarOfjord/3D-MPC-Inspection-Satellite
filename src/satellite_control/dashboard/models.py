"""Pydantic request/response models for the dashboard API."""

from typing import Annotated, List, Optional, Literal, Union
from pydantic import BaseModel, Field, field_validator


class ObstacleModel(BaseModel):
    position: List[float]
    radius: float


class MeshScanConfigModel(BaseModel):
    obj_path: str
    standoff: float = 0.5
    levels: int = 8
    level_spacing: Optional[float] = None  # distance between levels in meters
    points_per_circle: int = 72
    speed_max: float = 0.2
    speed_min: float = 0.05
    lateral_accel: float = 0.05
    z_margin: float = 0.0
    scan_axis: str = "Z"  # "X", "Y", or "Z"
    pattern: str = "rings"  # "rings" or "spiral"


class PathAssetSaveRequest(BaseModel):
    name: str
    obj_path: str
    path: List[List[float]]
    open: bool = True
    relative_to_obj: bool = True
    notes: Optional[str] = None


class PoseModel(BaseModel):
    frame: Literal["ECI", "LVLH"]
    position: List[float]
    orientation: Optional[List[float]] = None  # quaternion [w, x, y, z]

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: List[float]) -> List[float]:
        if len(value) != 3:
            raise ValueError("position must have length 3")
        return value

    @field_validator("orientation")
    @classmethod
    def validate_orientation(
        cls, value: Optional[List[float]]
    ) -> Optional[List[float]]:
        if value is None:
            return value
        if len(value) != 4:
            raise ValueError("orientation must have length 4")
        return value


class ConstraintsModel(BaseModel):
    speed_max: Optional[float] = None
    accel_max: Optional[float] = None
    angular_rate_max: Optional[float] = None


class SplineControlModel(BaseModel):
    position: List[float]
    weight: float = 1.0

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: List[float]) -> List[float]:
        if len(value) != 3:
            raise ValueError("spline control position must have length 3")
        return value


class TransferSegmentModel(BaseModel):
    type: Literal["transfer"]
    target_id: Optional[str] = None
    end_pose: PoseModel
    constraints: Optional[ConstraintsModel] = None


class ScanConfigModel(BaseModel):
    frame: Literal["ECI", "LVLH"] = "LVLH"
    axis: Literal["+X", "-X", "+Y", "-Y", "+Z", "-Z", "custom"] = "+Z"
    standoff: float = 10.0
    overlap: float = 0.25
    fov_deg: float = 60.0
    pitch: Optional[float] = None
    revolutions: int = 4
    direction: Literal["CW", "CCW"] = "CW"
    sensor_axis: Literal["+Y", "-Y"] = "+Y"


class ScanSegmentModel(BaseModel):
    type: Literal["scan"]
    target_id: str
    target_pose: Optional[PoseModel] = None
    scan: ScanConfigModel
    path_asset: Optional[str] = None
    constraints: Optional[ConstraintsModel] = None


class HoldSegmentModel(BaseModel):
    type: Literal["hold"]
    duration: float = 0.0
    constraints: Optional[ConstraintsModel] = None


MissionSegmentModel = Annotated[
    Union[TransferSegmentModel, ScanSegmentModel, HoldSegmentModel],
    Field(discriminator="type"),
]


class MissionOverridesModel(BaseModel):
    spline_controls: List[SplineControlModel] = Field(default_factory=list)


class UnifiedMissionModel(BaseModel):
    epoch: str
    start_pose: PoseModel
    start_target_id: Optional[str] = None
    segments: List[MissionSegmentModel]
    obstacles: List[ObstacleModel] = Field(default_factory=list)
    overrides: Optional[MissionOverridesModel] = None


class PreviewUnifiedMissionResponse(BaseModel):
    path: List[List[float]]
    path_length: float
    path_speed: float


class ControlCommand(BaseModel):
    action: Literal["pause", "resume", "step"]
    steps: int = 1


class SpeedCommand(BaseModel):
    speed: float


class SaveUnifiedMissionRequest(BaseModel):
    name: str
    config: UnifiedMissionModel


class RunMissionRequest(BaseModel):
    mission_name: str
