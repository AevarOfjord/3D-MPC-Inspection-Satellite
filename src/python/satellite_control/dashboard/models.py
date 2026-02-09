"""Pydantic request/response models for the dashboard API."""

from typing import Annotated, Literal

from pydantic import BaseModel, Field, field_validator


class ObstacleModel(BaseModel):
    position: list[float]
    radius: float


class MeshScanConfigModel(BaseModel):
    obj_path: str
    standoff: float = 0.5
    levels: int = 8
    level_spacing: float | None = None  # distance between levels in meters
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
    path: list[list[float]]
    open: bool = True
    relative_to_obj: bool = True
    notes: str | None = None


class PoseModel(BaseModel):
    frame: Literal["ECI", "LVLH"]
    position: list[float]
    orientation: list[float] | None = None  # quaternion [w, x, y, z]

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        if len(value) != 3:
            raise ValueError("position must have length 3")
        return value

    @field_validator("orientation")
    @classmethod
    def validate_orientation(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 4:
            raise ValueError("orientation must have length 4")
        return value


class ConstraintsModel(BaseModel):
    speed_max: float | None = None
    accel_max: float | None = None
    angular_rate_max: float | None = None


class SplineControlModel(BaseModel):
    position: list[float]
    weight: float = 1.0

    @field_validator("position")
    @classmethod
    def validate_position(cls, value: list[float]) -> list[float]:
        if len(value) != 3:
            raise ValueError("spline control position must have length 3")
        return value


class TransferSegmentModel(BaseModel):
    type: Literal["transfer"]
    target_id: str | None = None
    end_pose: PoseModel
    constraints: ConstraintsModel | None = None


class ScanConfigModel(BaseModel):
    frame: Literal["ECI", "LVLH"] = "LVLH"
    axis: Literal["+X", "-X", "+Y", "-Y", "+Z", "-Z", "custom"] = "+Z"
    standoff: float = 10.0
    overlap: float = 0.25
    fov_deg: float = 60.0
    pitch: float | None = None
    revolutions: int = 4
    direction: Literal["CW", "CCW"] = "CW"
    sensor_axis: Literal["+Y", "-Y"] = "+Y"


class ScanSegmentModel(BaseModel):
    type: Literal["scan"]
    target_id: str
    target_pose: PoseModel | None = None
    scan: ScanConfigModel
    path_asset: str | None = None
    constraints: ConstraintsModel | None = None


class HoldSegmentModel(BaseModel):
    type: Literal["hold"]
    duration: float = 0.0
    constraints: ConstraintsModel | None = None


MissionSegmentModel = Annotated[
    TransferSegmentModel | ScanSegmentModel | HoldSegmentModel,
    Field(discriminator="type"),
]


class MissionOverridesModel(BaseModel):
    spline_controls: list[SplineControlModel] = Field(default_factory=list)


class UnifiedMissionModel(BaseModel):
    epoch: str
    start_pose: PoseModel
    start_target_id: str | None = None
    segments: list[MissionSegmentModel]
    obstacles: list[ObstacleModel] = Field(default_factory=list)
    overrides: MissionOverridesModel | None = None


class PreviewUnifiedMissionResponse(BaseModel):
    path: list[list[float]]
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
