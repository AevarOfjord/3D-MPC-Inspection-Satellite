"""Pydantic request/response models for the dashboard API."""

from typing import Annotated, Any, Literal

from pydantic import BaseModel, Field, field_validator, model_validator


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
    passes: list["MeshScanPassModel"] | None = None


class MeshScanPassModel(BaseModel):
    label: str | None = None
    enabled: bool = True
    standoff: float = 0.5
    levels: int = 8
    level_spacing: float | None = None
    points_per_circle: int = 72
    speed_max: float = 0.2
    speed_min: float = 0.05
    lateral_accel: float = 0.05
    z_margin: float = 0.0
    scan_axis: str = "Z"  # "X", "Y", or "Z"
    pattern: str = "rings"  # "rings" or "spiral"
    region_enabled: bool = False
    region_center: list[float] | None = None
    region_size: list[float] | None = None
    section_mode: Literal["none", "aabb", "plane_slab"] = "none"
    plane_normal: list[float] | None = None
    plane_offset_min: float | None = None
    plane_offset_max: float | None = None

    @field_validator("region_center")
    @classmethod
    def validate_region_center(
        cls, value: list[float] | None
    ) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 3:
            raise ValueError("region_center must have length 3")
        return value

    @field_validator("region_size")
    @classmethod
    def validate_region_size(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 3:
            raise ValueError("region_size must have length 3")
        return value

    @field_validator("plane_normal")
    @classmethod
    def validate_plane_normal(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 3:
            raise ValueError("plane_normal must have length 3")
        return value


class PathAssetSaveRequest(BaseModel):
    name: str
    obj_path: str
    path: list[list[float]]
    open: bool = True
    relative_to_obj: bool = True
    notes: str | None = None


class ScanKeyLevelModel(BaseModel):
    id: str
    t: float
    center_offset: list[float] = Field(default_factory=lambda: [0.0, 0.0])
    radius_x: float = 1.0
    radius_y: float = 1.0
    rotation_deg: float = 0.0

    @field_validator("t")
    @classmethod
    def validate_t(cls, value: float) -> float:
        if value < 0.0 or value > 1.0:
            raise ValueError("key level t must be in [0, 1]")
        return value

    @field_validator("center_offset")
    @classmethod
    def validate_center_offset(cls, value: list[float]) -> list[float]:
        if len(value) != 2:
            raise ValueError("center_offset must have length 2")
        return value


class ScanDefinitionModel(BaseModel):
    id: str
    name: str
    axis: Literal["X", "Y", "Z"] = "Z"
    plane_a: list[float] = Field(default_factory=lambda: [0.0, 0.0, -0.5])
    plane_b: list[float] = Field(default_factory=lambda: [0.0, 0.0, 0.5])
    level_spacing_m: float = 0.1
    turns: float | None = None
    coarse_points_per_turn: int = 4
    densify_multiplier: int = 8
    speed_max: float = 0.2
    key_levels: list[ScanKeyLevelModel] = Field(default_factory=list)

    @field_validator("plane_a")
    @classmethod
    def validate_plane_a(cls, value: list[float]) -> list[float]:
        if len(value) != 3:
            raise ValueError("plane_a must have length 3")
        return value

    @field_validator("plane_b")
    @classmethod
    def validate_plane_b(cls, value: list[float]) -> list[float]:
        if len(value) != 3:
            raise ValueError("plane_b must have length 3")
        return value

    @field_validator("coarse_points_per_turn")
    @classmethod
    def validate_points_per_turn(cls, value: int) -> int:
        if value < 4:
            raise ValueError("coarse_points_per_turn must be at least 4")
        return value

    @field_validator("densify_multiplier")
    @classmethod
    def validate_densify(cls, value: int) -> int:
        if value < 1:
            raise ValueError("densify_multiplier must be at least 1")
        return value

    @field_validator("level_spacing_m")
    @classmethod
    def validate_level_spacing(cls, value: float) -> float:
        if value <= 0:
            raise ValueError("level_spacing_m must be > 0")
        return value

    @field_validator("turns")
    @classmethod
    def validate_turns(cls, value: float | None) -> float | None:
        if value is None:
            return value
        if value < 1:
            raise ValueError("turns must be at least 1 when provided")
        return value

    @model_validator(mode="after")
    def validate_key_levels(self) -> "ScanDefinitionModel":
        if len(self.key_levels) < 2:
            raise ValueError("scan must define at least 2 key levels")
        return self


class ScanConnectorModel(BaseModel):
    id: str
    from_scan_id: str
    to_scan_id: str
    from_endpoint: Literal["start", "end"] = "end"
    to_endpoint: Literal["start", "end"] = "start"
    control1: list[float] | None = None
    control2: list[float] | None = None
    samples: int = 24

    @field_validator("control1")
    @classmethod
    def validate_control1(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 3:
            raise ValueError("control1 must have length 3")
        return value

    @field_validator("control2")
    @classmethod
    def validate_control2(cls, value: list[float] | None) -> list[float] | None:
        if value is None:
            return value
        if len(value) != 3:
            raise ValueError("control2 must have length 3")
        return value

    @field_validator("samples")
    @classmethod
    def validate_samples(cls, value: int) -> int:
        if value < 4:
            raise ValueError("samples must be at least 4")
        return value


class ScanProjectModel(BaseModel):
    schema_version: int = 2
    id: str | None = None
    name: str
    obj_path: str
    path_density_multiplier: float = 1.0
    scans: list[ScanDefinitionModel] = Field(default_factory=list)
    connectors: list[ScanConnectorModel] = Field(default_factory=list)
    created_at: str | None = None
    updated_at: str | None = None

    @field_validator("path_density_multiplier")
    @classmethod
    def validate_path_density_multiplier(cls, value: float) -> float:
        if value <= 0:
            raise ValueError("path_density_multiplier must be > 0")
        return float(value)

    @model_validator(mode="after")
    def validate_references(self) -> "ScanProjectModel":
        if not self.scans:
            raise ValueError("scan project must include at least one scan")
        scan_ids = {scan.id for scan in self.scans}
        for connector in self.connectors:
            if connector.from_scan_id not in scan_ids:
                raise ValueError(
                    f"connector {connector.id} references unknown from_scan_id"
                )
            if connector.to_scan_id not in scan_ids:
                raise ValueError(
                    f"connector {connector.id} references unknown to_scan_id"
                )
            if connector.from_scan_id == connector.to_scan_id:
                raise ValueError(
                    f"connector {connector.id} must connect two different scans"
                )
        return self


class ScanProjectSummaryModel(BaseModel):
    id: str
    name: str
    obj_path: str
    scans: int
    connectors: int
    created_at: str | None = None
    updated_at: str | None = None


class ScanPathDiagnosticsModel(BaseModel):
    id: str
    kind: Literal["scan", "connector"]
    points: int
    path_length: float
    path: list[list[float]] | None = None
    min_clearance_m: float | None = None
    collision_points_count: int = 0
    clearance_per_point: list[float] | None = None


class ScanCompileDiagnosticsModel(BaseModel):
    min_clearance_m: float | None = None
    collision_points_count: int = 0
    clearance_threshold_m: float = 0.05
    combined_clearance_per_point: list[float] | None = None
    warnings: list[str] = Field(default_factory=list)


class ScanCompileResponseModel(BaseModel):
    status: str
    combined_path: list[list[float]]
    path_length: float
    estimated_duration: float
    points: int
    endpoints: dict[str, dict[str, list[float]]]
    scan_paths: list[ScanPathDiagnosticsModel] = Field(default_factory=list)
    connector_paths: list[ScanPathDiagnosticsModel] = Field(default_factory=list)
    diagnostics: ScanCompileDiagnosticsModel


class CompileScanProjectRequestModel(BaseModel):
    project: ScanProjectModel
    quality: Literal["preview", "final"] = "preview"
    include_collision: bool = True
    collision_threshold_m: float = 0.05


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
    manual_path: list[list[float]] = Field(default_factory=list)
    path_density_multiplier: float = 1.0

    @field_validator("manual_path")
    @classmethod
    def validate_manual_path(cls, value: list[list[float]]) -> list[list[float]]:
        for point in value:
            if len(point) != 3:
                raise ValueError("manual_path points must have length 3")
        return value

    @field_validator("path_density_multiplier")
    @classmethod
    def validate_path_density_multiplier(cls, value: float) -> float:
        if value <= 0:
            raise ValueError("path_density_multiplier must be > 0")
        return float(value)


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


# ============================================================================
# V2 mission authoring models
# ============================================================================


class MissionMetadataV2Model(BaseModel):
    version: int = 1
    created_at: str | None = None
    updated_at: str | None = None
    tags: list[str] = Field(default_factory=list)


class TransferSegmentV2Model(BaseModel):
    segment_id: str
    title: str | None = None
    notes: str | None = None
    type: Literal["transfer"]
    target_id: str | None = None
    end_pose: PoseModel
    constraints: ConstraintsModel | None = None


class ScanSegmentV2Model(BaseModel):
    segment_id: str
    title: str | None = None
    notes: str | None = None
    type: Literal["scan"]
    target_id: str
    target_pose: PoseModel | None = None
    scan: ScanConfigModel
    path_asset: str | None = None
    constraints: ConstraintsModel | None = None


class HoldSegmentV2Model(BaseModel):
    segment_id: str
    title: str | None = None
    notes: str | None = None
    type: Literal["hold"]
    duration: float = 0.0
    constraints: ConstraintsModel | None = None


MissionSegmentV2Model = Annotated[
    TransferSegmentV2Model | ScanSegmentV2Model | HoldSegmentV2Model,
    Field(discriminator="type"),
]


class UnifiedMissionV2Model(BaseModel):
    schema_version: Literal[2] = 2
    mission_id: str
    name: str
    epoch: str
    start_pose: PoseModel
    start_target_id: str | None = None
    segments: list[MissionSegmentV2Model]
    obstacles: list[ObstacleModel] = Field(default_factory=list)
    overrides: MissionOverridesModel | None = None
    metadata: MissionMetadataV2Model = Field(default_factory=MissionMetadataV2Model)

    @model_validator(mode="after")
    def validate_v2_identity(self) -> "UnifiedMissionV2Model":
        if not self.mission_id.strip():
            raise ValueError("mission_id is required")
        if not self.name.strip():
            raise ValueError("name is required")
        segment_ids = [segment.segment_id for segment in self.segments]
        if len(segment_ids) != len(set(segment_ids)):
            raise ValueError("segment_id values must be unique")
        return self


class MissionConstraintSummaryV2Model(BaseModel):
    speed_max: float | None = None
    accel_max: float | None = None
    angular_rate_max: float | None = None


class ValidationIssueV2Model(BaseModel):
    code: str
    severity: Literal["error", "warning", "info"] = "error"
    path: str
    message: str
    suggestion: str | None = None


class ValidationSummaryV2Model(BaseModel):
    errors: int = 0
    warnings: int = 0
    info: int = 0


class ValidationReportV2Model(BaseModel):
    valid: bool
    issues: list[ValidationIssueV2Model] = Field(default_factory=list)
    summary: ValidationSummaryV2Model


class PreviewMissionV2ResponseModel(BaseModel):
    path: list[list[float]]
    path_length: float
    path_speed: float
    eta_s: float
    risk_flags: list[str] = Field(default_factory=list)
    constraint_summary: MissionConstraintSummaryV2Model


class SaveMissionV2Request(BaseModel):
    name: str
    mission: UnifiedMissionV2Model


class SaveMissionV2ResponseModel(BaseModel):
    mission_id: str
    version: int
    saved_at: str
    filename: str


class MissionSummaryV2Model(BaseModel):
    name: str
    mission_id: str
    updated_at: str | None = None
    segments_count: int
    filename: str
    schema_version: int = 2


class MissionDraftSaveRequestV2Model(BaseModel):
    draft_id: str | None = None
    base_revision: int | None = None
    mission: UnifiedMissionV2Model


class MissionDraftResponseV2Model(BaseModel):
    draft_id: str
    revision: int
    saved_at: str
    mission: UnifiedMissionV2Model


class LegacyMissionMigrateRequestV2Model(BaseModel):
    payload: dict[str, Any]
    name_hint: str | None = None
