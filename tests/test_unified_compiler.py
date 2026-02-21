import numpy as np
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.mission import unified_compiler
from satellite_control.mission.unified_mission import (
    Frame,
    MissionDefinition,
    MissionObstacle,
    MissionOverrides,
    Pose,
    ScanConfig,
    ScanSegment,
    SegmentType,
    TransferSegment,
)


def test_build_asset_path_trims_duplicate_closure_when_open(monkeypatch):
    monkeypatch.setattr(
        unified_compiler,
        "load_path_asset",
        lambda _asset_id: {
            "path": [[10.0, 0.0, 0.0], [11.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            "open": True,
            "relative_to_obj": True,
        },
    )

    path, apply_orientation = unified_compiler._build_asset_path(
        "dummy", np.array([100.0, 200.0, 300.0], dtype=float)
    )

    assert apply_orientation is True
    assert len(path) == 2
    assert path[0] == (110.0, 200.0, 300.0)
    assert path[-1] == (111.0, 200.0, 300.0)
    assert path[0] != path[-1]


def test_compile_unified_mission_path_prefers_manual_path_override():
    mission = MissionDefinition(
        epoch="2026-01-01T00:00:00Z",
        start_pose=Pose(frame=Frame.ECI, position=[10.0, 0.0, 0.0]),
        segments=[],
        overrides=MissionOverrides(
            manual_path=[
                [10.0, 0.0, 0.0],
                [11.0, 1.0, 0.0],
                [12.0, 1.0, 0.0],
            ]
        ),
    )
    sim_cfg = SimulationConfig.create_default()

    path, path_length, path_speed, _origin = unified_compiler.compile_unified_mission_path(
        mission=mission,
        sim_config=sim_cfg,
        output_frame="ECI",
    )

    assert path == [(10.0, 0.0, 0.0), (11.0, 1.0, 0.0), (12.0, 1.0, 0.0)]
    assert path_length > 0.0
    assert path_speed == float(sim_cfg.app_config.mpc.path_speed)


def test_compile_unified_mission_path_ignores_obstacles_for_generation(monkeypatch):
    captured = {"obstacles": None}

    def _fake_builder(waypoints, obstacles, step_size=0.1, safety_margin=0.0):
        captured["obstacles"] = obstacles
        return [tuple(map(float, p)) for p in waypoints]

    monkeypatch.setattr(unified_compiler, "build_point_to_point_path", _fake_builder)

    mission = MissionDefinition(
        epoch="2026-01-01T00:00:00Z",
        start_pose=Pose(frame=Frame.ECI, position=[0.0, 0.0, 0.0]),
        segments=[
            TransferSegment(
                type=SegmentType.TRANSFER,
                end_pose=Pose(frame=Frame.ECI, position=[5.0, 0.0, 0.0]),
            )
        ],
        obstacles=[MissionObstacle(position=[1.0, 0.0, 0.0], radius=1.0)],
    )
    sim_cfg = SimulationConfig.create_default()

    path, _path_length, _path_speed, _origin = unified_compiler.compile_unified_mission_path(
        mission=mission,
        sim_config=sim_cfg,
        output_frame="ECI",
    )

    assert captured["obstacles"] in ([], ())
    assert path[-1] == (5.0, 0.0, 0.0)


def test_compile_unified_mission_path_scales_transfer_step_size_with_density(monkeypatch):
    captured_step_sizes: list[float] = []

    def _fake_builder(waypoints, obstacles, step_size=0.1, safety_margin=0.0):
        captured_step_sizes.append(float(step_size))
        return [tuple(map(float, p)) for p in waypoints]

    monkeypatch.setattr(unified_compiler, "build_point_to_point_path", _fake_builder)

    mission = MissionDefinition(
        epoch="2026-01-01T00:00:00Z",
        start_pose=Pose(frame=Frame.LVLH, position=[0.0, 0.0, 0.0]),
        segments=[
            TransferSegment(
                type=SegmentType.TRANSFER,
                end_pose=Pose(frame=Frame.LVLH, position=[5.0, 0.0, 0.0]),
            )
        ],
        overrides=MissionOverrides(path_density_multiplier=2.0),
    )
    sim_cfg = SimulationConfig.create_default()

    unified_compiler.compile_unified_mission_path(
        mission=mission,
        sim_config=sim_cfg,
        output_frame="LVLH",
    )

    assert captured_step_sizes
    assert abs(captured_step_sizes[0] - 0.5) < 1e-9


def test_compile_unified_mission_path_scales_asset_scan_path_density(monkeypatch):
    monkeypatch.setattr(
        unified_compiler,
        "load_path_asset",
        lambda _asset_id: {
            "path": [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            "open": True,
            "relative_to_obj": False,
        },
    )
    monkeypatch.setattr(
        unified_compiler,
        "build_point_to_point_path",
        lambda waypoints, obstacles, step_size=0.1, safety_margin=0.0: [
            tuple(map(float, p)) for p in waypoints
        ],
    )

    mission = MissionDefinition(
        epoch="2026-01-01T00:00:00Z",
        start_pose=Pose(frame=Frame.LVLH, position=[0.0, 0.0, 0.0]),
        segments=[
            ScanSegment(
                type=SegmentType.SCAN,
                target_id="TEST",
                target_pose=Pose(frame=Frame.ECI, position=[0.0, 0.0, 0.0]),
                scan=ScanConfig(frame=Frame.LVLH),
                path_asset="asset_1",
            )
        ],
        overrides=MissionOverrides(path_density_multiplier=2.0),
    )
    sim_cfg = SimulationConfig.create_default()

    path, _length, _speed, _origin = unified_compiler.compile_unified_mission_path(
        mission=mission,
        sim_config=sim_cfg,
        output_frame="LVLH",
    )

    # 1 start + 1 connector endpoint + (resampled asset points - first point)
    # Asset has 3 points, density 2.0 -> 6 points, total expected points = 7.
    assert len(path) == 7
    assert path[0] == (0.0, 0.0, 0.0)
    assert path[-1] == (3.0, 0.0, 0.0)
