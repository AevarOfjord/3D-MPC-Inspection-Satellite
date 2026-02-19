import numpy as np
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.mission import unified_compiler
from satellite_control.mission.unified_mission import (
    Frame,
    MissionDefinition,
    MissionObstacle,
    MissionOverrides,
    Pose,
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
