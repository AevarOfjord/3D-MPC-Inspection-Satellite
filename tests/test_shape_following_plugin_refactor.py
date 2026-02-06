import numpy as np

from src.satellite_control.config.mission_state import create_mission_state
from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.mission.mission_manager import MissionManager
from src.satellite_control.mission.plugins.shape_following_plugin import (
    ShapeFollowingMissionPlugin,
)


def test_shape_following_plugin_required_parameters_use_canonical_path_fields():
    plugin = ShapeFollowingMissionPlugin()
    required = plugin.get_required_parameters()

    assert "path_following_active" in required
    assert "path_waypoints" in required
    assert "path_speed" in required
    assert "path_length" in required
    assert "dxf_shape_mode_active" not in required
    assert "dxf_shape_path" not in required
    assert "dxf_target_speed" not in required
    assert "dxf_path_length" not in required


def test_shape_following_plugin_configure_uses_canonical_manager_entrypoint():
    plugin = ShapeFollowingMissionPlugin()
    expected_config = SimulationConfig.create_default()

    called = {"value": False}

    def _fake_run_shape_following_mode(return_simulation_config=False):
        called["value"] = True
        assert return_simulation_config is True
        return {"simulation_config": expected_config}

    plugin.manager.run_shape_following_mode = _fake_run_shape_following_mode

    mission_state = plugin.configure(SimulationConfig.create_default())
    assert called["value"] is True
    assert mission_state is expected_config.mission_state


def test_mission_manager_legacy_dxf_entrypoint_delegates_to_canonical():
    manager = MissionManager()

    called = {"value": False}
    expected = {"mission_type": "shape_following"}

    def _fake_run_shape_following_mode(return_simulation_config=False):
        called["value"] = True
        assert return_simulation_config is True
        return expected

    manager.run_shape_following_mode = _fake_run_shape_following_mode

    result = manager.run_dxf_shape_mode(return_simulation_config=True)
    assert called["value"] is True
    assert result == expected


def test_shape_following_plugin_target_checks_use_canonical_path_fields():
    plugin = ShapeFollowingMissionPlugin()
    current_state = np.zeros(13, dtype=float)
    mission_state = create_mission_state()

    # Inactive path mode should no-op.
    out = plugin.get_target_state(current_state, 0.0, mission_state)
    assert np.array_equal(out, current_state)

    # Active mode but empty path should no-op.
    mission_state.path_following_active = True
    out = plugin.get_target_state(current_state, 0.0, mission_state)
    assert np.array_equal(out, current_state)

    # Active mode with path is still placeholder behavior (current_state passthrough).
    mission_state.path_waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    out = plugin.get_target_state(current_state, 0.0, mission_state)
    assert np.array_equal(out, current_state)


def test_shape_following_plugin_completion_uses_canonical_active_flag():
    plugin = ShapeFollowingMissionPlugin()
    current_state = np.zeros(13, dtype=float)
    mission_state = create_mission_state()

    mission_state.path_following_active = True
    mission_state.dxf_shape_phase = "COMPLETE"
    assert plugin.is_complete(current_state, 0.0, mission_state) is True

    mission_state.path_following_active = False
    assert plugin.is_complete(current_state, 0.0, mission_state) is False
