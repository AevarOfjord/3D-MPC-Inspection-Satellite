import pytest

from src.satellite_control.config.io import ConfigIO
from src.satellite_control.config.mission_state import MissionState


class TestMissionStateRefactor:
    def test_initialization_defaults(self):
        """Test default initialization for path-only mission state."""
        ms = MissionState()
        assert ms.get_current_mission_type() == "NONE"
        assert ms.mpcc_path_waypoints == []
        assert ms.path_following_active is False

    def test_path_properties(self):
        """Test path-related properties map correctly."""
        ms = MissionState()
        ms.mpcc_path_waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        ms.mpcc_path_length = 1.0
        ms.mpcc_path_speed = 0.2

        assert ms.path_waypoints == [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        assert ms.path_length == pytest.approx(1.0)
        assert ms.path_speed == pytest.approx(0.2)
        assert ms.get_current_mission_type() == "PATH_FOLLOWING"

    def test_serialization_roundtrip(self):
        """Test serialization and deserialization via ConfigIO."""
        ms = MissionState()
        ms.mpcc_path_waypoints = [(10.0, 20.0, 30.0), (11.0, 22.0, 33.0)]
        ms.mpcc_path_length = 5.0
        ms.mpcc_path_speed = 0.8

        data = ConfigIO._mission_state_to_dict(ms)
        ms_new = ConfigIO._dict_to_mission_state(data)

        assert ms_new.mpcc_path_waypoints == [(10.0, 20.0, 30.0), (11.0, 22.0, 33.0)]
        assert ms_new.mpcc_path_length == pytest.approx(5.0)
        assert ms_new.mpcc_path_speed == pytest.approx(0.8)

    def test_flat_dict_loading(self):
        """Test loading from a flat dictionary."""
        flat_data = {
            "path_waypoints": [[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]],
            "obstacles": ["obs1", "obs2"],
        }

        ms = ConfigIO._dict_to_mission_state(flat_data)

        assert ms.mpcc_path_waypoints == [[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]]
        assert ms.obstacle_state.enabled is False  # Not set in dict
        assert ms.obstacles == ["obs1", "obs2"]

    def test_legacy_dxf_aliases_map_to_canonical_path_state(self):
        """Test legacy DXF fields map to canonical path fields."""
        ms = MissionState()
        ms.dxf_shape_path = [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        ms.dxf_path_length = 5.0
        ms.dxf_path_speed = 0.25
        ms.dxf_target_speed = 0.3

        assert ms.mpcc_path_waypoints == [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        assert ms.path_waypoints == [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        assert ms.mpcc_path_length == pytest.approx(5.0)
        assert ms.path_length == pytest.approx(5.0)
        assert ms.mpcc_path_speed == pytest.approx(0.3)
        assert ms.path_speed == pytest.approx(0.3)

    def test_resolved_path_length_falls_back_to_waypoint_polyline(self):
        """Test resolved path length computes from waypoints when unset."""
        ms = MissionState()
        ms.path_waypoints = [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0), (3.0, 4.0, 12.0)]
        ms.path_length = 0.0

        assert ms.get_resolved_path_length(compute_if_missing=True) == pytest.approx(17.0)
        assert ms.get_resolved_path_length(compute_if_missing=False) == pytest.approx(0.0)

    def test_flat_dict_loading_supports_legacy_dxf_keys(self):
        """Test loading from flat dictionary with legacy DXF keys."""
        flat_data = {
            "dxf_shape_path": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            "dxf_path_length": 1.0,
            "dxf_path_speed": 0.12,
            "dxf_shape_mode_active": True,
        }

        ms = ConfigIO._dict_to_mission_state(flat_data)

        assert ms.mpcc_path_waypoints == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
        assert ms.path_length == pytest.approx(1.0)
        assert ms.path_speed == pytest.approx(0.12)
        assert ms.path_following_active is True

    def test_dxf_shape_mode_alias_maps_to_path_active(self):
        """Test legacy DXF mode flag maps to canonical path active state."""
        ms = MissionState()
        ms.dxf_shape_mode_active = True
        assert ms.path_following_active is True
        ms.path_following_active = False
        assert ms.dxf_shape_mode_active is False

    def test_dxf_runtime_fields_survive_serialization_roundtrip(self):
        """Test explicit DXF runtime fields round-trip through ConfigIO."""
        ms = MissionState()
        ms.dxf_shape_phase = "TRACKING"
        ms.dxf_closest_point_index = 7
        ms.dxf_current_target_position = (1.0, 2.0, 3.0)
        ms.dxf_return_position = (0.1, 0.2, 0.3)
        ms.dxf_return_angle = (0.0, 0.0, 1.57)

        data = ConfigIO._mission_state_to_dict(ms)
        ms_new = ConfigIO._dict_to_mission_state(data)

        assert ms_new.dxf_shape_phase == "TRACKING"
        assert ms_new.dxf_closest_point_index == 7
        assert ms_new.dxf_current_target_position == (1.0, 2.0, 3.0)
        assert ms_new.dxf_return_position == (0.1, 0.2, 0.3)
        assert ms_new.dxf_return_angle == (0.0, 0.0, 1.57)

    def test_reset_clears_dxf_runtime_fields(self):
        """Test reset clears explicit legacy DXF runtime fields."""
        ms = MissionState()
        ms.dxf_shape_phase = "STABILIZING"
        ms.dxf_current_target_position = (9.0, 8.0, 7.0)
        ms.dxf_has_return = True
        ms.dxf_return_position = (1.0, 0.0, 0.0)

        ms.reset()

        assert ms.dxf_shape_phase == "POSITIONING"
        assert ms.dxf_current_target_position is None
        assert ms.dxf_has_return is False
        assert ms.dxf_return_position is None
