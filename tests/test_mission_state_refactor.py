import pytest

from src.satellite_control.config.io import ConfigIO
from src.satellite_control.config.mission_state import MissionState


class TestMissionStateRefactor:
    def test_initialization_defaults(self):
        """Test default initialization for path-only mission state."""
        ms = MissionState()
        assert ms.get_current_mission_type() == "NONE"
        assert ms.path_waypoints == []
        assert ms.path_following_active is False

    def test_path_properties(self):
        """Test path-related properties map correctly."""
        ms = MissionState()
        ms.path_waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        ms.path_length = 1.0
        ms.path_speed = 0.2

        assert ms.path_waypoints == [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        assert ms.path_length == pytest.approx(1.0)
        assert ms.path_speed == pytest.approx(0.2)
        assert ms.get_current_mission_type() == "PATH_FOLLOWING"

    def test_serialization_roundtrip(self):
        """Test serialization and deserialization via ConfigIO."""
        ms = MissionState()
        ms.path_waypoints = [(10.0, 20.0, 30.0), (11.0, 22.0, 33.0)]
        ms.path_length = 5.0
        ms.path_speed = 0.8

        data = ConfigIO._mission_state_to_dict(ms)
        ms_new = ConfigIO._dict_to_mission_state(data)

        assert ms_new.path_waypoints == [(10.0, 20.0, 30.0), (11.0, 22.0, 33.0)]
        assert ms_new.path_length == pytest.approx(5.0)
        assert ms_new.path_speed == pytest.approx(0.8)

    def test_flat_dict_loading(self):
        """Test loading from a flat dictionary."""
        flat_data = {
            "path_waypoints": [[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]],
            "obstacles": ["obs1", "obs2"],
        }

        ms = ConfigIO._dict_to_mission_state(flat_data)

        assert ms.path_waypoints == [[1.0, 2.0, 3.0], [2.0, 3.0, 4.0]]
        assert ms.obstacle_state.enabled is False  # Not set in dict
        assert ms.obstacles == ["obs1", "obs2"]

    def test_canonical_path_state_roundtrip(self):
        """Test canonical path fields remain consistent."""
        ms = MissionState()
        ms.path_waypoints = [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        ms.path_length = 5.0
        ms.path_speed = 0.3

        assert ms.path_waypoints == [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0)]
        assert ms.path_length == pytest.approx(5.0)
        assert ms.path_speed == pytest.approx(0.3)

    def test_resolved_path_length_falls_back_to_waypoint_polyline(self):
        """Test resolved path length computes from waypoints when unset."""
        ms = MissionState()
        ms.path_waypoints = [(0.0, 0.0, 0.0), (3.0, 4.0, 0.0), (3.0, 4.0, 12.0)]
        ms.path_length = 0.0

        assert ms.get_resolved_path_length(compute_if_missing=True) == pytest.approx(
            17.0
        )
        assert ms.get_resolved_path_length(compute_if_missing=False) == pytest.approx(
            0.0
        )

    def test_flat_dict_loading_supports_canonical_path_keys(self):
        """Test loading from flat dictionary with canonical path keys."""
        flat_data = {
            "path_waypoints": [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            "path_length": 1.0,
            "path_speed": 0.12,
            "path_following_active": True,
        }

        ms = ConfigIO._dict_to_mission_state(flat_data)

        assert ms.path_waypoints == [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
        assert ms.path_length == pytest.approx(1.0)
        assert ms.path_speed == pytest.approx(0.12)
        assert ms.path_following_active is True

    def test_path_following_active_roundtrip(self):
        """Test path-following active flag state roundtrip."""
        ms = MissionState()
        ms.path_following_active = True
        assert ms.path_following_active is True
        ms.path_following_active = False
        assert ms.path_following_active is False

    def test_path_tracking_runtime_fields_survive_serialization_roundtrip(self):
        """Test path-tracking runtime fields round-trip through ConfigIO."""
        ms = MissionState()
        ms.path_tracking_phase = "TRACKING"
        ms.path_tracking_closest_point_index = 7
        ms.path_tracking_current_target_position = (1.0, 2.0, 3.0)
        ms.path_tracking_return_position = (0.1, 0.2, 0.3)
        ms.path_tracking_return_angle = (0.0, 0.0, 1.57)

        data = ConfigIO._mission_state_to_dict(ms)
        ms_new = ConfigIO._dict_to_mission_state(data)

        assert ms_new.path_tracking_phase == "TRACKING"
        assert ms_new.path_tracking_closest_point_index == 7
        assert ms_new.path_tracking_current_target_position == (1.0, 2.0, 3.0)
        assert ms_new.path_tracking_return_position == (0.1, 0.2, 0.3)
        assert ms_new.path_tracking_return_angle == (0.0, 0.0, 1.57)

    def test_reset_clears_path_tracking_runtime_fields(self):
        """Test reset clears path-tracking runtime fields."""
        ms = MissionState()
        ms.path_tracking_phase = "STABILIZING"
        ms.path_tracking_current_target_position = (9.0, 8.0, 7.0)
        ms.path_tracking_has_return = True
        ms.path_tracking_return_position = (1.0, 0.0, 0.0)

        ms.reset()

        assert ms.path_tracking_phase == "POSITIONING"
        assert ms.path_tracking_current_target_position is None
        assert ms.path_tracking_has_return is False
        assert ms.path_tracking_return_position is None
