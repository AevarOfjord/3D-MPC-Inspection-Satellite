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
