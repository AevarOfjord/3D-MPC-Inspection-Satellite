import pytest
from src.satellite_control.config.mission_state import (
    MissionState,
    WaypointState,
    ShapeFollowingState,
)
from src.satellite_control.config.io import ConfigIO


class TestMissionStateRefactor:
    def test_initialization_defaults(self):
        """Test default initialization creates component states."""
        ms = MissionState()
        assert isinstance(ms.waypoint, WaypointState)
        assert isinstance(ms.shape, ShapeFollowingState)
        assert ms.waypoint.enabled is False
        assert ms.get_current_mission_type() == "NONE"

    def test_backward_compatibility_properties(self):
        """Test that legacy flat properties map correctly to nested state."""
        ms = MissionState()

        # Test Waypoint
        ms.enable_waypoint_mode = True
        assert ms.waypoint.enabled is True
        assert ms.enable_waypoint_mode is True

        targets = [(1.0, 2.0, 3.0)]
        ms.waypoint_targets = targets
        assert ms.waypoint.targets == targets
        assert ms.waypoint_targets == targets

        # Test Shape
        ms.dxf_shape_mode_active = True
        assert ms.shape.active is True
        assert ms.dxf_shape_mode_active is True

        ms.dxf_target_speed = 0.5
        assert ms.shape.target_speed == 0.5
        assert ms.dxf_target_speed == 0.5

    def test_mission_type_logic(self):
        """Test mission type determination."""
        ms = MissionState()
        assert ms.get_current_mission_type() == "NONE"

        ms.waypoint.enabled = True
        ms.waypoint.targets = [(1, 1, 1)]
        assert ms.get_current_mission_type() == "WAYPOINT_NAVIGATION"

        ms.waypoint.targets = [(1, 1, 1), (2, 2, 2)]
        assert ms.get_current_mission_type() == "WAYPOINT_NAVIGATION_MULTI"

        ms.shape.active = True
        # Shape has precedence over waypoint in current logic
        assert ms.get_current_mission_type() == "SHAPE_FOLLOWING"

        ms.trajectory.active = True
        # Trajectory has precedence over shape
        assert ms.get_current_mission_type() == "TRAJECTORY"

    def test_reset(self):
        """Test reset functionality."""
        ms = MissionState()
        ms.waypoint.enabled = True
        ms.shape.active = True
        ms.scan.active = True

        ms.reset()

        assert ms.waypoint.enabled is False
        assert ms.shape.active is False
        assert ms.scan.active is False
        assert ms.trajectory.active is False

    def test_serialization_roundtrip(self):
        """Test serialization and deserialization via ConfigIO."""
        ms = MissionState()
        ms.waypoint.enabled = True
        ms.waypoint.targets = [(10.0, 20.0, 30.0)]
        ms.shape.target_speed = 0.8

        # Serialize
        data = ConfigIO._mission_state_to_dict(ms)

        # Verify nested structure
        assert "waypoint" in data
        assert data["waypoint"]["enabled"] is True
        assert data["waypoint"]["targets"] == [(10.0, 20.0, 30.0)]

        # Deserialize
        ms_new = ConfigIO._dict_to_mission_state(data)

        assert ms_new.waypoint.enabled is True
        assert ms_new.waypoint.targets == [(10.0, 20.0, 30.0)]
        assert ms_new.shape.target_speed == 0.8

    def test_legacy_dict_loading(self):
        """Test loading from a legacy flat dictionary."""
        legacy_data = {
            "enable_waypoint_mode": True,
            "waypoint_targets": [[1.0, 2.0, 3.0]],
            "dxf_shape_mode_active": True,
            "dxf_target_speed": 1.5,
            "obstacles": ["obs1", "obs2"],
        }

        ms = ConfigIO._dict_to_mission_state(legacy_data)

        assert ms.waypoint.enabled is True
        assert ms.waypoint.targets == [[1.0, 2.0, 3.0]]
        assert ms.shape.active is True
        assert ms.shape.target_speed == 1.5
        assert ms.obstacle_state.enabled is False  # Not set in dict
        assert ms.obstacles == ["obs1", "obs2"]
