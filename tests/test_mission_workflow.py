"""
Mission Workflow Tests.

Tests mission state management, serialization, and end-to-end mission execution flow.
Combines `test_mission_state_refactor.py` and `test_unified_mission_workflow.py`.
"""

import json
from dataclasses import asdict

from satellite_control.config.mission_state import MissionState, PathFollowingState
from satellite_control.mission import repository as mission_repo


class TestMissionState:
    """Tests for MissionState serialization and logic."""

    def test_mission_state_roundtrip(self, tmp_path):
        """Test serialization and deserialization of MissionState."""
        path_state = PathFollowingState(
            active=True,
            waypoints=[(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
            path_speed=0.5,
        )
        state = MissionState(path=path_state)
        state.path_tracking_phase = "TRACKING"

        mission_state_file = tmp_path / "mission_state.json"
        mission_state_file.write_text(json.dumps(asdict(state)))
        loaded = json.loads(mission_state_file.read_text())

        restored = MissionState(
            path=PathFollowingState(
                active=loaded["path"]["active"],
                waypoints=[tuple(wp) for wp in loaded["path"]["waypoints"]],
                path_speed=float(loaded["path"]["path_speed"]),
                path_length=float(loaded["path"]["path_length"]),
            )
        )
        restored.path_tracking_phase = loaded["path_tracking_phase"]

        assert restored.path.active is True
        assert restored.path_waypoints == state.path_waypoints
        assert restored.path_speed == 0.5
        assert restored.path_tracking_phase == "TRACKING"
        assert restored.get_current_mission_type() == "PATH_FOLLOWING"
        assert restored.get_resolved_path_length(compute_if_missing=True) == 1.0

    def test_mission_reset(self):
        """Test resetting mission state."""
        state = MissionState()
        state.path.active = True
        state.path_tracking_phase = "TRACKING"

        state.reset()

        assert state.path.active is False
        assert state.path_tracking_phase == "POSITIONING"


class TestMissionExecutionFlow:
    """Tests for mission repository discovery/load workflow."""

    def test_mission_repository_discovery_and_load(self, tmp_path, monkeypatch):
        """Test listing, resolving, loading, and saving via mission repository."""
        alpha_payload = {"mission": {"name": "Alpha"}}
        beta_payload = {"mission": {"name": "Beta"}}
        (tmp_path / "alpha.json").write_text(json.dumps(alpha_payload))
        (tmp_path / "beta.json").write_text(json.dumps(beta_payload))
        (tmp_path / "ignore.txt").write_text("not a mission")

        monkeypatch.setitem(mission_repo.SOURCE_DIRS, "local", tmp_path)

        names = mission_repo.list_mission_names(source_priority=("local",))
        assert names == ["alpha.json", "beta.json"]

        resolved_alpha = mission_repo.resolve_mission_file(
            "alpha", source_priority=("local",)
        )
        assert resolved_alpha == tmp_path / "alpha.json"

        loaded_alpha = mission_repo.load_mission_json(
            "alpha", source_priority=("local",)
        )
        assert loaded_alpha == alpha_payload

        saved_payload = {"mission": {"name": "Gamma"}}
        saved_file = mission_repo.save_mission_json(
            name="Gamma Mission", payload=saved_payload, source="local"
        )
        assert saved_file == tmp_path / "GammaMission.json"
        assert json.loads(saved_file.read_text()) == saved_payload
