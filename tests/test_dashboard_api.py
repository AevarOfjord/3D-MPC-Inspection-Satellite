"""
Dashboard API Tests.

Tests FastAPI endpoints for the dashboard backend.
Combines `test_dashboard.py` and `test_mission_saving.py`.
"""

import pytest
from fastapi.testclient import TestClient
from satellite_control.dashboard.app import app


class TestDashboardAPI:
    """Tests for the dashboard API."""

    @pytest.fixture
    def client(self):
        with TestClient(app) as test_client:
            yield test_client

    def test_health_check(self, client):
        """Test root/health endpoint."""
        # /simulations is a valid GET endpoint
        response = client.get("/simulations")
        assert response.status_code == 200
        body = response.json()
        assert "runs" in body
        assert isinstance(body["runs"], list)

    def test_list_missions(self, client):
        """Test listing available missions."""
        response = client.get("/saved_missions_v2")
        assert response.status_code == 200
        assert "missions" in response.json()
        assert isinstance(response.json()["missions"], list)

    def test_control_actions(self, client):
        """Test control endpoints (start/stop/reset)."""
        # Control uses /control with ControlCommand payload
        # Fields: action (pause, resume, step), steps (int)

        # Reset first so state transitions are deterministic
        resp_reset = client.post("/reset")
        assert resp_reset.status_code == 200
        reset_body = resp_reset.json()
        assert reset_body["status"] == "success"
        assert "reset" in reset_body["message"].lower()

        # Resume (Start)
        resp_resume = client.post("/control", json={"action": "resume"})
        assert resp_resume.status_code == 200
        resume_body = resp_resume.json()
        assert resume_body["status"] == "success"
        assert resume_body["paused"] is False
        assert isinstance(resume_body["pending_steps"], int)

        # Pause
        resp_pause = client.post("/control", json={"action": "pause"})
        assert resp_pause.status_code == 200
        pause_body = resp_pause.json()
        assert pause_body["status"] == "success"
        assert pause_body["paused"] is True

        # Step
        resp_step = client.post("/control", json={"action": "step", "steps": 3})
        assert resp_step.status_code == 200
        step_body = resp_step.json()
        assert step_body["status"] == "success"
        assert step_body["paused"] is True
        assert isinstance(step_body["pending_steps"], int)
        assert step_body["pending_steps"] >= 1

    def test_mission_upload_validation(self, client):
        """Test validation on mission upload."""
        # /mission_v2 expects a valid UnifiedMissionModel
        # Empty dict should fail validation
        response = client.post("/mission_v2", json={})
        assert response.status_code == 422  # Validation Error
