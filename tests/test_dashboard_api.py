"""
Dashboard API Tests.

Tests FastAPI endpoints for the dashboard backend.
Combines `test_dashboard.py` and `test_mission_saving.py`.
"""

import json

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

    def test_runner_config_update_and_reset(self, client):
        """Runner config overrides should apply, then reset to defaults."""
        # Ensure clean default baseline
        reset_resp = client.post("/runner/config/reset")
        assert reset_resp.status_code == 200
        assert reset_resp.json().get("status") == "reset"

        base_resp = client.get("/runner/config")
        assert base_resp.status_code == 200
        base_cfg = base_resp.json()
        base_horizon = base_cfg["mpc"]["prediction_horizon"]
        assert isinstance(base_horizon, int)
        assert base_cfg.get("config_meta", {}).get("overrides_active") is False

        # Apply one override
        target_horizon = base_horizon + 7
        update_resp = client.post(
            "/runner/config",
            json={
                "mpc": {
                    "prediction_horizon": target_horizon,
                }
            },
        )
        assert update_resp.status_code == 200
        assert update_resp.json().get("status") == "updated"

        after_update = client.get("/runner/config")
        assert after_update.status_code == 200
        updated_cfg = after_update.json()
        assert updated_cfg["mpc"]["prediction_horizon"] == target_horizon
        meta = updated_cfg.get("config_meta", {})
        assert meta.get("overrides_active") is True
        assert isinstance(meta.get("config_hash"), str)
        assert len(meta["config_hash"]) == 12

        # Reset should restore defaults
        reset_again = client.post("/runner/config/reset")
        assert reset_again.status_code == 200
        reset_cfg = reset_again.json().get("config", {})
        assert reset_cfg["mpc"]["prediction_horizon"] == base_horizon
        assert reset_cfg.get("config_meta", {}).get("overrides_active") is False

    def test_runner_presets_crud_and_apply(self, client):
        """Runner presets should persist via API and be applicable."""
        clear_resp = client.post("/runner/presets/reset")
        assert clear_resp.status_code == 200
        assert clear_resp.json().get("status") == "reset"

        save_resp = client.post(
            "/runner/presets",
            json={
                "name": "fast-test",
                "config": {
                    "mpc": {
                        "prediction_horizon": 33,
                        "control_horizon": 33,
                        "dt": 0.05,
                    },
                    "simulation": {
                        "dt": 0.001,
                        "control_dt": 0.05,
                        "max_duration": 120.0,
                    },
                },
            },
        )
        assert save_resp.status_code == 200
        assert save_resp.json().get("status") == "saved"

        list_resp = client.get("/runner/presets")
        assert list_resp.status_code == 200
        presets = list_resp.json().get("presets", {})
        assert "fast-test" in presets
        assert presets["fast-test"]["config"]["mpc"]["prediction_horizon"] == 33

        apply_resp = client.post("/runner/presets/apply", json={"name": "fast-test"})
        assert apply_resp.status_code == 200
        assert apply_resp.json().get("status") == "applied"

        cfg_resp = client.get("/runner/config")
        assert cfg_resp.status_code == 200
        cfg = cfg_resp.json()
        assert cfg["mpc"]["prediction_horizon"] == 33
        assert cfg.get("config_meta", {}).get("overrides_active") is True
        assert cfg.get("config_meta", {}).get("active_preset_name") == "fast-test"

        delete_resp = client.delete("/runner/presets/fast-test")
        assert delete_resp.status_code == 200
        assert delete_resp.json().get("status") == "deleted"

    def test_simulations_runs_ws_snapshot(self, client):
        """Runs websocket should push an initial snapshot payload."""
        with client.websocket_connect("/simulations/runs/ws") as ws:
            payload = json.loads(ws.receive_text())
            assert payload.get("type") == "runs_snapshot"
            assert isinstance(payload.get("runs"), list)
