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
        assert base_cfg.get("schema_version") == "app_config_v3"
        assert isinstance(base_cfg.get("app_config"), dict)
        assert isinstance(base_cfg["app_config"].get("mpc_core"), dict)
        assert base_cfg.get("config_meta", {}).get("overrides_active") is False
        assert base_cfg.get("config_meta", {}).get("config_version") == "app_config_v3"

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
        assert meta.get("config_version") == "app_config_v3"
        assert isinstance(meta.get("config_hash"), str)
        assert len(meta["config_hash"]) == 12

        # Reset should restore defaults
        reset_again = client.post("/runner/config/reset")
        assert reset_again.status_code == 200
        reset_cfg = reset_again.json().get("config", {})
        assert reset_cfg["mpc"]["prediction_horizon"] == base_horizon
        assert reset_cfg.get("config_meta", {}).get("overrides_active") is False
        assert reset_cfg.get("config_meta", {}).get("config_version") == "app_config_v3"

    def test_runner_config_dual_read_payload_shapes(self, client):
        """Runner config should accept legacy, v1-flat, and v2 envelope payloads."""
        reset_resp = client.post("/runner/config/reset")
        assert reset_resp.status_code == 200

        legacy_resp = client.post(
            "/runner/config",
            json={
                "control": {"mpc": {"prediction_horizon": 42}},
                "sim": {"duration": 90.0},
            },
        )
        assert legacy_resp.status_code == 200
        cfg = client.get("/runner/config").json()
        assert cfg["mpc"]["prediction_horizon"] == 42
        assert cfg["simulation"]["max_duration"] == 90.0

        v1_resp = client.post(
            "/runner/config",
            json={
                "mpc": {"prediction_horizon": 43},
                "simulation": {"max_duration": 91.0},
            },
        )
        assert v1_resp.status_code == 200
        cfg = client.get("/runner/config").json()
        assert cfg["mpc"]["prediction_horizon"] == 43
        assert cfg["simulation"]["max_duration"] == 91.0

        v2_resp = client.post(
            "/runner/config",
            json={
                "schema_version": "app_config_v2",
                "app_config": {
                    "mpc": {"prediction_horizon": 44},
                    "simulation": {"max_duration": 92.0},
                },
            },
        )
        assert v2_resp.status_code == 200
        cfg = client.get("/runner/config").json()
        assert cfg.get("schema_version") == "app_config_v3"
        assert cfg["mpc"]["prediction_horizon"] == 44
        assert cfg["simulation"]["max_duration"] == 92.0

        v3_resp = client.post(
            "/runner/config",
            json={
                "schema_version": "app_config_v3",
                "app_config": {
                    "mpc_core": {"prediction_horizon": 45},
                    "simulation": {"max_duration": 93.0},
                    "actuator_policy": {
                        "enable_thruster_hysteresis": True,
                        "thruster_hysteresis_on": 0.02,
                        "thruster_hysteresis_off": 0.01,
                    },
                },
            },
        )
        assert v3_resp.status_code == 200
        cfg = client.get("/runner/config").json()
        assert cfg.get("schema_version") == "app_config_v3"
        assert cfg["mpc"]["prediction_horizon"] == 45
        assert cfg["simulation"]["max_duration"] == 93.0

    def test_runner_config_warn_ignores_removed_mpc_fields(self, client):
        """Removed MPC fields should be ignored with deprecation metadata."""
        reset_resp = client.post("/runner/config/reset")
        assert reset_resp.status_code == 200

        v3_resp = client.post(
            "/runner/config",
            json={
                "schema_version": "app_config_v3",
                "app_config": {
                    "mpc_core": {
                        "prediction_horizon": 46,
                        "coast_pos_tolerance": 0.25,
                        "progress_taper_distance": 1.5,
                    }
                },
            },
        )
        assert v3_resp.status_code == 200

        legacy_resp = client.post(
            "/runner/config",
            json={
                "control": {
                    "mpc": {
                        "prediction_horizon": 47,
                        "path_following": {
                            "coast_min_speed": 0.02,
                            "progress_slowdown_distance": 0.8,
                        },
                    }
                }
            },
        )
        assert legacy_resp.status_code == 200

        cfg = client.get("/runner/config").json()
        assert cfg["mpc"]["prediction_horizon"] == 47
        for removed_key in (
            "coast_pos_tolerance",
            "coast_vel_tolerance",
            "coast_min_speed",
            "progress_taper_distance",
            "progress_slowdown_distance",
        ):
            assert removed_key not in cfg["mpc"]

        deprecations = cfg.get("config_meta", {}).get("deprecations", {})
        seen = set(deprecations.get("removed_mpc_fields_seen", []))
        assert {
            "coast_pos_tolerance",
            "coast_min_speed",
            "progress_taper_distance",
            "progress_slowdown_distance",
        }.issubset(seen)
        assert deprecations.get("removed_mpc_fields_policy") == "warn_ignore"
        assert deprecations.get("removed_mpc_fields_sunset") == "next_major"

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
        assert presets["fast-test"]["config"].get("schema_version") == "app_config_v3"
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

    def test_scan_projects_validation_endpoint(self, client):
        """Scan project endpoint should enforce schema validation."""
        response = client.post("/scan_projects", json={"name": "bad"})
        assert response.status_code == 422
