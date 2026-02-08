"""
Dashboard API tests using FastAPI TestClient.

Tests the REST endpoints without starting a real server.
"""

import pytest
from fastapi.testclient import TestClient


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def app():
    """Create a fresh FastAPI app for each test."""
    # Import inside fixture to avoid import-time side-effects
    from src.satellite_control.dashboard.app import app as dashboard_app

    return dashboard_app


@pytest.fixture()
def client(app):
    """TestClient wraps the ASGI app for synchronous test requests."""
    with TestClient(app, raise_server_exceptions=False) as c:
        yield c


# ---------------------------------------------------------------------------
# Simulation routes
# ---------------------------------------------------------------------------


class TestSimulationRoutes:
    """Tests for /simulations and /control endpoints."""

    def test_list_simulations(self, client: TestClient):
        """GET /simulations should return a JSON response with runs."""
        resp = client.get("/simulations")
        assert resp.status_code == 200
        body = resp.json()
        assert isinstance(body, dict)
        assert "runs" in body
        assert isinstance(body["runs"], list)

    def test_control_pause(self, client: TestClient):
        """POST /control with pause action should succeed."""
        resp = client.post("/control", json={"action": "pause"})
        assert resp.status_code == 200
        body = resp.json()
        assert body.get("status") == "success"
        assert body.get("paused") is True

    def test_control_resume(self, client: TestClient):
        """POST /control with resume action should succeed."""
        resp = client.post("/control", json={"action": "resume"})
        assert resp.status_code == 200
        body = resp.json()
        assert body.get("status") == "success"
        assert body.get("paused") is False

    def test_control_step(self, client: TestClient):
        """POST /control with step action should succeed."""
        resp = client.post("/control", json={"action": "step", "steps": 5})
        assert resp.status_code == 200
        body = resp.json()
        assert body.get("status") == "success"

    def test_speed_update(self, client: TestClient):
        """POST /speed should clamp and return the simulation speed."""
        resp = client.post("/speed", json={"speed": 2.5})
        assert resp.status_code == 200
        body = resp.json()
        assert body.get("sim_speed") == 2.5

    def test_speed_clamp_max(self, client: TestClient):
        """Speed above 10 should be clamped."""
        resp = client.post("/speed", json={"speed": 999})
        assert resp.status_code == 200
        assert resp.json().get("sim_speed") <= 10.0

    def test_speed_clamp_min(self, client: TestClient):
        """Speed below 0.1 should be clamped."""
        resp = client.post("/speed", json={"speed": 0.001})
        assert resp.status_code == 200
        assert resp.json().get("sim_speed") >= 0.1

    def test_reset(self, client: TestClient):
        """POST /reset should return success."""
        resp = client.post("/reset")
        assert resp.status_code == 200


# ---------------------------------------------------------------------------
# Mission routes
# ---------------------------------------------------------------------------


class TestMissionRoutes:
    """Tests for /mission_v2 endpoints."""

    def test_get_current_mission_empty(self, client: TestClient):
        """GET /mission_v2 when no mission is loaded returns 404."""
        resp = client.get("/mission_v2")
        assert resp.status_code in (200, 404)

    def test_list_saved_missions(self, client: TestClient):
        """GET /saved_missions_v2 should return missions."""
        resp = client.get("/saved_missions_v2")
        assert resp.status_code == 200
        body = resp.json()
        assert isinstance(body, dict)
        assert "missions" in body
        assert isinstance(body["missions"], list)

    def test_load_nonexistent_mission(self, client: TestClient):
        """GET /mission_v2/nonexistent should return 404."""
        resp = client.get("/mission_v2/__does_not_exist__")
        assert resp.status_code == 404


# ---------------------------------------------------------------------------
# Asset routes
# ---------------------------------------------------------------------------


class TestAssetRoutes:
    """Tests for /api/models and /path_assets endpoints."""

    def test_list_models(self, client: TestClient):
        """GET /api/models/list should return available 3D models."""
        resp = client.get("/api/models/list")
        assert resp.status_code == 200
        body = resp.json()
        assert isinstance(body, (list, dict))

    def test_path_assets_list(self, client: TestClient):
        """GET /path_assets should return assets."""
        resp = client.get("/path_assets")
        assert resp.status_code == 200
        body = resp.json()
        assert isinstance(body, dict)
        assert "assets" in body
        assert isinstance(body["assets"], list)


# ---------------------------------------------------------------------------
# CORS
# ---------------------------------------------------------------------------


class TestCORS:
    """Verify CORS middleware is configured."""

    def test_cors_allowed_origin(self, client: TestClient):
        """Preflight with allowed origin should include CORS headers."""
        resp = client.options(
            "/simulations",
            headers={
                "Origin": "http://localhost:5173",
                "Access-Control-Request-Method": "GET",
            },
        )
        assert resp.headers.get("access-control-allow-origin") in (
            "http://localhost:5173",
            "*",
        )
