"""Contract and compatibility tests for mission authoring API v2."""

from __future__ import annotations

from typing import Any

import pytest
from fastapi.testclient import TestClient
from satellite_control.dashboard import mission_v2_service
from satellite_control.dashboard.app import app
from satellite_control.mission import repository as mission_repo


def _sample_mission_payload(*, mission_id: str = "mission_test") -> dict[str, Any]:
    return {
        "schema_version": 2,
        "mission_id": mission_id,
        "name": "Mission Test",
        "epoch": "2026-01-01T00:00:00Z",
        "start_pose": {
            "frame": "ECI",
            "position": [0.0, 0.0, 0.0],
        },
        "segments": [
            {
                "type": "hold",
                "segment_id": "seg_hold_001",
                "duration": 12.0,
            }
        ],
        "metadata": {
            "version": 1,
            "tags": ["test"],
        },
    }


@pytest.fixture
def client(tmp_path, monkeypatch):
    missions_dir = tmp_path / "missions"
    drafts_dir = tmp_path / "mission_drafts_v2"
    monkeypatch.setattr(mission_v2_service, "MISSIONS_DIR", missions_dir)
    monkeypatch.setattr(mission_v2_service, "DRAFTS_DIR", drafts_dir)
    monkeypatch.setitem(mission_repo.SOURCE_DIRS, "local", missions_dir)
    with TestClient(app) as test_client:
        yield test_client


def test_v2_validate_returns_structured_report(client: TestClient):
    response = client.post("/api/v2/missions/validate", json=_sample_mission_payload())
    assert response.status_code == 200
    payload = response.json()
    assert payload["valid"] is True
    assert payload["summary"]["errors"] == 0
    assert isinstance(payload["issues"], list)


def test_v2_validate_detects_missing_segments(client: TestClient):
    mission = _sample_mission_payload()
    mission["segments"] = []
    response = client.post("/api/v2/missions/validate", json=mission)
    assert response.status_code == 200
    payload = response.json()
    assert payload["valid"] is False
    assert any(issue["code"] == "MISSION_SEGMENTS_REQUIRED" for issue in payload["issues"])


def test_v2_save_list_and_load_roundtrip(client: TestClient):
    mission = _sample_mission_payload(mission_id="mission_roundtrip")
    save = client.post(
        "/api/v2/missions",
        json={
            "name": "Mission Roundtrip",
            "mission": mission,
        },
    )
    assert save.status_code == 200
    saved = save.json()
    assert saved["mission_id"] == "mission_roundtrip"
    assert saved["version"] == 1
    assert saved["filename"].endswith(".json")

    list_response = client.get("/api/v2/missions")
    assert list_response.status_code == 200
    missions = list_response.json()
    assert len(missions) == 1
    assert missions[0]["mission_id"] == "mission_roundtrip"
    assert missions[0]["segments_count"] == 1

    load = client.get(f"/api/v2/missions/{saved['mission_id']}")
    assert load.status_code == 200
    loaded = load.json()
    assert loaded["schema_version"] == 2
    assert loaded["name"] == "Mission Roundtrip"
    assert loaded["segments"][0]["segment_id"] == "seg_hold_001"


def test_v2_draft_save_load_and_conflict(client: TestClient):
    mission = _sample_mission_payload(mission_id="mission_draft")

    draft_first = client.post(
        "/api/v2/missions/drafts",
        json={
            "mission": mission,
        },
    )
    assert draft_first.status_code == 200
    first = draft_first.json()
    assert first["revision"] == 1
    assert first["draft_id"].startswith("draft_")

    draft_second = client.post(
        "/api/v2/missions/drafts",
        json={
            "draft_id": first["draft_id"],
            "base_revision": first["revision"],
            "mission": mission,
        },
    )
    assert draft_second.status_code == 200
    second = draft_second.json()
    assert second["revision"] == 2

    conflict = client.post(
        "/api/v2/missions/drafts",
        json={
            "draft_id": first["draft_id"],
            "base_revision": 1,
            "mission": mission,
        },
    )
    assert conflict.status_code == 409
    assert conflict.json()["detail"]["expected_revision"] == 2

    loaded = client.get(f"/api/v2/missions/drafts/{first['draft_id']}")
    assert loaded.status_code == 200
    assert loaded.json()["revision"] == 2


def test_legacy_endpoints_emit_deprecation_headers(client: TestClient):
    response = client.get("/saved_missions_v2")
    assert response.status_code == 200
    assert response.headers.get("Deprecation") == "true"
    assert response.headers.get("Sunset")
    assert "deprecation" in (response.headers.get("Link") or "")


def test_legacy_save_adapter_persists_v2_payload(client: TestClient):
    legacy_payload = {
        "epoch": "2026-01-01T00:00:00Z",
        "start_pose": {
            "frame": "ECI",
            "position": [0.0, 0.0, 0.0],
        },
        "segments": [
            {
                "type": "hold",
                "duration": 5.0,
            }
        ],
    }
    save_response = client.post(
        "/save_mission_v2",
        json={
            "name": "Legacy Adapter Mission",
            "config": legacy_payload,
        },
    )
    assert save_response.status_code == 200
    assert save_response.json()["status"] == "success"

    loaded = client.get("/api/v2/missions/LegacyAdapterMission")
    assert loaded.status_code == 200
    payload = loaded.json()
    assert payload["schema_version"] == 2
    assert payload["name"] == "Legacy Adapter Mission"
    assert payload["segments"][0]["segment_id"]
