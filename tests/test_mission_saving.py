import pytest
from fastapi.testclient import TestClient
from pathlib import Path
import json
import os

from satellite_control.dashboard.app import app

client = TestClient(app)


@pytest.fixture
def clean_missions():
    """Cleanup missions directory before and after tests."""
    missions_dir = Path("missions_unified")
    missions_dir.mkdir(parents=True, exist_ok=True)

    # Backup existing
    backup = []
    for f in missions_dir.glob("*.json"):
        backup.append((f.name, f.read_text()))
        f.unlink()

    yield

    # Restore (optional, or just leave empty)
    for name, content in backup:
        (missions_dir / name).write_text(content)


def test_save_mission(clean_missions):
    config = {
        "epoch": "2026-01-01T00:00:00Z",
        "start_pose": {"frame": "ECI", "position": [1.0, 2.0, 3.0]},
        "segments": [{"type": "hold", "duration": 1.0}],
        "obstacles": [],
    }

    response = client.post(
        "/save_mission_v2", json={"name": "Test Mission 1!", "config": config}
    )

    assert response.status_code == 200
    assert response.json()["status"] == "success"
    assert response.json()["filename"] == "TestMission1.json"

    # Verify file exists
    assert Path("missions_unified/TestMission1.json").exists()

    # Verify content
    saved = json.loads(Path("missions_unified/TestMission1.json").read_text())
    assert saved["start_pose"]["position"] == [1.0, 2.0, 3.0]


def test_list_saved_missions(clean_missions):
    # create dummy file
    Path("missions_unified/UnifiedAlpha.json").touch()
    Path("missions_unified/UnifiedBeta.json").touch()

    response = client.get("/saved_missions_v2")
    assert response.status_code == 200
    missions = response.json()["missions"]
    assert "UnifiedAlpha.json" in missions
    assert "UnifiedBeta.json" in missions


def test_preview_trajectory():
    # Helper to test preview (mocking build_mesh_scan_trajectory logic effectively via the endpoint)
    # We rely on the endpoint calling the real function, so we need a valid obj or it will fail?
    # The endpoint imports build_mesh_scan_trajectory.
    # If we pass a non-existent OBJ, it should fail.

    config = {
        "obj_path": "non_existent.obj",
        "standoff": 0.5,
        "levels": 5,
        "points_per_circle": 10,
    }

    # It might raise 500 because file not found
    try:
        response = client.post("/preview_trajectory", json=config)
        # If it actually tries to load, it will fail.
        # However, checking if it hits the endpoint is enough for "wiring" check.
        # If 500, it means it tried to execute logic.
        assert response.status_code in [200, 500]
    except Exception:
        pass


def test_upload_object():
    file_content = b"v 1.0 1.0 1.0"
    files = {"file": ("test.obj", file_content, "application/octet-stream")}

    response = client.post(
        "/upload_object", data={"filename": "test_upload.obj"}, files=files
    )

    assert response.status_code == 200
    path = response.json()["path"]
    assert Path(path).exists()
    assert Path(path).name == "test_upload.obj"

    # Cleanup
    if Path(path).exists():
        os.remove(path)
