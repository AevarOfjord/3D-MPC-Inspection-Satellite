import io
import json
import zipfile
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from satellite_control.dashboard.app import app
from satellite_control.dashboard.routes import runner as runner_routes
from satellite_control.mission import repository as mission_repo


def _zip_bytes(entries: dict[str, bytes | str]) -> bytes:
    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path, payload in entries.items():
            if isinstance(payload, str):
                zf.writestr(path, payload.encode("utf-8"))
            else:
                zf.writestr(path, payload)
    return buffer.getvalue()


@pytest.fixture
def client():
    with TestClient(app) as test_client:
        yield test_client


@pytest.fixture
def isolated_workspace(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    missions_dir = tmp_path / "missions"
    sim_dir = tmp_path / "data" / "simulation_data"
    release_dir = tmp_path / "release"
    presets_file = tmp_path / "data" / "dashboard" / "runner_presets.json"
    missions_dir.mkdir(parents=True, exist_ok=True)
    sim_dir.mkdir(parents=True, exist_ok=True)
    release_dir.mkdir(parents=True, exist_ok=True)
    presets_file.parent.mkdir(parents=True, exist_ok=True)

    monkeypatch.setattr(mission_repo, "MISSIONS_DIR", missions_dir, raising=True)
    monkeypatch.setattr(runner_routes, "MISSIONS_DIR", missions_dir, raising=True)
    monkeypatch.setitem(mission_repo.SOURCE_DIRS, "local", missions_dir)
    monkeypatch.setattr(runner_routes, "_SIMULATION_DIR", sim_dir, raising=True)
    monkeypatch.setattr(runner_routes, "_RELEASE_DIR", release_dir, raising=True)

    manager = runner_routes.get_runner_manager()
    manager._presets_path = presets_file
    manager.clear_presets()
    manager.reset_config()

    return {
        "missions_dir": missions_dir,
        "sim_dir": sim_dir,
        "presets_file": presets_file,
    }


def test_workspace_export_includes_simulation_runs(
    client: TestClient, isolated_workspace
):
    mission_repo.save_mission_json(
        name="MissionAlpha",
        payload={"segments": [], "start_pose": {"position": [0, 0, 0]}},
        source="local",
    )
    manager = runner_routes.get_runner_manager()
    manager.save_preset(
        "preset-alpha",
        {"mpc": {"prediction_horizon": 10, "control_horizon": 10, "dt": 0.05}},
    )

    run_dir = isolated_workspace["sim_dir"] / "run_001"
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "physics_data.csv").write_text("time,x\n0,0\n", encoding="utf-8")

    response = client.get("/runner/workspace/export?include_simulation_data=true")
    assert response.status_code == 200
    assert response.headers.get("content-type", "").startswith("application/zip")

    zf = zipfile.ZipFile(io.BytesIO(response.content))
    names = set(zf.namelist())
    assert "workspace_manifest.json" in names
    assert "runner_presets.json" in names
    assert "runner_overrides.json" in names
    assert "missions/MissionAlpha.json" in names
    assert "simulation_runs/run_001/physics_data.csv" in names


def test_workspace_inspect_reports_conflicts(client: TestClient, isolated_workspace):
    mission_repo.save_mission_json(
        name="MissionConflict",
        payload={"segments": [], "start_pose": {"position": [0, 0, 0]}},
        source="local",
    )
    manager = runner_routes.get_runner_manager()
    manager.save_preset(
        "preset-conflict",
        {"mpc": {"prediction_horizon": 10, "control_horizon": 10, "dt": 0.05}},
    )
    sim_conflict_dir = isolated_workspace["sim_dir"] / "run_conflict"
    sim_conflict_dir.mkdir(parents=True, exist_ok=True)

    payload = _zip_bytes(
        {
            "missions/MissionConflict.json": json.dumps(
                {"segments": [], "start_pose": {"position": [1, 1, 1]}}
            ),
            "runner_presets.json": json.dumps(
                {
                    "presets": {
                        "preset-conflict": {
                            "config": {
                                "mpc": {
                                    "prediction_horizon": 11,
                                    "control_horizon": 11,
                                    "dt": 0.05,
                                }
                            }
                        }
                    }
                }
            ),
            "simulation_runs/run_conflict/physics_data.csv": "time,x\n0,0\n",
        }
    )
    response = client.post(
        "/runner/workspace/inspect",
        files={"file": ("workspace.zip", payload, "application/zip")},
    )
    assert response.status_code == 200
    body = response.json()
    assert body["counts"]["mission_conflicts"] == 1
    assert body["counts"]["preset_conflicts"] == 1
    assert body["counts"]["simulation_run_conflicts"] == 1
    assert "MissionConflict.json" in body["conflicts"]["missions"]
    assert "preset-conflict" in body["conflicts"]["presets"]
    assert "run_conflict" in body["conflicts"]["simulation_runs"]


def test_workspace_import_per_item_overwrite(client: TestClient, isolated_workspace):
    original_mission = {"segments": [], "start_pose": {"position": [0, 0, 0]}}
    mission_repo.save_mission_json(
        name="MissionKeep",
        payload=original_mission,
        source="local",
    )
    mission_repo.save_mission_json(
        name="MissionOverwrite",
        payload=original_mission,
        source="local",
    )

    manager = runner_routes.get_runner_manager()
    manager.save_preset(
        "preset-keep",
        {"mpc": {"prediction_horizon": 9, "control_horizon": 9, "dt": 0.05}},
    )
    manager.save_preset(
        "preset-overwrite",
        {"mpc": {"prediction_horizon": 9, "control_horizon": 9, "dt": 0.05}},
    )

    run_keep = isolated_workspace["sim_dir"] / "run_keep"
    run_keep.mkdir(parents=True, exist_ok=True)
    (run_keep / "physics_data.csv").write_text("time,x\n0,0\n", encoding="utf-8")
    run_overwrite = isolated_workspace["sim_dir"] / "run_overwrite"
    run_overwrite.mkdir(parents=True, exist_ok=True)
    (run_overwrite / "physics_data.csv").write_text("time,x\n0,0\n", encoding="utf-8")

    bundle = _zip_bytes(
        {
            "missions/MissionKeep.json": json.dumps(
                {"segments": [], "start_pose": {"position": [10, 0, 0]}}
            ),
            "missions/MissionOverwrite.json": json.dumps(
                {"segments": [], "start_pose": {"position": [20, 0, 0]}}
            ),
            "runner_presets.json": json.dumps(
                {
                    "presets": {
                        "preset-keep": {
                            "config": {
                                "mpc": {
                                    "prediction_horizon": 12,
                                    "control_horizon": 12,
                                    "dt": 0.05,
                                }
                            }
                        },
                        "preset-overwrite": {
                            "config": {
                                "mpc": {
                                    "prediction_horizon": 13,
                                    "control_horizon": 13,
                                    "dt": 0.05,
                                }
                            }
                        },
                    }
                }
            ),
            "simulation_runs/run_keep/physics_data.csv": "time,x\n1,1\n",
            "simulation_runs/run_overwrite/physics_data.csv": "time,x\n2,2\n",
        }
    )

    response = client.post(
        "/runner/workspace/import",
        data={
            "replace_existing_missions": "false",
            "replace_existing_presets": "false",
            "replace_existing_simulation_runs": "false",
            "apply_runner_config": "false",
            "overwrite_missions_json": json.dumps(["MissionOverwrite.json"]),
            "overwrite_presets_json": json.dumps(["preset-overwrite"]),
            "overwrite_simulation_runs_json": json.dumps(["run_overwrite"]),
        },
        files={"file": ("workspace.zip", bundle, "application/zip")},
    )
    assert response.status_code == 200
    body = response.json()
    assert body["missions_imported"] == 1
    assert body["missions_skipped"] == 1
    assert body["presets_imported"] == 1
    assert body["presets_skipped"] == 1
    assert body["simulation_runs_imported"] == 1
    assert body["simulation_runs_skipped"] == 1
    assert body["config_imported"] is False

    mission_keep = json.loads(
        (isolated_workspace["missions_dir"] / "MissionKeep.json").read_text(
            encoding="utf-8"
        )
    )
    mission_overwrite = json.loads(
        (isolated_workspace["missions_dir"] / "MissionOverwrite.json").read_text(
            encoding="utf-8"
        )
    )
    assert mission_keep["start_pose"]["position"] == [0, 0, 0]
    assert mission_overwrite["start_pose"]["position"] == [20, 0, 0]

    presets = manager.list_presets()
    assert presets["preset-keep"]["config"]["mpc"]["prediction_horizon"] == 9
    assert presets["preset-overwrite"]["config"]["mpc"]["prediction_horizon"] == 13

    keep_csv = (
        isolated_workspace["sim_dir"] / "run_keep" / "physics_data.csv"
    ).read_text(encoding="utf-8")
    overwrite_csv = (
        isolated_workspace["sim_dir"] / "run_overwrite" / "physics_data.csv"
    ).read_text(encoding="utf-8")
    assert "1,1" not in keep_csv
    assert "2,2" in overwrite_csv
