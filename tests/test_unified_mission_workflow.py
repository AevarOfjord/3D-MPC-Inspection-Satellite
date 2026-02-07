import builtins
import json
import subprocess
from pathlib import Path

import pytest
from fastapi.testclient import TestClient
from typer.testing import CliRunner

from src.satellite_control import cli as cli_module
from src.satellite_control.dashboard.app import app
from src.satellite_control.mission import repository as mission_repo


def _sample_unified_mission() -> dict:
    return {
        "epoch": "2026-01-01T00:00:00Z",
        "start_pose": {"frame": "ECI", "position": [0.0, 0.0, 0.0]},
        "segments": [
            {
                "type": "transfer",
                "end_pose": {"frame": "ECI", "position": [1.0, 0.0, 0.0]},
            }
        ],
        "obstacles": [],
    }


@pytest.fixture
def isolated_mission_repo(tmp_path, monkeypatch):
    missions_dir = tmp_path / "missions_unified"
    missions_dir.mkdir(parents=True, exist_ok=True)
    monkeypatch.setattr(mission_repo, "MISSIONS_UNIFIED_DIR", missions_dir)
    monkeypatch.setattr(mission_repo, "SOURCE_DIRS", {"unified": missions_dir})
    return missions_dir


def test_web_save_then_terminal_discovery_prompt(isolated_mission_repo, monkeypatch):
    with TestClient(app) as client:
        response = client.post(
            "/save_mission_v2",
            json={"name": "Web Mission", "config": _sample_unified_mission()},
        )
    assert response.status_code == 200
    assert response.json()["filename"] == "WebMission.json"

    entries = mission_repo.list_mission_entries(source_priority=("unified",))
    assert [entry.name for entry in entries] == ["WebMission.json"]

    real_import = builtins.__import__

    def _import_without_questionary(
        name,
        globals=None,
        locals=None,
        fromlist=(),
        level=0,
    ):
        if name == "questionary":
            raise ImportError("disabled in test")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", _import_without_questionary)
    monkeypatch.setattr("builtins.input", lambda _: "1")
    selected = cli_module._prompt_saved_mission_file()
    assert selected == str(entries[0].path)


def test_cli_runs_unified_mission_file_path(isolated_mission_repo, monkeypatch):
    mission_path = isolated_mission_repo / "CliRunMission.json"
    mission_path.write_text(json.dumps(_sample_unified_mission()))

    class DummySimulation:
        init_kwargs = None
        run_called = False
        close_called = False
        show_animation = None

        def __init__(self, *args, **kwargs):
            type(self).init_kwargs = kwargs

        def run_simulation(self, show_animation=True):
            type(self).run_called = True
            type(self).show_animation = show_animation

        def close(self):
            type(self).close_called = True

    monkeypatch.setattr(
        cli_module, "SatelliteMPCLinearizedSimulation", DummySimulation
    )

    runner = CliRunner()
    result = runner.invoke(
        cli_module.app, ["--mission", str(mission_path), "--no-anim"]
    )

    assert result.exit_code == 0, result.output
    assert DummySimulation.run_called is True
    assert DummySimulation.close_called is True
    assert DummySimulation.show_animation is False

    kwargs = DummySimulation.init_kwargs
    assert kwargs is not None
    sim_config = kwargs["simulation_config"]
    assert sim_config.mission_state.path_waypoints
    assert kwargs["start_pos"] == tuple(sim_config.mission_state.path_waypoints[0])
    assert kwargs["end_pos"] == tuple(sim_config.mission_state.path_waypoints[-1])


def test_run_mission_endpoint_spawns_cli_with_saved_mission(
    isolated_mission_repo, monkeypatch
):
    captured = {}

    class DummyPopen:
        def __init__(self, cmd, cwd, stdout, stderr, start_new_session):
            captured["cmd"] = cmd
            captured["cwd"] = cwd
            captured["stdout"] = stdout
            captured["stderr"] = stderr
            captured["start_new_session"] = start_new_session
            self.pid = 4242

    monkeypatch.setattr(subprocess, "Popen", DummyPopen)

    with TestClient(app) as client:
        save = client.post(
            "/save_mission_v2",
            json={"name": "Run Endpoint Mission", "config": _sample_unified_mission()},
        )
        assert save.status_code == 200

        run_resp = client.post(
            "/run_mission", json={"mission_name": "RunEndpointMission"}
        )

    assert run_resp.status_code == 200
    body = run_resp.json()
    assert body["status"] == "started"
    assert body["pid"] == 4242

    cmd = captured["cmd"]
    assert "--mission" in cmd
    assert "python" in Path(cmd[0]).name
    assert cmd[1:4] == ["-m", "src.satellite_control.cli", "--mission"]
    assert Path(cmd[4]).name == "RunEndpointMission.json"
    assert cmd[5] == "--no-anim"
    assert captured["start_new_session"] is True
