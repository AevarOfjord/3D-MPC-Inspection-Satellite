from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

from controller.shared.python.simulation import io as simulation_io_module
from controller.shared.python.simulation.io import SimulationIO


class _DummySim:
    def __init__(self, profile: str | None = None) -> None:
        self.controller_profile_mode = profile
        self.mpc_controller = None
        self.simulation_config = None


def test_create_data_directories_includes_controller_profile(
    tmp_path: Path, monkeypatch
) -> None:
    monkeypatch.setattr(simulation_io_module, "SIMULATION_DATA_ROOT", tmp_path)

    sim = _DummySim(profile="cpp_nonlinear_rti_hpipm")
    io = SimulationIO(sim)

    run_dir = io.create_data_directories()

    assert run_dir.exists()
    assert "__nonlinear_rti_hpipm__auto" in run_dir.name
    assert (run_dir / "Data").exists()
    assert (run_dir / "Plots").exists()


def test_sanitize_run_token_normalizes_unsafe_characters() -> None:
    token = SimulationIO._sanitize_run_token("  cpp nonlinear/rti:hpipm  ")
    assert token == "cpp_nonlinear_rti_hpipm"


def test_resolve_unique_run_dir_adds_numeric_suffix(tmp_path: Path) -> None:
    sim = _DummySim(profile="cpp_linearized_rti_osqp")
    io = SimulationIO(sim)

    base_name = "04-03-2026_15-30-00__linearized_rti_osqp__simplesway"
    existing = tmp_path / base_name
    existing.mkdir(parents=True, exist_ok=True)

    unique = io._resolve_unique_run_dir(tmp_path, base_name)
    assert unique.name == base_name + "__02"

    unique.mkdir(parents=True, exist_ok=True)
    unique_2 = io._resolve_unique_run_dir(tmp_path, base_name)
    assert unique_2.name == base_name + "__03"


def test_resolve_mission_token_prefers_env_name(monkeypatch) -> None:
    sim = _DummySim(profile="cpp_hybrid_rti_osqp")
    io = SimulationIO(sim)

    monkeypatch.setenv("SATCTRL_RUNNER_MISSION_NAME", "Starlink FullScan")
    monkeypatch.setenv("SATCTRL_RUNNER_MISSION_PATH", "/tmp/ShouldNotBeUsed.json")

    assert io._resolve_mission_token() == "Starlink_FullScan"


def test_resolve_mission_token_falls_back_to_input_path(monkeypatch) -> None:
    sim = _DummySim(profile="cpp_hybrid_rti_osqp")
    sim.simulation_config = SimpleNamespace(
        app_config=SimpleNamespace(input_file_path="/tmp/missions/SimpleSway.json")
    )
    io = SimulationIO(sim)

    monkeypatch.delenv("SATCTRL_RUNNER_MISSION_NAME", raising=False)
    monkeypatch.delenv("SATCTRL_RUNNER_MISSION_PATH", raising=False)

    assert io._resolve_mission_token() == "SimpleSway"
