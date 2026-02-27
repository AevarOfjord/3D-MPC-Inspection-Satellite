import json
from pathlib import Path
from types import SimpleNamespace

from simulation.io import SimulationIO


def _write(path: Path, content: str = "x") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def test_build_plots_index_v2_with_manifest(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_001"
    plots_dir = run_dir / "Plots"
    _write(plots_dir / "01_overview" / "01_mission_overview.png")
    _write(plots_dir / "02_trajectory" / "05_trajectory_3d_interactive.html", "<html/>")
    _write(run_dir / "interactive_3d_plot.html", "<html/>")

    manifest = {
        "suite_version": "postrun_v2_full",
        "groups": [
            {
                "id": "overview",
                "order": 1,
                "title": "Overview",
                "path": "Plots/01_overview",
                "file_count": 1,
            },
            {
                "id": "trajectory",
                "order": 2,
                "title": "Trajectory",
                "path": "Plots/02_trajectory",
                "file_count": 1,
            },
        ],
        "files": [
            {
                "plot_id": "overview.mission_overview",
                "title": "Mission Overview",
                "path": "Plots/01_overview/01_mission_overview.png",
                "order": 10100,
                "group_id": "overview",
                "format": "png",
                "interactive": False,
                "status": "ok",
            },
            {
                "plot_id": "trajectory.3d_interactive",
                "title": "Trajectory 3D Interactive",
                "path": "Plots/02_trajectory/05_trajectory_3d_interactive.html",
                "order": 20500,
                "group_id": "trajectory",
                "format": "html",
                "interactive": True,
                "status": "ok",
            },
        ],
        "failures": [
            {"plot_id": "solver.fallback_breach_timeline", "reason": "no data"}
        ],
    }
    _write(plots_dir / "plot_manifest.json", json.dumps(manifest))

    io = SimulationIO(SimpleNamespace())
    index = io._build_plots_index(run_dir)
    assert index["schema_version"] == "plots_index_v2"
    assert index["suite_version"] == "postrun_v2_full"
    assert len(index["groups"]) == 2
    assert len(index["files"]) == 2
    assert len(index["failures"]) == 1
    # Backward-compatible field retained.
    assert len(index["plot_files"]) >= 2
    assert any(
        item["path"] == "interactive_3d_plot.html" for item in index["top_level_html"]
    )


def test_build_plots_index_v2_legacy_fallback(tmp_path: Path) -> None:
    run_dir = tmp_path / "run_legacy"
    _write(run_dir / "Plots" / "04_actuators" / "01_thruster_usage_summary.png")
    io = SimulationIO(SimpleNamespace())
    index = io._build_plots_index(run_dir)
    assert index["schema_version"] == "plots_index_v2"
    assert len(index["groups"]) == 1
    assert len(index["files"]) == 1
    assert index["files"][0]["status"] == "ok"
