#!/usr/bin/env python3
"""
MuJoCo replay viewer.
Loads simulation CSV data and replays it using the MuJoCo physics engine.
"""

import argparse
import sys
from pathlib import Path

import pandas as pd

# Add project root to sys.path
ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.satellite_control.visualization.mujoco_replay_visualizer import MuJoCoReplayVisualizer


def _latest_run_dir(base: Path) -> Path:
    if not base.exists():
        raise FileNotFoundError(f"Base path not found: {base}")
    dirs = [p for p in base.iterdir() if p.is_dir()]
    if not dirs:
        raise FileNotFoundError(f"No run folders in {base}")
    return max(dirs, key=lambda p: p.stat().st_mtime)


def _resolve_csv(run_dir: Path, source: str) -> Path:
    if source == "physics":
        csv_path = run_dir / "physics_data.csv"
    else:
        csv_path = run_dir / "control_data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    return csv_path


def _preprocess_argv(argv: list) -> list:
    # Handle direct path arguments that might look like flags to argparse
    if len(argv) == 2:
        candidate = argv[1]
        if candidate.startswith("-") and not candidate.startswith("--"):
            if "/" in candidate or "\\" in candidate:
                return [argv[0], candidate[1:]]
    return argv


def main() -> int:
    argv = _preprocess_argv(sys.argv)

    parser = argparse.ArgumentParser(
        description="Replay MuJoCo simulation with timeline control."
    )
    parser.add_argument("run_dir", nargs="?", help="Simulation run directory.")
    parser.add_argument(
        "--csv", type=str, help="Path to control_data.csv or physics_data.csv."
    )
    parser.add_argument(
        "--source",
        choices=["control", "physics"],
        default="control",
        help="Which CSV to use when run_dir is provided.",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="models/satellite_3d.xml",
        help="MuJoCo model XML to render.",
    )
    args = parser.parse_args(argv[1:])

    # Locate CSV Data
    if args.csv:
        csv_path = Path(args.csv)
        if not csv_path.exists():
            raise FileNotFoundError(f"CSV not found: {csv_path}")
    else:
        base = Path("Data") / "Simulation"
        if args.run_dir:
            run_dir = Path(args.run_dir)
        else:
            run_dir = _latest_run_dir(base)
        csv_path = _resolve_csv(run_dir, args.source)

    print(f"Loading replay data from: {csv_path}")
    df = pd.read_csv(csv_path)
    if len(df) == 0:
        raise RuntimeError(f"Empty CSV: {csv_path}")

    # Launch Visualizer
    print("Launching MuJoCo Viewer...")
    print("Controls:")
    print("  SPACE       : Pause/Play")
    print("  LEFT/RIGHT  : Step Backward/Forward")
    print("  UP/DOWN     : Increase/Decrease Playback Speed")
    print("  R           : Reset to Start")
    
    visualizer = MuJoCoReplayVisualizer(model_path=args.model, df=df)
    visualizer.launch()

    return 0


if __name__ == "__main__":
    sys.exit(main())
