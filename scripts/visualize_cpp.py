import sys
import os
import glob
from pathlib import Path
import subprocess

# Paths
ROOT_DIR = Path(__file__).resolve().parent.parent
LEGACY_DIR = ROOT_DIR / "_legacy_v1"
CPP_BUILD_DIR = ROOT_DIR / "Cplusplus_build" / "build"
CPP_DATA_DIR = CPP_BUILD_DIR / "Data" / "Simulation"

def main():
    # Find latest C++ run
    if not CPP_DATA_DIR.exists():
        print(f"No C++ data directory found at {CPP_DATA_DIR}")
        print("Please run the C++ simulation first.")
        sys.exit(1)

    # Filter for directories (timestamps)
    runs = [p for p in CPP_DATA_DIR.iterdir() if p.is_dir()]
    if not runs:
        print("No C++ simulation runs found.")
        sys.exit(1)

    # Sort by name (timestamp format YYYY... helps, but format is DD-MM-YYYY which sorts badly lexicographically)
    # Format: 11-01-2026_17-14-13
    # We should sort by modification time or parse timestamp.
    # Modification time is easiest.
    latest_run = max(runs, key=lambda p: p.stat().st_mtime)
    
    print(f"Visualizing latest run: {latest_run.name}")
    print(f"Path: {latest_run}")

    # Visualizer Script Path
    viz_script = LEGACY_DIR / "src" / "satellite_control" / "visualization" / "unified_visualizer.py"
    
    # Environment with PYTHONPATH
    env = os.environ.copy()
    env["PYTHONPATH"] = str(LEGACY_DIR) + os.pathsep + env.get("PYTHONPATH", "")

    # Command
    cmd = [
        sys.executable,
        str(viz_script),
        "--data-dir",
        str(latest_run),
        "--mode", "simulation"
    ]

    print(f"Running Visualizer...")
    try:
        subprocess.run(cmd, env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Visualizer failed with code {e.returncode}")
    except KeyboardInterrupt:
        print("\nVisualization interrupted.")

if __name__ == "__main__":
    main()
