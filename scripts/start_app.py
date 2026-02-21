#!/usr/bin/env python3
"""Zero-terminal app launcher for local users."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
import webbrowser
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _venv_python(project_root: Path) -> Path | None:
    candidates = [
        project_root / ".venv311" / "bin" / "python",
        project_root / ".venv311" / "Scripts" / "python.exe",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Start Mission Control app.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    root = _project_root()
    ui_index = root / "ui" / "dist" / "index.html"
    if not ui_index.exists():
        print("Error: prebuilt UI missing at ui/dist/index.html")
        print("Run 'make ui-build' first (or use a packaged app bundle).")
        return 2

    python_exec = _venv_python(root) or Path(sys.executable)
    cmd = [
        str(python_exec),
        "-m",
        "satellite_control.cli",
        "serve",
        "--host",
        args.host,
        "--port",
        str(args.port),
    ]

    print(f"Starting Mission Control on http://{args.host}:{args.port}")
    print(f"Using Python: {python_exec}")
    proc = subprocess.Popen(cmd, cwd=str(root))

    if not args.no_browser:
        time.sleep(1.0)
        webbrowser.open(f"http://{args.host}:{args.port}")

    print("Press Ctrl+C to stop.")
    try:
        proc.wait()
        return int(proc.returncode or 0)
    except KeyboardInterrupt:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
