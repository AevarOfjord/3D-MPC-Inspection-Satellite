#!/usr/bin/env python3
"""Smoke test for packaged Mission Control binary."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path


def _project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _default_app_dir() -> Path:
    root = _project_root() / "release" / "pyinstaller"
    candidates = sorted(
        root.glob("*/SatelliteControl"), key=lambda p: p.stat().st_mtime
    )
    if not candidates:
        raise FileNotFoundError(
            "No packaged app directory found under release/pyinstaller/*/SatelliteControl"
        )
    return candidates[-1]


def _resolve_executable(app_dir: Path) -> Path:
    candidate = app_dir / (
        "SatelliteControl.exe" if sys.platform.startswith("win") else "SatelliteControl"
    )
    if candidate.exists():
        return candidate
    raise FileNotFoundError(f"Packaged executable not found: {candidate}")


def _port_open(host: str, port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.5)
        return sock.connect_ex((host, port)) == 0


def _wait_for_http_ready(host: str, port: int, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    url = f"http://{host}:{port}/"
    while time.time() < deadline:
        if _port_open(host, port):
            try:
                with urllib.request.urlopen(url, timeout=1.5) as response:
                    if 200 <= int(response.status) < 500:
                        return True
            except Exception:
                pass
        time.sleep(0.5)
    return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke test packaged Mission Control binary."
    )
    parser.add_argument(
        "--app-dir", default=None, help="Path to packaged app directory."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args()

    app_dir = Path(args.app_dir).resolve() if args.app_dir else _default_app_dir()
    executable = _resolve_executable(app_dir)
    print(f"Smoke testing packaged app: {executable}")

    proc = subprocess.Popen(
        [
            str(executable),
            "--host",
            args.host,
            "--port",
            str(args.port),
            "--no-browser",
        ],
        cwd=str(app_dir),
    )
    try:
        if not _wait_for_http_ready(args.host, int(args.port), float(args.timeout_s)):
            print("Packaged app failed readiness check.", file=sys.stderr)
            return 1
        print("Packaged app smoke test passed.")
        return 0
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except Exception:
            proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
