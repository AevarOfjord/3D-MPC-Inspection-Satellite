#!/usr/bin/env python3
"""Runtime entrypoint used by PyInstaller bundles."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import webbrowser
from pathlib import Path

import uvicorn


def _runtime_root() -> Path:
    frozen_root = getattr(sys, "_MEIPASS", None)
    if frozen_root:
        return Path(str(frozen_root)).resolve()
    return Path(__file__).resolve().parents[1]


def _open_browser_later(host: str, port: int) -> None:
    time.sleep(1.0)
    webbrowser.open(f"http://{host}:{port}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Start packaged Mission Control app.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()

    runtime_root = _runtime_root()
    os.environ.setdefault("SATELLITE_CONTROL_ROOT", str(runtime_root))

    if not args.no_browser:
        threading.Thread(
            target=_open_browser_later,
            args=(args.host, args.port),
            daemon=True,
        ).start()

    uvicorn.run(
        "controller.shared.python.dashboard.app:app",
        host=args.host,
        port=args.port,
        reload=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
