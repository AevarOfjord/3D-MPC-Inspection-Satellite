import sys
from pathlib import Path

import uvicorn


def _ensure_src_on_path() -> None:
    repo_root = Path(__file__).resolve().parent
    src_path = repo_root / "src"
    if src_path.exists():
        sys.path.insert(0, str(src_path))


if __name__ == "__main__":
    _ensure_src_on_path()
    uvicorn.run(
        "satellite_control.dashboard.app:app",
        host="127.0.0.1",
        port=8000,
        reload=True,
        reload_dirs=["src"],
    )
