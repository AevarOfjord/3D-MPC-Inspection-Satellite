"""Helper module to load compiled C++ extensions."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


def _inject_local_build_paths() -> None:
    root = Path(__file__).resolve().parents[3]
    for build_dir in sorted((root / "build").glob("cp*")):
        if not build_dir.is_dir():
            continue
        build_path = str(build_dir.resolve())
        if build_path not in sys.path:
            sys.path.insert(0, build_path)


def _load_extension(name: str):
    _inject_local_build_paths()
    candidates = (
        f"cpp.{name}",
        name,  # flat install fallback
        f"satellite_control.cpp.{name}",  # legacy editable layout
    )
    for module_name in candidates:
        try:
            return importlib.import_module(module_name)
        except Exception:
            continue
    return None


for _mod in ("_cpp_mpc", "_cpp_sim", "_cpp_physics"):
    _module = _load_extension(_mod)
    if _module is None:
        continue
    globals()[_mod] = _module
    sys.modules.setdefault(f"cpp.{_mod}", _module)
