"""Helper module to load compiled C++ extensions."""

from __future__ import annotations

import importlib
import sys
from pathlib import Path


def _inject_local_build_paths() -> None:
    root = Path(__file__).resolve().parents[4]
    for build_dir in sorted((root / "build").glob("cp*")):
        if not build_dir.is_dir():
            continue
        build_path = str(build_dir.resolve())
        if build_path not in sys.path:
            sys.path.insert(0, build_path)
        # Also add to package __path__ so importlib finds submodules
        if build_path not in __path__:
            __path__.append(build_path)


def _load_extension(name: str):
    _inject_local_build_paths()
    try:
        return importlib.import_module(f"{__name__}.{name}")
    except Exception:
        try:
            return importlib.import_module(name)
        except Exception:
            return None


for _mod in (
    "_cpp_mpc",
    "_cpp_mpc_nonlinear",
    "_cpp_mpc_linear",
    "_cpp_sim",
    "_cpp_physics",
):
    _module = _load_extension(_mod)
    if _module is None:
        continue
    globals()[_mod] = _module
    # Backward-compat alias
    sys.modules.setdefault(f"cpp.{_mod}", _module)
