"""
Helper module to load compiled C++ extensions.

The project imports extensions via the `satellite_control` namespace during
development, but the compiled modules are installed under `satellite_control`.
This shim aliases the installed modules so `satellite_control.cpp._cpp_*`
imports resolve without copying binaries into the source tree.
"""

from __future__ import annotations

import importlib
import sys


def _alias_extension(name: str) -> None:
    try:
        module = importlib.import_module(f"satellite_control.cpp.{name}")
    except Exception:
        return
    sys.modules.setdefault(f"satellite_control.cpp.{name}", module)


for _mod in ("_cpp_mpc", "_cpp_sim", "_cpp_physics"):
    _alias_extension(_mod)
