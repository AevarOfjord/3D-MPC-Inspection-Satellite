"""
CasADi C Code Generator for V2 MPC.

Generates C code from CasADi symbolic expressions (dynamics + cost) and
caches the output in `codegen_cache/`. Re-generates only when the source
hash changes.

Usage:
    python -m control.codegen.generate          # generate all
    python -m control.codegen.generate --force   # force regeneration
"""

import hashlib
import json
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

CACHE_DIR = Path(__file__).resolve().parent.parent.parent.parent / "codegen_cache"


def _source_hash(*source_files: Path) -> str:
    """Compute a combined SHA256 hash of source files."""
    h = hashlib.sha256()
    for f in sorted(source_files):
        if f.exists():
            h.update(f.read_bytes())
    return h.hexdigest()[:16]


def _needs_regen(tag: str, current_hash: str) -> bool:
    meta = CACHE_DIR / f"{tag}.meta.json"
    if not meta.exists():
        return True
    try:
        data = json.loads(meta.read_text())
        return data.get("hash") != current_hash
    except Exception:
        return True


def _write_meta(tag: str, current_hash: str) -> None:
    meta = CACHE_DIR / f"{tag}.meta.json"
    meta.write_text(json.dumps({"hash": current_hash}))


def generate_dynamics(
    num_thrusters: int = 6,
    num_rw: int = 3,
    dt: float = 0.05,
    force: bool = False,
) -> Path:
    """Generate C code for satellite dynamics + Jacobians."""
    from .satellite_dynamics import SatelliteDynamicsSymbolic

    src_dir = Path(__file__).resolve().parent
    src_files = [
        src_dir / "satellite_dynamics.py",
    ]
    h = _source_hash(*src_files) + f"_t{num_thrusters}_r{num_rw}_dt{dt}"

    tag = "dynamics"
    if not force and not _needs_regen(tag, h):
        logger.info("Dynamics codegen cache hit (%s)", h)
        return CACHE_DIR / tag

    logger.info("Generating CasADi C code for dynamics...")
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    out_dir = CACHE_DIR / tag
    out_dir.mkdir(parents=True, exist_ok=True)

    dyn = SatelliteDynamicsSymbolic(
        num_thrusters=num_thrusters,
        num_rw=num_rw,
        dt=dt,
    )
    dyn.generate_c_code(str(out_dir))

    _write_meta(tag, h)
    logger.info("Dynamics codegen complete → %s", out_dir)
    return out_dir


def generate_cost(
    num_thrusters: int = 6,
    num_rw: int = 3,
    force: bool = False,
) -> Path:
    """Generate C code for MPCC cost functions."""
    from .cost_functions import MPCCStageCost, MPCCTerminalCost

    src_dir = Path(__file__).resolve().parent
    src_files = [
        src_dir / "cost_functions.py",
    ]
    h = _source_hash(*src_files) + f"_t{num_thrusters}_r{num_rw}"

    tag = "cost"
    if not force and not _needs_regen(tag, h):
        logger.info("Cost codegen cache hit (%s)", h)
        return CACHE_DIR / tag

    logger.info("Generating CasADi C code for cost functions...")
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    out_dir = CACHE_DIR / tag
    out_dir.mkdir(parents=True, exist_ok=True)

    nx = 17
    nu = num_rw + num_thrusters + 1

    stage_cost = MPCCStageCost(nx=nx, nu=nu, num_thrusters=num_thrusters, num_rw=num_rw)
    terminal_cost = MPCCTerminalCost(nx=nx)

    # Generate CasADi C code for stage cost Hessian + gradient
    import casadi as ca

    cg = ca.CodeGenerator("mpcc_cost", {"mex": False, "with_header": True})
    cg.add(stage_cost.H_and_g_func)
    cg.add(terminal_cost.terminal_func)
    cg.generate(str(out_dir) + "/")

    _write_meta(tag, h)
    logger.info("Cost codegen complete → %s", out_dir)
    return out_dir


def generate_all(
    num_thrusters: int = 6,
    num_rw: int = 3,
    dt: float = 0.05,
    force: bool = False,
) -> dict[str, Path]:
    """Generate all CasADi C code."""
    results = {}
    results["dynamics"] = generate_dynamics(num_thrusters, num_rw, dt, force)
    results["cost"] = generate_cost(num_thrusters, num_rw, force)
    return results


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    force = "--force" in sys.argv
    generate_all(force=force)
    print("Done.")
