"""Check attribute names on the loaded acados solver (name mangling)."""

import logging
import os
import sys

sys.path.insert(0, ".")
os.environ.setdefault("SATELLITE_HEADLESS", "1")
logging.disable(logging.CRITICAL)
from controller.acados_rti.python.controller import AcadosRtiController
from controller.configs.simulation_config import SimulationConfig

cfg = SimulationConfig.create_default()
ctrl = AcadosRtiController(cfg.app_config)
solver = ctrl._acados_solver

# Check what C-level attributes exist
attrs = [
    a
    for a in dir(solver)
    if "nlp" in a.lower() or "config" in a.lower() or "lib" in a.lower()
]
print("Relevant attributes:")
for a in attrs:
    try:
        v = getattr(solver, a)
        print(f"  {a}: {type(v).__name__}")
    except Exception as e:
        print(f"  {a}: ERROR {e}")

# Check mangled names specifically
for mangle in [
    "_AcadosOcpSolver__acados_lib",
    "_AcadosOcpSolver__nlp_config",
    "_AcadosOcpSolver__nlp_dims",
    "_AcadosOcpSolver__nlp_in",
]:
    if hasattr(solver, mangle):
        print(f"  {mangle}: {type(getattr(solver, mangle)).__name__} ✓")
    else:
        print(f"  {mangle}: NOT FOUND")
