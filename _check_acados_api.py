"""Quick check that AcadosOcpSolver.set supports 'W' and 'W_e' fields."""

import sys

sys.path.insert(0, ".")
import inspect

try:
    from acados_template import AcadosOcpSolver

    # Check the set method signature and docstring
    src = inspect.getsource(AcadosOcpSolver.set)
    # Look for 'W' in supported fields
    matching_lines = [
        line for line in src.split("\n") if "W" in line or "cost" in line.lower()
    ]
    for line in matching_lines[:30]:
        print(repr(line))
except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()
