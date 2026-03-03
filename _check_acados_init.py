"""Check full AcadosOcpSolver.__init__ source around the JSON loading."""

import sys

sys.path.insert(0, ".")
import inspect

from acados_template import AcadosOcpSolver

src = inspect.getsource(AcadosOcpSolver.__init__)
lines = src.split("\n")
# Print lines 40-90 (covers JSON reading)
for i, line in enumerate(lines[35:95], start=36):
    print(f"{i:3d}: {line}")
