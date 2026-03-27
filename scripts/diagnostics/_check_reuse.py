"""Check is_code_reuse_possible source."""

import sys

sys.path.insert(0, ".")
import inspect

from acados_template import AcadosOcpSolver

src = inspect.getsource(AcadosOcpSolver.is_code_reuse_possible)
print(src[:3000])
