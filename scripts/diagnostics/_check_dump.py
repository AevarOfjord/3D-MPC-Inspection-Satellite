"""Check AcadosOcp.dump_to_json signature."""

import sys

sys.path.insert(0, ".")
import inspect

from acados_template import AcadosOcp

src = inspect.getsource(AcadosOcp.dump_to_json)
print(src[:2000])
