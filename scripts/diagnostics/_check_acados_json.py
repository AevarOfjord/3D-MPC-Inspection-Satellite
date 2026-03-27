"""Check whether AcadosOcpSolver with build=False writes JSON or reads existing."""

import sys

sys.path.insert(0, ".")
try:
    import inspect

    from acados_template import AcadosOcpSolver

    src = inspect.getsource(AcadosOcpSolver.__init__)
    # Find the block that handles JSON writing
    lines = src.split("\n")
    for i, line in enumerate(lines):
        if any(kw in line for kw in ["json", "dump", "generate", "build", "json_file"]):
            print(f"{i:3d}: {line}")
        if i > 200:
            break
except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()
