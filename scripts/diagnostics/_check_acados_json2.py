"""Check acados JSON serialization utilities available."""

import sys

sys.path.insert(0, ".")
import inspect

try:
    import acados_template
    from acados_template import AcadosOcp

    # Check what serialization is available
    print("Module:", acados_template.__file__)
    try:
        from acados_template.acados_ocp_solver import ocp_check_against_json

        print("ocp_check_against_json: available", bool(ocp_check_against_json))
    except ImportError:
        pass
    # Look at serialize method
    ocp = AcadosOcp()
    methods = [
        m
        for m in dir(ocp)
        if "json" in m.lower() or "serial" in m.lower() or "dump" in m.lower()
    ]
    print("OCP json methods:", methods)
    # Look at AcadosOcpSolver.generate source
    from acados_template import AcadosOcpSolver

    src = inspect.getsource(AcadosOcpSolver.generate)
    # Find the json writing part
    for line in src.split("\n")[:40]:
        if any(kw in line for kw in ["json", "dump", "serialize", "write", "open"]):
            print(f"  {line.rstrip()}")
except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()
