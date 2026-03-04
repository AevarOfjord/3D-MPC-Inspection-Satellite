"""acados full SQP controller profile — iterate to convergence each control step."""

from controller.acados_shared.python.base import AcadosBaseController


class AcadosSqpController(AcadosBaseController):
    """
    acados full SQP: iterate to convergence each control step.

    Equivalent in solution quality to NmpcController (IPOPT) but typically
    10-20x faster due to HPIPM's structured QP solver exploiting the banded
    MPC block structure (O(N) Riccati factorization vs. IPOPT's O(N³) sparse LU).
    """

    controller_profile = "cpp_nonlinear_sqp_hpipm"
    solver_type = "ACADOS-SQP"
    _acados_nlp_solver_type = "SQP"
    _acados_globalization = "MERIT_BACKTRACKING"  # line-search like IPOPT
