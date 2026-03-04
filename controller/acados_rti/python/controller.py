"""acados SQP_RTI controller profile — one Real-Time Iteration per control step."""

from controller.acados_shared.python.base import AcadosBaseController


class AcadosRtiController(AcadosBaseController):
    """
    acados SQP_RTI: one SQP step per control step.

    Equivalent iteration budget to the RTI-SQP profiles (hybrid/nonlinear/linear)
    but uses acados's exact nonlinear RK4 dynamics and HPIPM QP sub-solver instead
    of the custom C++ OSQP backend. Fastest of the two acados profiles (~1-5ms/step
    on warm-start).
    """

    controller_profile = "cpp_nonlinear_rti_hpipm"
    solver_type = "ACADOS-SQP_RTI"
    _acados_nlp_solver_type = "SQP_RTI"
