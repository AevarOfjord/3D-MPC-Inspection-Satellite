# Nonlinear C++

This directory holds the C++ backend used by the `cpp_nonlinear_rti_osqp` profile.

The controller keeps the OSQP-family QP runtime, but it is driven by exact stage-wise nonlinear linearizations generated from the shared CasADi model.

For the mathematical formulation and paper-facing explanation, use `MATH/cpp_nonlinear_rti_osqp.md`.
