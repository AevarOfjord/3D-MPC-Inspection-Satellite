# Hybrid C++

This directory contains the C++ RTI-SQP + OSQP backend used by the `cpp_hybrid_rti_osqp` controller profile.

Primary files:

- `sqp_controller.cpp` and `sqp_controller.hpp` for QP assembly and solve orchestration
- `sqp_types.*` for sparse-structure and runtime data containers
- `bindings.cpp` for the Python extension bridge

For the mathematical formulation and paper-facing explanation, use `MATH/cpp_hybrid_rti_osqp.md`.
