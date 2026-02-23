"""
CasADi-based code generation for MPC V2.

This package provides symbolic dynamics, cost functions, and C code generation
for the SQP-based MPC controller (V2 backend).

Modules:
    satellite_dynamics: Full nonlinear 6DOF satellite dynamics in CasADi
    cost_functions:     MPCC cost components (contouring, lag, attitude, etc.)
    generate:           Orchestrates C code generation and caching
"""
