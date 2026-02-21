# Orbital Inspector Satellite Control

Welcome to the documentation for the **Orbital Inspector Satellite Control** engine!

This repository contains a high-fidelity space simulation environment and a mathematically proven Model Predictive Control (MPC) engine designed for path-following and 6-DOF inspection maneuvers.

## What is it?

This system allows you to:

1. **Plan** 3D trajectory missions with an intuitive WebGL React User Interface.
2. **Execute** those missions via a headless C++ physics engine that simulates realistic thruster delays and reaction wheel momentum.
3. **Analyze** the 16Hz MPC telemetry to ensure stability, fuel efficiency, and precise state tracking.

## Navigation

- [User Guide](user-guide.md): Learn how to build missions in the UI and run basic terminal simulations.
- [Core Physics](physics.md): Dive into the equations of motion and MPC optimization constraints.
- [Developer Guide](developer-guide.md): Get your local environment set up to compile the C++ `pybind11` extension and contribute.
- [API Reference](api.md): Programmatic interface reference for the `satellite_control` Python package.
