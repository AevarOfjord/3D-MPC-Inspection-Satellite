# Project Status: C++ Control System

## Completed Milestones
1. **Scaffolding**: Created `Cplusplus_build` directory structure.
2. **Build System**: 
    - Configured `CMakeLists.txt` for C++20.
    - Integrated Dependencies via `FetchContent`:
        - `Eigen` (Math)
        - `yaml-cpp` (Configuration)
        - `spdlog` (Logging)
        - `MuJoCo 3.1.1` (Physics)
        - `OSQP` (Solver)
    - **Fixes Applied**:
        - Patched `osqp` and `yaml-cpp` legacy `CMakeLists.txt` to be compatible with CMake 3.30+ (Policy CMP0048).
        - Resolved `spdlog` target conflicts with MuJoCo's internal fetch.
3. **Configuration Module**:
    - Implemented `ConfigLoader` to parse `VehicleConfig` from YAML.
    - Verified with `vehicle_test.yaml`.
4. **Integration Testing**:
    - `sat_control` executable compiles and runs.
    - Successfully initializes all libraries.
5. **Physics Engine**:
    - Implemented `PhysicsEngine` wrapper for MuJoCo.
    - Verified force application (`qfrc_applied`) and state extraction.
    - Confirmed physics integration with 1g acceleration test.

## Next Steps
1. **Dynamics & Control**:
    - Implement `ControlAllocator` to generate B-Matrix from `VehicleConfig`.
    - Port MPC Logic (OSQP formulation) from Python to C++.
2. **Simulation Loop**:
    - Build the main simulation loop connecting Physics -> Sensor -> Control -> Actuator -> Physics.
