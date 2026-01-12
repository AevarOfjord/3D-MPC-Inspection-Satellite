# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2026-01-12

### Added
- **Phase 1: Configuration System**
  - Hydra-style YAML configuration hierarchy
  - Vehicle templates (CubeSat 6U, Inspection Drone)
  - MPC profiles (smooth, aggressive)
  - Mission templates (station keeping, hull inspection)

- **Phase 2: Universal Physics Engine**
  - Dynamic B-matrix generation from config
  - Per-thruster continuous thrust bounds
  - Removed PWM modulation layer

- **Phase 3: Fault Tolerance**
  - FaultManager with fault injection/clearing
  - Real-time B-matrix adaptation
  - Degraded mode operation

- **Phase 4: Autonomous Inspection**
  - STL/OBJ mesh loader
  - Zone clustering and viewpoint generation
  - TSP path optimization
  - InspectionPlanner (mesh → waypoints)

- **Phase 5: Flight Readiness**
  - C99 code generator for ARM/FPGA
  - HIL bridge (Serial/CAN/UDP)
  - Telemetry server (JSON pub/sub)

### Changed
- Migrated from Python to C++ for performance
- Switched to continuous variable thrust (no PWM)

### Removed
- PulseWidthModulator class
- Hardcoded thruster configurations
