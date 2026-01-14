# Changelog

## Version 4.0.0 - January 13, 2026

### Features
- **Mission Control UI:** Modern React/Three.js web interface replacing legacy viewers.
- **Interactive Editor:** Design missions (waypoints, obstacles) directly in the browser.
- **Real-time Telemetry:** Heads-Up Display (HUD) for position, velocity, thrusters, and RW torque.
- **REST API:** Control the simulation backend via HTTP endpoints (`POST /mission`).

### Technical Details
- **Frontend:** Built with Vite, React, React Three Fiber, and Tailwind CSS.
- **Backend:** FastAPI server (`app.py`) bridging C++ physics and WebSockets.
- **Architecture:** Decoupled UI from simulation loop, enabling remote mission operations.

---

## Version 3.0.0 - January 13, 2026

### Features
- **Collision Avoidance:** Implemented dynamic obstacle avoidance using hard QP constraints.
- **Obstacle Types:** Support for Sphere, Cylinder, and Box obstacles.
- **3D Visualization:** Real-time rendering of obstacles in the simulation view.
- **Safety:** Hard constraints guarantee minimum separation distances are respected.

### Technical Details
- **C++ Backend:**
  - Added `Obstacle` and `ObstacleSet` classes for efficient geometry handling.
  - Modified `MPCControllerCpp` to dynamically insert linear constraints into the OSQP formulation.
  - Implemented closest-point calculations for supported geometries.
- **Python Integration:**
  - Updated `_cpp_mpc` bindings to expose obstacle management.
  - Added `Obstacle` dataclasses to `mission_types.py`.
  - Simplifed `MPCRunner` to pass obstacle data directly to the C++ controller.
- **Configuration:**
  - Added `enable_collision_avoidance` and `obstacle_margin` to `MPCParams`.

### Compatibility
- Fully backward compatible with V2.x missions (obstacles are optional).

---

# Changelog - January 3, 2026

## Summary

Comprehensive codebase improvements to bring the Satellite_3D_MuJoCo project to production-ready form.

---

## Test Results

- **232 tests passed** (6 skipped)
- **11 property tests passed** (Hypothesis)

---

## Phase 1: Failing Test Fixes

### New Files

- `src/satellite_control/core/model.py` - Backward compatibility module

### Modified Files

- `src/satellite_control/core/__init__.py` - Added `model` to exports

**Issue:** 7 tests failing due to `ImportError: src.satellite_control.core.model`  
**Fix:** Created compatibility module re-exporting from `config.physics` and `config.timing`

---

## Phase 2: File Refactoring

### New Files

- `src/satellite_control/core/simulation_io.py` (141 lines)

  - `SimulationIO` class with extracted methods:
    - `create_data_directories()`
    - `save_csv_data()`
    - `save_mission_summary()`
    - `save_animation_mp4()`

- `src/satellite_control/mission/mission_handler_base.py` (98 lines)
  - `MissionHandler` abstract base class

### Modified Files

- `src/satellite_control/core/simulation.py`
  - Reduced from 1302 → 1229 lines (-73)
  - IO methods now delegate to `SimulationIO`

---

## Phase 3: MPC Profiling

### New Files

- `src/satellite_control/utils/profiler.py` (140 lines)
  - `PerformanceProfiler` class
  - `@profiler.profile` decorator
  - `profiler.measure()` context manager
  - Global `mpc_profiler` instance

### Modified Files

- `src/satellite_control/control/pwm_mpc.py`

  - Added profiler import
  - Wrapped `model.optimize()` with `mpc_profiler.measure()`

- `src/satellite_control/control/binary_mpc.py`
  - Added profiler import
  - Wrapped `model.optimize()` with `mpc_profiler.measure()`

---

## Phase 4: Logging Standardization

### Modified Files

- `src/satellite_control/config/physics.py`

  - Added `logging` import
  - Replaced all `print()` with `logger.info/warning/error()`

- `src/satellite_control/config/timing.py`

  - Added `logging` import
  - Replaced all `print()` with `logger.info/warning/error()`

- `src/satellite_control/mission/mission_state_manager.py`

  - Added `logging` import
  - Replaced 20+ `print()` with `logger.info()`

- `src/satellite_control/mission/mission_logic.py`
  - Added `logging` import
  - Replaced warning prints with `logger.warning()`

---

## Phase 5: Property-Based Testing

### Dependencies Added

- `hypothesis>=6.0` in `pyproject.toml`

### New Files

- `tests/test_property_based.py` (200+ lines)
  - `TestStateConverterProperties`
  - `TestMPCConstraintProperties`
  - `TestThrusterProperties`
  - `TestDynamicsProperties`
  - `TestObstacleAvoidanceProperties`
  - `TestStateValidationEdgeCases`

---

## Phase 6: Documentation

### Generated

- `docs/_build/html/` - Sphinx API documentation

### Dependencies Installed

- `sphinx`
- `sphinx-rtd-theme`
- `myst-parser`
- `vulture` (dead code scanner)

---

## Phase 7: Dead Code Removal

### Modified Files

- `src/satellite_control/mission/mission_state_manager.py`
  - Removed unreachable `return None` at line 769

---

## Files Changed Summary

| File                               | Change Type | Lines Changed |
| ---------------------------------- | ----------- | ------------- |
| `core/model.py`                    | NEW         | +34           |
| `core/__init__.py`                 | MODIFIED    | +2            |
| `core/simulation.py`               | MODIFIED    | -73           |
| `core/simulation_io.py`            | NEW         | +141          |
| `mission/mission_handler_base.py`  | NEW         | +98           |
| `mission/mission_state_manager.py` | MODIFIED    | ~25 (logging) |
| `mission/mission_logic.py`         | MODIFIED    | +5            |
| `control/pwm_mpc.py`               | MODIFIED    | +3            |
| `control/binary_mpc.py`            | MODIFIED    | +3            |
| `config/physics.py`                | MODIFIED    | ~10           |
| `config/timing.py`                 | MODIFIED    | ~10           |
| `utils/profiler.py`                | NEW         | +140          |
| `tests/test_property_based.py`     | NEW         | +200          |
| `pyproject.toml`                   | MODIFIED    | +1            |
