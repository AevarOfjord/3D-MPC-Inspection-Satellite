# Tests

Compact, high-value test suite for the Satellite control system.
Focused on behavioral coverage (unit + integration + API + E2E smoke + benchmarks).

## Running Tests

```bash
# Run all tests (fast)
.venv311/bin/python -m pytest

# Run slow integration/E2E tests
.venv311/bin/python -m pytest -m slow

# Run opt-in E2E simulation test (slow + environment-gated)
RUN_E2E_SIM_TESTS=1 .venv311/bin/python -m pytest -m slow tests/test_e2e_simulation.py

# Run benchmarks
.venv311/bin/python -m pytest tests/test_benchmark.py --benchmark-only
```

## Test Files

| File | Scope |
|------|-------|
| `test_config.py` | Configuration validation, presets, parameter constraints. |
| `test_math.py` | Pure math: orbital dynamics, navigation, angle normalization. |
| `test_mpc.py` | MPC Controller logic: initialization, control action generation. |
| `test_thruster_logic.py` | Thruster management: PWM duty cycles, continuous mode, valve delays. |
| `test_state_validation.py` | State vector checks: NaN/Inf detection, bounds, trajectory continuity. |
| `test_path_planning.py` | Obstacle avoidance, waypoint generation, safe path calculation. |
| `test_mission_workflow.py` | MissionState serialization, mission context management. |
| `test_dashboard_api.py` | FastAPI endpoints: mission listing, simulation status, control. |
| `test_property_based.py` | Hypothesis fuzzing for math invariants and edge cases. |
| `test_cpp_integration.py` | C++ simulation engine bindings and physics stepping. |
| `test_e2e_simulation.py` | Full headless simulation run (smoke test, opt-in via `RUN_E2E_SIM_TESTS=1`). |
| `test_benchmark.py` | Performance regression tests for the solver. |
