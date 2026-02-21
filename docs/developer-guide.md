# Developer Guide

This guide is for engineers and developers looking to understand the architecture, run tests, or contribute to the Orbital Inspector Satellite Control repository.

---

## 1. Quick Start / Workspace Setup

The project uses Python 3.11, `scikit-build-core` for C++ compilation, and Node.js for the UI.

```bash
# 1. Create and activate a virtual environment
python3.11 -m venv .venv311
source .venv311/bin/activate

# 2. Install all dependencies (Backend + Frontend + C++ Engine)
make install

# 3. Verify the environment
make check     # Lints Python code via ruff and black
make test      # Runs the pytest suite
```

---

## 2. High-Level Architecture (V6)

The repository implements a strict separation of concerns across three language domains:

1. **Python Runtime (`src/python/`)**: Orchestrates the mission logic, configuration, file I/O, telemetry, and runs the FastAPI backend.
2. **C++ Engine (`src/cpp/`)**: High-performance execution of the receding horizon Model Predictive Control (MPC) logic and the simulated physics environment. Compiled via `pybind11` and `scikit-build-core`.
3. **TypeScript / React UI (`ui/`)**: The frontend SPA providing the WebGL 3D planner, simulation runner, and telemetry viewer.

### The Two-Rate Control Loop

The simulation uses a two-rate architecture to mimic real spacecraft embedded systems:

- **Physics Loop (`200 Hz`)**: High-fidelity dynamics integration (C++).
- **Control Loop (`16.67 Hz`)**: MPC optimization and thruster duty cycle allocation (Python/C++).

---

## 3. Testing & Validation

All tests are written in Pytest and located in the `tests/` directory. By default, the testing suite validates the Python wrapper, the C++ bindings, and the V6 control contracts.

```bash
# Run the full test suite
python -m pytest

# Run fast unit tests only
python -m pytest -m "not slow"

# Evaluate MPC Quality Contracts (Tracking, timing, chatter)
python scripts/run_mpc_quality_suite.py --fail-on-breach
```

### V6 Terminal Completion Contracts

Missions succeed when the `TerminalSupervisorV6` validates all the following continuously for 10 seconds:

- Position error `<= 0.10 m`
- Angle error `<= 2 deg`
- Linear velocity error `<= 0.05 m/s`
- Angular velocity error `<= 2 deg/s`

---

## 4. Code Structure & Guidelines

We rely heavily on static analysis and automated formatters to maintain code quality.

- **Formatting**: `black`
- **Linting**: `ruff`
- **Typing**: `mypy` (Strict type hints are required on all functions)

### Configuration Philosophy

Never hardcode magic numbers. All constants and default behaviors are injected via the configuration system located in `src/python/satellite_control/config/`.
If you need to change the damping, the cost weights (Q, R matrices), or the satellite mass, you should adjust the Pydantic models in `config/models.py`.

### Git Workflow and Contributions

We follow the Conventional Commits format for PRs:

- `feat:` for new features
- `fix:` for bug fixes
- `docs:` for documentation updates
- `refactor:` for code restructuring without behavior change

Before committing, always run `make check` and ensure `pytest` runs green.
