# Testing Guide

Comprehensive guide for testing the Satellite Control System, including unit tests, integration tests, and simulation validation.

---

## Table of Contents

- [Quick Start](#quick-start)
- [Running Simulations for Testing](#running-simulations-for-testing)
- [Test Suite (Pytest)](#test-suite-pytest)
- [Writing Tests](#writing-tests)
- [Simulation Testing & Validation](#simulation-testing--validation)
- [Performance Benchmarks](#performance-benchmarks)
- [Debugging](#debugging)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

### Install Testing Tools

```bash
pip install pytest pytest-cov
```

### Run All Tests

```bash
# Run full pytest test suite
.venv311/bin/python -m pytest

# Run with coverage
.venv311/bin/python -m pytest --cov=src/satellite_control --cov-report=html
```

### Run Your First Simulation Test

```bash
python run_simulation.py run
```

Select a saved unified mission from `missions_unified/` to run a basic test.

---

## Running Simulations for Testing

### Interactive Mode

```bash
python run_simulation.py run
```

**Use Cases:**

- Manual verification of control behavior
- Visual inspection of trajectories
- Parameter tuning and experimentation

### Automated Mode (for CI)

```bash
# Default auto mode
python run_simulation.py run --auto

# Custom duration
python run_simulation.py run --auto --duration 30.0

# Headless (no visualization)
python run_simulation.py run --no-anim --auto
```

### Mission Flow for Testing

Tests unified mission compile/hydration and path tracking runtime behavior.

**What to validate:**

- Mission save/load (`save_mission_v2`, `saved_missions_v2`)
- Terminal mission discovery (`make sim` / CLI selection list)
- Path tracking progress and phase transitions
- Generated logs/plots/animation outputs

---

## Test Suite (Pytest)

### Test Structure

```
tests/
├── __init__.py
├── conftest.py              # Shared fixtures
├── benchmarks/              # Benchmark tests
├── e2e/                     # End-to-end tests
├── integration/             # Integration tests
├── physics/                 # Physics unit tests
├── test_*.py                # Unit/component tests at repo root
```

### Running Tests

```bash
# Run all tests
.venv311/bin/python -m pytest

# Run specific category
.venv311/bin/python -m pytest tests/integration/
.venv311/bin/python -m pytest tests/e2e/

# Run specific file
.venv311/bin/python -m pytest tests/test_mpc_controller.py

# Run specific test
.venv311/bin/python -m pytest tests/test_mpc_controller.py::test_mpc_converges_to_target

# Run with verbose output
.venv311/bin/python -m pytest -v

# Run with print statements visible
.venv311/bin/python -m pytest -s

# Stop at first failure
.venv311/bin/python -m pytest -x

# Run tests matching pattern
.venv311/bin/python -m pytest -k "mpc"
```

### Test Markers

```bash
# Run only fast tests (skip slow E2E)
.venv311/bin/python -m pytest -m "not slow"

# Run tests by custom marker
.venv311/bin/python -m pytest -m "integration"
```

### Understanding Test Results

**Symbols:**

- ✓ `PASSED`: Test succeeded
- ✗ `FAILED`: Test failed
- ⊘ `SKIPPED`: Test was skipped
- ⚠ `XFAIL`: Expected failure
- `ERROR`: Setup/teardown failed

**Example output:**

```
tests/test_config.py::test_mass_is_positive PASSED       [10%]
tests/test_mpc.py::test_solve_time PASSED                [20%]
...
====== 47 passed in 1.23s ======
```

---

## Writing Tests

### Basic Test Structure

```python
# tests/test_example.py
import pytest

class TestPhysics:
    """Test suite for physics calculations."""

    def test_acceleration_is_positive_for_positive_force(self):
        """Test that positive force produces positive acceleration."""
        mass = 10.0
        force = 5.0
        acceleration = force / mass

        assert acceleration > 0
        assert acceleration == pytest.approx(0.5)

    def test_acceleration_raises_for_zero_mass(self):
        """Test that zero mass raises error."""
        with pytest.raises(ZeroDivisionError):
            acceleration = 5.0 / 0.0
```

### Naming Conventions

```python
# Good test names describe what is tested
def test_mass_is_positive():
    """Test that satellite mass is positive."""
    pass

def test_thruster_force_increases_with_pressure():
    """Test that higher pressure produces more force."""
    pass

# Avoid unclear names
def test_physics():  # Too vague
    pass

def test_1():  # No meaning
    pass
```

### Common Assertions

```python
# Equality
assert result == 5

# Approximate equality (for floats)
assert result == pytest.approx(3.14159, abs=1e-5)

# Comparisons
assert value > 0
assert value >= threshold

# Membership
assert element in collection

# Type checking
assert isinstance(obj, MyClass)

# Exception handling
with pytest.raises(ValueError):
    function_that_should_raise()

with pytest.raises(ValueError, match="error message"):
    function_that_raises_with_message()
```

### Using Fixtures

```python
# conftest.py
import pytest
from src.satellite_control.config import SatelliteConfig

@pytest.fixture
def satellite_params():
    """Provide standard satellite parameters."""
    return SatelliteConfig.get_app_config().physics

@pytest.fixture
def mpc_config():
    """Provide standard MPC configuration."""
    return SatelliteConfig.get_mpc_params()

# Use in test
def test_with_fixture(satellite_params):
    assert satellite_params['mass'] > 0
```

### Parametrized Tests

```python
@pytest.mark.parametrize("mass,expected_inertia", [
    (10.0, 0.45),
    (12.5, 0.56),
    (15.0, 0.67),
])
def test_inertia_scales_with_mass(mass, expected_inertia):
    """Test that inertia scales correctly with mass."""
    calculated = mass * 0.045
    assert calculated == pytest.approx(expected_inertia, rel=0.01)
```

---

## Simulation Testing & Validation

### Configuration & Tuning

All parameters are in `src/satellite_control/config/`:

**Key MPC Parameters** (`mpc_params.py`):

```python
MPC_PREDICTION_HORIZON = 50      # 3.0s lookahead
MPC_CONTROL_HORIZON = 50
SOLVER_TIME_LIMIT = 0.05         # 50ms max

# Cost weights
Q_POSITION = 1000.0              # Position tracking
Q_VELOCITY = 10000.0             # Velocity damping
Q_ANGLE = 1000.0                 # Orientation tracking
Q_ANGULAR_VELOCITY = 1500.0      # Angular damping
R_THRUST = 1.0                   # Control effort penalty
```

**Tuning Guidelines:**

For **faster response**:

```python
Q_POSITION = 1500.0    # Increase from 1000
R_THRUST = 0.5         # Decrease from 1.0
```

For **smoother motion**:

```python
Q_VELOCITY = 15000.0             # Increase from 10000
Q_ANGULAR_VELOCITY = 2000.0      # Increase from 1500
```

### Output & Analysis

Every simulation creates:

```
Data/Simulation/DD-MM-YYYY_HH-MM-SS/
├── physics_data.csv           # State history (200 Hz)
├── control_data.csv           # MPC commands (16.67 Hz)
└── simulation_animation.mp4   # Visual playback
```

**Analyze Results:**

```python
import pandas as pd
import numpy as np

physics = pd.read_csv('Data/Simulation/<timestamp>/physics_data.csv')
control = pd.read_csv('Data/Simulation/<timestamp>/control_data.csv')

# Position error over time
pos_error = np.sqrt(physics['Current_X']**2 + physics['Current_Y']**2)
print(f"Mean error: {pos_error.mean():.4f} m")
print(f"Final error: {pos_error.iloc[-1]:.4f} m")

# MPC performance
print(f"Avg solve time: {control['mpc_solve_time'].mean()*1000:.2f} ms")
print(f"Solver failures: {(control['mpc_status'] != 'OPTIMAL').sum()}")
```

### Batch Parameter Sweep

```python
import subprocess

q_positions = [500, 1000, 2000]
q_velocities = [5000, 10000, 15000]

for q_pos in q_positions:
    for q_vel in q_velocities:
        # Update config (or use environment variables)
        # Run simulation
        subprocess.run([
            'python', 'run_simulation.py', 'run',
            '--auto', '--no-anim', '--duration', '20'
        ])
        # Collect and analyze results
```

---

## Performance Benchmarks

### Expected Performance

On modern hardware (2020+ laptop):

| Metric             | Target | Acceptable | Poor  |
| ------------------ | ------ | ---------- | ----- |
| MPC solve time     | <2ms   | <5ms       | >10ms |
| Position error     | <0.02m | <0.05m     | >0.1m |
| Angle error        | <2°    | <5°        | >10°  |
| Settling time (1m) | <15s   | <30s       | >45s  |

---

## Debugging

### Using Logging

```python
import logging

logger = logging.getLogger(__name__)

logger.debug(f"Current state: x={x:.3f}, y={y:.3f}")
logger.info("Starting MPC optimization")
logger.warning(f"Solve time {solve_time:.3f}s exceeds limit")
logger.error(f"MPC solve failed: {error_msg}")
```

### Interactive Debugging with pdb

```python
import pdb; pdb.set_trace()  # Debugger pauses here

# Or use built-in breakpoint()
breakpoint()
```

### Testing with Debug Output

```bash
# Run tests with print output visible
.venv311/bin/python -m pytest tests/test_mpc_controller.py -v -s

# -v: verbose test names
# -s: show print statements
```

### Live Simulation Debugging

The rich terminal dashboard shows real-time telemetry. Check for:

- Position and velocity errors
- Active thrusters
- MPC solve times
- Mission phase transitions

---

## Test Coverage

### Generate Coverage Report

```bash
pytest --cov=src/satellite_control --cov-report=html
open htmlcov/index.html
```

### Understanding Coverage

```
Name                  Stmts   Miss  Cover   Missing
--------------------------------------------------
config/__init__.py       45      2    96%   12-13
model.py               120     10    92%   45-48, 67
mpc.py                 200      5    98%   150-151
--------------------------------------------------
TOTAL                  615     37    94%
```

### Coverage Goals

- **Overall:** Aim for >80% coverage
- **Critical:** MPC, physics model >95%
- **Utilities:** Test helpers >70%

### Coverage Commands

```bash
# Minimum coverage threshold
pytest --cov=. --cov-fail-under=80

# Coverage per module
pytest --cov=. --cov-report=term-missing

# XML report (for CI/CD)
pytest --cov=. --cov-report=xml
```

---

## Troubleshooting

### Common Issues

#### MPC Solver Too Slow

**Symptoms:** Solve times >10ms consistently

**Solutions:**

```python
# 1. Reduce horizon
MPC_PREDICTION_HORIZON = 30  # Down from 50

# 2. Relax solver tolerances
eps_abs = 1e-3  # Looser (was 1e-4)

# 3. Increase time limit
SOLVER_TIME_LIMIT = 0.08  # Up from 0.05s
```

####Simulation Unstable

**Symptoms:** Satellite spinning, oscillating, or diverging

**Solutions:**

```python
# Increase damping
Q_VELOCITY = 15000.0          # Increase from 10000
Q_ANGULAR_VELOCITY = 2000.0   # Increase from 1500
```

#### Tests Fail to Run

```bash
# Ensure you're in project root
cd /Users/aevar/Desktop/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel
pwd  # Should show project root

# Install pytest
pip install pytest pytest-cov

# Run from project root
.venv311/bin/python -m pytest
```

#### Tests Timeout

```bash
# Set timeout for slow tests
.venv311/bin/python -m pytest --timeout=10

# Run only fast tests
.venv311/bin/python -m pytest -m "not slow"

# Run integration tests only
.venv311/bin/python -m pytest tests/integration/
```

### Quick Reference

```bash
# Interactive test
python run_simulation.py run

# Fast automated test
python run_simulation.py run --auto --no-anim --duration 10

# Run pytest suite
.venv311/bin/python -m pytest -v

# Smoke-check CLI wiring with a saved mission
python run_simulation.py run --mission missions_unified/<YourMission>.json --no-anim

# Generate plots from existing data
python -m src.satellite_control.visualization.unified_visualizer
```

---

## Next Steps

After testing:

1. **Validate Results** - Review CSV data and animations
2. **Tune Parameters** - Iterate on MPC weights if needed
3. **Run Full Test Suite** - Ensure no regressions
4. **Document Changes** - Update parameter files with comments
5. **Performance Baseline** - Record metrics for comparison

See [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) for contributing guidelines and [README.md](../README.md) for getting started.
