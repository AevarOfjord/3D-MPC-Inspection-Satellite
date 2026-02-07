# Tests

This directory is for unit tests and integration tests for the Satellite Thruster Control System.

## Running Tests

Once tests are added, you can run them with:

```bash
.venv311/bin/python -m pytest tests/
```

## Test Structure

- `test_mpc_controller.py` - Tests for MPC controller
- `test_config.py` - Tests for configuration validation
- `test_model.py` - Tests for physical model
- `test_integration_basic.py` - Basic integration tests
- `test_integration_missions.py` - Mission integration tests
- Additional test files as needed

## Testing Tools

For manual testing and verification:
- Use `run_simulation.py` (or `make sim`) to run missions in the simulator
- Inspect generated outputs under `Data/Simulation/` for validation
