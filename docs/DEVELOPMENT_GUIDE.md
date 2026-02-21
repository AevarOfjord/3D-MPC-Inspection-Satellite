# Development Guide

This guide provides technical details for developers and engineers interested in understanding the codebase architecture and modification process.

**Note**: This project is a demonstration of advanced control systems engineering. While it is primarily a portfolio piece, forks and independent experimentation are encouraged.

---

## Table of Contents

- [Quick Start for Development](#quick-start-for-development)
- [Project Structure](#project-structure)
- [Code Style Guidelines](#code-style-guidelines)
- [Working with Data and CSV Logging](#working-with-data-and-csv-logging)
- [Testing Your Changes](#testing-your-changes)
- [Adding New Features](#adding-new-features)
- [Modifying Parameters](#modifying-parameters)
- [Debugging](#debugging)
- [Git Workflow](#git-workflow)
- [Contributing](#contributing)

---

## Quick Start for Development

### Setup

````bash
# Clone the repository (or your fork)
git clone https://github.com/AevarOfjord/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel.git
cd Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel

# Create virtual environment (Python 3.11 required)
python3.11 -m venv .venv311
source .venv311/bin/activate  # On Windows: .venv311\Scripts\activate

# Install all dependencies
pip install -r requirements.txt
```text
### Verify Everything Works

```bash
# Run the simulation with default settings
.venv311/bin/python scripts/run_simulation.py run

# Run tests
.venv311/bin/python -m pytest

# Check code quality
python -m ruff check src tests
python -m black --check src tests
```text
---

## Project Structure

For a complete project structure and design overview, see [ARCHITECTURE.md](ARCHITECTURE.md).

### Key Design Principles

1. **Centralized Configuration**: All parameters in `src/python/satellite_control/config/` package

    - Never hardcode parameters in other files
    - Import configuration using `SimulationConfig`

2. **Modular Architecture**: Each module has a single responsibility

   - `mpc_controller.py`: Optimization only
   - `cpp_satellite.py`: Physics simulation only
   - `simulation.py`: Control loop orchestration

3. **Type Hints**: All functions use comprehensive type hints

    - Enables static analysis and IDE support

4. **Docstrings**: Google-style docstrings for all classes and functions

   ```python
   def my_function(x: float, y: float) -> float:
       """Brief description.

       Longer description if needed.

       Args:
           x: Description of x
           y: Description of y

       Returns:
           Description of return value

       Raises:
           ValueError: If x is negative
       """
       return x + y
````

---

## Code Style Guidelines

### Automated Formatting

We use `black` for formatting and `ruff` for linting/import checks.

````bash
# Auto-format code
black src/ tests/

# Check style and imports
ruff check src tests
```text
### Style Rules

- **Line length**: 88 characters (configured in `pyproject.toml`)
- **Naming**:

  - Classes: `PascalCase` (e.g., `SatelliteConfig`, `MPCController`)
  - Functions: `snake_case` (e.g., `calculate_thrust`, `update_state`)
  - Constants: `UPPER_CASE` (e.g., `CONTROL_DT`, `MAX_FORCE`)
  - Private: Leading underscore (e.g., `_helper_function`, `_internal_state`)

- **Imports**: Group in standard order (checked by `ruff`):

  1. Standard library (`import os`, `import sys`)
  2. Third-party (`import numpy`, `import osqp`)
  3. Local (`from satellite_control.config.simulation_config import SimulationConfig`)

- **Functions**: Keep < 50 lines when possible

  - Break into smaller functions
  - Each function does one thing

- **Comments**: Explain _why_, not _what_

  ```python
  # Good
  # MPC requires linearization around current state for convex approximation
  A_lin = self._linearize_dynamics(x_current)

  # Avoid
  # Linearize dynamics
  A_lin = self._linearize_dynamics(x_current)
````

---

## Working with Data and CSV Logging

### CSV Data Format

The simulation produces several CSV files:

**Physics Data**: `physics_data.csv`

- Raw simulation state at every physics timestep (high frequency, ~200 Hz)
- Columns include `Current_X`, `Current_Y`, `Current_Z`, `Current_VX`, `Current_VY`, `Current_VZ`,
  `Current_Roll`, `Current_Pitch`, `Current_Yaw`, and tracking error fields

**Control Data**: `control_data.csv`

- Controller decisions and commands at control frequency (lower frequency, ~20-50 Hz)
- Columns include `Control_Time`, `Current_*`, `Target_*`, `Command_Vector`, `MPC_Solve_Time`, and solver status fields

For complete column definitions, see [VISUALIZATION.md](VISUALIZATION.md).

### Using the DataLogger

The `DataLogger` class in `utils/data_logger.py` handles buffered CSV writing:

````python
from satellite_control.utils.data_logger import DataLogger

# Create logger
logger = DataLogger(
    output_dir="Data/Simulation/test_run",
    buffer_size=1000  # Flush every 1000 entries
)

# Log physics data
logger.log_physics_state({
    "Time": current_time,
    "Current_X": x_position,
    "Current_Y": y_position,
    "Current_Z": z_position,
    "Current_Yaw": yaw_angle,
    # ... other columns
})

# Log control data
logger.log_control_step({
    "Control_Step": step_count,
    "MPC_Solve_Time": solve_time,
    "Command_Vector": command_vector,
    # ... other columns
})

# Flush and save when done
logger.close()
```text
**Important**: The logger uses buffered writing for performance. Data is periodically flushed to disk, not immediately.

### Modifying CSV Format

To change the CSV format:

1. **Update DataLogger**: Edit `_get_physics_headers()` or `_get_control_headers()` in `utils/data_logger.py`
2. **Update visualization**: Modify `visualization/unified_visualizer.py` to read new columns
3. **Update documentation**: Reflect changes in [VISUALIZATION.md](VISUALIZATION.md) CSV reference
4. **Update tests**: Verify new format in `tests/test_data_logger.py`

---

## Testing Your Changes

### Running Tests

```bash
# Run all tests
.venv311/bin/python -m pytest

# Run specific test file
.venv311/bin/python -m pytest tests/test_mpc_controller.py

# Run specific test function
.venv311/bin/python -m pytest tests/test_mpc_controller.py::test_mpc_solve_time

# Run with verbose output
.venv311/bin/python -m pytest -v

# Run with coverage report
.venv311/bin/python -m pytest --cov=src/python/satellite_control --cov-report=html
# Open htmlcov/index.html to view coverage

# Run E2E tests only
.venv311/bin/python -m pytest tests/e2e/ -v

# Run fast tests (skip slow E2E)
.venv311/bin/python -m pytest -m "not slow"
```text
### Recommended Validation Commands

```bash
# Focused fast suite
.venv311/bin/python -m pytest -m "not slow"

# Full suite
.venv311/bin/python -m pytest
```text
### Writing Tests

Create tests following existing patterns:

```python
# tests/test_myfeature.py
import pytest
import numpy as np
from satellite_control.config.simulation_config import SimulationConfig
from satellite_control.mymodule import my_function


class TestMyFeature:
    """Test suite for my new feature."""

    def test_basic_functionality(self):
        """Test basic case."""
        result = my_function(1.0, 2.0)
        assert result == pytest.approx(3.0)

    def test_edge_case(self):
        """Test edge case."""
        with pytest.raises(ValueError):
            my_function(-1.0, 2.0)

    @pytest.fixture
    def sample_config(self):
        """Provide sample configuration for tests."""
        return SimulationConfig.create_default().app_config

    def test_with_fixture(self, sample_config):
        """Test using fixture."""
        assert sample_config.physics.mass > 0
```text
### Testing Workflow

```bash
# 1. Make changes to code
# (Edit files in src/python/satellite_control/)

# 2. Format and check
black src/ tests/
ruff check src tests

# 3. Run tests to catch regressions
pytest

# 4. Test in simulation
.venv311/bin/python scripts/run_simulation.py run
# Select mission and verify behavior
```text
---

## Adding New Features

### Example: Adding a New Mission Type

**Goal**: Add "Figure-8 Navigation" mission

#### Step 1: Define Mission Logic

Create a helper in `src/python/satellite_control/mission/` (e.g., `figure8.py`):

```python
def configure_figure8_mission(
    center: Tuple[float, float],
    size: float,
    speed: float
) -> Dict[str, Any]:
    """Configure figure-8 navigation mission.

    Args:
        center: Center point of figure-8
        size: Size of each lobe (meters)
        speed: Target velocity along path (m/s)

    Returns:
        Mission configuration dictionary
    """
    # Generate figure-8 path points
    t = np.linspace(0, 2*np.pi, 100)
    x = center[0] + size * np.sin(t)
    y = center[1] + size * np.sin(t) * np.cos(t)

    mission = {
        'type': 'figure8',
        'path_points': list(zip(x, y)),
        'target_speed': speed,
        'center': center,
    }
    return mission
```text
#### Step 2: Attach Path to MissionState

Attach the generated path to the mission state so MPCC can follow it:

```python
from satellite_control.config.simulation_config import SimulationConfig

simulation_config = SimulationConfig.create_default()
mission_state = simulation_config.mission_state

path = [(float(px), float(py), 0.0) for px, py in zip(x, y)]
mission_state.path_waypoints = path
mission_state.path_speed = speed
```text
#### Step 3: Wire into Mission Loading

Integrate the mission into the saved-mission flow used by terminal simulation:

```python
from satellite_control.mission.repository import list_mission_entries

entries = list_mission_entries(source_priority=("unified", "dev"))
# Add your mission JSON and select it when running `.venv311/bin/python scripts/run_simulation.py run`
```text
#### Step 4: Add Configuration

If needed, add parameters to `src/python/satellite_control/config/mission_state.py`:

```python
# Figure-8 mission defaults
FIGURE8_DEFAULT_SIZE = 2.0  # meters
FIGURE8_DEFAULT_SPEED = 0.5  # m/s
```text
#### Step 5: Add Visualization Support

Use the existing auto-generated plots/video pipeline (no mission-specific overlay required).

#### Step 6: Write Tests

Add a test for the new path in `tests/`:

```python
def test_figure8_mission_generation():
    """Test figure-8 path generation."""
    mission = configure_figure8_mission(
        center=(0, 0),
        size=2.0,
        speed=0.5
    )

    assert mission['type'] == 'figure8'
    assert len(mission['path_points']) > 0
    assert mission['target_speed'] == 0.5
```text
#### Step 7: Document

Update [README.md](../README.md) with new mission type description.

#### Step 8: Test End-to-End

```bash
# Run tests
.venv311/bin/python -m pytest

# Test in simulation
.venv311/bin/python scripts/run_simulation.py run
# Select: Figure-8 Navigation
# Verify trajectory follows expected path
```text
---

## Modifying Parameters

### Where Parameters Live

All parameters are in `src/python/satellite_control/config/`:

```text
config/
├── models.py             # Pydantic validation models
├── simulation_config.py  # Top-level configuration wrapper
├── mission_state.py      # Mission parameters
├── defaults.py           # Default factories
├── physics.py            # Mass, inertia, thruster specs
├── timing.py             # Control rate, timesteps
├── constants.py          # System constants
├── presets.py            # FAST/BALANCED/STABLE/PRECISION presets
├── thruster_config.py    # Thruster configuration
└── reaction_wheel_config.py # Reaction wheel configuration
```text
### Guidelines

1. **Never hardcode values** in implementation files

   ```python
   # Good
    from satellite_control.config import SimulationConfig
    config = SimulationConfig.create_default()
    mass = config.app_config.physics.total_mass

   # Bad
   mass = 12.5  # Hardcoded!
````

2. **Group related parameters**

   ```python
    # Good - related parameters together in models.py (AppConfig.mpc)
    prediction_horizon = 15
    control_horizon = 10
    solver_time_limit = 0.04  # seconds
   ```

3. **Use descriptive names with units**

   ```python
   # Good
   STABILIZATION_VELOCITY_THRESHOLD = 0.03  # m/s
   VALVE_DELAY_TIME = 0.015  # seconds

   # Unclear
   V_THRESH = 0.03
   DELAY = 0.015
   ```

4. **Use Pydantic models for validation**

   ```python
   # In config/models.py
   from pydantic import BaseModel, Field

   class PhysicsConfig(BaseModel):
       mass: float = Field(gt=0, description="Satellite mass in kg")
       inertia: float = Field(gt=0, description="Moment of inertia in kg⋅m²")

       @field_validator('mass')
       def validate_mass(cls, v):
           if v > 100:
               warnings.warn("Mass seems unusually high")
           return v
   ```

### Adding New Parameters

1. Add to appropriate config file:

   ```python
    # In config/models.py (PhysicsParams)
    linear_damping_coeff = 0.05  # N⋅s/m
   ```

2. If using Pydantic models, add to the model:

   ```python
   # In config/models.py
   class PhysicsConfig(BaseModel):
       # ...existing fields...
       damping_coefficient: float = 0.05
   ```

3. Access in your code:

   ```python
    from satellite_control.config.simulation_config import SimulationConfig

    config = SimulationConfig.create_default()
    damping = config.app_config.physics.damping_coefficient
   ```

### Configuration Validation

````bash
Configuration is validated at simulation startup. Use a short run to sanity check:

```bash
.venv311/bin/python scripts/run_simulation.py run --auto --no-anim --duration 2
```text
```text
---

## Debugging

### Using Logging

The project uses Python's `logging` module configured in `utils/logging_config.py`:

```python
import logging
from satellite_control.utils.logging_config import setup_logging

# Setup logging (done automatically at startup)
setup_logging()

# Get logger for your module
logger = logging.getLogger(__name__)

# Log at different levels
logger.debug(f"Current state: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")
logger.info("Starting MPC optimization")
logger.warning(f"Solve time {solve_time:.3f}s exceeds limit {limit:.3f}s")
logger.error(f"MPC solve failed: {error_msg}")
```text
### Interactive Debugging

```python
# Add breakpoint in code
import pdb; pdb.set_trace()

# Or use built-in breakpoint() (Python 3.7+)
breakpoint()

# Debugger commands:
# n: next line
# s: step into function
# c: continue execution
# p variable: print variable value
# l: list surrounding code
# h: help
# q: quit debugger
```text
### Testing with Debug Output

```bash
# Run tests with print output visible
.venv311/bin/python -m pytest tests/test_mpc_controller.py -v -s

# -v: verbose test names
# -s: show print statements and logs
```text
### Live Simulation Debugging

The rich terminal dashboard shows real-time telemetry. To add custom debug info:

```python
# In simulation_logger.py, add custom panel
from rich.panel import Panel

debug_text = f"Custom Debug Info:\n{my_variable}"
console.print(Panel(debug_text, title="Debug"))
```text
---

## Git Workflow

### Working with Your Fork

Since this is primarily a portfolio demonstration project, if you want to make modifications:

1. **Fork the repository** on GitHub
2. **Clone your fork** locally
3. **Make changes** on feature branches
4. **Commit and push** to your fork
5. **Report bugs** back as issues if you find problems

### Making Changes

```bash
# Create feature branch
git checkout -b feature/my-enhancement

# Make changes and test
# (edit files, run tests, etc.)

# Stage changes
git add src/python/satellite_control/myfile.py tests/test_myfile.py

# Commit with descriptive message
git commit -m "feat: add new feature X

Detailed explanation of changes:
- Added Y functionality
- Modified Z for better performance
- Tests included"

# Push to your fork
git push -u origin feature/my-enhancement
```text
### Commit Message Guidelines

Follow conventional commits format:

```text
<type>: <brief description>

<optional detailed explanation>

<optional footer>
```text
Types:

- `feat`: New feature
- `fix`: Bug fix
- `refactor`: Code restructuring without behavior change
- `perf`: Performance improvement
- `test`: Adding or updating tests
- `docs`: Documentation changes
- `chore`: Maintenance tasks
- `style`: Code style changes (formatting, etc.)

Examples:

```text
feat: add figure-8 mission type

Implements a new mission type that follows a figure-8 trajectory.
Includes interactive CLI integration and visualization support.

fix: correct thruster force calculation in realistic physics mode

The force was previously not accounting for valve delay properly.
This fix ensures thrust ramp-up is applied after valve opening.

refactor: extract thruster management into separate module

Moves thruster valve delay and PWM logic from simulation.py
into new thruster_manager.py for better separation of concerns.
```text
### Before Committing

```bash
# Format code
black src/ tests/

# Run linters
ruff check src/ tests/

# Run tests
.venv311/bin/python -m pytest

# If all pass, commit
git add .
git commit -m "your message"
```text
### Pre-commit Hooks (Optional)

Install pre-commit hooks to automate checks:

```bash
# Install pre-commit
pip install pre-commit

# Install the git hooks
pre-commit install

# Now checks run automatically before each commit
```text
The project includes a `.pre-commit-config.yaml` that runs:

- `black` (formatting)
- `ruff` (linting)

---

## Contributing

### For Your Own Fork

When working on your own fork, maintain these quality standards:

### Code Quality Checklist

Before committing:

- [ ] Code follows style checks (checked by `ruff`)
- [ ] All code formatted with `black`
- [ ] All functions have docstrings (Google style)
- [ ] Type hints on all function signatures
- [ ] Tests written for new functionality
- [ ] All tests pass (`.venv311/bin/python -m pytest`)
- [ ] No new warnings from `ruff`
- [ ] Documentation updated if needed
- [ ] Commit messages follow conventional commits

### Adding Tests

For every new feature or bug fix:

1. **Write test first** (TDD approach)
2. **Implement feature/fix** until test passes
3. **Add edge case tests** for robustness
4. **Verify coverage** with `.venv311/bin/python -m pytest --cov`

Example test structure:

```python
# tests/test_new_feature.py
import pytest
from satellite_control.mymodule import MyClass


class TestMyClass:
    """Test suite for MyClass."""

    @pytest.fixture
    def instance(self):
        """Provide MyClass instance for tests."""
        return MyClass(param=1.0)

    def test_basic_operation(self, instance):
        """Test basic functionality."""
        result = instance.process(2.0)
        assert result == pytest.approx(3.0)

    def test_edge_case_zero(self, instance):
        """Test with zero input."""
        result = instance.process(0.0)
        assert result == pytest.approx(1.0)

    def test_invalid_input(self, instance):
        """Test with invalid input."""
        with pytest.raises(ValueError, match="must be positive"):
            instance.process(-1.0)
```text
### Documentation & Style

When making significant changes:

- **README.md**: Update for user-facing changes
- **ARCHITECTURE.md**: Update for design/structure changes
- **Docstrings**: Keep inline with code changes
- **This guide**: Update for development process changes
- **VISUALIZATION.md**: Update CSV reference details if data format changes

### Reporting Bugs

If you discover bugs in the original project:

1. Create an issue on GitHub with:

   - Clear problem description
   - Steps to reproduce
   - Environment info (OS, Python version, compiler/toolchain)
   - Error messages or log snippets
   - Suggested fix if you have one

2. The maintainer will evaluate and fix verified bugs when available

3. You'll be credited in the issue and commit messages

---

## Common Development Tasks

### Running Simulation with Custom Parameters

```bash
# Run with specific duration
.venv311/bin/python scripts/run_simulation.py run --duration 60

# Run in auto mode (skip menu)
.venv311/bin/python scripts/run_simulation.py run --auto

# Run without animation (faster)
.venv311/bin/python scripts/run_simulation.py run --no-anim

```text
### Generating Visualizations Only

If you have existing simulation data:

```python
from satellite_control.visualization.unified_visualizer import UnifiedVisualizationGenerator

# Generate visualizations from existing data
viz = UnifiedVisualizationGenerator(
    data_directory="Data/Simulation",
    interactive=True  # Let user select which run
)
```text
### Comparing Different Configurations

```bash
# Run baseline
.venv311/bin/python scripts/run_simulation.py run
# Results saved to Data/Simulation/<timestamp-1>/

# Modify parameters in config/
# Edit src/python/satellite_control/config/models.py or constants in src/python/satellite_control/config/

# Run with new parameters
.venv311/bin/python scripts/run_simulation.py run
# Results saved to Data/Simulation/<timestamp-2>/

# Compare results by examining mission_summary.txt in each folder
```text
---

## Troubleshooting Development

### Import Errors

```python
# Problem: "ModuleNotFoundError: No module named 'satellite_control'"

# Solution 1: Always run from project root
cd /path/to/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel
.venv311/bin/python scripts/run_simulation.py run

# Solution 2: Install package in editable mode
pip install -e .

# Solution 3: Add to PYTHONPATH
export PYTHONPATH="${PYTHONPATH}:/path/to/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel"
```text
### Test Discovery Issues

```bash
# Ensure proper test structure
# tests/__init__.py should exist
touch tests/__init__.py

# Run with explicit test directory
.venv311/bin/python -m pytest

# Debug test discovery
.venv311/bin/python -m pytest --collect-only -v
```text
### Linting Issues

```bash
# Re-run ruff to see detailed output
ruff check src/ tests/
```text
### C++ Extension Build Issues

```bash
# Rebuild the native extension in editable mode
pip install -e .

# Verify installation
python -c "from satellite_control.cpp import _cpp_sim; print('ok')"
```text
### OSQP Solver Issues

```bash
# Reinstall OSQP if needed
pip uninstall osqp
pip install osqp

# Verify installation
python -c "import osqp; print(osqp.__version__)"
```text
---

## Resources

### Documentation & Style

- **OSQP**: <https://osqp.org/docs/>
- **NumPy**: <https://numpy.org/doc/>
- **Rich**: <https://rich.readthedocs.io/> (for terminal UI)
- **Questionary**: <https://questionary.readthedocs.io/> (for interactive menus)

### Python Best Practices

- **PEP 8**: <https://www.python.org/dev/peps/pep-0008/>
- **Google Python Style Guide**: <https://google.github.io/styleguide/pyguide.html>
- **Type Hints**: <https://docs.python.org/3/library/typing.html>
- **Pytest Documentation**: <https://docs.pytest.org/>

### Control Theory

- **MPC Introduction**: <https://www.mpc.berkeley.edu/mpc-course-material>
- **Linearization**: Understanding system linearization for MPC
- **Quadratic Programming**: OSQP is a QP solver

---

## Related Documentation

This guide complements other important documents:

- **[README.md](../README.md)** - Project overview and quick start
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System design and component relationships
- **[VISUALIZATION.md](VISUALIZATION.md)** - Output analysis and CSV format reference
- **[MATHEMATICS.md](MATHEMATICS.md)** - Mathematical formulation of the MPC problem
- **[TESTING.md](TESTING.md)** - Comprehensive testing and simulation validation guide
- **[SIMULATION.md](SIMULATION.md)** - Mission types and simulation loop architecture

---

## Getting Help

If you're stuck:

1. **Check existing documentation** listed above
2. **Review related code** for examples of similar functionality
3. **Run tests** to understand expected behavior
4. **Create an issue** on GitHub with:
   - What you're trying to do
   - What you've tried
   - Error messages or unexpected behavior
   - Your environment (OS, Python version, etc.)

Happy developing! 🚀
````
