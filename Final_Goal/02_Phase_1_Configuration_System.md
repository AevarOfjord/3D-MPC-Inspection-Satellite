# Phase 1: Data-Driven Configuration (Hydra)

**Goal**: Replace all hardcoded Python constants with a composable, type-safe YAML configuration system using `Hydra` and `OmegaConf`.

## 1. New Directory Structure
We will create a structured configuration tree in `config/` that mirrors the system architecture.

```text
config/
├── main.yaml                   # Root config (defaults)
├── vehicle/                    # Vehicle definitions
│   ├── cube_sat_6u.yaml
│   └── inspection_drone_v1.yaml
├── mission/                    # Mission scenarios
│   ├── station_keeping.yaml
│   └── hull_inspection.yaml
├── control/                    # Controller tuning
│   ├── mpc/
│   │   ├── aggressive.yaml
│   │   └── smooth.yaml
│   └── pid/                    # (Future proofing)
└── env/                        # Environment/Sim settings
    ├── deep_space.yaml
    └── leo_perturbed.yaml
```

## 2. Configuration Schema (The "Contract")
We will define strict `dataclasses` (Structured Configs) to validate YAMLs at runtime.

**`src/satellite_control/config/schema.py`**:
```python
@dataclass
class ThrusterConfig:
    position: List[float]  # [x, y, z]
    direction: List[float] # [x, y, z] normal vector
    max_thrust: float
    min_impulse_bit: float
    group: str = "main"    # e.g., "main", "rcs"

@dataclass
class VehicleConfig:
    mass: float
    inertia: List[float]   # [Ixx, Iyy, Izz] (diagonal assumption for simplicity initially)
    center_of_mass: List[float]
    thrusters: List[ThrusterConfig]  # DYNAMIC list size!
    reaction_wheels: List[RWConfig]
```

## 3. Code Refactoring
*   **Remove**: `mpc_params.py`, `satellite_config.py`, `constants.py` (eventually).
*   **Modify**: `SatelliteMPCLinearizedSimulation` constructor.
    *   *Old*: `def __init__(self)` (imports constants implicitly).
    *   *New*: `def __init__(self, cfg: DictConfig)`.
*   **Dependency Injection**: The `MPCController` will no longer import `Q_POSITION`. It will receive `cfg.control.mpc.weights.position` in its `__init__`.
