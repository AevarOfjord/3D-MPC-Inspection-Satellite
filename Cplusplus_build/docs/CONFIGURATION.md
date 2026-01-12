# Configuration Guide

Complete YAML schema reference for the satellite control system.

## Config Directory Structure

```
config/
├── main.yaml                    # Root config with defaults
├── vehicle/
│   ├── cube_sat_6u.yaml        # 6-thruster CubeSat
│   └── inspection_drone_v1.yaml # 12-thruster drone
├── control/mpc/
│   ├── smooth.yaml             # Fuel-efficient tuning
│   └── aggressive.yaml         # Fast maneuvering
├── mission/
│   ├── station_keeping.yaml    # Hold position
│   └── hull_inspection.yaml    # Circumnavigation
└── env/
    ├── deep_space.yaml         # No perturbations
    └── leo_perturbed.yaml      # Full orbital dynamics
```

## Vehicle Configuration

```yaml
vehicle:
  name: "CubeSat_6U"
  mass: 12.0                        # [kg]
  inertia_diag: [1.2, 1.2, 0.8]    # [Ixx, Iyy, Izz] kg*m²
  center_of_mass: [0.0, 0.0, 0.0]  # [m]

  reaction_wheels:
    max_torque: 0.1                 # [Nm] per axis
    max_speed_rad_s: 600.0          # [rad/s]
    inertia: 0.001                  # [kg*m²]
    enabled: true

  thrusters:
    - id: "T1_PX"
      position: [0.15, 0.0, 0.0]    # [m] from CoM
      direction: [-1.0, 0.0, 0.0]   # Unit vector
      max_thrust: 1.0               # [N]
      group: "main"                 # Optional grouping
```

## MPC Configuration

```yaml
control:
  mpc:
    horizon: 20                     # Prediction steps
    dt: 0.05                        # Control timestep [s]

    weights:
      position: 100.0               # Position tracking
      velocity: 1.0
      attitude: 10.0
      angular_velocity: 1.0
      thrust: 0.1                   # Fuel penalty
      reaction_wheel: 1.0

    max_iterations: 1000
    tolerance: 1e-4
    verbose: false
```

## Mission Configuration

```yaml
mission:
  name: "Hull Inspection"
  max_duration: 600.0               # [s]
  loop: false                       # Repeat waypoints?

  waypoints:
    - position: [5.0, 0.0, 0.0]     # [m]
      attitude: [0.707, 0, 0.707, 0] # [w, x, y, z]
      hold_time: 5.0                # [s]
      pos_tolerance: 0.1            # [m]
      att_tolerance: 0.1            # [rad]
```

## Environment Configuration

```yaml
environment:
  name: "leo_perturbed"
  gravity: 0.0                      # Free-fall

  orbital_altitude_km: 400.0        # ISS-like
  mean_motion: 0.00113              # [rad/s]

  enable_drag: true
  enable_j2: true
  enable_gravity_gradient: true
```

## Loading Configs Programmatically

```cpp
#include "io/ConfigLoader.h"

// Load individual configs
auto vehicle = ConfigLoader::load_vehicle_config("vehicle/cube_sat_6u.yaml");
auto mission = ConfigLoader::load_mission_config("mission/station_keeping.yaml");

// Or load complete config tree
auto config = ConfigLoader::load("main.yaml");
// config.vehicle, config.control, config.mission, config.environment
```
