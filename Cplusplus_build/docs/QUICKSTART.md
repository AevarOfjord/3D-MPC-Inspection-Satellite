# Quick Start Guide

Get the satellite control system running in 5 minutes.

## Prerequisites

- CMake 3.16+
- C++17 compiler (GCC 9+, Clang 10+, MSVC 2019+)
- Git

## Build

```bash
# From project root, navigate to C++ build directory
cd Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel/Cplusplus_build

# Build (first time)
mkdir -p build && cd build
cmake ..
make sat_control

# Build (subsequent times, from Cplusplus_build/)
cd build && make sat_control
```

First build takes ~2 minutes (downloads dependencies automatically).

## Run Your First Mission

```bash
./sat_control ../config/mission_test.yaml
```

Expected output:
```
[info] Loaded Vehicle 'TestSatellite' with 6 thrusters
[info] Loaded Mission 'Square Pattern' with 5 waypoints
[info] Starting Simulation Loop (Max 120.0s)...
[info] T=3.2s | WP:0 | Pos=[0.47 0.47 0.47] | Err=[0.06m]
[info] Transition: Reached Waypoint 0
```

## Try Different Vehicles

```bash
# 6-thruster CubeSat
./sat_control ../config/vehicle/cube_sat_6u.yaml

# 12-thruster Inspection Drone  
./sat_control ../config/vehicle/inspection_drone_v1.yaml
```

## Create Your Own Mission

1. Copy a template:
```bash
cp config/mission/station_keeping.yaml config/mission/my_mission.yaml
```

2. Edit waypoints:
```yaml
mission:
  name: "My Custom Mission"
  waypoints:
    - position: [1.0, 0.0, 0.0]
      attitude: [1.0, 0.0, 0.0, 0.0]
      hold_time: 5.0
```

3. Run:
```bash
./sat_control ../config/mission/my_mission.yaml
```

## Next Steps

- [Configuration Guide](CONFIGURATION.md) - Full YAML reference
- [Architecture](ARCHITECTURE.md) - System design
- [Fault Tolerance](FAULT_TOLERANCE.md) - Test failure scenarios
