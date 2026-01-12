# Satellite Control System (C++)

A commercial-grade, fault-tolerant spacecraft control kernel for autonomous inspection satellites.

## Features

- **Universal Adaptability** - Any thruster configuration from YAML, no code changes
- **Hybrid Control** - Seamless fusion of reaction wheels + thrusters via MPC
- **Fault Tolerance** - Real-time B-matrix adaptation on actuator failure
- **Autonomous Inspection** - One-click hull inspection from STL/OBJ meshes
- **Flight Ready** - Export to C99 for ARM/FPGA deployment

## Quick Start

```bash
# Build
cd Cplusplus_build
mkdir build && cd build
cmake ..
make sat_control

# Run
./sat_control ../config/mission_test.yaml
```

## Architecture

```
YAML Config → ConfigLoader → MPCController → PhysicsEngine
                   ↓               ↓
           FaultManager    ControlAllocator (B-matrix)
```

## Configuration

Vehicle, control, and mission parameters are fully configurable:

```yaml
# config/vehicle/cube_sat_6u.yaml
vehicle:
  name: "CubeSat_6U"
  mass: 12.0
  thrusters:
    - id: "T1_PX"
      position: [0.15, 0.0, 0.0]
      direction: [-1.0, 0.0, 0.0]
      max_thrust: 1.0
```

## Documentation

| Document | Description |
|----------|-------------|
| [Quick Start](docs/QUICKSTART.md) | Build and run in 5 minutes |
| [Architecture](docs/ARCHITECTURE.md) | System design overview |
| [Configuration](docs/CONFIGURATION.md) | YAML schema reference |
| [API Reference](docs/API_REFERENCE.md) | Class documentation |
| [Flight Deployment](docs/FLIGHT_DEPLOYMENT.md) | Embedded export guide |
| [Inspection Planning](docs/INSPECTION_PLANNING.md) | Mesh-to-mission workflow |
| [Fault Tolerance](docs/FAULT_TOLERANCE.md) | FDIR system guide |

## Dependencies

- **Eigen** - Linear algebra
- **yaml-cpp** - Configuration parsing  
- **spdlog** - Logging
- **OSQP** - QP solver for MPC
- **MuJoCo** - Physics simulation

All dependencies are fetched automatically via CMake.

## License

Proprietary - All rights reserved.
