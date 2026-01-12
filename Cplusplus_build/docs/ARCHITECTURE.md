# System Architecture

## Overview

The satellite control system implements a Model Predictive Control (MPC) kernel with hybrid actuation (reaction wheels + thrusters).

## Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                        YAML CONFIGURATION                        │
│  vehicle/*.yaml  │  control/mpc/*.yaml  │  mission/*.yaml       │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                       ConfigLoader                               │
│  Hydra-style composition → RootConfig struct                    │
└────────────────────────────┬────────────────────────────────────┘
                             │
         ┌───────────────────┼───────────────────┐
         ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────────┐    ┌──────────────┐
│ FaultManager│    │ ControlAllocator│    │MissionManager│
│ (FDIR)      │    │ (B-matrix)      │    │ (Waypoints)  │
└──────┬──────┘    └────────┬────────┘    └──────┬───────┘
       │                    │                    │
       └────────────────────┼────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      MPCController                               │
│  1. Linearize dynamics (A, B matrices)                          │
│  2. Build QP problem (costs, constraints)                       │
│  3. Solve via OSQP → [RW torques, Thruster forces]              │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│                      PhysicsEngine                               │
│  MuJoCo simulation: apply forces → integrate → new state        │
└─────────────────────────────────────────────────────────────────┘
```

## Module Descriptions

### Core

| Module | File | Purpose |
|--------|------|---------|
| **Types** | `core/Types.h` | StateVector (13-DOF), ControlInput |
| **ConfigSchema** | `config/ConfigSchema.h` | Structured config types |

### Control

| Module | File | Purpose |
|--------|------|---------|
| **ControlAllocator** | `control/ControlAllocator.h` | B-matrix: thrust → wrench |
| **MPCController** | `control/MPCController.h` | Optimal control via QP |

### I/O

| Module | File | Purpose |
|--------|------|---------|
| **ConfigLoader** | `io/ConfigLoader.h` | YAML → structs |
| **DataLogger** | `io/DataLogger.h` | CSV telemetry output |

### Logic

| Module | File | Purpose |
|--------|------|---------|
| **MissionManager** | `logic/MissionManager.h` | Waypoint sequencing |
| **FaultManager** | `fdir/FaultManager.h` | Actuator health tracking |

### Planning

| Module | File | Purpose |
|--------|------|---------|
| **MeshAnalyzer** | `planner/MeshAnalyzer.h` | STL/OBJ loading, zones |
| **InspectionPlanner** | `planner/InspectionPlanner.h` | Mesh → waypoints |

### Deployment

| Module | File | Purpose |
|--------|------|---------|
| **FlightCodeGenerator** | `export/FlightCodeGenerator.h` | C99 code export |
| **HILBridge** | `hil/HILBridge.h` | Serial/CAN/UDP comms |
| **TelemetryServer** | `telemetry/TelemetryServer.h` | JSON pub/sub |

## State Vector (13-DOF)

```cpp
struct StateVector {
    Vector3 position;       // [x, y, z] meters
    Quaternion attitude;    // [w, x, y, z] body-to-inertial
    Vector3 velocity;       // [vx, vy, vz] m/s
    Vector3 angular_velocity; // [ωx, ωy, ωz] rad/s
};
```

## Control Vector

```cpp
struct ControlInput {
    Vector3 rw_torques;                 // [τx, τy, τz] Nm
    std::vector<double> thrust;         // [T1, T2, ...] Newtons
};
```

## Control Allocation

The B-matrix maps thrust commands to body wrench:

```
[Fx]     [d1x  d2x  ... dNx] [T1]
[Fy]     [d1y  d2y  ... dNy] [T2]
[Fz]  =  [d1z  d2z  ... dNz] [ ⋮]
[Tx]     [r1×d1  ...       ] [TN]
[Ty]     [     ...         ]
[Tz]     [           r×dN  ]
```

Where `d` = thrust direction, `r` = thruster position.
