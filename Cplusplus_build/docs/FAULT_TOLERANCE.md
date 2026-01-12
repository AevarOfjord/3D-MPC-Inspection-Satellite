# Fault Tolerance Guide

The FDIR (Fault Detection, Isolation, Recovery) system enables mission success despite actuator failures.

## Architecture

```
Healthy State     Fault Detected      Recovery
     │                  │                 │
     ▼                  ▼                 ▼
[6 Thrusters] → [T1 STUCK_OFF] → [5 Thrusters]
     │                  │                 │
   6×6 B         Recompute B          6×5 B
```

## Fault Types

| Type | Description |
|------|-------------|
| `STUCK_OFF` | Actuator cannot produce output |
| `STUCK_ON` | Actuator stuck at max output |
| `DEGRADED` | Reduced capability (0-100%) |
| `INTERMITTENT` | Random failures |

## Basic Usage

```cpp
#include "fdir/FaultManager.h"

FaultManager faults(vehicle_config);

// Inject fault
faults.inject_fault("T1", FaultType::STUCK_OFF);

// Check status
if (faults.has_active_faults()) {
    int healthy = faults.get_healthy_thruster_count();
    spdlog::warn("Operating with {}/{} thrusters", 
        healthy, vehicle_config.thrusters.size());
}

// Get config with failed actuators removed
auto effective = faults.get_effective_vehicle_config();

// Recompute B-matrix
allocator.update_config(effective);
```

## Real-Time Adaptation

```cpp
// Register callback for fault events
faults.set_fault_callback([&](const auto& status) {
    auto eff_cfg = faults.get_effective_vehicle_config();
    allocator.update_config(eff_cfg);
    mpc.rebuild(eff_cfg);  // Resize MPC matrices
});

// Now faults trigger automatic recomputation
faults.inject_fault("T3", FaultType::DEGRADED, 0.5);  // 50% thrust
```

## Degraded Mode

For partial failures, thrust is scaled:

```cpp
// Thruster operates at 50% capacity
faults.inject_fault("T2", FaultType::DEGRADED, 0.5);

// Effective config has max_thrust reduced
auto cfg = faults.get_effective_vehicle_config();
// cfg.thrusters[idx].max_thrust *= 0.5
```

## Testing Fault Scenarios

```cpp
// Simulate progressive failures
void test_graceful_degradation() {
    for (int i = 1; i <= 4; ++i) {
        faults.inject_fault("T" + std::to_string(i), FaultType::STUCK_OFF);
        
        // Run mission
        bool success = run_mission(mission);
        
        spdlog::info("Thrusters: {}/6 → Mission {}", 
            6-i, success ? "SUCCESS" : "FAILED");
    }
}
```

## Fault Clearing

```cpp
// Clear single fault
faults.clear_fault("T1");

// Clear all faults
faults.clear_all_faults();
```

## Controllability Check

Before deploying, verify minimum actuator set:

```cpp
// At least 3 thrusters needed for 3-DOF translation
if (faults.get_healthy_thruster_count() < 3) {
    trigger_safe_mode();
}

// Reaction wheels required for fine attitude
if (faults.get_healthy_rw_count() < 3) {
    switch_to_thruster_attitude_control();
}
```
