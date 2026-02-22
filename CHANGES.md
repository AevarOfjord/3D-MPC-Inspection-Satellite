# MPC Controller Improvements

## Summary

Three improvements were made to the OSQP-based MPC controller, along with a full sweep to remove the deprecated obstacle-avoidance subsystem.

---

## 1. Obstacle Avoidance Removed

All obstacle-avoidance code has been stripped from the codebase.

**C++ / config:**

- Removed `enable_collision_avoidance` and `obstacle_margin` from `MPCParams` (`mpc_controller.hpp`)

**Python config layer:**

- `src/python/config/defaults.py` — removed `enable_collision_avoidance` default
- `src/python/config/simulation_config.py` — removed fields from `to_dict()`
- `src/python/config/io.py` — removed `obstacle_state` serialization
- `src/python/config/models.py` — removed fields from `MPCParams` schema

**Runtime / mission:**

- `src/python/mission/unified_compiler.py` — removed `margin`/`obstacles` args from `_build_segment_path` and `_build_compiled_path_and_spans`
- `src/python/mission/runtime_loader.py` — removed `mission_state.obstacles`, `obstacles_enabled`, and `_to_runtime_obstacles()`
- `src/python/core/simulation_initialization.py` — removed obstacle log block
- `src/python/core/simulation_io.py` — removed obstacle telemetry, breach tracking, and margin calculations
- `src/python/core/mpc_runner.py` — fixed `SimulationConfig` vs `AppConfig` physics attribute resolution

**Tests:**

- Removed `test_obstacle_constraint_api`, `test_collision_avoidance_flag_passthrough`, and `test_compile_unified_mission_path_ignores_obstacles_for_generation`

---

## 2. LTV-MPC (Trajectory Linearization) — Verified

The controller already had per-step trajectory linearization in place. Each call to `get_control_action()` builds a predicted state trajectory from the warm-start solution and calls `update_dynamics(x_traj)`, which evaluates distinct $(A_k, B_k)$ at each horizon step $k$ (rather than a single linearization at the current state). Index maps are pre-allocated per step at initialization.

No code changes required — verification confirmed it was working correctly.

---

## 3. Exact DARE Terminal Costs

**Before:** terminal cost = `Q_diag * 10.0` (arbitrary heuristic)

**After:** terminal cost = diagonal of $P^\infty$ from the Discrete Algebraic Riccati Equation

The new `compute_dare_terminal_cost()` method in `mpc_controller.cpp`:

1. Linearizes the dynamics at the zero state (nominal hover) to obtain $(A, B)$ in the 16-state physics space
2. Runs value iteration (up to 500 iterations, tolerance $10^{-4}$) using the closed-loop Riccati update:

$$P_{k+1} = Q + (A - BK)^\top P_k (A - BK) + K^\top R K, \quad K = (R + B^\top P_k B)^{-1} B^\top P_k A$$

3. Embeds the result into the full 17-state augmented terminal cost (the path parameter $s$ at index 16 keeps a conservative 10× stage cost)

This computes once at controller construction — **zero runtime overhead**.

---

## Test Results

```
96 passed, 1 skipped
MPC solve time: ~35ms (unchanged)
```
