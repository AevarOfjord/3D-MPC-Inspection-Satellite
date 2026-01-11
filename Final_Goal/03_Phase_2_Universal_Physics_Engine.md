# Phase 2: Universal Adaptability (The "Engine")

**Goal**: Rewrite the physics and control engine to accept *arbitrary* vehicle definitions without code changes.

## 1. Dynamic Dynamics Model (`physics.py`)
Currently, the B-matrix (Control Matrix) is likely hardcoded for a specific 12-thruster layout.
*   **Change**: Implement `compute_b_matrix(vehicle_config)` that procedurally generates the $12 \times N_{thrusters}$ matrix based on the `position` and `direction` vectors in the YAML.
*   **Math**: $\tau = r \times F$. The torque contribution of each thruster is calculated at runtime.

## 2. Generalized MPC Formulation (`mpc_controller.py`)
*   **Objective Function**: The cost matrix $R$ must dynamically resize based on `len(cfg.vehicle.thrusters)`.
*   **Constraints**:
    *   *Old*: `u_min = 0, u_max = MAX_THRUST` (scalar constant).
    *   *New*: `u_max` vector constructed from `[t.max_thrust for t in cfg.vehicle.thrusters]`. This allows mixing large main engines with tiny RCS thrusters in the same optimization.

## 3. Visualizer Adaptability (`mujoco_viewer.py`)
The visualizer currently hardcodes "12 thrusters".
*   **Update**: Read `vehicle.yaml` to dynamically spawn thruster geometries (sites) in the MuJoCo XML scene or update their positions programmatically if using a template.
*   **Asset Pipeline**: If the user provides a new `.obj` mesh in `vehicle.yaml`, the sim must automatically load it.

## 4. Verification Plan

1.  **Unit Test**: `test_dynamic_b_matrix.py`.
    *   Compare the procedural B-matrix against the old hardcoded one for the standard 12-thruster CubeSat. They must match exactly.
2.  **Regression Test**: Run the standard `waypoint` mission with the new Hydra config.
    *   Success criteria: Trajectory matches the pre-refactor trajectory within floating-point tolerance (< 1e-6).
3.  **New Capability Test**: Define a "3-Thruster" asymmetric satellite in YAML.
    *   Run simulation.
    *   Success criteria: Solver finds a valid solution (even if performance is poor) without crashing due to matrix dimension mismatches.
