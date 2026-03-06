# `cpp_nonlinear_sqp_hpipm`

See [README.md](./README.md) in this folder for shared notation and the common path-following problem.

## 1. Controller Identity

- Profile ID: `cpp_nonlinear_sqp_hpipm`
- Solver backend: acados with HPIPM
- Controller family: nonlinear MPC using full `SQP`
- Intended comparison role: higher-iteration acados nonlinear benchmark that pushes the same acados OCP harder than the RTI variant

## 2. Problem This Controller Solves

This controller uses the same acados optimal control problem as `cpp_nonlinear_rti_hpipm`, but it allows multiple SQP iterations per control step with globalization.

Its implemented problem is still

```text
min sum ||r_k(x_k, u_k)||^2_{W_k} + ||r_N(x_N)||^2_{W_N}
```

subject to

```text
x_{k+1} = f_d(x_k, u_k)
x_0 = x_meas
u_k in U
h(x_k, u_k) in H
```

The difference is not the OCP definition. The difference is that the controller spends more computational effort trying to solve that OCP more completely at each timestep.

## 3. Mathematical Transcription Used Here

### Decision Variables

Exactly the same augmented-state acados transcription as the RTI profile:

```text
x_aug,k = [x_sat,k, u_prev,k]
```

with `u_k` as the stage control.

### Dynamics

Exactly the same exact nonlinear discrete dynamics:

```text
x_sat,k+1 = f_d(x_sat,k, u_k)
u_prev,k+1 = u_k
```

### Cost Structure Actually Implemented

Exactly the same `NONLINEAR_LS` residual structure as the RTI acados controller:

- contouring
- lag
- progress
- velocity alignment
- attitude residual
- angular velocity
- control effort
- opposing-thruster residuals
- smoothness through state augmentation

The terminal residual is also the same:

- position
- attitude
- angular velocity
- velocity

So the scientific distinction between the two acados profiles is **not** the mathematical problem statement. It is the iteration budget and globalization strategy.

### Constraints

Exactly the same acados constraints as the RTI variant:

- control bounds
- hard delta-`u` bounds
- terminal angular-velocity bounds
- nonlinear dynamics equalities

### Reference Construction

The path-reference generation is shared with the RTI acados controller and is therefore not the differentiating factor between the two.

## 4. How The Solve Proceeds Each Control Step

1. Build the augmented initial state from the measured satellite state and the last applied control.
2. Build the horizon path/attitude reference.
3. Load solver parameters and warm-start from the shifted previous solution.
4. Run acados in `SQP` mode rather than `SQP_RTI`.
5. Allow multiple SQP iterations up to the configured iteration cap.
6. Use merit-backtracking globalization to improve robustness of the nonlinear iterations.
7. Extract the first control move and cache the solved trajectories.

This makes the controller mathematically identical to the RTI acados profile at the OCP level, but algorithmically closer to a converged nonlinear MPC solve.

## 5. Why This Formulation Exists

This formulation exists to answer the question:

If the exact-model acados OCP is worth solving, how much more can be gained by spending more SQP iterations per control step?

That is a meaningful paper question because it separates:

- model fidelity
- solver structure
- iteration budget

from one another.

For a real inspection satellite, this profile is attractive when better nonlinear solution quality is needed and more onboard computation can be tolerated than in strict RTI mode.

## 6. Scientific Comparison Notes

### What makes it comparable

- Same exact OCP transcription as `cpp_nonlinear_rti_hpipm`
- Same nonlinear dynamics source and same reference generation
- Same control and state meaning as the rest of the stack
- Same broad tracking/effort/smoothness objective family

### What makes it not perfectly identical

- It still differs from the IPOPT and RTI-QP formulations in cost transcription and hard constraints
- It spends more work per control step than the acados RTI profile
- Convergence quality depends directly on the allowed SQP iterations and tolerances

### Main paper caveat

This profile is the "solve the acados OCP harder" benchmark, not a distinct control law relative to the RTI acados profile.

## 7. When This Controller Is Likely Best

Likely strongest when:

- the exact nonlinear acados OCP is already judged appropriate
- one RTI step is not enough for difficult path segments
- more compute can be traded for higher-quality nonlinear solutions

Likely weakest when:

- hard real-time deadlines dominate all other concerns
- the RTI acados profile already performs adequately
- the mission cannot tolerate the larger per-step iteration budget

## 8. Implementation Anchors

- [`../controller/acados_sqp/python/controller.py`](../controller/acados_sqp/python/controller.py)
- [`../controller/acados_shared/python/base.py`](../controller/acados_shared/python/base.py)
- [`../controller/shared/python/control_common/codegen/satellite_dynamics.py`](../controller/shared/python/control_common/codegen/satellite_dynamics.py)
