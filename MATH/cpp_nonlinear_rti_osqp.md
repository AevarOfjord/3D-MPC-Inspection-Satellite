# `cpp_nonlinear_rti_osqp`

See [README.md](./README.md) in this folder for shared notation and the common path-following problem.

## 1. Controller Identity

- Profile ID: `cpp_nonlinear_rti_osqp`
- Solver backend: CasADi stage-wise nonlinear linearization in Python, sparse QP solve in C++ with OSQP
- Controller family: RTI-SQP / sequential QP
- Intended comparison role: highest-fidelity nonlinear member of the OSQP-based RTI family

## 2. Problem This Controller Solves

This controller targets the same path-following finite-horizon problem as the hybrid profile, but it is stricter about recomputing the local nonlinear model and can optionally perform multiple outer SQP iterations in one control step.

The local subproblem solved at each outer iteration is

```text
min sum ell_k(x_k, u_k; ref_k) + ell_N(x_N)
```

subject to

```text
x_{k+1} = A_k x_k + B_k u_k + d_k
x_0 = x_meas
```

with `A_k`, `B_k`, and `d_k` recomputed from the nonlinear model at every horizon stage.

Physically, it still tries to:

- move the satellite along the path through `v_s`
- reduce contouring and lag error
- align the spacecraft attitude to the path-based frame
- regulate angular motion and actuation effort

## 3. Mathematical Transcription Used Here

### Decision Variables

The decision variables are the same horizon-stacked state/control sequence used by the OSQP RTI family:

```text
z = [x_0, ..., x_N, u_0, ..., u_{N-1}]
```

### Dynamics

The controller uses the shared symbolic nonlinear discrete dynamics to generate exact stage-wise Jacobians through CasADi automatic differentiation. That produces the affine stage model

```text
x_{k+1} = A_k x_k + B_k u_k + d_k
```

for each stage `k`, but unlike the linearized profile, these matrices are refreshed at every stage, and unlike the hybrid profile, integrity can be enforced strictly.

### Cost Structure

The QP cost is the same C++ RTI runtime cost family used by the hybrid and linearized profiles:

- position/path penalty from contour and lag weighting
- attitude tracking
- translational and angular-rate penalties
- progress and `s` anchoring
- smoothness coupling
- control effort
- thruster-pair and optional `L1` fuel bias
- terminal progress, velocity, and optional DARE shaping

### Constraints

The main constraints are the same as the OSQP RTI family:

- affine dynamics equalities
- initial-state equality
- state and input bounds
- control-horizon tying after `control_horizon`
- bounded virtual path speed

### Reference Construction

References are generated from the current path and current path progress estimate. The important point for this controller is that every outer SQP relinearization is still anchored to the same path-reference machinery used in the rest of the stack.

## 4. How The Solve Proceeds Each Control Step

1. Form the augmented measured state including `s`.
2. Build stage-wise linearizations at every stage using CasADi.
3. Solve the resulting sparse QP with OSQP.
4. If `sqp_max_iter > 1`, use the returned trajectory as a warm start and repeat the process as an outer SQP loop.
5. Stop early if the outer-loop control change falls below the configured SQP tolerance.
6. Return the first physical control move and update the path-progress state.

The controller therefore ranges from RTI behavior at `sqp_max_iter = 1` to a small sequential-QP nonlinear solve when `sqp_max_iter > 1`.

## 5. Why This Formulation Exists

This profile exists to answer a specific research question: how much benefit is gained by keeping the efficient RTI-QP architecture, but insisting on more faithful nonlinear relinearization?

Relative to the hybrid profile, it removes the pragmatic stale-stage tolerance and instead treats linearization quality as part of the control law. Relative to the full NLP controller, it retains the sparse QP computational structure that is much more realistic for tight onboard timing budgets.

For inspection-satellite feasibility, this profile is valuable because it isolates the benefit of stronger local nonlinear fidelity without fully abandoning real-time structure.

## 6. Scientific Comparison Notes

### What makes it comparable

- Same RTI/OSQP solver family as the hybrid and linearized profiles
- Same shared state, control, and reference definitions
- Same broad QP objective family
- Same fairness baseline parameters when `shared.parameters=true`

### What makes it not perfectly identical

- It is still a sequential-QP method, not a direct nonlinear program
- When `sqp_max_iter > 1`, it no longer has the same per-step iteration budget as the one-shot RTI profiles
- Its strict linearization integrity policy can change failure behavior relative to the hybrid profile

### Main paper caveat

This is the cleanest nonlinear benchmark inside the OSQP RTI family, but it is still not mathematically equivalent to the full nonlinear IPOPT or acados formulations.

## 7. When This Controller Is Likely Best

Likely strongest when:

- nonlinear local model fidelity matters
- the platform still needs structured sparse QP solves
- the experiment wants a cleaner RTI-family scientific benchmark than the hybrid controller

Likely weakest when:

- computation budget is too tight for repeated outer SQP iterations
- strict linearization integrity causes more fail-closed behavior than the mission can tolerate
- the study needs the exact nonlinear problem rather than repeated affine subproblems

## 8. Implementation Anchors

- [`../controller/nonlinear/python/controller.py`](../controller/nonlinear/python/controller.py)
- [`../controller/shared/python/control_common/mpc_controller.py`](../controller/shared/python/control_common/mpc_controller.py)
- [`../controller/nonlinear/cpp/nonlinear_sqp_controller.cpp`](../controller/nonlinear/cpp/nonlinear_sqp_controller.cpp)
- [`../controller/shared/python/control_common/codegen/satellite_dynamics.py`](../controller/shared/python/control_common/codegen/satellite_dynamics.py)
