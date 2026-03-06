# `cpp_nonlinear_rti_hpipm`

See [README.md](./README.md) in this folder for shared notation and the common path-following problem.

## 1. Controller Identity

- Profile ID: `cpp_nonlinear_rti_hpipm`
- Solver backend: acados with HPIPM
- Controller family: nonlinear MPC using `SQP_RTI`
- Intended comparison role: acados real-time nonlinear controller with one SQP step per control update

## 2. Problem This Controller Solves

This controller solves an exact-model nonlinear MPC problem in acados, but with one real-time SQP iteration per control step rather than a full convergence loop.

At a high level, it solves

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

where `f_d` is the exact nonlinear discrete model and `r_k` is a nonlinear least-squares residual vector.

Physically, it still tries to:

- follow the inspection path
- regulate forward progress
- align attitude with the path-based reference frame
- suppress excessive angular motion
- keep control effort and control-rate changes reasonable

## 3. Mathematical Transcription Used Here

### Decision Variables

The acados formulation uses an augmented state:

```text
x_aug,k = [x_sat,k, u_prev,k]
```

where

- `x_sat,k` is the 17-state satellite/path state
- `u_prev,k` stores the previous control so that smoothness can be represented directly inside the OCP

The control variable is the current command `u_k`.

### Dynamics

The dynamics are exact nonlinear discrete dynamics:

```text
x_sat,k+1 = f_d(x_sat,k, u_k)
u_prev,k+1 = u_k
```

The second equation is the augmentation that makes intra-horizon smoothness a true part of the model, not just a separate warm-start heuristic.

### Cost Structure Actually Implemented

The stage cost is implemented as a nonlinear least-squares residual vector with Gauss-Newton Hessian approximation. The residual includes:

- contouring residual
- lag residual
- progress residual
- velocity-alignment residual
- attitude residual from quaternion error
- angular-velocity residual
- wheel-effort residual
- thruster-effort residual
- opposing-thruster residuals
- smoothness residual `u_k - u_prev,k`

The terminal residual includes:

- terminal position error
- terminal attitude error
- terminal angular velocity
- terminal velocity

Important implementation caveats:

- this controller does **not** currently use the generic symbolic scalar stage cost directly
- it does **not** include the quaternion-normalization term used by the IPOPT and OSQP symbolic cost family
- it does **not** include an explicit `s`-anchor or terminal `s` residual
- it uses reduced stage attitude and angular-rate weights relative to the raw shared MPC weights
- it sets zero `omega` reference and relies on terminal attitude/rotation shaping to avoid pathological spin-up

### Constraints

The acados RTI controller enforces:

- exact nonlinear discrete dynamics
- initial-state equality
- control bounds
- hard delta-`u` constraints through `u_k - u_prev,k`
- terminal angular-velocity state bounds

These delta-`u` and terminal-omega hard constraints are important mathematical differences from the other controllers.

### Reference Construction

The horizon reference is built from the path in the same broad way as the IPOPT controller:

- interpolate position and tangent along the path
- build a path-following reference quaternion
- keep frame continuity across calls

The path-reference machinery is shared conceptually with the rest of the stack even though the cost transcription is different.

## 4. How The Solve Proceeds Each Control Step

1. Build the 17-state measured satellite/path state.
2. Augment it with the last applied control to form the initial augmented state.
3. Build the horizon reference trajectory.
4. Write stage parameters `[satellite params, p_ref, t_ref, q_ref]` into the acados solver.
5. Set zero residual targets and warm-start with shifted previous trajectories.
6. Run one `SQP_RTI` step.
7. Extract the first control move and cache the solved trajectories for the next warm start.

This is a real-time nonlinear MPC formulation, but with a deliberately minimal SQP iteration budget.

## 5. Why This Formulation Exists

This formulation exists to combine nonlinear-model fidelity with real-time structure.

Compared with the IPOPT controller, it keeps exact nonlinear dynamics but uses the acados/HPIPM structure designed for embedded MPC. Compared with the OSQP RTI profiles, it avoids the explicit user-level affine QP transcription and instead relies on the acados RTI pipeline.

For a real inspection satellite, this profile is attractive because it is much closer to what an embedded nonlinear MPC implementation would look like if exact-model fidelity is desired but full NLP convergence is too expensive.

## 6. Scientific Comparison Notes

### What makes it comparable

- Same physical state, control, and path-following mission meaning
- Same shared nonlinear dynamics source
- Same broad tracking/effort/smoothness design intent
- Same fairness baseline parameters at the configuration level

### What makes it not perfectly identical

- Uses `NONLINEAR_LS` residuals and Gauss-Newton, not the scalar objective form used elsewhere
- Uses augmented state for smoothness
- Adds hard delta-`u` constraints and terminal angular-rate bounds
- Omits some shared terms such as quaternion-normalization and explicit `s` anchoring
- Uses one RTI step, so it is not converged SQP

### Main paper caveat

This controller is best described as the embedded real-time nonlinear MPC candidate, but not as the exact same optimization problem solved by a different numerical library.

## 7. When This Controller Is Likely Best

Likely strongest when:

- nonlinear dynamics fidelity is important
- real-time solve speed still matters
- the mission benefits from hard control-rate constraints
- the target architecture resembles embedded NMPC

Likely weakest when:

- one SQP step is insufficient for difficult path segments
- strict comparability to the RTI-QP or IPOPT objective is required
- the experiment wants converged nonlinear solutions rather than RTI behavior

## 8. Implementation Anchors

- [`../controller/acados_rti/python/controller.py`](../controller/acados_rti/python/controller.py)
- [`../controller/acados_shared/python/base.py`](../controller/acados_shared/python/base.py)
- [`../controller/shared/python/control_common/codegen/satellite_dynamics.py`](../controller/shared/python/control_common/codegen/satellite_dynamics.py)
