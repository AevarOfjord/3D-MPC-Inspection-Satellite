# Controller Mathematics

This folder is the authoritative mathematical description of the 6 implemented controller profiles for the satellite path-following problem. It is written from the current implementation in `controller/`, not from any older Markdown notes elsewhere in the repository.

Files:

- [cpp_hybrid_rti_osqp.md](./cpp_hybrid_rti_osqp.md)
- [cpp_nonlinear_rti_osqp.md](./cpp_nonlinear_rti_osqp.md)
- [cpp_linearized_rti_osqp.md](./cpp_linearized_rti_osqp.md)
- [cpp_nonlinear_fullnlp_ipopt.md](./cpp_nonlinear_fullnlp_ipopt.md)
- [cpp_nonlinear_rti_hpipm.md](./cpp_nonlinear_rti_hpipm.md)
- [cpp_nonlinear_sqp_hpipm.md](./cpp_nonlinear_sqp_hpipm.md)

## Shared Path-Following Optimal Control Problem

All 6 controllers are trying to solve the same physical guidance and control task: push an inspection satellite along a prescribed spatial path while keeping translational, rotational, and actuator behavior well behaved.

At the conceptual level, the finite-horizon problem is

```text
min_{x_0,...,x_N, u_0,...,u_{N-1}}  sum_{k=0}^{N-1} ell_k(x_k, u_k; ref_k) + ell_N(x_N; ref_N)
```

subject to

```text
x_{k+1} = f_d(x_k, u_k),    k = 0,...,N-1
x_0 = x_meas
u_k in U,   x_k in X
```

where the path reference depends on the virtual path coordinate `s` carried inside the state.

## Common Notation

State:

```text
x = [p, q, v, omega, omega_rw, s]
```

with

- `p in R^3`: satellite position in the LVLH/inertial working frame
- `q in R^4`: scalar-first quaternion `[q_w, q_x, q_y, q_z]`
- `v in R^3`: translational velocity
- `omega in R^3`: body angular velocity
- `omega_rw in R^3`: reaction-wheel speeds
- `s in R`: virtual path progress coordinate

Control:

```text
u = [tau_rw, u_thr, v_s]
```

with

- `tau_rw`: reaction-wheel torque commands
- `u_thr`: thruster commands or duty cycles
- `v_s`: virtual path-speed command

Path reference:

```text
p_ref(s), t_ref(s), q_ref(s)
```

with

- `p_ref(s)`: reference point on the path
- `t_ref(s)`: unit tangent to the path
- `q_ref(s)`: reference attitude, primarily built by aligning body `+X` with the tangent

## Shared Dynamics Summary

The shared symbolic dynamics are defined in [`controller/shared/python/control_common/codegen/satellite_dynamics.py`](../controller/shared/python/control_common/codegen/satellite_dynamics.py).

### Translational Dynamics

The common model uses Clohessy-Wiltshire relative-orbit dynamics:

```text
a_grav =
[ 3 n^2 x + 2 n v_y,
 -2 n v_x,
 -n^2 z ]
```

where `n = sqrt(mu / R^3)` is the orbital mean motion.

Thruster forces are accumulated in the body frame, rotated into the world frame through the quaternion, and divided by mass.

### Quaternion Kinematics

The quaternion dynamics are

```text
q_dot = 0.5 Xi(q) omega
```

with `Xi(q)` in scalar-first convention.

### Rotational Dynamics with Reaction Wheels

The rigid-body rotational dynamics include reaction-wheel angular momentum:

```text
I omega_dot + omega x (I omega + h_rw) = tau_thr + tau_rw
```

and the wheel-speed dynamics are driven by the wheel torques.

### Progress-State Update

The augmented progress state advances through the virtual path-speed input:

```text
s_{k+1} = s_k + dt * v_s,k
```

This is the mechanism that lets the optimizer decide how aggressively to move along the path while still tracking the geometric path itself.

## Shared Objective Summary

The full symbolic cost library lives in [`controller/shared/python/control_common/codegen/cost_functions.py`](../controller/shared/python/control_common/codegen/cost_functions.py). Across the controller family, the intended cost ingredients are:

- Contouring error: penalize cross-track displacement from the path
- Lag error: penalize along-track mismatch relative to the path parameterization
- Progress tracking: regulate `v_s` toward the desired path speed
- Progress reward: optionally bias forward motion
- Velocity alignment: keep velocity aligned with the path tangent
- Attitude tracking: align the spacecraft attitude with the path-based reference
- Angular-rate damping: suppress excessive body rotation
- Control effort: penalize wheel torque and thruster effort
- Smoothness: penalize control increments
- Thruster-pair penalty: discourage wasteful co-firing of opposing thrusters
- Fuel bias: linear penalty on thruster usage
- Terminal terms: endpoint position, progress, attitude, velocity, angular-rate, and optionally DARE-style terminal shaping

## What Is Held Fair Across Profiles

These features are intentionally shared or intended to be shared for scientific comparison:

- Same physical state and control meaning
- Same mission/path input and the same runtime mission state machine
- Same base `AppConfig.mpc` parameter block in fairness mode
- Same underlying satellite geometry, mass, inertia, wheel, and thruster data
- Same broad path-following objective family: contouring, progress, attitude, effort, smoothness, and terminal regulation
- Same controller-profile contract hashing mechanism for identifying the shared baseline

## What Is Not Mathematically Identical in the Current Implementation

This is the key section to acknowledge in a scientific paper. The 6 controllers are comparable, but they are not literally solving the same algebraic program.

### OSQP RTI-SQP family vs exact nonlinear controllers

The three OSQP profiles solve a sparse quadratic program built from affine discrete dynamics and a QP cost model. They do not solve the full nonlinear program directly.

### OSQP runtime cost is implementation-specific

In the C++ RTI-SQP runtime, contouring and lag are collapsed into a combined position quadratic around `p_ref`, progress anchoring is handled through `s`, control-horizon tying is explicit, and terminal DARE shaping can be added. This is not identical to the most general symbolic MPCC cost expression.

### IPOPT controller is a true nonlinear program, but not feature-complete relative to the shared symbolic library

The IPOPT controller uses exact RK4 nonlinear dynamics, but its implemented objective currently omits some shared terms that exist elsewhere in the stack, including explicit terminal `s` pull, explicit terminal velocity shaping from the shared RTI runtime, online DARE terminal shaping, and the linear thrust `L1` bias term.

### acados controllers use a different nonlinear least-squares transcription

The two acados controllers use exact nonlinear discrete dynamics, but the objective is implemented as a `NONLINEAR_LS` residual model with Gauss-Newton Hessian approximation, augmented state for smoothness, hard delta-`u` constraints, and terminal angular-rate bounds. They also omit some terms present in the other stacks, such as quaternion-normalization cost and explicit `s`-anchor / terminal-`s` costs.

### The two acados profiles share the same OCP but not the same solve policy

`cpp_nonlinear_rti_hpipm` uses `SQP_RTI`, while `cpp_nonlinear_sqp_hpipm` uses full `SQP` with globalization. That means the mathematical transcription is shared, but the solve budget per control step is different.

## Comparison Table

| Profile | Solver Family | Dynamics Transcription | Linearization Policy | Iteration Budget Per Step | Expected Tradeoff | Main Scientific Caveat |
| --- | --- | --- | --- | --- | --- | --- |
| `cpp_linearized_rti_osqp` | RTI-SQP + OSQP | Affine discrete model in QP | One frozen Jacobian reused across horizon | 1 QP | Cheapest, least faithful | Strongest approximation of nonlinear dynamics |
| `cpp_hybrid_rti_osqp` | RTI-SQP + OSQP | Affine discrete model in QP | Stage-wise linearization with tolerant stale reuse | 1 QP | Real-time and robust | Can reuse stale stage Jacobians on failures |
| `cpp_nonlinear_rti_osqp` | RTI-SQP + OSQP | Affine discrete model in QP | Exact stage-wise relinearization | 1 to `sqp_max_iter` QPs | Best nonlinear fidelity in OSQP family | Still solves sequential QPs, not the full NLP |
| `cpp_nonlinear_fullnlp_ipopt` | NMPC + IPOPT | Exact nonlinear RK4 constraints | None | Full NLP solve | Highest fidelity, slowest | Objective is not feature-identical to other profiles |
| `cpp_nonlinear_rti_hpipm` | acados SQP_RTI + HPIPM | Exact nonlinear discrete model | Internal acados RTI linearization | 1 SQP step | Fast exact-model nonlinear MPC | Uses different residual-form cost and added hard constraints |
| `cpp_nonlinear_sqp_hpipm` | acados SQP + HPIPM | Exact nonlinear discrete model | Internal acados SQP linearization | Multiple SQP steps | Higher-quality nonlinear solution than RTI | Same OCP as RTI acados, different convergence budget |

## Recommended Use in the Paper

For the paper, the cleanest wording is:

- The 6 controllers share one intended path-following satellite GNC problem and one shared baseline parameter set in fairness mode.
- They differ in how that problem is transcribed and solved.
- Therefore, the comparison is scientifically meaningful as a comparison of feasible control architectures, but it is not a comparison of six numerically identical optimizers.

That framing matches the implementation and is defensible.
