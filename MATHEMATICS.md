# Mathematics Behind the Implemented MPC/MPCC Controller

This document describes the *actual equations implemented today* in:

- `src/python/control/codegen/satellite_dynamics.py` (CasADi symbolic dynamics)
- `src/python/control/codegen/cost_functions.py` (CasADi symbolic cost terms)
- `src/cpp/mpc/sqp_controller.cpp` (RTI-SQP solver, OSQP backend)
- `src/cpp/sim/orbital_dynamics.cpp`

The MPC uses CasADi automatic differentiation for exact Jacobians and Hessians,
injected into a C++ RTI-SQP (Real-Time Iteration Sequential Quadratic Programming)
loop that solves the resulting QP via OSQP at each control step.

## 1) Decision variables and dimensions

At each control step, the solver uses a finite-horizon QP with:

- Horizon: `N`
- Control period: `dt`
- Augmented state size: `nx = 17`
- Control size: `nu = n_rw + n_thr + 1`

Stage state:

```text
x_k = [p_k, q_k, v_k, w_k, w_rw,k, s_k]
```

with blocks:

- `p_k in R^3` position
- `q_k in R^4` quaternion `[qw, qx, qy, qz]`
- `v_k in R^3` linear velocity
- `w_k in R^3` angular velocity
- `w_rw,k in R^3` wheel speeds
- `s_k in R` path progress

Stage control:

```text
u_k = [tau_rw,k, u_thr,k, v_s,k]
```

- `tau_rw,k` reaction-wheel torque commands (normalized to `[-1, 1]`)
- `u_thr,k` thruster commands (`[0, 1]`)
- `v_s,k` virtual path-speed control

The QP decision vector is stacked as:

```text
z = [x_0, ..., x_N, u_0, ..., u_{N-1}]
```

## 2) Dynamics model used inside MPC

The 16 physical states (all except `s`) are linearized online:

```text
x_phys,k+1 = A_k x_phys,k + B_k u_phys,k + a_k
```

where `u_phys` excludes `v_s`.

Progress-state augmentation is exact linear:

```text
s_{k+1} = s_k + dt * v_s,k
```

So each stage equality in the QP is:

```text
x_{k+1} - Ahat_k x_k - Bhat_k u_k = ahat_k
```

with `Ahat_k, Bhat_k, ahat_k` built from the 16-state linearization plus the `s` row.

### 2.1 Quaternion kinematics block

In `satellite_dynamics.py`, quaternion dynamics are propagated (via CasADi AD) from the continuous kinematic relation:

```text
q_dot = 0.5 * Xi(q) * w
```

with `Xi(q)` (4×3 matrix, scalar-first convention):

```text
Xi(q) = [ -qx  -qy  -qz
           qw  -qz   qy
           qz   qw  -qx
          -qy   qx   qw ]
```

The full nonlinear dynamics are propagated by RK4, and CasADi computes the exact discrete Jacobians `A_k = df/dx` and `B_k = df/du` via automatic differentiation.

### 2.2 Orbital terms

The MPC dynamics model uses Clohessy-Wiltshire (CW/Hill's) linearised gravity in the LVLH frame. Mean motion: `n = sqrt(mu / R^3)`.

CW accelerations:

```text
ax =  3n^2 x + 2n vy   (radial)
ay = -2n vx             (along-track)
az = -n^2 z             (cross-track)
```

These are exact linear functions of position and velocity and appear directly in the `A` matrix without a residual term.

Note: the simulation engine (`PHYSICS-ENGINE.md`) propagates the plant with nonlinear two-body differential gravity. The MPC intentionally uses the simpler CW model for real-time linearization.

### 2.3 Angular-rate gyroscopic term

Euler's rotational equation including reaction-wheel angular momentum:

```text
I w_dot + w x (I w + h_rw) = tau
```

where `h_rw = sum_i axis_i * I_rw,i * w_rw,i` is the total wheel angular momentum in the body frame. This is linearized around current `w` and `w_rw`, adding a Jacobian block to `A(10:12, 10:15)`.

### 2.4 Input matrix mapping

- RW torque contributes to body angular acceleration and wheel-speed dynamics:
  - `w_dot += -I^{-1}(axis_i * tau_max_i * u_i)`
  - `w_rw_dot += (tau_max_i / I_rw_i) * u_i`
- Thrusters:
  - body force `F_i = d_i * F_max_i * u_i`
  - world force `F_world = R(q) sum_i F_i`
  - `v_dot += F_world / m`
  - `w_dot += I^{-1} sum_i (r_i x F_i)`

## 3) QP form

OSQP solves:

```text
min (1/2) z^T P z + q^T z
s.t. l <= A z <= u
```

`P` is assembled as an upper-triangular sparse matrix with pre-allocated sparsity for diagonal state/control costs, smoothness cross-terms `(u_k, u_{k-1})`, and opposing-thruster pair cross-terms.

## 4) Objective function terms (implemented)

The cost is stage-wise MPCC plus regularization/effort terms.

## 4.1 Contouring + lag (path geometry)

At each stage `k`, the path reference is evaluated at the current progress estimate `s_k`:

- `p_ref = p(s_k)` (reference position)
- `t_ref = dp/ds |_(s_k)` (unit tangent)

The cross-track (contouring) and along-track (lag) errors are:

```text
e_c = (p - p_ref) - [(p - p_ref) . t_ref] t_ref   (perpendicular to tangent)
e_l = (p - p_ref) . t_ref                           (along tangent)
```

Costs:

```text
Q_contour * ||e_c||^2  +  Q_lag * e_l^2
```

In the C++ QP builder these are combined into a single diagonal position quadratic:

```text
(Q_contour + Q_lag) * ||p - p_ref||^2
```

with linear term `q_pos = -2*(Q_contour + Q_lag)*p_ref`. The progress state `s` is anchored separately (§4.3).

## 4.2 Velocity damping

The C++ QP builder penalises velocity magnitude:

```text
Q_v * ||v||^2
```

The CasADi symbolic cost function (`velocity_alignment_cost`) implements a full tangent-tracking term `Q_v * ||v - v_ref * t_ref||^2`, which is used for gradient/Hessian export but the runtime C++ QP builder uses the simpler damping form.

## 4.3 Progress term (`v_s`) and path anchor

Progress tracking:

```text
Q_progress * (v_s - v_ref)^2
```

or, when `progress_reward > 0`, an added linear incentive:

```text
q[v_s] -= progress_reward
```

Path progress anchor (keeps `s` near its current estimate):

```text
Q_s_anchor * (s - s_ref)^2
```

In `error_priority` progress policy, the upper speed bound is reduced by path error (see §7); cost weights are not modified.

## 4.4 Attitude and angular-rate terms

Quaternion tracking (4-component, in C++ QP builder):

```text
Q_att * ||q - q_ref||^2
```

where `Q_att = Q_attitude + Q_axis_align` and `q_ref` is built from path tangent plus scan-frame logic.

Additional quaternion regularizer:

```text
Q_quat_norm * ||q - q_current||^2
```

(diagonal + linear terms around current quaternion).

Angular velocity damping:

```text
Q_angvel * ||w||^2
```

with runtime mode scaling.

## 4.5 Control effort and shaping

Base effort:

```text
R_rw * ||tau_rw||^2 + R_thr * ||u_thr||^2
```

Smoothness (cross-stage control increment):

```text
Q_smooth * sum_{k=1}^{N-1} ||u_k - u_{k-1}||^2
```

Cross-stage Hessian entries `(u_{k-1}, u_k)` are pre-allocated in the sparsity pattern.

Opposing-thruster cofire penalty (per pair `(i, j)`):

```text
w_pair * (u_i + u_j)^2
```

Fuel/coasting bias:

```text
thrust_l1_weight * sum_i u_thr,i
```

## 4.6 Terminal terms

The terminal stage (`k = N`) applies a 10× scale on position, attitude, velocity, and angular-velocity costs relative to the stage cost. Additionally:

- `Q_terminal_vel` added to velocity diagonal
- `Q_terminal_s` added to `s` anchor diagonal
- DARE-based terminal diagonal (see §5)

## 5) Terminal cost approximation

For the 16 physical states, `compute_dare_terminal_diag()` computes a diagonal terminal cost using a heuristic scaling that approximates an infinite-horizon LQR cost-to-go:

```text
P_ii = Q_ii * N * dt * factor_i
```

where `factor_i` is state-dependent (0.5 for position, 0.3 for quaternion/angular velocity, 0.2 for linear velocity, 0.01 for wheel speeds). This approximates the steady-state Riccati solution without an explicit Riccati iteration.

The result is added to the terminal stage diagonal:

```text
P[x_N_offset + i, x_N_offset + i] += dare_diag[i]   for i = 0..15
```

Online updates refresh this block every `dare_update_period_steps` control steps.

## 6) Constraints

The QP constraint matrix includes:

1. Dynamics equalities for `k=0..N-1`
2. Initial-state equality (`x_0 = x_current`)
3. State bounds for all `k=0..N`
4. Control bounds for all `k=0..N-1`
5. Control-horizon tying (`u_k = u_{M-1}` for `k >= M` when `M < N`)

Typical bounds:

- RW commands in `[-1, 1]`
- Thrusters in `[0, 1]`
- wheel speeds bounded by configured limits
- optional velocity and angular-rate bounds (exponentially tightened over the horizon; k=0 is unconstrained since the initial-state equality pins it)
- progress state `s` bounded to `[0, path_length]`
- `v_s` bounded by `[path_speed_min, path_speed_max]` (runtime-adjusted)

## 7) Progress-bound adaptation and endpoint behavior

Near path end (`s_runtime > 0.9 * total_length`), minimum `v_s` is relaxed to allow stopping:

```text
vs_min = 0
```

In `error_priority` progress policy, the upper speed cap is reduced by path error:

```text
v_s_max_adaptive = v_s_max / (1 + gain * path_error^2)
```

and lower bound is clamped to `max(base_min, error_priority_min_vs)` (except endpoint relaxation above).

## 8) Runtime mode scaling (TRACK/RECOVER/SETTLE/HOLD/COMPLETE)

Mode changes rescale major weights:

- contour/lag/progress/attitude
- terminal position/attitude
- velocity alignment and angular-rate damping
- smoothness and thruster-pair penalties

This is done by `apply_mode_scaling()` updating cached effective weight scalars; QP matrices are rebuilt from these on the next `update_cost()` call.

## 9) Solver and fallback behavior

OSQP is used with warm-start and bounded solve time.

If non-success status occurs, control fallback is:

1. hold last feasible command for `T_hold`
2. linearly decay over `T_decay`
3. force zero after `T_hold + T_decay`

This preserves bounded, graceful behavior under solver failures/timeouts.
