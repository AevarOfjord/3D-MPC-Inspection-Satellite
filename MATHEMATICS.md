# Mathematics Behind the Implemented MPC/MPCC Controller

This document describes the *actual equations implemented today* in:

- `src/python/control/codegen/satellite_dynamics.py` (CasADi symbolic dynamics)
- `src/python/control/codegen/cost_functions.py` (CasADi symbolic cost terms)
- `src/cpp/mpc_v2/sqp_controller.cpp` (RTI-SQP solver, OSQP backend)
- `src/cpp/orbital_dynamics.cpp`

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

In `satellite_dynamics.py`, quaternion dynamics are linearized (via CasADi AD) from the continuous kinematic relation:

```text
q_{k+1} approx q_k + 0.5 * G(q_k) * w_k * dt
```

with:

```text
G(q) = [ -qx  -qy  -qz
          qw  -qz   qy
          qz   qw  -qx
         -qy   qx   qw ]
```

and quaternion normalized before Jacobian evaluation.

### 2.2 Orbital terms

Two-body mode (default in current setup):

```text
a(r) = -mu * r / |r|^3
```

For relative dynamics, the gravity-gradient Jacobian is:

```text
J = -mu * ( I/|r|^3 - 3 rr^T/|r|^5 )
```

and applied to velocity-position coupling (`A(7:9,0:2)`), with residual in affine term:

```text
a_k(vel) = dt * (a_rel - J * r_rel)
```

CW mode is also supported via Hill/Clohessy-Wiltshire terms.

### 2.3 Angular-rate gyroscopic Jacobian

When enabled:

```text
I w_dot + w x (I w) = tau
```

is linearized around current `w`, adding a Jacobian block to `A(10:12,10:12)`.

### 2.4 Input matrix mapping

- RW torque contributes to body angular acceleration and wheel-speed dynamics:
  - `w_dot += -I^{-1}(axis_i * tau_max_i * u_i)`
  - `w_rw_dot += (tau_max_i / I_rw_i) * u_i`
- Thrusters:
  - body force `F_body = f_i * d_i`
  - world force `F_world = R(q) F_body`
  - `v_dot += F_world / m`
  - `w_dot += I^{-1} (r_i x F_body)`

## 3) QP form and variable scaling

OSQP solves:

```text
min (1/2) z^T P z + q^T z
s.t. l <= A z <= u
```

The implementation optionally solves in scaled coordinates:

```text
z = S * z_tilde
```

with per-variable scales (`state_var_scale_`, `control_var_scale_`), and matrix entries assembled/updated directly in scaled form.

## 4) Objective function terms (implemented)

The cost is stage-wise MPCC plus regularization/effort terms.

## 4.1 Contouring + lag (path geometry)

At each stage `k`, with linearization point `s_bar`:

- `p_ref = p(s_bar)`
- `t_ref = dp/ds |_(s_bar)` (unit tangent)

Linearized path point:

```text
p(s) approx p_ref + t_ref * (s - s_bar)
```

Contouring uses the expansion of:

```text
||p - p(s)||^2
```

which yields:

- position diagonal terms
- `(p_i, s)` cross terms
- `s` diagonal terms
- linear terms in `p` and `s`

Lag is implemented through a tangent projector contribution (`t_ref t_ref^T`) on position block and matching linear term.

## 4.2 Velocity alignment term

Velocity tracking along tangent:

```text
Q_v * ||v - v_ref * t_ref||^2
```

implemented as velocity Hessian diagonals and linear term toward `t_ref`.

## 4.3 Progress term (`v_s`)

In speed-tracking mode:

```text
Q_progress * (v_s - v_ref)^2
```

or, when `progress_reward > 0`, linear reward:

```text
-2 * progress_reward * v_s
```

In `error_priority` mode:

- progress quadratic is reduced to tiny regularization (`1e-4` on `v_s`)
- linear progress drive is removed
- velocity-align and `s`-anchor terms are disabled

so path-error reduction dominates.

## 4.4 Attitude and angular-rate terms

Quaternion tracking:

```text
Q_att * ||q - q_ref||^2
```

where `Q_att = Q_attitude + Q_axis_align` and `q_ref` is built from path tangent plus scan-frame logic.

Additional quaternion regularizer is implemented as:

```text
Q_quat_norm * ||q - q_current||^2
```

(implemented through diagonal + linear terms around current quaternion).

Angular velocity damping:

```text
Q_angvel * ||w||^2
```

with runtime mode scaling.

## 4.5 Control effort and shaping

Base effort:

```text
R_rw * ||tau_rw||^2 + R_thr * ||u_thr||^2 + Q_progress * v_s^2
```

Smoothness:

```text
Q_smooth * sum_{k=1}^{N-1} ||u_k - u_{k-1}||^2
```

with optional explicit cross-stage Hessian coupling (`enable_delta_u_coupling`).

Opposing-thruster cofire penalty (per pair):

```text
w_pair * (u_i + u_j)^2
```

Fuel/coasting bias:

```text
thrust_l1_weight * sum_i u_thr,i
```

## 4.6 Terminal terms

Terminal stage receives:

- endpoint position pull to final path point
- endpoint progress pull to total path length
- DARE-based terminal physics cost (see next section)

plus mode-dependent scaling (SETTLE/HOLD/COMPLETE can increase terminal stabilization emphasis).

## 5) DARE terminal cost (implemented)

For the 16 physical states, a discrete Riccati iteration computes `P` from local linearization:

```text
S = R + B^T P B
K = S^{-1} B^T P A
P_next = Q + (A-BK)^T P (A-BK) + K^T R K
```

This terminal `P` is used as:

- diagonal-only (default `diagonal` profile), or
- with added dense off-diagonal terminal couplings (`dense_terminal`)

Online periodic updates can refresh this terminal block around the latest predicted tail state.

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
- optional velocity and angular-rate bounds
- progress state `s` bounded to path range (with startup margin)
- `v_s` bounded by `[path_speed_min, path_speed_max]` (runtime-adjusted)

## 7) Progress-bound adaptation and endpoint behavior

Near path end, minimum `v_s` is relaxed to allow stop:

```text
if remaining_distance <= horizon_min_distance: v_s_min = 0
```

In `error_priority` mode, upper speed cap is reduced by path error:

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
- control horizon length (shorter in some modes)

This is done by updating effective weights and rebuilding/updating QP terms accordingly.

## 9) Robust scaffold (`tube` mode)

Two mechanisms are implemented:

1. **Constraint tightening**
   - state/control bounds contracted by a configured fraction.
2. **Ancillary feedback correction**
   - after nominal MPC solve:

```text
u_phys <- u_phys - alpha * K * (x - x_nom)
```

with `K` from local DARE/LQR and correction clipped per channel.

## 10) Solver and fallback behavior

OSQP is used with warm-start and bounded solve time.

If non-success status occurs, control fallback is:

1. hold last feasible command for `T_hold`
2. linearly decay over `T_decay`
3. force zero after `T_zero`

This preserves bounded, graceful behavior under solver failures/timeouts.
