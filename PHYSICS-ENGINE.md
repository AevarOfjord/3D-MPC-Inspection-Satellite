# Physics Engine Mathematics

This document explains the math implemented by the simulation/physics stack, primarily:

- `src/cpp/simulation_engine.cpp`
- `src/cpp/orbital_dynamics.cpp`
- `src/python/core/thruster_manager.py`
- `src/python/physics/orbital_dynamics.py`

## 1. State and Inputs

The C++ simulation engine propagates a 16-state rigid-body model:

```text
x = [r(3), q(4), v(3), w(3), wr(3)]
  = [x,y,z, qw,qx,qy,qz, vx,vy,vz, wx,wy,wz, wrx,wry,wrz]
```

- `r`: relative position (Hill/LVLH frame)
- `q`: body attitude quaternion, scalar-first `[w,x,y,z]`
- `v`: relative velocity
- `w`: body angular velocity
- `wr`: reaction wheel angular speeds

Inputs each step:

- Thruster commands `u_thr,i` (normalized, usually `[0,1]`)
- Reaction wheel torques `tau_rw` in N.m

## 2. Continuous-Time Dynamics

## 2.1 Kinematics

```text
r_dot = v
```

Quaternion kinematics (scalar-first convention):

```text
q_dot = 0.5 * q ⊗ [0, w]
```

Expanded in code:

```text
qw_dot = -0.5*(qx*wx + qy*wy + qz*wz)
qx_dot =  0.5*(qw*wx + qy*wz - qz*wy)
qy_dot =  0.5*(qw*wy - qx*wz + qz*wx)
qz_dot =  0.5*(qw*wz + qx*wy - qy*wx)
```

## 2.2 Translational Dynamics

```text
v_dot = a_gravity + a_thrust
```

Thruster force is summed in body frame then rotated to inertial/Hill frame:

```text
F_body = sum_i ( d_i * F_i * u_thr,i )
F_inertial = R(q) * F_body
a_thrust = F_inertial / m
```

where:

- `d_i`: unit thruster direction (body frame)
- `F_i`: max force for thruster `i`
- `m`: spacecraft mass

## 2.3 Rotational Dynamics

Body torque sum:

```text
tau_body = tau_thrusters + tau_rw_reaction
```

Thruster torque:

```text
tau_thrusters = sum_i (r_i - r_com) × (d_i * F_i * u_thr,i)
```

Reaction wheel body reaction torque:

```text
tau_rw_reaction = -tau_rw
```

Euler rigid-body equation with gyroscopic term:

```text
I w_dot + w × (I w) = tau_body
w_dot = I^{-1} (tau_body - w × (I w))
```

with `I` diagonal from configuration.

## 2.4 Reaction Wheel Speed Dynamics

Per axis:

```text
wr_dot,i = tau_rw,i / I_rw,i
```

where `I_rw,i` is wheel inertia.

## 3. Orbital/Relative Gravity Models

Two alternatives are implemented.

## 3.1 CW / Hill-Clohessy-Wiltshire model

Relative acceleration:

```text
ax =  3 n^2 x + 2 n vy
ay = -2 n vx
az = -n^2 z
```

where `n` is mean motion.

## 3.2 Two-body differential gravity model

Absolute gravity:

```text
a(r) = -mu / ||r||^3 * r
```

Relative acceleration (inspector minus target):

```text
a_rel = a(target + r_rel) - a(target)
```

The target orbit is propagated as a circular orbit each step:

```text
phase_{k+1} = phase_k + n*dt
r_target = [R cos(phase), R sin(phase), 0]
v_target = [-nR sin(phase), nR cos(phase), 0]
```

## 4. Numerical Integration

The physics engine uses fixed-step RK4 on the full 16-state system:

```text
k1 = f(x_k)
k2 = f(x_k + dt/2 * k1)
k3 = f(x_k + dt/2 * k2)
k4 = f(x_k + dt   * k3)

x_{k+1} = x_k + dt/6 * (k1 + 2k2 + 2k3 + k4)
```

Quaternion is normalized during intermediate RK stages and after the final update:

```text
q <- q / ||q||
```

This prevents drift from numerical integration.

## 5. Thruster Actuation Physics (Command -> Actual Output)

`ThrusterManager` models actuator timing and PWM behavior.

## 5.1 Valve delay

For each thruster, command changes are timestamped:

- Open command time: `t_open_cmd`
- Close command time: `t_close_cmd`

Effective valve times:

```text
t_open_valve  = t_open_cmd  + VALVE_DELAY
t_close_valve = t_close_cmd + VALVE_DELAY
```

No thrust before valve-open time.

## 5.2 Ramp-up

If realistic physics is enabled and `THRUST_RAMPUP_TIME > 0`:

```text
output(t) = clamp((t - t_open_valve) / THRUST_RAMPUP_TIME, 0, 1)
```

Otherwise output is binary full-on when valve is open.

## 5.3 PWM duty-cycle realization

In PWM mode, each control interval `T_ctrl` turns command duty `d` into a pulse of length:

```text
T_pulse_raw = d * T_ctrl
```

Then quantized to physics step `dt_sim`:

```text
steps = round(T_pulse_raw / dt_sim)
T_pulse = steps * dt_sim
```

with a minimum of one physics step for small nonzero duty.

Within each control interval:

```text
binary_on = 1  if (time_since_control_update < T_pulse)
         = 0  otherwise
```

This binary signal then passes through valve delay/ramp logic to produce actual thrust level.

## 6. Force/Torque Consistency Across Stack

- High-level commands are produced by MPC.
- `ThrusterManager` converts commanded duty to actual per-thruster output.
- C++ `SimulationEngine` uses actual thruster levels with configured geometry (`position`, `direction`, `force`) to compute net body force/torque.
- Dynamics are propagated with RK4 and chosen gravity model.

## 7. Practical Notes

- The dynamic model is fully nonlinear in simulation (rotation matrix from quaternion, gyroscopic coupling, optional two-body gravity).
- MPC uses a linearized model for optimization, while the simulation engine executes the nonlinear plant.
- Quaternion normalization is an explicit constraint-enforcement step for numerical stability.
