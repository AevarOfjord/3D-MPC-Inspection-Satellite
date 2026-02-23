# Physics Engine Mathematics (Code-Accurate)

This document describes the physics that is currently implemented in:

- `src/cpp/sim/simulation_engine.cpp`
- `src/cpp/sim/orbital_dynamics.cpp`
- `src/python/runtime/thruster_manager.py`
- `src/python/simulation/cpp_backend.py`

It is intentionally implementation-specific.

## 1. Runtime Physics Path

The runtime plant is the C++ simulation engine (`cpp._cpp_sim`), wrapped by `CppSatelliteSimulator`.

- Python computes actuator commands.
- `ThrusterManager` converts commanded thruster pattern to actual per-thruster output.
- C++ `SimulationEngine` propagates the 16-state plant with RK4.

`src/python/physics/orbital_dynamics.py` is not the active runtime integrator path for this engine.

## 2. State and Inputs

The propagated C++ state is:

```text
x = [r(3), q(4), v(3), w(3), wr(3)]
  = [x,y,z, qw,qx,qy,qz, vx,vy,vz, wx,wy,wz, wrx,wry,wrz]
```

- `r`: relative position
- `q`: quaternion, scalar-first `[w,x,y,z]`
- `v`: relative velocity
- `w`: body angular velocity
- `wr`: reaction-wheel speeds (3 channels in plant state)

Step inputs:

- `thruster_cmds[i]` (one value per configured thruster, typically in `[0,1]`)
- `rw_torques[i]` (N.m, only first 3 channels used by plant torque/speed equations)

## 3. Continuous-Time Dynamics

## 3.1 Kinematics

```text
r_dot = v
```

Quaternion derivative (scalar-first convention):

```text
qw_dot = -0.5*(qx*wx + qy*wy + qz*wz)
qx_dot =  0.5*(qw*wx + qy*wz - qz*wy)
qy_dot =  0.5*(qw*wy - qx*wz + qz*wx)
qz_dot =  0.5*(qw*wz + qx*wy - qy*wx)
```

## 3.2 Translation

```text
v_dot = a_gravity + a_thrust
```

Body-frame thruster force sum:

```text
F_body = sum_i (d_i * F_i * u_i)
```

Rotate to inertial/relative frame:

```text
F_world = R(q) * F_body
a_thrust = F_world / m
```

where:

- `d_i`: configured thruster direction vector
- `F_i`: configured max force
- `u_i`: actual thruster output level from `ThrusterManager`

## 3.3 Rotation

Thruster torque:

```text
tau_thr = sum_i (r_i - r_com) x (d_i * F_i * u_i)
```

Reaction torque from wheels (plant implementation):

```text
tau_body(i) += -rw_torques(i),  i = 0..min(2, rw_torques.size-1)
```

Rigid-body angular dynamics:

```text
I w_dot + w x (I w) = tau_body
w_dot = I^{-1}(tau_body - w x (I w))
```

with diagonal inertia from config.

## 3.4 Wheel-Speed Dynamics (Plant)

For plant state indices `13..15`:

```text
wr_dot,i = rw_torque(i) / I_rw,i
```

Implementation details:

- Computed for `i=0,1,2` only.
- If configured `rw_inertia[i]` is missing or too small, fallback inertia `1e-3` is used.

## 4. Gravity Models

`SimulationEngine` supports two gravity modes:

## 4.1 Two-body differential gravity (`use_nonlinear = true`)

Absolute gravity:

```text
a(r) = -mu * r / |r|^3
```

Relative acceleration:

```text
a_rel = a(target + r_rel) - a(target)
```

Target orbit is propagated as circular:

```text
phase_{k+1} = phase_k + n*dt
r_target = [R cos(phase), R sin(phase), 0]
v_target = [-nR sin(phase), nR cos(phase), 0]
```

## 4.2 CW/Hill (`use_nonlinear = false`)

```text
ax =  3 n^2 x + 2 n vy
ay = -2 n vx
az = -n^2 z
```

## 4.3 Default runtime setting

Via `cpp._cpp_sim.SimulationEngine` binding defaults and `CppSatelliteSimulator` constructor usage, runtime currently defaults to:

- `use_nonlinear = true` (two-body mode),
- Earth-like defaults for `mu` and target radius unless explicitly overridden at C++ constructor call.

## 5. Numerical Integration

The C++ engine uses fixed-step RK4:

```text
k1 = f(x_k)
k2 = f(x_k + dt/2 * k1)
k3 = f(x_k + dt/2 * k2)
k4 = f(x_k + dt   * k3)
x_{k+1} = x_k + dt/6 * (k1 + 2k2 + 2k3 + k4)
```

Quaternion normalization is applied at:

- intermediate RK states (`s2`, `s3`, `s4`),
- final updated state.

## 6. Thruster Command-to-Output Model (`ThrusterManager`)

## 6.1 PWM mode (`thruster_type == "PWM"`)

Within each control interval `T_ctrl`, duty command `d` is converted to a binary pulse:

```text
T_pulse_raw = d * T_ctrl
steps = round(T_pulse_raw / dt_sim)
T_pulse = steps * dt_sim
```

If `d > 0.01` and `steps == 0`, one minimum physics step is enforced.

Binary on/off then passes through valve-delay and optional ramp-up logic:

```text
t_open_valve  = t_open_cmd  + VALVE_DELAY
t_close_valve = t_close_cmd + VALVE_DELAY
```

When valve has opened:

```text
output = min(1, (t - t_open_valve)/THRUST_RAMPUP_TIME)   (if realistic physics and ramp > 0)
output = 1                                                (otherwise)
```

## 6.2 Continuous mode (`thruster_type == "CON"`)

Current implementation behavior:

- `thruster_actual_output = commanded_target` (direct pass-through),
- even when realistic physics is enabled (no separate valve delay/ramp shaping branch is applied).

## 7. Stack Consistency

- MPC/controller computes commanded actuators.
- `ThrusterManager` computes actual per-thruster output.
- `CppSatelliteSimulator` writes those levels into C++ engine commands.
- `SimulationEngine` computes force/torque from configured geometry and propagates state with RK4.

## 8. Important Implementation Limits

- Plant state is fixed to 16 elements with 3 wheel-speed states.
- Plant torque reaction currently uses first 3 RW torque channels directly in body XYZ (no arbitrary RW-axis projection in simulation engine).
- MPC linearization/controller may model richer RW geometry, but simulation plant follows the implementation above.
