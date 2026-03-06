# `cpp_linearized_rti_osqp`

See [README.md](./README.md) in this folder for shared notation and the common path-following problem.

## 1. Controller Identity

- Profile ID: `cpp_linearized_rti_osqp`
- Solver backend: CasADi linearization in Python, sparse QP solve in C++ with OSQP
- Controller family: RTI-SQP with frozen-horizon affine model
- Intended comparison role: cheapest member of the OSQP family and the most aggressively linear approximation

## 2. Problem This Controller Solves

This controller solves the same horizon-stacked path-following QP structure as the other OSQP profiles, but it approximates the full horizon using one frozen linearization rather than a separately updated model at every stage.

The subproblem can be written as

```text
min sum ell_k(x_k, u_k; ref_k) + ell_N(x_N)
```

subject to

```text
x_{k+1} = A_f x_k + B_f u_k + d_f
x_0 = x_meas
```

where the same frozen affine model is reused across the horizon for that control step, and may even be reused across multiple control steps depending on the refresh interval.

Physically, the intent is unchanged:

- follow the path
- regulate progress along the path
- align attitude and suppress unwanted angular motion
- keep control effort and switching moderate

## 3. Mathematical Transcription Used Here

### Decision Variables

The decision variables are the same stacked state/control sequence:

```text
z = [x_0, ..., x_N, u_0, ..., u_{N-1}]
```

### Dynamics

The critical approximation is the linearization policy. Instead of computing `A_k`, `B_k`, and `d_k` separately at every stage, the controller computes one affine model from the current step anchor and injects the same model across the horizon:

```text
x_{k+1} = A_f x_k + B_f u_k + d_f
```

That makes the horizon mathematically cheaper, but it also means model fidelity degrades as the predicted trajectory moves away from the point where the model was frozen.

The implementation also supports a `freeze_refresh_interval_steps`, so the frozen model may be reused across several MPC updates.

### Cost Structure

The cost structure is the same C++ RTI-QP family used by the hybrid and nonlinear OSQP profiles:

- path-position penalty
- progress and `s` regulation
- attitude and angular-rate terms
- effort and smoothness
- thruster-pair and optional fuel bias
- terminal shaping

The key difference is not the cost family, but the much stronger simplification in the dynamics model.

### Constraints

Constraints match the other OSQP RTI controllers:

- affine dynamics equalities
- initial-state equality
- state and control bounds
- control-horizon tying
- path-speed bounds

### Reference Construction

The path/attitude reference generation is unchanged. The reference is still nonlinear in the path parameter, even though the prediction model is frozen.

## 4. How The Solve Proceeds Each Control Step

1. Read and augment the measured state.
2. Decide whether the frozen linearization cache should be refreshed.
3. If refresh is needed, evaluate one affine model through CasADi at the current anchor.
4. Reuse that same linearization across the full horizon.
5. If refresh is not needed, reuse the cached frozen model from earlier steps.
6. Assemble and solve one sparse OSQP QP.
7. Return the first physical control move.

This makes the controller the most computationally lightweight of the 3 OSQP profiles.

## 5. Why This Formulation Exists

This profile exists as the extreme low-cost approximation in the comparison set. It answers the practical question:

How much performance can be retained if the satellite controller is forced to operate with a very cheap local model update?

That matters for small inspection satellites because onboard compute is often scarce, and a lower-order or less frequently updated controller may still be attractive if performance loss is modest.

## 6. Scientific Comparison Notes

### What makes it comparable

- Same RTI-QP architecture and OSQP backend family as the hybrid/nonlinear profiles
- Same control/state meaning and same fairness baseline parameter block
- Same reference generation and same mission runtime layer

### What makes it not perfectly identical

- It uses the most aggressive approximation of the nonlinear dynamics among the six profiles
- Model mismatch grows across the horizon because one linearization is reused everywhere
- If the freeze interval is greater than 1, the model can also lag in time across controller updates

### Main paper caveat

This controller is best interpreted as the computational lower bound of the RTI family, not as a high-fidelity nonlinear benchmark.

## 7. When This Controller Is Likely Best

Likely strongest when:

- onboard compute is extremely constrained
- the path segment is locally smooth enough that one frozen model remains adequate
- the mission values deterministic low solve time over modeling fidelity

Likely weakest when:

- the path geometry or attitude reference changes rapidly across the horizon
- the vehicle operates far from the linearization point
- the study is trying to measure best achievable control quality rather than cheapest feasible control

## 8. Implementation Anchors

- [`../controller/linear/python/controller.py`](../controller/linear/python/controller.py)
- [`../controller/shared/python/control_common/mpc_controller.py`](../controller/shared/python/control_common/mpc_controller.py)
- [`../controller/linear/cpp/linear_sqp_controller.cpp`](../controller/linear/cpp/linear_sqp_controller.cpp)
- [`../controller/shared/python/control_common/codegen/satellite_dynamics.py`](../controller/shared/python/control_common/codegen/satellite_dynamics.py)
