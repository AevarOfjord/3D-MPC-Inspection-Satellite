# Path-Following MPC (MPCC) Proposal for Inspection Satellite

## A) Explanation: Tracking vs. Path-Following MPC

**Target-Tracking MPC (Current)** works by penalizing the error between the satellite's state $x_k$ and a specific reference state $x_{ref,k}$ at a specific time $k$.
*   **Concept**: "Be exactly at $[10m, 0m]$ at $t=5.0s$."
*   **Behavior**: If the satellite lags behind (e.g., due to a disturbance), it will burn extra fuel to "catch up" to the reference point, which has moved on. This can lead to aggressive/unnecessary maneuvering.
*   **Analogy**: A dog running after a mechanical rabbit moving at a fixed speed.

**Path-Following MPC (MPCC)** separates the "where" from the "when". It works by penalizing the distance to the *geometric path* and separately managing the progress along that path.
*   **Concept**: "Be on the circle of radius $10m$, moving at roughly $0.5 rad/s$."
*   **Behavior**: If the satellite lags, it simply accelerates to resume the desired speed, but does not try to "catch up" to a virtual point that is far ahead. It prioritizes staying *on* the path (contouring error) over being at a specific angle at a specific split-second (lag error).
*   **Mechanism**: We introduce a "path progress" variable $s$ (e.g., phase angle) as an **optimization variable** in the MPC. The solver decides $s_{k+1}$ alongside inputs $u_k$.

## B) Mathematical Formulation (Linearized MPCC)

We propose a **Linearized MPCC** formulation to keep the problem convex (QP), allowing efficient solution with your existing **OSQP** stack.

### 1. Extended State & Control
We augment the state decision vector with the path parameters.
*   **Original State** ($n_x=16$): $x = [r, v, q, \omega, \omega_{rw}]^T$
*   **Augmented State** ($n_x=17$): $\hat{x} = [x, s]^T$, where $s$ is the path parameter (angle in radians).
*   **Augmented Control** ($n_u+1$): $\hat{u} = [u, v_s]^T$, where $v_s$ is the "virtual control" (rate of change of $s$).

### 2. Dynamics
The path dynamics are simple integrators, added to your CW/Hill dynamics:
$$ s_{k+1} = s_k + v_{s,k} \cdot \Delta t $$
$$ x_{k+1} = A x_k + B u_k $$

### 3. Path & Linearization
**Path Definition** (Circle in Hill Frame):
$$ p(s) = \begin{bmatrix} R \cos(s) \\ R \sin(s) \\ 0 \end{bmatrix} $$
**Tangent Vector**:
$$ t(s) = \frac{\partial p}{\partial s} = \begin{bmatrix} -R \sin(s) \\ R \cos(s) \\ 0 \end{bmatrix} $$

Since $p(s)$ is nonlinear, we cannot put $\| r - p(s) \|^2$ directly into a QP. We use **linearization** around a predicted reference $\bar{s}_k$ (generated from the previous solution or a simple guess):
$$ p(s_k) \approx p(\bar{s}_k) + t(\bar{s}_k) (s_k - \bar{s}_k) $$

### 4. Cost Function
The cost minimizes Contouring Error ($e^C$) and Lag Error ($e^L$), plus Control Effort.

**Contouring Error** (Distance to tangent):
Minimizing $\| r_k - p(s_k) \|^2$ with the linearized path:
$$ J_{path} = \sum_{k=0}^{N} \| r_k - (p(\bar{s}_k) + t(\bar{s}_k)(s_k - \bar{s}_k)) \|_Q^2 $$
This is a standard quadratic form $(Hx + g)^T Q (Hx + g)$ which fits OSQP.

**Progress / Speed Tracking**:
To ensure the satellite moves, we track a reference path velocity $\dot{s}_{ref}$:
$$ J_{progress} = \sum_{k=0}^{N-1} w_{vel} (v_{s,k} - \dot{s}_{ref})^2 $$

**Total Cost**:
$$ J = J_{path} + J_{progress} + \| u \|_R^2 + \| \Delta u \|_{R_{\Delta}}^2 $$

### 5. Constraints
*   **State/Control Bounds**: Same as before.
*   **Path Constraints**: $0 \le v_{s,k} \le v_{s,max}$ (can stop, but max speed limited).
*   **Monotonicity**: $v_{s,k} \ge 0$ (optional, prevents moving backward on path).

## C) Implementation Plan

We recommend a "Minimum Viable Model" approach that extends your existing C++ controller.

### Phase 1: Core C++ Changes
1.  **Modify `MPCParams`**: Add `mode` (TRACKING vs. PATH_FOLLOWING), `path_radius`, `target_speed`.
2.  **Modify `MPCControllerCpp`**:
    *   Dynamic sizing: Set `nx_ = 17`, `nu_ += 1` if mode is PATH.
    *   Update `build_A_matrix`: Add row/col for $s_{k+1} = s_k + v_s dt$.
    *   Update `build_P_matrix`: Add weight for $(v_s - v_{ref})^2$.
    *   **New Function**: `linearize_path_constraints()`.
        *   Takes current estimate of $s$ (e.g., propagated by $v_{ref}$).
        *   Computes $p(\bar{s})$ and $t(\bar{s})$.
        *   Updates the $q$ (linear cost) and $P$ (quadratic cost) vectors to penalize deviation from this linearized line.
3.  **Update Bindings**: Expose new params to Python.

### Phase 2: Python Integration
1.  Update `mpc_controller.py` to handle the variable state size in returned results.
2.  Update `Mission` logic:
    *   Instead of generating a full $x_{ref}[k]$ trajectory, the mission simply sets the `path_radius` and `path_mode = True`.
    *   The MPC handles the rest autonomously.

### Phase 3: Testing
1.  **Zero-Disturbance**: Verify it circles strictly at $R$ and $\omega_{ref}$.
2.  **Disturbance Rejection**: Manually push satellite off-path; verify it returns to the circle smoothly rather than racing to a specific phase angle.

## D) Complexity & Feasibility Analysis

*   **Complexity**:
    *   Adding 1 state and 1 control variable to OSQP is **negligible**. Solve time will increase by $< 5\%$.
    *   **Linearization**: Computing sin/cos for N steps (N=50) is computationally free ($< 0.1ms$).
    *   **Convexity**: By linearizing the path constraint around a reference guess trajectory (which is trivially just moving forward at constant speed), the problem remains a **Convex QP**.
*   **Recommendation**:
    *   **Do NOT use NLP (Ipopt/Acados)** yet. It introduces massive complexity (new dependencies, compilation, interface).
    *   **Use Linearized MPCC** within your existing OSQP framework. It is robust enough for circular references in Hill frames.

---

## Appendix: Pseudocode (C++ Logic)

```cpp
// Inside MPCControllerCpp::get_control_action()

// 1. Initial Guess for s trajectory (warm start or heuristic)
// If we had a previous solution, shift it. Otherwise:
double s_guess[N];
s_guess[0] = x_current_augmented[16]; // Current s
for(int k=0; k<N; k++) {
    s_guess[k+1] = s_guess[k] + mpc_params_.target_speed * dt;
}

// 2. Build QP Matrices (A, P, q, l, u)
if (mpc_params_.mode == PATH_FOLLOWING) {
    // Standard dynamics for first 16 states...
    // ...
    
    // Add path dynamics: s_next = s + v_s * dt
    // A_matrix row for s: 1.0 * s_k + dt * v_s_k
    
    // 3. Linearized Path Cost
    for(int k=0; k<=N; k++) {
        double sk = s_guess[k];
        Vector3d p_ref = get_circle_pos(sk);    // [R cos(sk), R sin(sk), 0]
        Vector3d t_ref = get_circle_tangent(sk);// [-R sin(sk), R cos(sk), 0]
        
        // We want to minimize || pos_k - (p_ref + t_ref * (s_k - sk)) ||^2
        // This expands to quadratic terms in pos_k and s_k
        // Update P (Hessian) and q (Gradient) accordingly
    }
}

// 4. Solve OSQP
solve();

// 5. Extract Control
u_applied = result.u.head(nu_original); // Thrusters + RW
// result.u(last) is v_s (virtual speed), useful for debugging/logging
```
