# Control System Specifications

This document defines the mathematical models, state definitions, and functional requirements for the Universal 3D Satellite Controller.

## 1. State Space Definition

The system operates on a 13-dimensional state vector representing the full 6-DOF rigid body state.

### State Vector ($x$)
The state vector $x \in \mathbb{R}^{13}$ is defined as:

$$
x = \begin{bmatrix} 
p \\ 
q \\ 
v \\ 
\omega 
\end{bmatrix}
$$

Where:
*   $p \in \mathbb{R}^3$: Position in Inertial Frame (ECI or World).
*   $q \in \mathbb{R}^4$: Attitude Quaternion (scalar-last or scalar-first convention to be standardized in `Core`).
*   $v \in \mathbb{R}^3$: Linear Velocity in Inertial Frame.
*   $\omega \in \mathbb{R}^3$: Angular Velocity in Body Frame.

### Control Vector ($u$)
The control vector $u \in \mathbb{R}^{3 + N}$ is a composite of continuous torques and discrete thruster forces:

$$
u = \begin{bmatrix} 
u_{rw} \\ 
u_{thrusters} 
\end{bmatrix}
$$

Where:
*   $u_{rw} \in \mathbb{R}^3$: Torque commands for Reaction Wheels (Body Frame).
*   $u_{thrusters} \in \mathbb{R}^N$: Force commands (PWM or continuous) for $N$ generic thrusters.

## 2. Mathematical Model

### Linearized Discrete Model
For the Model Predictive Control (MPC) formulation, we utilize a linearized discrete-time state-space model:

$$
x_{k+1} = A x_k + B u_k
$$

*   $A \in \mathbb{R}^{13 \times 13}$: State transition matrix (Jacobian of system dynamics $\frac{\partial f}{\partial x}$).
*   $B \in \mathbb{R}^{13 \times (3+N)}$: Control input matrix.

### Universal B-Matrix Requirement
The system must be "Universal," meaning the control effectiveness matrix $B$ is **not hardcoded**. It must be constructed at runtime.

For each thruster $i$ from $1$ to $N$:
1.  **Input**: Position $r_i$ (relative to CoM) and Force Direction $d_i$ (unit vector).
2.  **Force Contribution**: $F_i = d_i \cdot u_i$.
3.  **Torque Contribution**: $\tau_i = r_i \times d_i \cdot u_i$.

The matrix $B$ is populated such that input $u_i$ maps to the corresponding force and torque rows in the state derivative $\dot{x}$.

```cpp
// Pseudocode Concept
for (const auto& thruster : thrusters) {
    Vector3 force = thruster.direction;
    Vector3 torque = cross_product(thruster.position, thruster.direction);
    // Append column to B matrix
}
```

## 3. Configuration Interface

The `ConfigLoader` module must ingest the following parameters to fully define the vehicle for the `Dynamics` module.

### Inputs
*   **Mass Properties**:
    *   `mass` (scalar, kg)
    *   `inertia` (3x3 matrix or diagonal vector, $kg \cdot m^2$)
*   **Actuators**:
    *   `reaction_wheels`: Configuration for max torque and saturation.
    *   `thrusters`: A list of objects, where each object contains:
        *   `id`: Unique identifier.
        *   `position`: $[x, y, z]$ relative to Center of Mass.
        *   `direction`: $[x, y, z]$ unit vector of thrust.
        *   `max_thrust`: scalar (Newtons).

## 4. Requirements Traceability

| ID | Requirement | Implementation Module |
|----|-------------|-----------------------|
| REQ-01 | Support 13-state vector (Quaternions). | `Core` / `Dynamics` |
| REQ-02 | Dynamically generating B-matrix from YAML. | `Dynamics` |
| REQ-03 | Hybrid control (RW + Thrusters). | `Control` (MPC Cost Function) |
| REQ-04 | Real-time performance (Solve < 10ms). | `Control` (OSQP) |
