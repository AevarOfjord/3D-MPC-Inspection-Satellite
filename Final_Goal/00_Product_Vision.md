# Final Product Specification: Autonomous Inspection Satellite Control System

## 1. Executive Summary
**Goal**: Develop a commercial-grade, "black-box" control algorithm for autonomous inspection satellites.
**Target Customer**: Aerospace manufacturers and satellite operators.
**Value Proposition**: A universal, configurable control kernel that guarantees precision, safety, and efficiency for hull inspection missions, adaptable to *any* satellite chassis design without code changes.

---

## 2. Key Pillars of the Final Version

### A. Universal Adaptability ("Configure, Don't Code")
The core algorithm must accept a "Vehicle Definition File" (YAML) and instantly adapt behavior. It must not require recompilation for hardware changes.
*   **Configurable Parameters**:
    *   Mass Properties (Mass, Inertia Tensor, Center of Mass).
    *   Thruster Layout (Positions, Orientations, Max Thrust, Min Impulse).
    *   Reaction Wheels (Max Torque, Max Speed, Axis alignment).
    *   Geometric Envelope (Visual mesh, Collision bounding box).

### B. High-Precision Hybrid Control
*   **Actuation**: seamless sensor fusion of Reaction Wheels (RW) for fine pointing and Thrusters for translation/slew.
*   **Performance**: "Zero-Overshoot" trajectory execution (critical for proximity ops).
*   **Constraint Satisfaction**: Strict adherence to hardware limits (e.g., RW saturation).

### C. Active Fault Tolerance
*   **Scenario**: "Thruster #4 Valve Stuck Closed" or "RW #2 Failure".
*   **Response**: The MPC instantly updates its internal model (B-matrix), re-optimizing the control allocation to compensate using remaining healthy actuators. No abort required if physical authority remains.

### D. The "Killer Feature": Autonomous Structural Inspection
*   **Input**: 3D Reference Model of the Target Satellite (e.g., `.obj`, `.stl`).
*   **Process**:
    1.  **Surface Analysis**: Discretize target hull into inspection zones.
    2.  **Viewpoint Generation**: Calculate optimal observation points (distance + angle) for each zone.
    3.  **Path Optimization**: Solve the "Traveling Salesperson Problem" constrained by satellite dynamics (fuel optimal).
*   **Output**: An executable trajectory that guarantees 100% visual coverage of the target.

### E. Commercial-Grade Safety & Verification
To sell to aerospace, "it works in my sim" isn't enough. We need proof.
*   **Keep-Out Zone Enforcement**: Mathematically guaranteed geometric exclusion zones (ellipsoids) around the target.
*   **Passive Safety "Safe-Return"**: If control is lost, the last commanded trajectory ensures drift *away* from the target.
*   **Automated Validation Report**: A generated PDF report after every config change, running 1,000 Monte Carlo sims to prove the config is stable (e.g., "99.99% success rate with ±5% mass uncertainty").

### F. Developer Experience
*   **Telemetry Stream API**: Expose a ZMQ/WebSocket stream of state data so customers can plug it into their own ground station software (Cosmos, OpenMCT).
*   **Headless "Cloud" Mode**: Optimize for running on AWS Batch to train policies or valid configurations in parallel.

### G. Flight Readiness & IP Protection
*   **C++ Code Generation**: Flight computers don't run Python. The system must export the verified MPC logic to pure, static C++ (using `cppad` or `acados`) that can compile on an ARM Cortex-M or FPGA.
*   **Hardware-in-the-Loop (HIL) Bridge**: A module to treat the simulation as a "Mock Hardware" device, talking over serial/CAN to a real flight computer on the desk.
*   **Licensing System**: A method to deliver the control core as a compiled binary (shared object/DLL) to protect your IP, preventing clients from reverse-engineering the source.

---

## 3. System Architecture

The software is divided into three distinct layers:

### Layer 1: The Planner (Strategic)
*   *Role*: "Where should I go?"
*   *Input*: Target 3D Model.
*   *Output*: Waypoints and Orientations (State References).
*   *Tech*: Computational Geometry, Graph Search (A*/RRT).

### Layer 2: The MPC Controller (Tactical)
*   *Role*: "How do I get there safely?"
*   *Logic*: Linear MPC (Linearized Model Predictive Control).
*   *Features*: Prediction Horizon (N=50+), Collision Avoidance constraints, Fuel minimization cost function.

### Layer 3: The Allocator & Driver (Execution)
*   *Role*: "Which hardware do I fire?"
*   *Logic*: Maps required Forces/Torques to PWM signals for thrusters and Voltage/Current for RWs.
*   *Safety*: Enforces hardware interlocks.
