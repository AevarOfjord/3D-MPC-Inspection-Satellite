# V5.0 Autonomy Update Documentation

## 1. Overview
The **V5.0 Autonomy Update** introduces global path planning and advanced trajectory tracking capabilities to the satellite control simulation. This update enables the satellite to autonomously navigate complex environments, avoid obstacles, and execute smooth trajectories using a Model Predictive Control (MPC) tracking formulation.

## 2. Architecture

The autonomy stack is composed of three main layers that interact with the core simulation loop:

1.  **Planning Layer (`RRTStarPlanner`)**:
    *   Generates collision-free geometric paths (waypoints) from start to goal.
    *   Uses RRT* (Rapidly-exploring Random Tree Star) for asymptotic optimality.
    *   Handles spherical obstacles dynamically defined in the mission configuration.

2.  **Trajectory Layer (`TrajectoryGenerator`)**:
    *   Converts geometric waypoints into a time-parameterized trajectory state $x(t) = [p(t), v(t)]$.
    *   Uses cubic spline interpolation to ensure $C^2$ continuity.
    *   Resamples the path to respect velocity constraints (maximum velocity).

3.  **Control Layer (MPC Tracking)**:
    *   The existing Linearized MPC controller now tracks the time-varying reference provided by the trajectory generator.
    *   The simulation loop feeds independent reference states for each horizon step based on the planned trajectory $x_{ref}(t+k\Delta t)$.

## 3. Key Features

### 3.1 RRT* Path Planner
*   **File**: `src/satellite_control/planning/rrt_star.py`
*   **Algorithm**: RRT* (Karaman & Frazzoli, 2011)
*   **Configuration**:
    *   `max_iter`: Maximum number of nodes to sample (limits computation time).
    *   `step_size`: Maximum distance between nodes (controls path resolution).
    *   `search_radius`: Radius for rewiring neighbors (optimization range).
*   **Obstacle Avoidance**: Checks for line-segment intersections with defined spherical obstacles.

### 3.2 Trajectory Generation
*   **File**: `src/satellite_control/planning/trajectory_generator.py`
*   **Method**:
    1.  **Waypoint Extraction**: Takes raw path from RRT*.
    2.  **Spline Interpolation**: Fits cubic splines to x, y, z coordinates parameterized by cumulative distance.
    3.  **Velocity Profiling**: Maps distance to time using a trapezoidal or constant velocity profile to strictly enforce `max_velocity`.

### 3.3 Simulation Integration
*   **Dynamic Replanning**:
    *   The simulation exposes a `replan_path()` method.
    *   Replanning is triggered automatically (conceptually) or manually via the UI when targets/obstacles change.
    *   **Caching**: To ensure performance, the planner caches the last solution and only re-computes if the "problem signature" (start, target, obstacles) changes.

## 4. Dashboard & Telemetry

### 4.1 Path Visualization
*   **3D Viewport**: The planned path is visualized as a **Dashed Yellow Line** connecting the start to the target.
*   **Obstacles**: Rendered as red wireframe spheres.
*   **Trajectory Tracking**: The "Ghost Target" (Green Cube) moves along this yellow line, guiding the satellite.

### 4.2 Controller Metrics
Real-time telemetry now includes specific metrics for the controller's performance:
*   **Solve Time (ms)**: Time taken by the MPC solver (OSQP) to find the optimal control input.
*   **Position Error (m)**: Euclidean distance between current satellite position and the active reference point.
*   **Angle Error (deg)**: Geodesic distance (principal angle) between current orientation and target orientation.

### 4.3 Charts
New charts have been added to the bottom panel to track these metrics over time:
*   **Position Error** (Blue)
*   **Angle Error** (Purple)
*   **Velocity** (Green)
*   **Solve Time** (Yellow) - *New in V5.0*

## 5. Usage (Interactive Editor)
1.  Open the **Mission Control** dashboard.
2.  Enable **"Edit Mode"** toggle in the top-right settings.
3.  Use the **Editor Panel** to:
    *   Add/Move **Obstacles** to block the direct path.
    *   The system will automatically replan and display the new path (Yellow Line).
4.  Watch the satellite autonomously execute the maneuver.

## 6. Mission Sequencing
The update introduces a `MissionSequencer` capable of executing multi-stage missions defined in YAML.

### 6.1 Supported Stages
*   `goto`: Navigate to a 3D coordinate using RRT* planning.
*   `orbit`: Execute a circular orbit around a center point (legacy DXF mode logic).
*   `return`: Autonomously return to the starting position (0,0,0).

### 6.2 Example Configuration
```yaml
name: "Patrol Mission"
stages:
  - type: goto
    target: [2.0, 2.0, 0.0]
    hold_time: 5.0
  - type: orbit
    center: [0.0, 0.0, 0.0]
    radius: 3.0
    duration: 30.0
  - type: return
```

## 7. Validation Scenarios
The following scenarios verify the V5.0 capabilities:

1.  **"Patrol"**: Validates the **Sequencer** by executing a Goto -> Orbit -> Return mission.
    *   Config: `config/missions/mission_sequence_demo.yaml`
2.  **"The Maze"**: Validates **RRT* Obstacle Avoidance** by placing multiple spheres between the satellite and target.
    *   Config: `config/missions/maze.yaml`

