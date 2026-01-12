# Inspection Planning Guide

Automatically generate inspection missions from CAD meshes.

## Workflow

```
STL/OBJ File → MeshAnalyzer → Zones → Viewpoints → TSP → Mission
```

## Quick Example

```cpp
#include "planner/InspectionPlanner.h"

InspectionPlanner planner;
Vector3 start_pos(10, 0, 0);  // Inspector starting position

InspectionParams params;
params.standoff_distance = 3.0;  // 3m from surface
params.zone_size = 2.0;          // ~2m² per zone
params.hold_time = 2.0;          // 2s at each viewpoint

auto mission = planner.plan_mission("target_hull.stl", start_pos, params);
// mission.waypoints now contains ordered viewpoints
```

## Supported File Formats

| Format | Notes |
|--------|-------|
| **STL (ASCII)** | `solid name ... endsolid` |
| **STL (Binary)** | 80-byte header + triangle data |
| **OBJ** | Vertices + faces, auto-triangulates quads |

## Zone Generation

The mesh surface is divided into inspection zones using spatial grid clustering:

```cpp
MeshAnalyzer mesh;
mesh.load("hull.stl");

// Create zones (~2m² each)
auto zones = mesh.create_zones(2.0);
// Each zone has: centroid, normal, area, triangle indices
```

## Viewpoint Generation

Viewpoints are placed at standoff distance along zone normals:

```cpp
auto viewpoints = mesh.generate_viewpoints(zones, 3.0);
// Each viewpoint has: position, look_at, zone_id
```

## Path Optimization

TSP solver finds shortest tour:

```cpp
auto order = mesh.solve_tsp(viewpoints, start_position);
// order = [4, 2, 0, 3, 1, ...]  (visit sequence)
```

## Output

The `plan_mission()` function returns a complete `MissionConfig`:

```yaml
mission:
  name: "Auto-Generated Inspection"
  waypoints:
    - position: [5.0, 2.1, 0.3]
      attitude: [0.707, 0.0, 0.707, 0.0]  # Look at zone
      hold_time: 2.0
    - position: [4.8, 2.5, 1.1]
      ...
```

## Integration with Simulation

```cpp
// Generate mission
auto mission = planner.plan_mission("target.stl", start);

// Run with MPC
MissionManager mgr(mission);
while (!mgr.is_complete()) {
    auto target = mgr.update(time, state);
    auto u = mpc.compute_control(state, target);
    physics->step(u);
}
```
