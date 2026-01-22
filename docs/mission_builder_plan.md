# Mission Builder Unification Plan

Date: 2026-01-22

## Goals
- Replace separate mission modes with a single, unified mission builder in the web UI.
- Model missions as a sequence of path segments (transfer + scan) that are optimized as one trajectory.
- Use a high-fidelity orbital environment (Earth, ISS, Starlink) with static, offline orbital data.
- Allow full user freedom: place the satellite anywhere in ECI, generate paths, and edit via spline handles.

## Locked Decisions
- High-fidelity orbit dynamics.
- Static orbital data (TLE snapshot, no live updates).
- Planning frame for scan paths: LVLH relative to target.
- Path planner: full optimization-based MPC for the entire mission path.
- User path edits are soft constraints.
- Satellite placement and mission start pose: ECI.
- Scan uses mesh-based standoff and collision checks.
- Sensor pointing: +Y or -Y body axis must face the target.
- Spiral scan overlap: 25% with 60 deg FOV.
- Spiral axis is user-defined.

## Conceptual Model
### Two Layers
1) Physics Layer (truth)
   - Earth and orbital dynamics
   - ISS + Starlink positions from TLE snapshot
2) Planning Layer (authoring)
   - Frozen epoch snapshot
   - User places satellite and authors mission segments
   - Planner generates a continuous, optimized trajectory

### Mission = Sequence of Segments
- Transfer: point-to-point travel in orbit
- Scan: approach + spiral scan around target mesh + depart
- Optional Hold: maintain pose for time window

## Data Model (Draft)
```yaml
mission:
  epoch: "2026-01-22T00:00:00Z"
  start_pose:
    frame: ECI
    position: [x, y, z]
    orientation: [qx, qy, qz, qw]
  segments:
    - type: transfer
      end_pose:
        frame: ECI
        position: [x, y, z]
        orientation: [qx, qy, qz, qw]
      constraints:
        speed_max: 0.5
        accel_max: 0.1
    - type: scan
      target_id: "ISS"
      target_pose: { frame: ECI, position: [...], orientation: [...] }
      scan:
        frame: LVLH
        axis: "+Z" | "+Y" | "custom"
        standoff: 10.0
        overlap: 0.25
        fov_deg: 60
        pitch: "auto"
        revolutions: 4
        direction: "CW"
      constraints:
        speed_max: 0.2
        accel_max: 0.05
  overrides:
    spline_controls:
      - position: [x, y, z]
        weight: 0.7
  obstacles:
    - position: [x, y, z]
      radius: 1.0
```

## Scan Spiral Generator (LVLH)
### Pitch Rule (auto)
```
footprint = 2 * standoff * tan(30 deg)
pitch = 0.75 * footprint   # 25% overlap
```
### Orientation Rule
- The spacecraft body +Y or -Y axis points to the target at all scan points.
- Optional: roll stabilization to avoid uncontrolled spin about the pointing axis.

## Path Planning Pipeline
1) Snapshot orbit state at the mission epoch (ECI).
2) Expand segments:
   - Transfer: target pose in ECI
   - Scan: generate LVLH spiral around target mesh, transform to ECI
3) Generate an initial guess path (stitched waypoints).
4) Run full MPC optimization for the entire mission:
   - High-fidelity orbital dynamics
   - Mesh-based collision avoidance
   - Pointing constraints (+Y/-Y to target)
   - Soft spline constraints (user edits)
5) Output a time-parameterized trajectory (positions + attitudes).

## UI Layout (Unified Builder)
### Left Panel: Mission Context
- Epoch selector
- Start pose (ECI)
- Obstacles + constraints

### Center: Orbit Scene
- Earth, ISS, Starlink, user satellite
- Click object to create scan segment
- Path and constraints visualized

### Right Panel: Segment Stack + Tools
- Add segment: Transfer / Scan / Hold
- Edit segment parameters
- Path preview stats + solver status
- Spline editing tools (add/drag/delete control points)

## Spline Edits (Soft Constraints)
- User control points influence the optimizer but are not hard requirements.
- If a control point is unsafe (collision/constraints), the solver will deviate.
- UI should show a warning and display deviation magnitude.

## Implementation Phases
### Phase 0: Planning Artifacts
- Document the unified mission schema
- Define path segment types and constraints
- Add a README for planner inputs/outputs

### Phase 1: UI Refactor
- Remove mode-specific builders
- Implement segment stack UI
- Implement spline editing controls for the unified path

### Phase 2: Orbital Environment
- Integrate Earth and TLE snapshot data
- Spawn ISS and Starlink objects in ECI
- Add UI selection + metadata display

### Phase 3: Scan Generator
- LVLH spiral generator around mesh
- Pointing constraints in preview
- Mesh distance + standoff computation

### Phase 4: Full MPC Planner
- Global optimization for full mission path
- Soft spline constraints integrated into cost
- Mesh collision avoidance integrated

### Phase 5: Validation
- Bench tests for planner outputs
- Visual validation in UI
- Export/import mission JSONs

## Open Questions
- Orbit propagator choice (SGP4 vs higher-order)
- Planner runtime budget and solver library
- Mesh acceleration structure for collision queries
- Attitude control coupling (position-only vs full state)

## Success Criteria
- User can author any mission in a single UI flow.
- Planner produces a single continuous trajectory for mixed segments.
- Scan paths respect standoff, overlap, and pointing constraints.
- Spline edits are supported without breaking safety constraints.

## Status (Repo)
- Added unified mission schema: `src/satellite_control/mission/unified_mission.py`
- Added example JSON: `docs/unified_mission_example.json`
- Added UI types: `ui/src/api/unifiedMission.ts`
- Added backend endpoints: `/mission_v2`, `/save_mission_v2`, `/saved_missions_v2`
- Wired UI builder segment stack + save/load to `mission_v2` endpoints
- Added spline controls UI wired to `overrides.spline_controls`
- Added orbit snapshot layer (Earth + ISS/Starlink) to plan scene
- Orbit objects clickable; selecting an object sets scan target_id
- Selection feedback: highlighted orbit objects + selected target shown in scan segment editor
- Orbit selection now focuses camera and prefills scan standoff/pitch
- Pitch input shows auto state (placeholder + computed value) until user edits; AUTO toggle resets
- Unified missions compile into a single MPCC path on push (v2), using mesh/cylinder scan fallback
- Unified missions now include obstacles; compiler inserts simple detours around spheres
- CLI now accepts unified mission JSON (segments/start_pose) and compiles path
