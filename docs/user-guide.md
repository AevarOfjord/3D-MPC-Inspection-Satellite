# User Guide

Welcome to the Orbital Inspector Satellite Control system! This guide covers how to launch the application, plan missions via the web interface, execute simulations, and analyze your results.

---

## 1. Quick Start

### Option A: Standard Local Run

```bash
make run-app
```

Then open your browser to: `http://localhost:8000`

### Option B: macOS Zero-Terminal Launch

Double-click the `scripts/Start_Mission_Control.command` file from Finder to automatically start the background server and open your browser.

---

## 2. Main Web Workflow (The Planner)

The web UI is organized around a guided 5-step rail that takes you from concept to a runnable mission.

1. **Path Maker**: Define plane pairs, shape spirals with drag handles, and connect endpoints in 3D space.
2. **Transfer**: Set the satellite's starting pose, select a tangent spiral endpoint, and generate the transfer trajectory.
3. **Obstacles**: Place spherical keep-out zones by defining their position and radius. (Visual diagnostics).
4. **Path Edit**: Manually drag, add, or delete spline points to refine your path and resolve collision warnings.
5. **Mission Saver**: Validate your trajectory and save the mission to disk.

### Helpful Features

- **Auto-Recovery**: Drafts autosave every ~5 seconds. If you accidentally close the tab, the Planner will offer a one-shot restore card upon reload.
- **Onboarding Tour**: The first time you open the Planner, a 60-second interactive tour will guide you through the controls.
- **Keyboard Shortcuts**: Press `Ctrl/Cmd + K` to open the command palette, or `?` for the shortcut cheat sheet. Use `Ctrl/Cmd + 1..5` to quickly switch between app modes (Viewer, Planner, Runner, Data, Settings).
- **Workspace Backup**: Go to `Settings -> Build & Package -> Export Workspace` to download a `.zip` of all your saved missions, presets, and simulation configurations.

---

## 3. Running Simulations

While missions are authored in the web UI, the high-fidelity physics simulations are executed in your terminal. This uses the unified path-based MPC controller.

**Quick Launch:**

```bash
make sim
# OR use the CLI directly:
satellite-control run --auto
```

**Workflow:**

1. Launch the simulation from the terminal.
2. The CLI will present an interactive menu of your saved missions. Select one.
3. Confirm the run parameters.
4. Watch the terminal for progress as the C++ physics engine and OSQP solver step through the mission.
5. Once complete, the output artifacts are generated automatically.

---

## 4. Analyzing Output & Visualizations

After a simulation completes, results are saved to timestamped directories in the `Data/Simulation/` folder:

```text
Data/
└── Simulation/
   └── DD-MM-YYYY_HH-MM-SS/
      ├── physics_data.csv           # 200Hz physical state history
      ├── control_data.csv           # 16.67Hz thruster/MPC performance
      ├── Simulation_3D_Render.mp4   # Animated playback of the mission
      ├── mission_metadata.json
      └── Plots/                     # Generated PNG analysis plots
```

### What gets generated?

1. **Simulation_3D_Render.mp4**: A 30 FPS animated playback of the trajectory. It overlays active thrusters (highlighted when firing), target waypoints, and real-time telemetry markers.
2. **physics_data.csv**: The raw telemetry including position, velocity, orientation (quaternions), and thruster values over time. Perfect for import into MATLAB or Pandas.
3. **Performance Plots (in `Plots/`)**:
   - `trajectory_plot.png`: Static X-Y path overview with start/end markers.
   - `velocity_profile.png`: Ensuring velocities stayed within the 0.5 m/s linear bounds.
   - `thruster_activity.png`: A heatmap showing the duty cycle of all 8 thrusters over time to identify chattering or inefficiencies.
   - `mpc_performance.png`: Ensure solver times remain under 10ms (the control period is 60ms).

You can also browse these outputs interactively using the **Data** and **Viewer** tabs in the Web UI!
