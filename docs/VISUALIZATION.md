# Simulation Visualization Guide

## Overview

After each simulation run, the system automatically generates comprehensive visualizations to help you analyze satellite performance, control behavior, and mission success. All outputs are saved to timestamped directories in the `Data/` folder.

---

## Quick Start

### Running a Simulation

```bash
python run_simulation.py run
```

After the simulation completes, find your results:

```bash
ls Data/Simulation/  # Lists timestamped directories
cd Data/Simulation/2026-01-06_10-30-45  # Navigate to latest run
```

### What Gets Generated

Every simulation automatically creates:

1. **CSV Data Files** - Raw telemetry data
2. **Trajectory Animation (MP4)** - Animated playback of the mission
3. **Performance Plots (PNG)** - Static analysis charts

---

## Output Directory Structure

After running a simulation, you'll find:

```
Data/
└── Simulation/
   └── 06-01-2026_10-30-45/           # Timestamp: DD-MM-YYYY_HH-MM-SS
      ├── physics_data.csv           # Position, velocity, acceleration
      ├── control_data.csv           # Thruster commands, MPC performance
      ├── Simulation_3D_Render.mp4   # Matplotlib animation
      ├── mission_metadata.json      # Planned path for UI
      ├── mission_summary.txt        # Run summary
      └── Plots/                     # Generated on demand
```

---

## 1. CSV Data Files

### physics_data.csv

**Contains:** Complete state history of the satellite

**Columns (subset):**

- `Time` - Simulation time (seconds)
- `Current_X`, `Current_Y`, `Current_Z` - Position in world frame (meters)
- `Current_Roll`, `Current_Pitch`, `Current_Yaw` - Orientation (radians)
- `Current_VX`, `Current_VY`, `Current_VZ` - Linear velocity (m/s)
- `Target_X`, `Target_Y`, `Target_Z` - Target position
- `Target_Roll`, `Target_Pitch`, `Target_Yaw` - Target orientation
- `Error_*` - Position/orientation tracking errors
- `Command_Vector`, `Thruster_*_Cmd`, `Thruster_*_Val` - Thruster command/actual levels

See the CSV header for the full column list.

**Example Row (truncated):**

```csv
Time,Current_X,Current_Y,Current_Z,Current_Roll,Current_Pitch,Current_Yaw,...,Thruster_1_Cmd,Thruster_1_Val
12.3400,0.45200,-0.23100,0.00000,0.00010,-0.00020,0.12500,...,0.000,0.000
```

**Use Case:** Import into your own analysis tools, MATLAB, Python pandas, etc.

```python
import pandas as pd
df = pd.read_csv('Data/Simulation/06-01-2026_10-30-45/physics_data.csv')
print(df.describe())
```

---

### control_data.csv

**Contains:** Control inputs and MPC solver performance

**Columns (subset):**

- `Control_Time` - Control tick time (seconds)
- `Current_*`, `Target_*`, `Error_*` - Full 3D state and tracking errors
- `Command_Vector`, `Command_Hex`, `Total_Active_Thrusters`, `Thruster_Switches`
- `MPC_Solve_Time`, `MPC_Status`, `MPC_Objective`, `MPC_Iterations`

See the CSV header for the full column list.

**Example Row (truncated):**

```csv
Control_Time,Current_X,Current_Y,Current_Z,...,Command_Vector,Total_Active_Thrusters,MPC_Solve_Time,MPC_Status
12.3400,0.45200,-0.23100,0.00000,...,"[0.000, 0.000, ...]",2,0.00142,solved
```

**Use Case:** Analyze control effort, MPC performance, thruster usage patterns

```python
import pandas as pd
df = pd.read_csv('Data/Simulation/06-01-2026_10-30-45/control_data.csv')

# Average solve time
print(f"Avg MPC solve time: {df['mpc_solve_time'].mean()*1000:.2f} ms")

# Total control effort
total_effort = df[[f'thruster_{i}' for i in range(1,9)]].sum().sum()
print(f"Total thrust effort: {total_effort:.2f}")
```

---

## 2. Animated Mission Playback (MP4)

### Simulation_3D_Render.mp4

**What it shows:**

- Full mission trajectory in X-Y plane
- Satellite position and orientation over time
- Active thrusters highlighted when firing
- Target/waypoint positions
- Real-time telemetry overlay:
  - Current position and velocity
  - Distance to target
  - Mission phase
  - Active thruster list

**Playback Controls:**

- Standard video player controls
- Scrub timeline to specific moments
- Pause to inspect details
- Speed up/slow down in most players

**Technical Details:**

- **Resolution:** 1920×1080 (Full HD)
- **Frame Rate:** 30 FPS
- **Duration:** Matches real simulation time (e.g., 30s mission = 30s video)
- **Codec:** H.264 (widely compatible)

**Viewing:**

```bash
# macOS
open Data/Simulation/06-01-2026_10-30-45/Simulation_3D_Render.mp4

# Linux
xdg-open Data/Simulation/06-01-2026_10-30-45/Simulation_3D_Render.mp4

# Windows
start Data/Simulation/06-01-2026_10-30-45/Simulation_3D_Render.mp4
```

---

## 3. Performance Analysis Plots

Generate detailed static plots on demand:

```bash
python -m src.satellite_control.visualization.unified_visualizer
```

This creates multiple PNG plots in the same `Data/Simulation/<timestamp>/Plots/` directory.

---

### 3.1 Trajectory Plot

**Filename:** `trajectory_plot.png`

**Shows:**

- Complete X-Y path of satellite (blue line)
- Starting position (green marker)
- Waypoints/targets (red markers)
- Trajectory highlights where thrusters fired
- Workspace boundaries (±3m)

**What to Look For:**

- ✓ Smooth path to target
- ✓ Minimal oscillations
- ✓ Respects workspace bounds
- ✗ Overshooting targets
- ✗ Erratic path segments

**Example Interpretation:**

```
┌─────────────────────────────────────┐
│                                     │
│     Start (0,0) ●────────→ ● Target│
│                  \                  │
│                   \ (slight curve)  │
│                    \                │
│                     ● Waypoint 1    │
└─────────────────────────────────────┘

Good: Smooth path with gentle curve
```

---

### 3.2 Position vs Time

**Filename:** `position_vs_time.png`

**Shows:**

- X position over time (top subplot)
- Y position over time (bottom subplot)
- Target positions (dashed reference lines)
- Position error shaded region

**What to Look For:**

- ✓ Converges to target positions
- ✓ Minimal overshoot
- ✓ Smooth approach (no sharp jumps)
- ✗ Oscillations around target
- ✗ Never reaching target

**Key Metrics:**

- **Settling time:** How long to reach and stay at target
- **Overshoot:** Maximum distance past target
- **Steady-state error:** Final position error at mission end

---

### 3.3 Velocity Profile

**Filename:** `velocity_profile.png`

**Shows:**

- Linear velocity magnitude over time (top)
- Angular velocity over time (bottom)
- Velocity limits shown as reference lines

**What to Look For:**

- ✓ Velocity approaches zero near target
- ✓ Stays within limits (0.5 m/s linear, π/2 rad/s angular)
- ✓ Smooth acceleration/deceleration
- ✗ Sustained high velocity near target
- ✗ Velocity limit violations

**Performance Indicators:**

- **Aggressive control:** High velocities, fast settling
- **Conservative control:** Low velocities, slow but stable
- **Good damping:** Velocity smoothly decreases to zero

---

### 3.4 Thruster Activity Heatmap

**Filename:** `thruster_activity.png`

**Shows:**

- 8 rows (one per thruster)
- Time on X-axis
- Color intensity = duty cycle (0% = dark, 100% = bright)

**What to Look For:**

- ✓ Balanced thruster usage (no single thruster overused)
- ✓ Clear firing patterns matching mission phases
- ✓ Symmetric pairs firing for rotation
- ✗ Continuous high duty cycles (inefficient)
- ✗ Chattering (rapid on/off switching)

**Interpretation:**

```
Thruster 1: ████░░░░░░░░░░  (fired early, then idle)
Thruster 2: ░░░░████████░░  (fired mid-mission)
Thruster 3: ░░░░░░░░████░░  (fired for final correction)
...
```

---

### 3.5 MPC Performance

**Filename:** `mpc_performance.png`

**Shows:**

- Solve time per MPC iteration (milliseconds)
- Cost function value over time
- Solver status (success/failure markers)

**What to Look For:**

- ✓ Solve times consistently < 10 ms
- ✓ Cost function decreasing over time
- ✓ All iterations marked as "OPTIMAL"
- ✗ Solve times approaching control period (60 ms)
- ✗ Frequent solver failures

**Performance Targets:**

- **Excellent:** < 2 ms average solve time
- **Good:** 2-5 ms average
- **Acceptable:** 5-10 ms average
- **Poor:** > 10 ms (may indicate problem too large or poorly conditioned)

---

### 3.6 Error Analysis

**Filename:** `error_analysis.png`

**Shows:**

- Position error magnitude over time (meters)
- Angle error over time (degrees)
- Error convergence rate

**What to Look For:**

- ✓ Error decreasing monotonically
- ✓ Final error < tolerance (typically 0.05 m, 3°)
- ✓ Exponential decay shape
- ✗ Error increasing over time
- ✗ Oscillating error (overshooting)

**Success Criteria:**

- **Position error:** < 0.05 m at mission end
- **Angle error:** < 3° at mission end
- **Convergence time:** Depends on mission (typically 10-30s)

---

## Customizing Visualizations

### Changing Plot Appearance

Edit `src/satellite_control/visualization/plot_style.py` for shared defaults:

```python
from src.satellite_control.visualization.plot_style import PlotStyle

# Figure sizing
PlotStyle.FIGSIZE_SUBPLOTS = (12, 9)

# Color defaults
PlotStyle.COLOR_SIGNAL_POS = "#2E86AB"
PlotStyle.COLOR_TARGET = "#A23B72"

# Export quality
PlotStyle.DPI = 300
```

Plot implementations are split by concern:

- `trajectory_plots.py` for trajectory and 3D path plots
- `actuator_plots.py` for thruster/PWM/actuator plots
- `state_plots.py` for state/phase/constraint plots
- `diagnostics_plots.py` for solver/timing/progress plots

### Changing Animation Settings

Animation output is generated by:

- `simulation_visualization.py` for runtime auto-generated simulation animation
- `video_renderer.py` for post-processed frame rendering and encoding

For rendering quality and style changes, update `video_renderer.py` and
`plot_style.py` together so plot/video visuals remain consistent.

### Generating Only Specific Plots

```python
from src.satellite_control.visualization.unified_visualizer import UnifiedVisualizationGenerator

viz = UnifiedVisualizationGenerator(data_directory="Data/Simulation")
viz.load_csv_data()

# Generate the full static plot suite
viz.generate_performance_plots()

# Generate animation artifact
viz.generate_animation("output.mp4")
```

If you need one specific plot, call the corresponding method on
`PlotGenerator` after loading data:

```python
plot_gen = viz._get_plot_generator()
plot_gen.generate_velocity_tracking_plot(viz.output_dir / "Plots")
```

---

## Comparing Multiple Runs

To compare different missions or parameter configurations:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load two different runs
run1 = pd.read_csv('Data/Simulation/06-01-2026_10-00-00/physics_data.csv')
run2 = pd.read_csv('Data/Simulation/06-01-2026_10-30-00/physics_data.csv')

# Compare position error
plt.figure(figsize=(10, 6))
plt.plot(run1['time'], run1['error'], label='Run 1: N=50')
plt.plot(run2['time'], run2['error'], label='Run 2: N=100')
plt.xlabel('Time (s)')
plt.ylabel('Position Error (m)')
plt.legend()
plt.grid(True)
plt.savefig('comparison.png')
```

---

## Exporting for Presentations

### High-Quality Still Images

```python
from src.satellite_control.visualization.unified_visualizer import UnifiedVisualizationGenerator

viz = UnifiedVisualizationGenerator(data_directory="Data/Simulation")
viz.load_csv_data()

# Export full static plot suite (high quality controlled by PlotStyle.DPI)
viz.generate_performance_plots()
# Saved under Data/Simulation/<timestamp>/Plots/

# Convert to PDF for LaTeX/presentations
import subprocess
subprocess.run([
    'convert',
   'Data/Simulation/<timestamp>/Plots/01_trajectory_2d.png',
    'trajectory_plot.pdf'
])
```

### Embedding Animation in Slides

1. **PowerPoint:** Insert → Video → From File
2. **Google Slides:** Insert → Video → Upload
3. **LaTeX Beamer:** Use `\movie` command or convert to GIF

**Convert MP4 to GIF (smaller for web):**

```bash
ffmpeg -i Simulation_3D_Render.mp4 -vf "fps=15,scale=800:-1" output.gif
```

---

## Troubleshooting

### No Plots Generated

**Check:**

1. Simulation completed successfully
2. CSV files exist in `Data/Simulation/<timestamp>/`
3. Run the visualizer manually:
   ```bash
   python -m src.satellite_control.visualization.unified_visualizer
   ```

### Animation File Corrupted

**Symptoms:** Can't open MP4, 0 byte file

**Solutions:**

1. Check FFmpeg installation: `ffmpeg -version`
2. Re-run visualizer with verbose output
3. Check disk space

### Poor Video Quality

**Improve quality:**

```python
# In plot_style.py
from src.satellite_control.visualization.plot_style import PlotStyle
PlotStyle.DPI = 300

# In video_renderer.py
# Tune writer fps / bitrate settings for your output target.
```

### Plots Look Cluttered

**Simplify:**

```python
# Reduce data points plotted
df_downsampled = df[::10]  # Plot every 10th point

# Increase figure size
plt.figure(figsize=(14, 10))

# Use cleaner style
plt.style.use('seaborn-v0_8-white')
```

---

## Best Practices

### For Analysis

1. **Always check solve times first** - MPC failures indicate parameter issues
2. **Compare error plot to velocity** - Slow convergence may need higher Q_vel
3. **Review thruster activity** - Excessive chattering wastes fuel
4. **Watch the animation** - Catches issues not obvious in plots

### For Presentations

1. **Use trajectory + animation** - Best visual demo
2. **Include error analysis** - Shows quantitative performance
3. **Highlight key metrics** - Settling time, final error, solve time
4. **Compare to baselines** - Show improvement over previous attempts

### For Debugging

1. **Check mission phase transitions** in CSV data
2. **Look for MPC solver failures** in control_data
3. **Identify oscillation patterns** in position plot
4. **Verify thruster symmetry** in heatmap

---

## Summary

After each simulation, you get:

| Output                       | Purpose            | Key Insights                       |
| ---------------------------- | ------------------ | ---------------------------------- |
| **physics_data.csv**         | Raw state data     | Import to custom analysis tools    |
| **control_data.csv**         | Control & MPC data | Solver performance, thruster usage |
| **Simulation_3D_Render.mp4** | Visual playback    | See actual mission execution       |
| **trajectory_plot.png**      | Path analysis      | Verify smooth navigation           |
| **velocity_profile.png**     | Speed analysis     | Check damping and limits           |
| **thruster_activity.png**    | Control effort     | Identify inefficiencies            |
| **mpc_performance.png**      | Solver metrics     | Detect computational issues        |
| **error_analysis.png**       | Accuracy metrics   | Quantify mission success           |

**Next Steps:**

- Run a simulation: `python run_simulation.py run`
- Find your data: `ls Data/Simulation/`
- Review the animation and plots
- Iterate on MPC parameters if needed
- See [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md) for tuning tips

---

## CSV Data Reference

The CSV headers are the source of truth for available fields. Use the column list
in the file header to drive analysis scripts, and prefer the `control_data.csv`
and `physics_data.csv` exports under `Data/Simulation/<timestamp>/`.
