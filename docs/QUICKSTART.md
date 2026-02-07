# Quick Start Guide

Get your first satellite control simulation running in 5 minutes.

---

## Prerequisites

- **Python 3.11.x** (check: `python3.11 --version`)
- **20 minutes** for full installation
- **macOS, Linux, or Windows**

---

## Installation

### 1. Clone and Navigate

```bash
git clone https://github.com/AevarOfjord/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel.git
cd Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel
```

### 2. Create Virtual Environment

```bash
python3.11 -m venv .venv311
source .venv311/bin/activate  # On Windows: .venv311\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Note:** This installs the OSQP solver and visualization tools (~150MB).

### 4. Install FFmpeg (for animations)

**macOS:**

```bash
brew install ffmpeg
```

**Linux:**

```bash
sudo apt update && sudo apt install ffmpeg
```

**Windows:** Download from [ffmpeg.org](https://ffmpeg.org/download.html) and add to PATH.

---

## Run Your First Simulation

### Launch Interactive Menu

```bash
make sim
```

### Run a Saved Mission

1. Create and save a unified mission in the web UI (`missions_unified/`)
2. Run `make sim`
3. Select your saved mission from the list

**What you'll see:**

- Real-time terminal dashboard with telemetry
- Mission completes in ~15 seconds
- Results saved to `Data/` folder

### View Results

```bash
# Find your simulation output
ls -lt Data/ | head -2

# Navigate to latest run
cd Data/<timestamp>

# View animation
open Simulation_animation.mp4  # macOS
# or: xdg-open Simulation_animation.mp4  # Linux
```

---

## Understanding the Output

Every simulation creates:

| File                       | Purpose                                    |
| -------------------------- | ------------------------------------------ |
| `physics_data.csv`         | Position, velocity, acceleration over time |
| `control_data.csv`         | Thruster commands and MPC performance      |
| `Simulation_animation.mp4` | Animated mission playback                  |

**Example analysis:**

```python
import pandas as pd
df = pd.read_csv('Data/<timestamp>/physics_data.csv')
print(f"Final position error: {df['error'].iloc[-1]:.4f} m")
```

---

## Next Steps

### Try Different Missions

Create mission paths in the web UI and save them, then run them from terminal:

- Open Mission Control UI and build/edit a mission path
- Save unified mission JSON to `missions_unified/`
- Run `make sim` and choose a saved mission file

### Run Tests

```bash
.venv311/bin/python -m pytest tests/
```

---

## Troubleshooting

### "ModuleNotFoundError"

```bash
# Ensure you're in project root and venv is activated
pwd  # Should show .../Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel
which python  # Should show .../.venv311/bin/python
```

### "ffmpeg not found"

```bash
# Verify installation
ffmpeg -version

# Run without animation if needed
python run_simulation.py --no-anim
```

### Slow Performance

```bash
# Run headless for max speed
python run_simulation.py --no-anim --auto
```

**More help:** See [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

---

## What Just Happened?

1. **MPC Controller** computed optimal thruster commands every 60ms
2. **C++ Physics** simulated satellite dynamics at 200Hz
3. **8 Thrusters** fired based on continuous PWM duty cycles
4. **Data Logger** recorded full mission telemetry
5. **Visualizer** created animation and plots

**Key Achievement:** Your satellite navigated to the target with <5cm accuracy using Model Predictive Control!

---

## Learn More

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System design
- **[MATHEMATICS.md](MATHEMATICS.md)** - MPC formulation
- **[DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)** - Contributing
- **[VISUALIZATION.md](VISUALIZATION.md)** - Understanding plots

---

**Ready for more?** Create a new mission path in the web UI and run it through terminal simulation.
