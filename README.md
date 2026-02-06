# Orbital Inspector Satellite Control System

[![Python 3.9-3.12](https://img.shields.io/badge/python-3.9--3.12-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Optimization: OSQP](https://img.shields.io/badge/Optimization-OSQP-green.svg)](https://osqp.org/)

**A 6-DOF orbital inspection satellite simulation with Model Predictive Control, reaction wheels, and multi-satellite coordination.**

---

## 🚀 Features

| Feature                    | Description                                            |
| -------------------------- | ------------------------------------------------------ |
| **Reaction Wheel Control** | 3-axis attitude control with 0.18° precision           |
| **Orbital Dynamics**       | Hill-Clohessy-Wiltshire gravity gradient model         |
| **Multi-Satellite Fleet**  | 3 inspectors with collision avoidance                  |
| **Obstacle Avoidance**     | Dynamic hard constraints for environment obstacles     |
| **Mission Designer UI**    | Three.js web interface for trajectory planning         |
| **Mission System**         | Pre-built flyby, circumnavigation, inspection missions |

## 🛠️ Technology Stack

| Component   | Technology                         |
| ----------- | ---------------------------------- |
| **Solver**  | OSQP (<5ms solve times)            |
| **Physics** | Custom C++ engine                  |
| **Control** | 16-state MPC with 9 control inputs |
| **UI**      | Three.js 3D visualization          |

## 📁 Project Structure

```
├── src/satellite_control/
│   ├── control/                # MPC controllers
│   ├── core/                   # Simulation loop + C++ engine bindings
│   ├── config/                 # Orbital & actuator configs
│   ├── fleet/                  # Multi-satellite coordination
│   ├── mission/                # Mission types & helpers
│   └── physics/                # Orbital dynamics (CW equations)
├── missions/                   # Sample mission JSON files
│   ├── examples/               # Curated example missions
│   └── dev/                    # User-saved and scratch missions
├── ui/                         # Web-based mission designer
├── missions_unified/           # Unified mission drafts/previews
└── scripts/                    # Test scripts
```

## ⚡ Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt
# Optional: dev tools + linters
pip install -r requirements-dev.txt
cd ui && npm install && cd ..

# 2. Start the Backend Simulation Server (Term 1)
python run_dashboard.py
# Server binds to localhost (127.0.0.1:8000)

# 3. Start the Mission Control UI (Term 2)
cd ui && npm run dev
# Open http://localhost:5173
```

## 🧭 Local-Only Usage Model

This project is intended to be cloned and run on a local machine from terminal sessions.
It is not configured as an internet-facing hosted service by default.

## ✅ Known-Good Local Verification

```bash
# Backend lint (high-signal correctness checks)
.venv311/bin/python -m ruff check src tests --select E9,F63,F7,F82,F401,F541,F841,E402,E722

# Backend tests
.venv311/bin/python -m pytest -q

# Frontend quality/build checks
cd ui
npm run lint
npm run build
```

## 🧪 Tests

| Test      | Command                |
| --------- | ---------------------- |
| All tests | `python -m pytest`     |

## 📊 Performance

| Metric                | Value  |
| --------------------- | ------ |
| Position control      | ±0.5mm |
| Attitude control      | ±0.18° |
| MPC solve time        | <1ms   |
| Station-keeping error | 0.00cm |
| Formation separation  | 8.66m  |

## 📋 Mission Types

Mission definitions live in `missions/examples/` (curated) and `missions/dev/` (saved)
and drive the path-following MPC setup.

## 📄 License

MIT License - See [LICENSE](LICENSE)
