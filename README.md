# Orbital Inspector Satellite Control System

[![Python 3.11](https://img.shields.io/badge/python-3.11-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Optimization: OSQP](https://img.shields.io/badge/Optimization-OSQP-green.svg)](https://osqp.org/)

**A 6-DOF orbital inspection satellite simulation with path-based MPC, reaction wheels, and a web mission builder.**

---

## 🚀 Features

| Feature                    | Description                                            |
| -------------------------- | ------------------------------------------------------ |
| **Reaction Wheel Control** | 3-axis attitude control with 0.18° precision           |
| **Orbital Dynamics**       | Hill-Clohessy-Wiltshire gravity gradient model         |
| **Obstacle Avoidance**     | Dynamic hard constraints for environment obstacles     |
| **Mission Designer UI**    | Three.js web interface for trajectory planning         |
| **Mission System**         | Unified mission JSON with scan, transfer, hold segments |

## 🛠️ Technology Stack

| Component   | Technology                         |
| ----------- | ---------------------------------- |
| **Solver**  | OSQP (<5ms solve times)            |
| **Physics** | Custom C++ engine                  |
| **Control** | 13-state MPC with thrusters + reaction wheels |
| **UI**      | Three.js 3D visualization          |

## 📁 Project Structure

```
├── src/satellite_control/
│   ├── control/                # MPC controllers
│   ├── core/                   # Simulation loop + C++ engine bindings
│   ├── config/                 # Orbital & actuator configs
│   ├── mission/                # Mission types & helpers
│   ├── dashboard/              # FastAPI backend + route modules
│   └── physics/                # Orbital dynamics (CW equations)
├── ui/                         # Web-based mission designer
├── missions/                   # Saved unified missions
└── Data/                       # Simulation outputs
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
| All tests | `.venv311/bin/python -m pytest` |

## 📊 Performance

| Metric                | Value  |
| --------------------- | ------ |
| Position control      | ±0.5mm |
| Attitude control      | ±0.18° |
| MPC solve time        | <1ms   |
| Station-keeping error | 0.00cm |

## 📋 Mission Types

Mission definitions live in `missions/` (saved)
and drive the path-following MPC setup.

## 📄 License

MIT License - See [LICENSE](LICENSE)
