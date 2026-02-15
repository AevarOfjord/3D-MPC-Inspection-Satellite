# Orbital Inspector Satellite Control System.

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
├── src/python/satellite_control/
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

## ⚡ Quick Start (Developer)

```bash
# 1) Install/build once
make install
make ui-build

# 2) Run packaged web app (single server, no Vite)
make run-app
# Open http://localhost:8000
#
# Dev mode (hot reload): make run
```

## 📘 Usage Guide

Detailed run/use instructions are in:

- `docs/HOW_TO_USE.md`

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
