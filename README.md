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
├── ui/                         # Web-based mission designer
└── scripts/                    # Test scripts
```

## ⚡ Quick Start

```bash
# 1. Install dependencies
pip install -r requirements.txt
cd ui && npm install && cd ..

# 2. Start the Backend Simulation Server (Term 1)
python run_dashboard.py

# 3. Start the Mission Control UI (Term 2)
cd ui && npm run dev
# Open http://localhost:5173
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

Mission definitions live in `missions/` and drive the path-following MPC setup.

## 📄 License

MIT License - See [LICENSE](LICENSE)
