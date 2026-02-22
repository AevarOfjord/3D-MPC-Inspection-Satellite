# Orbital Inspector Satellite Control

A local-first satellite simulation and mission control project with:

- Python backend simulation and APIs
- Web UI for mission design and runtime control
- MPC-based control and telemetry outputs

## Requirements

- Python 3.11
- Node.js 20+
- CMake + Ninja (for C++ extension builds)

## Quick Start

```bash
make install
make ui-build
make run-app
```

Open `http://127.0.0.1:8000`.

## Core Commands

```bash
make run            # backend + frontend dev
make run-app        # backend serving prebuilt ui/dist
make test           # backend tests
make lint           # backend + frontend lint
make package-pyinstaller
```

## Repository Layout

- `src/python/satellite_control/` backend code
- `src/cpp/` C++ solver/runtime modules
- `ui/` frontend application
- `scripts/` operations and release scripts
- `missions/` saved mission payloads
- `data/` canonical assets and runtime simulation/dashboard data
- `tests/` backend test suite
- `docs/architecture/` architecture notes

## Contributing & Policies

- [CONTRIBUTING.md](CONTRIBUTING.md)
- [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)
- [SECURITY.md](SECURITY.md)
