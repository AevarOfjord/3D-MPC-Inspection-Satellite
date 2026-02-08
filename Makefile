.PHONY: run run-backend run-frontend sim build rebuild clean-build install install-dev venv clean check-python

VENV_DIR ?= .venv311
VENV_BIN := $(VENV_DIR)/bin
VENV_PY := $(VENV_BIN)/python
VENV_PIP := $(VENV_BIN)/pip
REQS_FILE ?= requirements.txt
DEV_REQS_FILE ?= requirements-dev.txt
CMAKE_GENERATOR ?= Ninja
CMAKE_MAKE_PROGRAM ?= $(VENV_BIN)/ninja

# Find system python3.11 (skip the venv to avoid self-referencing)
SYSTEM_PYTHON := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v python3.11 2>/dev/null || echo "")

check-python:
	@if [ -z "$(SYSTEM_PYTHON)" ]; then \
		echo "Error: python3.11 not found on system PATH (outside of venv)"; \
		echo "Install it via: brew install python@3.11"; \
		exit 1; \
	fi
	@$(SYSTEM_PYTHON) -c "import sys; v=sys.version_info[:2]; raise SystemExit(0 if v==(3, 11) else f'Python 3.11.x is required, got {sys.version.split()[0]}')"
	@echo "Found system Python: $(SYSTEM_PYTHON)"

run:
	@$(MAKE) -j2 backend frontend

dashboard:
	@$(MAKE) -j2 backend frontend

backend:
	@echo "Stopping any existing process on port 8000..."
	@lsof -ti:8000 | xargs kill -9 || true
	$(VENV_PY) run_dashboard.py

frontend:
	cd ui && npm install && npm run dev

sim:
	$(VENV_PY) run_simulation.py

build: install
	@echo ""
	@echo "✅ Build complete. Run a mission with: make sim"

rebuild: clean-build install
	@echo ""
	@echo "✅ Rebuild complete. Run a mission with: make sim"

clean-build:
	@echo "Cleaning build artifacts..."
	@rm -rf build/cp3*
	@echo "Done."

venv: check-python
	@if [ ! -f $(VENV_BIN)/python ]; then \
		echo "Creating virtual environment..."; \
		$(SYSTEM_PYTHON) -m venv $(VENV_DIR); \
		$(VENV_PIP) install --upgrade pip; \
		echo "Virtual environment created at $(VENV_DIR)"; \
	else \
		echo "Virtual environment already exists, skipping creation."; \
	fi

install: venv
	@echo ""
	@echo "=== Installing Python dependencies ==="
	$(VENV_PIP) install -r $(REQS_FILE)
	@echo ""
	@echo "=== Building C++ extensions (takes ~2 min) ==="
	CMAKE_GENERATOR="$(CMAKE_GENERATOR)" CMAKE_MAKE_PROGRAM="$(CMAKE_MAKE_PROGRAM)" \
		$(VENV_PIP) install --no-build-isolation -e .
	@$(VENV_PY) -c "from satellite_control.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ modules loaded OK')"
	@echo "=== Copying .so files ==="
	@cp build/cp3*-cp3*-macosx_*_arm64/*.so src/satellite_control/cpp/ 2>/dev/null || true

install-dev: venv
	@echo ""
	@echo "=== Installing dev dependencies ==="
	$(VENV_PIP) install -r $(DEV_REQS_FILE)
	@echo ""
	@echo "=== Building C++ extensions (takes ~2 min) ==="
	CMAKE_GENERATOR="$(CMAKE_GENERATOR)" CMAKE_MAKE_PROGRAM="$(CMAKE_MAKE_PROGRAM)" \
		$(VENV_PIP) install --no-build-isolation -e .
	@$(VENV_PY) -c "from satellite_control.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ modules loaded OK')"
	@echo "=== Copying .so files ==="
	@cp build/cp3*-cp3*-macosx_*_arm64/*.so src/satellite_control/cpp/ 2>/dev/null || true

clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/satellite_control/cpp/*.so
	@rm -rf .venv311
	@rm -rf ui/node_modules/.vite
	@echo "Cleaned."
