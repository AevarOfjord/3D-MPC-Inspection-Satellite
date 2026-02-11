# ============================================================================
# Cross-platform detection
# ============================================================================

UNAME_S := $(shell uname -s 2>/dev/null || echo Windows)

ifeq ($(UNAME_S),Darwin)
  # macOS
  PLATFORM := macos
  EXT_SUFFIX := .so
  BUILD_GLOB := build/cp3*-cp3*-macosx_*/*.so
  KILL_PORT = lsof -ti:8000 | xargs kill -9 2>/dev/null || true
  SYSTEM_PYTHON := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v python3.11 2>/dev/null || echo "")
  INSTALL_HINT := brew install python@3.11
else ifeq ($(UNAME_S),Linux)
  # Linux
  PLATFORM := linux
  EXT_SUFFIX := .so
  BUILD_GLOB := build/cp3*-cp3*-linux_*/*.so
  KILL_PORT = fuser -k 8000/tcp 2>/dev/null || true
  SYSTEM_PYTHON := $(shell command -v python3.11 2>/dev/null || echo "")
  INSTALL_HINT := sudo apt install python3.11
else
  # Windows (Git Bash / MSYS2 / WSL fallback)
  PLATFORM := windows
  EXT_SUFFIX := .pyd
  BUILD_GLOB := build/cp3*-cp3*-win_*/*.pyd
  KILL_PORT = taskkill /F /PID $$(netstat -ano | findstr :8000 | head -1 | awk '{print $$NF}') 2>NUL || true
  SYSTEM_PYTHON := $(shell command -v python 2>/dev/null || command -v py 2>/dev/null || echo "")
  INSTALL_HINT := Download from https://python.org or: winget install Python.Python.3.11
endif

VENV_DIR ?= .venv311
CMAKE_GENERATOR ?= Ninja

ifeq ($(PLATFORM),windows)
  VENV_BIN := $(VENV_DIR)/Scripts
  VENV_PY := $(VENV_BIN)/python.exe
  VENV_PIP := $(VENV_BIN)/pip.exe
  CMAKE_MAKE_PROGRAM ?= $(VENV_BIN)/ninja.exe
else
  VENV_BIN := $(VENV_DIR)/bin
  VENV_PY := $(VENV_BIN)/python
  VENV_PIP := $(VENV_BIN)/pip
  CMAKE_MAKE_PROGRAM ?= $(VENV_BIN)/ninja
endif

REQS_FILE ?= requirements.txt
SYSTEM_CMAKE := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v cmake 2>/dev/null || echo "")

# ============================================================================
# Help
# ============================================================================

help:
	@echo ""
	@echo "Satellite Control System  ($(PLATFORM))"
	@echo "========================================"
	@echo ""
	@echo "  make build        Build everything (venv + deps + C++ extensions)"
	@echo "  make rebuild      Clean build artifacts, then build from scratch"
	@echo "  make run          Start backend + frontend together"
	@echo "  make sim          Run CLI simulation (prompts to run tests first)"
	@echo "  make test         Run pytest suite"
	@echo "  make lint         Lint Python + UI"
	@echo "  make clean        Remove venv, build artifacts, caches"
	@echo ""

# ============================================================================
# Python check
# ============================================================================

check-python:
	@if [ -z "$(SYSTEM_PYTHON)" ]; then \
		echo "Error: Python 3.11 not found on system PATH"; \
		echo "Install it via: $(INSTALL_HINT)"; \
		exit 1; \
	fi
	@$(SYSTEM_PYTHON) -c "import sys; v=sys.version_info[:2]; raise SystemExit(0 if v==(3,11) else f'Python 3.11.x required, got {sys.version.split()[0]}')"
	@echo "Found system Python: $(SYSTEM_PYTHON)"

check-cmake:
	@if [ -z "$(SYSTEM_CMAKE)" ]; then \
		echo "Error: CMake not found on system PATH"; \
		echo "Install it (macOS: brew install cmake, Ubuntu: apt install cmake)"; \
		exit 1; \
	fi
	@echo "Found system CMake: $(SYSTEM_CMAKE)"
	@$(SYSTEM_CMAKE) --version | head -1

# ============================================================================
# Run targets
# ============================================================================

run:
	@$(MAKE) -j2 backend frontend

dashboard: run

backend:
	@echo "Stopping any existing process on port 8000..."
	@$(KILL_PORT)
	PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) scripts/run_dashboard.py

frontend:
	cd ui && npm install && npm run dev

sim:
	@$(MAKE) venv
	@if ! $(VENV_PY) -c "import numpy, scipy, pydantic, typer" >/dev/null 2>&1; then \
		echo "Python runtime dependencies are missing in $(VENV_DIR)."; \
		echo "Running 'make install' to repair the environment..."; \
		$(MAKE) install || exit $$?; \
		if ! $(VENV_PY) -c "import numpy, scipy, pydantic, typer" >/dev/null 2>&1; then \
			echo "Error: Dependencies are still missing after install."; \
			echo "Check network access and pip output above."; \
			exit 1; \
		fi; \
	fi
	@printf "Run tests before simulation? [y/N] "; \
	read ans; \
	case "$$ans" in \
		y|Y|yes|YES) $(MAKE) test || exit $$? ;; \
		*) echo "Skipping tests."; ;; \
	esac
	PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) scripts/run_simulation.py

# ============================================================================
# Build targets
# ============================================================================

build: install
	@echo ""
	@echo "Build complete. Run a mission with: make sim"

rebuild: clean-build install
	@echo ""
	@echo "Rebuild complete. Run a mission with: make sim"

venv: check-python
	@if [ -f "$(VENV_PY)" ]; then \
		echo "Virtual environment already exists, skipping creation."; \
	else \
		echo "Creating virtual environment..."; \
		$(SYSTEM_PYTHON) -m venv $(VENV_DIR); \
		echo "Virtual environment created at $(VENV_DIR)"; \
	fi
	@if ! $(VENV_PY) -c "import pip" >/dev/null 2>&1; then \
		echo "pip missing in venv, repairing with ensurepip..."; \
		$(VENV_PY) -m ensurepip --upgrade; \
		$(VENV_PY) -m pip install --upgrade pip setuptools; \
	fi

install: venv check-cmake
	@echo ""
	@echo "=== Installing Python dependencies ==="
	$(VENV_PY) -m pip install -r $(REQS_FILE)
	@echo ""
	@echo "=== Installing C++ build dependencies ==="
	$(VENV_PY) -m pip install "scikit-build-core>=0.3.3" pybind11 "ninja>=1.10"
	@echo ""
	@echo "=== Building C++ extensions ==="
	CMAKE_GENERATOR="$(CMAKE_GENERATOR)" CMAKE_MAKE_PROGRAM="$(CMAKE_MAKE_PROGRAM)" \
	SKBUILD_CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" \
		$(VENV_PY) -m pip install --no-build-isolation -e .
	@$(VENV_PY) -c "from satellite_control.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ modules loaded OK')"

install-dev: install
	@echo ""
	@echo "=== Installing C++ build dependencies ==="
	$(VENV_PY) -m pip install "scikit-build-core>=0.3.3" pybind11 "ninja>=1.10"
	@echo ""
	@echo "=== Building C++ extensions ==="
	CMAKE_GENERATOR="$(CMAKE_GENERATOR)" CMAKE_MAKE_PROGRAM="$(CMAKE_MAKE_PROGRAM)" \
	SKBUILD_CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" \
		$(VENV_PY) -m pip install --no-build-isolation -e .
	@$(VENV_PY) -c "from satellite_control.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ modules loaded OK')"

# ============================================================================
# Quality targets
# ============================================================================

test:
	PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m pytest -q --tb=short

lint:
	PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m ruff check src/python tests
	cd ui && npx eslint .

# ============================================================================
# Clean targets
# ============================================================================

clean-build:
	@echo "Cleaning build artifacts..."
	@rm -rf build/cp3*
	@rm -f src/python/satellite_control/cpp/*$(EXT_SUFFIX)
	@rm -rf $(VENV_DIR)
	@rm -rf dist
	@echo "Done."

clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/python/satellite_control/cpp/*$(EXT_SUFFIX)
	@rm -rf ui/node_modules/.vite
	@rm -rf .pytest_cache .ruff_cache
	@echo "Cleaned."
