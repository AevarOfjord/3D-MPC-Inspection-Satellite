# ============================================================================
# Cross-platform detection
# ============================================================================

UNAME_S := $(shell uname -s 2>/dev/null || echo Windows)

ifeq ($(UNAME_S),Darwin)
  # macOS
  PLATFORM := macos
  EXT_SUFFIX := .so
  BUILD_GLOB := build/cp3*-cp3*-macosx_*/*.so
  KILL_BACKEND = lsof -ti:8000 | xargs kill -9 2>/dev/null || true
  KILL_FRONTEND = lsof -ti:5173 | xargs kill -9 2>/dev/null || true
  SYSTEM_PYTHON := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v python3.11 2>/dev/null || echo "")
  INSTALL_HINT := brew install python@3.11
else ifeq ($(UNAME_S),Linux)
  # Linux
  PLATFORM := linux
  EXT_SUFFIX := .so
  BUILD_GLOB := build/cp3*-cp3*-linux_*/*.so
  KILL_BACKEND = fuser -k 8000/tcp 2>/dev/null || true
  KILL_FRONTEND = fuser -k 5173/tcp 2>/dev/null || true
  SYSTEM_PYTHON := $(shell command -v python3.11 2>/dev/null || echo "")
  INSTALL_HINT := sudo apt install python3.11
else
  # Windows (Git Bash / MSYS2 / WSL fallback)
  PLATFORM := windows
  EXT_SUFFIX := .pyd
  BUILD_GLOB := build/cp3*-cp3*-win_*/*.pyd
  KILL_BACKEND = taskkill /F /PID $$(netstat -ano | findstr :8000 | head -1 | awk '{print $$NF}') 2>NUL || true
  KILL_FRONTEND = taskkill /F /PID $$(netstat -ano | findstr :5173 | head -1 | awk '{print $$NF}') 2>NUL || true
  SYSTEM_PYTHON := $(shell command -v python 2>/dev/null || command -v py 2>/dev/null || echo "")
  INSTALL_HINT := Download from https://python.org or: winget install Python.Python.3.11
endif

# Python venv location and CMake generator used for C++ extension builds.
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

# Dependency/build settings.
REQS_FILE ?= requirements.txt
SYSTEM_CMAKE := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v cmake 2>/dev/null || echo "")
UI_DIR ?= ui
UI_NODE_MODULES := $(UI_DIR)/node_modules
UI_LOCKFILES := $(UI_DIR)/package-lock.json $(UI_DIR)/package.json
RELEASE_DIR ?= release
APP_BUNDLE_NAME ?= satellite-control-app
APP_BUNDLE_DIR := $(RELEASE_DIR)/$(APP_BUNDLE_NAME)
# Stamp file used to avoid reinstalling Node dependencies on every `make run`.
UI_DEPS_STAMP := $(UI_NODE_MODULES)/.deps-installed
# Try to detect active scikit-build editable output directory.
SKBUILD_MATCHED_EXT := $(firstword $(wildcard $(BUILD_GLOB)))
SKBUILD_BUILD_DIR := $(if $(SKBUILD_MATCHED_EXT),$(patsubst %/,%,$(abspath $(dir $(SKBUILD_MATCHED_EXT)))))
# Skip scikit-build editable runtime rebuild checks by default for faster startup.
SKBUILD_SKIP_RUNTIME_REBUILD ?= 1
SKBUILD_RUNTIME_ENV := $(if $(and $(filter 1 true TRUE yes YES,$(SKBUILD_SKIP_RUNTIME_REBUILD)),$(SKBUILD_BUILD_DIR)),SKBUILD_EDITABLE_SKIP="$(SKBUILD_BUILD_DIR)")

# ============================================================================
# Help
# ============================================================================

.PHONY: help run run-app stop backend backend-prod frontend ui-build package-app package-clean \
	sim install test lint clean rebuild \
	check-python check-cmake venv build dashboard install-dev clean-build

# Show available high-level commands.
help:
	@echo ""
	@echo "Satellite Control System  ($(PLATFORM))"
	@echo "========================================"
	@echo ""
	@echo "  make install      Setup/repair env and build C++ extensions"
	@echo "  make rebuild      Clean + fresh install"
	@echo "  make run          Start backend + frontend dev servers (stops existing instances first)"
	@echo "  make run-app      Start backend only and serve prebuilt UI from ui/dist at :8000"
	@echo "  make stop         Stop running backend and frontend processes"
	@echo "  make ui-build     Build production UI bundle into ui/dist"
	@echo "  make package-app  Create distributable prebuilt app bundle under ./release"
	@echo "  make package-clean Remove generated app bundles in ./release"
	@echo "  make sim          Run CLI simulation (prompts to run tests first)"
	@echo "  make test         Run pytest suite"
	@echo "  make lint         Lint Python + UI"
	@echo "  make clean        Remove venv, build artifacts, caches"
	@echo ""

# ============================================================================
# Python check
# ============================================================================

# Validate system Python exists and is exactly 3.11.x.
check-python:
	@if [ -z "$(SYSTEM_PYTHON)" ]; then \
		echo "Error: Python 3.11 not found on system PATH"; \
		echo "Install it via: $(INSTALL_HINT)"; \
		exit 1; \
	fi
	@$(SYSTEM_PYTHON) -c "import sys; v=sys.version_info[:2]; raise SystemExit(0 if v==(3,11) else f'Python 3.11.x required, got {sys.version.split()[0]}')"
	@echo "Found system Python: $(SYSTEM_PYTHON)"

# Validate system CMake is available for C++ extension compilation.
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

# Start both backend and frontend after stopping old processes.
run: stop
	@$(MAKE) -j2 backend frontend

# Stop any process bound to dashboard ports.
stop:
	@echo "Stopping any existing process on port 8000 (Backend)..."
	@$(KILL_BACKEND)
	@echo "Stopping any existing process on port 5173 (Frontend)..."
	@$(KILL_FRONTEND)

# Start API/dashboard backend; auto-repair env if core deps are missing.
backend:
	@$(MAKE) venv
	@if ! $(VENV_PY) -c "import fastapi, uvicorn, pydantic" >/dev/null 2>&1; then \
		echo "Backend dependencies are missing in $(VENV_DIR)."; \
		echo "Running 'make install' to repair the environment..."; \
		$(MAKE) install || exit $$?; \
	fi
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) scripts/run_dashboard.py --dev

# Start backend in packaged-app mode (no Vite, serves ui/dist on :8000).
backend-prod:
	@$(MAKE) venv
	@if [ ! -f "$(UI_DIR)/dist/index.html" ]; then \
		echo "Error: prebuilt UI not found at $(UI_DIR)/dist/index.html"; \
		echo "Run 'make ui-build' once (or ship ui/dist in your release package)."; \
		exit 1; \
	fi
	@if ! $(VENV_PY) -c "import fastapi, uvicorn, pydantic" >/dev/null 2>&1; then \
		echo "Backend dependencies are missing in $(VENV_DIR)."; \
		echo "Running 'make install' to repair the environment..."; \
		$(MAKE) install || exit $$?; \
	fi
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) scripts/run_dashboard.py

# Install frontend dependencies only when lockfiles change.
$(UI_DEPS_STAMP): $(UI_LOCKFILES)
	cd $(UI_DIR) && npm install
	@mkdir -p "$(UI_NODE_MODULES)"
	@touch "$(UI_DEPS_STAMP)"

# Start frontend dev server.
frontend: $(UI_DEPS_STAMP)
	cd $(UI_DIR) && npm run dev

# Build production UI assets consumed by backend-prod/run-app.
ui-build: $(UI_DEPS_STAMP)
	cd $(UI_DIR) && npm run build

# One-command app startup using prebuilt UI bundle.
run-app: stop backend-prod

# Create a distributable app bundle with prebuilt UI and current runtime env.
package-app: ui-build
	@$(MAKE) venv
	@if ! $(VENV_PY) -c "import satellite_control, fastapi, uvicorn" >/dev/null 2>&1; then \
		echo "Runtime deps missing in $(VENV_DIR); running 'make install' first..."; \
		$(MAKE) install || exit $$?; \
	fi
	@mkdir -p "$(RELEASE_DIR)"
	@rm -rf "$(APP_BUNDLE_DIR)"
	@mkdir -p "$(APP_BUNDLE_DIR)"
	@echo "Staging app bundle at $(APP_BUNDLE_DIR)"
	@rsync -a \
		--exclude '.git/' \
		--exclude '.github/' \
		--exclude '.pytest_cache/' \
		--exclude '.ruff_cache/' \
		--exclude '.hypothesis/' \
		--exclude '.benchmarks/' \
		--exclude '__pycache__/' \
		--exclude '/build/' \
		--exclude '/dist/' \
		--exclude '/release/' \
		--exclude 'ui/node_modules/' \
		--exclude 'ui/npm_cache/' \
		--exclude '/Data/Simulation/' \
		./ "$(APP_BUNDLE_DIR)/"
	@mkdir -p "$(APP_BUNDLE_DIR)/Data/Simulation"
	@printf '%s\n' \
		'#!/usr/bin/env bash' \
		'set -euo pipefail' \
		'ROOT_DIR="$$(cd "$$(dirname "$$0")" && pwd)"' \
		'cd "$$ROOT_DIR"' \
		'python3 scripts/start_app.py' > "$(APP_BUNDLE_DIR)/RUN_APP.sh"
	@chmod +x "$(APP_BUNDLE_DIR)/RUN_APP.sh"
	@printf '%s\n' \
		'#!/usr/bin/env bash' \
		'set -euo pipefail' \
		'ROOT_DIR="$$(cd "$$(dirname "$$0")" && pwd)"' \
		'cd "$$ROOT_DIR"' \
		'if [ -x ".venv311/bin/python" ]; then' \
		'  exec ".venv311/bin/python" "scripts/start_app.py"' \
		'fi' \
		'exec python3 "scripts/start_app.py"' > "$(APP_BUNDLE_DIR)/RUN_APP.command"
	@chmod +x "$(APP_BUNDLE_DIR)/RUN_APP.command"
	@ARCHIVE_PATH="$(RELEASE_DIR)/$(APP_BUNDLE_NAME)-$$(date +%Y%m%d_%H%M%S).tar.gz"; \
	tar -czf "$$ARCHIVE_PATH" -C "$(RELEASE_DIR)" "$(APP_BUNDLE_NAME)"; \
	echo "Created app archive: $$ARCHIVE_PATH"

# Remove generated distributable bundles.
package-clean:
	@rm -rf "$(RELEASE_DIR)"
	@echo "Removed $(RELEASE_DIR)"

# Run CLI simulation, repairing Python/build prerequisites when needed.
sim:
	@$(MAKE) venv
	@if ! $(VENV_PY) -m pip --version >/dev/null 2>&1; then \
		echo "pip missing in $(VENV_DIR), bootstrapping with ensurepip..."; \
		$(VENV_PY) -m ensurepip --upgrade || true; \
	fi
	@if ! $(VENV_PY) -m pip --version >/dev/null 2>&1; then \
		echo "Error: pip is not available in $(VENV_DIR)."; \
		echo "Your Python 3.11 may be missing ensurepip support."; \
		echo "Try reinstalling Python 3.11, then run: make clean && make install"; \
		exit 1; \
	fi
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
	@if [ ! -x "$(CMAKE_MAKE_PROGRAM)" ]; then \
		echo "Build tool missing: $(CMAKE_MAKE_PROGRAM)"; \
		echo "Installing C++ build dependencies into $(VENV_DIR)..."; \
		$(VENV_PY) -m pip install "scikit-build-core>=0.3.3" pybind11 "ninja>=1.10" || exit $$?; \
	fi
	@printf "Run tests before simulation? [y/N] "; \
	read ans; \
	case "$$ans" in \
		y|Y|yes|YES) $(MAKE) test || exit $$? ;; \
		*) echo "Skipping tests."; ;; \
	esac
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) scripts/run_simulation.py

# ============================================================================
# Build targets
# ============================================================================

# Full reset + reinstall workflow.
rebuild: clean install
	@echo ""
	@echo "Rebuild complete. Run a mission with: make sim"

# Create/repair Python virtual environment and pip.
venv: check-python
	@if [ -d "$(VENV_DIR)" ] && [ ! -x "$(VENV_PY)" ]; then \
		echo "Detected broken virtual environment at $(VENV_DIR), recreating..."; \
		rm -rf "$(VENV_DIR)"; \
	fi
	@if [ -x "$(VENV_PY)" ]; then \
		echo "Virtual environment already exists, skipping creation."; \
	else \
		echo "Creating virtual environment..."; \
		$(SYSTEM_PYTHON) -m venv $(VENV_DIR); \
		echo "Virtual environment created at $(VENV_DIR)"; \
	fi
	@if [ ! -x "$(VENV_PY)" ]; then \
		echo "Error: Virtual environment python was not created at $(VENV_PY)."; \
		echo "Run 'make clean' and retry. If this persists, reinstall Python 3.11."; \
		exit 1; \
	fi
	@if ! $(VENV_PY) -m pip --version >/dev/null 2>&1; then \
		echo "pip missing in venv, repairing with ensurepip..."; \
		$(VENV_PY) -m ensurepip --upgrade; \
	fi
	@if ! $(VENV_PY) -m pip --version >/dev/null 2>&1; then \
		echo "Error: Failed to provision pip in $(VENV_DIR)."; \
		echo "Please reinstall Python 3.11 with ensurepip support."; \
		exit 1; \
	fi
	@$(VENV_PY) -m pip install --upgrade pip setuptools >/dev/null 2>&1 || \
		echo "Warning: pip/setuptools upgrade skipped (offline or cert issue)."

# Install Python deps and (re)build C++ extensions in editable mode.
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

# ============================================================================
# Quality targets
# ============================================================================

# Run test suite.
test:
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m pytest -q --tb=short

# Run Python + frontend lint checks.
lint:
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m ruff check src/python tests
	cd ui && npx eslint .

# ============================================================================
# Clean targets
# ============================================================================

# Remove local build, venv, and cache artifacts.
clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/python/satellite_control/cpp/*$(EXT_SUFFIX)
	@rm -rf ui/node_modules/.vite
	@rm -rf .pytest_cache .ruff_cache
	@echo "Cleaned."

# Removed targets: fail fast so stale scripts don't silently no-op.
build dashboard install-dev clean-build:
	@echo "Error: target '$@' was removed."
	@echo "Use one of: install, rebuild, run, stop, sim, test, lint, clean."
	@exit 2
