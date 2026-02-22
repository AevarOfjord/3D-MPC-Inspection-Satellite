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
SYSTEM_CMAKE := $(shell PATH=$$(echo "$$PATH" | sed 's|$(CURDIR)/$(VENV_BIN):||g; s|$(CURDIR)/$(VENV_BIN)||g') command -v cmake 2>/dev/null || echo "")
UI_DIR ?= ui
UI_NODE_MODULES := $(UI_DIR)/node_modules
UI_LOCKFILES := $(UI_DIR)/package-lock.json $(UI_DIR)/package.json
ASSET_MODEL_DIR := data/assets/model_files
UI_DIST_MODEL_DIR := $(UI_DIR)/dist/model_files
RELEASE_DIR ?= release
APP_BUNDLE_NAME ?= satellite-control-app
APP_BUNDLE_DIR := $(RELEASE_DIR)/$(APP_BUNDLE_NAME)
PYINSTALLER_APP_NAME ?= SatelliteControl
PYINSTALLER_BUNDLE_DIR := $(RELEASE_DIR)/pyinstaller/$(PLATFORM)/$(PYINSTALLER_APP_NAME)
PACKAGE_MAX_MB ?= 150
PYINSTALLER_MAX_MB ?= 900
# Stamp file used to avoid reinstalling Node dependencies on every `make run`.
UI_DEPS_STAMP := $(UI_NODE_MODULES)/.deps-installed
# Try to detect active scikit-build editable output directory.
SKBUILD_MATCHED_EXT := $(firstword $(wildcard $(BUILD_GLOB)))
SKBUILD_BUILD_DIR := $(if $(SKBUILD_MATCHED_EXT),$(patsubst %/,%,$(abspath $(dir $(SKBUILD_MATCHED_EXT)))))
# Skip scikit-build editable runtime rebuild checks by default for faster startup.
SKBUILD_SKIP_RUNTIME_REBUILD ?= 1
SKBUILD_RUNTIME_ENV := $(if $(and $(filter 1 true TRUE yes YES,$(SKBUILD_SKIP_RUNTIME_REBUILD)),$(SKBUILD_BUILD_DIR)),SKBUILD_EDITABLE_SKIP="$(SKBUILD_BUILD_DIR)")
LINT_BACKEND_CMD := $(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m ruff check src/python tests
TEST_COV_CMD := $(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m pytest -q --tb=short --cov=src/python --cov-report=term-missing --cov-fail-under=30

# ============================================================================
# Help
# ============================================================================

.PHONY: help run run-app stop backend backend-prod frontend ui-build sync-ui-model-assets package-app package-pyinstaller smoke-pyinstaller package-clean \
	sim install test test-cov test-ui test-ui-e2e lint lint-backend lint-ui docs-build release-v4-beta release-v4-final clean rebuild \
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
	@echo "  make sync-ui-model-assets Sync canonical data/assets/model_files -> ui/dist/model_files"
	@echo "  make ui-build     Build production UI bundle into ui/dist"
	@echo "  make package-app  Create distributable prebuilt app bundle under ./release"
	@echo "  make package-pyinstaller Build OS-native PyInstaller bundle + archive under ./release"
	@echo "  make smoke-pyinstaller Launch smoke test on latest PyInstaller bundle"
	@echo "  make package-clean Remove generated app bundles in ./release"
	@echo "  make sim          Run CLI simulation (prompts to run tests first)"
	@echo "  make test         Run pytest suite"
	@echo "  make test-cov     Run pytest with coverage gate (>=30%)"
	@echo "  make test-ui      Run frontend unit/component tests (Vitest)"
	@echo "  make test-ui-e2e  Run frontend Playwright smoke tests"
	@echo "  make lint-backend Run backend lint checks (canonical command)"
	@echo "  make lint-ui      Run frontend lint checks"
	@echo "  make lint         Run backend + frontend lint checks"
	@echo "  make docs-build   Build docs with warnings-as-errors"
	@echo "  make release-v4-beta Run V4 beta release gates + packaging and print tag command"
	@echo "  make release-v4-final Run V4 final release gates + packaging and print tag command"
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
	@pkill -f "cli serve" || true
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
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m cli serve --dev

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
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m cli serve

# Install frontend dependencies only when lockfiles change.
$(UI_DEPS_STAMP): $(UI_LOCKFILES)
	cd $(UI_DIR) && npm install
	@mkdir -p "$(UI_NODE_MODULES)"
	@touch "$(UI_DEPS_STAMP)"

# Start frontend dev server.
frontend: $(UI_DEPS_STAMP)
	cd $(UI_DIR) && npm run dev

# Keep `data/assets/model_files` as the canonical source and mirror into built UI assets.
sync-ui-model-assets:
	@mkdir -p "$(UI_DIST_MODEL_DIR)"
	@rsync -a --delete "$(ASSET_MODEL_DIR)/" "$(UI_DIST_MODEL_DIR)/"

# Build production UI assets consumed by backend-prod/run-app.
ui-build: $(UI_DEPS_STAMP)
	cd $(UI_DIR) && npm run build
	@$(MAKE) sync-ui-model-assets

# One-command app startup using prebuilt UI bundle.
run-app: stop backend-prod

# Create a distributable app bundle with prebuilt UI and current runtime env.
package-app: ui-build
	@$(MAKE) venv
	@if ! $(VENV_PY) -c "import cli, fastapi, uvicorn" >/dev/null 2>&1; then \
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
		--exclude '.venv311/' \
		--exclude 'ui/node_modules/' \
		--exclude 'ui/npm_cache/' \
		--exclude 'ui/public/model_files/' \
		--exclude 'ui/dist/model_files/' \
		--exclude '/data/simulation_data/' \
		--exclude '/Data/Simulation/' \
		./ "$(APP_BUNDLE_DIR)/"
	@mkdir -p "$(APP_BUNDLE_DIR)/data/simulation_data"
	@mkdir -p "$(APP_BUNDLE_DIR)/data/dashboard"
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
	ARCHIVE_BYTES=$$(wc -c < "$$ARCHIVE_PATH"); \
	ARCHIVE_MB=$$(( (ARCHIVE_BYTES + 1048575) / 1048576 )); \
	MAX_BYTES=$$(( $(PACKAGE_MAX_MB) * 1024 * 1024 )); \
	if [ "$(PACKAGE_MAX_MB)" -gt 0 ] && [ "$$ARCHIVE_BYTES" -gt "$$MAX_BYTES" ]; then \
		echo "Error: package archive exceeds limit ($${ARCHIVE_MB}MB > $(PACKAGE_MAX_MB)MB)."; \
		echo "Set PACKAGE_MAX_MB=0 to disable the check intentionally."; \
		exit 1; \
	fi; \
	echo "Created app archive: $$ARCHIVE_PATH ($${ARCHIVE_MB}MB)"

# Remove generated distributable bundles.
package-clean:
	@rm -rf "$(RELEASE_DIR)"
	@echo "Removed $(RELEASE_DIR)"

# Build OS-native PyInstaller bundle and archive.
package-pyinstaller: ui-build
	@$(MAKE) venv
	@if ! $(VENV_PY) -c "import cli, fastapi, uvicorn" >/dev/null 2>&1; then \
		echo "Runtime deps missing in $(VENV_DIR); running 'make install' first..."; \
		$(MAKE) install || exit $$?; \
	fi
	@if ! $(VENV_PY) -c "import PyInstaller" >/dev/null 2>&1; then \
		echo "PyInstaller missing in $(VENV_DIR); installing..."; \
		$(VENV_PY) -m pip install "pyinstaller>=6.0.0" || exit $$?; \
	fi
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" \
		$(VENV_PY) scripts/build_pyinstaller_bundle.py --max-mb "$(PYINSTALLER_MAX_MB)"

# Launch latest PyInstaller output and verify HTTP readiness.
smoke-pyinstaller:
	@$(MAKE) venv
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" \
		$(VENV_PY) scripts/smoke_test_packaged_app.py

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
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m cli run

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
	@echo "=== Installing C++ build dependencies ==="
	$(VENV_PY) -m pip install "scikit-build-core>=0.3.3" pybind11 "ninja>=1.10"
	@echo ""
	@echo "=== Installing project + development dependencies ==="
	CMAKE_GENERATOR="$(CMAKE_GENERATOR)" CMAKE_MAKE_PROGRAM="$(CMAKE_MAKE_PROGRAM)" \
	SKBUILD_CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" CMAKE_EXECUTABLE="$(SYSTEM_CMAKE)" \
		$(VENV_PY) -m pip install --no-build-isolation -e ".[dev]"
	@$(VENV_PY) -c "from cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ modules loaded OK')"

# ============================================================================
# Quality targets
# ============================================================================

# Run test suite.
test:
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_PY) -m pytest -q --tb=short

# Run test suite with coverage quality gate.
test-cov:
	$(TEST_COV_CMD)

# Run frontend unit/component tests.
test-ui: $(UI_DEPS_STAMP)
	cd ui && npm run test

# Run frontend E2E smoke tests.
test-ui-e2e: $(UI_DEPS_STAMP)
	cd ui && npm run test:e2e

# Build docs with warnings treated as errors.
docs-build:
	@echo "Building MkDocs documentation..."
	@if ! $(VENV_PY) -c "import mkdocs" >/dev/null 2>&1; then \
		echo "Missing docs dependencies. Install with: $(VENV_PY) -m pip install -e \".[docs]\""; \
		exit 1; \
	fi
	$(SKBUILD_RUNTIME_ENV) PYTHONPATH="$(CURDIR)/src/python$${PYTHONPATH:+:$$PYTHONPATH}" $(VENV_ACTIVATE) && mkdocs build --strict

# Run full V4 beta release verification and packaging.
release-v4-beta: lint test-cov test-ui test-ui-e2e docs-build package-app package-pyinstaller
	@echo "V4 beta gates passed."
	@echo "Next: git tag -a v4.0.0-beta.1 -m \"V4.0.0 beta.1\""

# Run full V4 final release verification and packaging.
release-v4-final: lint test-cov test-ui test-ui-e2e docs-build package-app package-pyinstaller
	@echo "V4 final gates passed."
	@echo "Next: git tag -a v4.0.0 -m \"V4.0.0\""

# Canonical backend lint command reused by CI and README.
lint-backend:
	$(LINT_BACKEND_CMD)

# Frontend lint command.
lint-ui: $(UI_DEPS_STAMP)
	cd ui && npx eslint .

# Run Python + frontend lint checks.
lint: lint-backend lint-ui

# ============================================================================
# Clean targets
# ============================================================================

# Remove local build, venv, and cache artifacts.
clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/python/cpp/*$(EXT_SUFFIX)
	@rm -rf ui/node_modules/.vite
	@rm -rf .pytest_cache .ruff_cache
	@echo "Cleaned."

# Removed targets: fail fast so stale scripts don't silently no-op.
build dashboard install-dev clean-build:
	@echo "Error: target '$@' was removed."
	@echo "Use one of: install, rebuild, run, stop, sim, test, lint, clean."
	@exit 2
