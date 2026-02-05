.PHONY: run run-backend run-frontend sim install install-dev venv clean

VENV_DIR ?= .venv311
PYTHON ?= python3.11
VENV_BIN := $(VENV_DIR)/bin
VENV_PY := $(VENV_BIN)/python
VENV_PIP := $(VENV_BIN)/pip
REQS_FILE ?= requirements.txt
DEV_REQS_FILE ?= requirements-dev.txt

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

venv:
	@$(PYTHON) -m venv $(VENV_DIR)
	@$(VENV_PIP) install --upgrade pip
	@echo "Virtual environment created."
	@echo "To activate it manually, run: source $(VENV_DIR)/bin/activate"

install: venv
	@$(VENV_PIP) install -r $(REQS_FILE)
	@$(VENV_PIP) install --no-build-isolation -e .
	@cp build/cp3*-cp3*-macosx_*_arm64/*.so src/satellite_control/cpp/ || echo "Warning: Could not auto-copy .so files"

install-dev: venv
	@$(VENV_PIP) install -r $(DEV_REQS_FILE)
	@$(VENV_PIP) install --no-build-isolation -e .
	@cp build/cp3*-cp3*-macosx_*_arm64/*.so src/satellite_control/cpp/ || echo "Warning: Could not auto-copy .so files"

clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/satellite_control/cpp/*.so
	@rm -rf .venv311
	@rm -rf ui/node_modules/.vite
