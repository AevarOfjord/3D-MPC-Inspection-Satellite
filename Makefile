.PHONY: run run-backend run-frontend sim install venv clean

VENV_DIR ?= .venv311
PYTHON ?= python3.11
VENV_BIN := $(VENV_DIR)/bin
VENV_PY := $(VENV_BIN)/python
VENV_PIP := $(VENV_BIN)/pip
REQS_FILE ?= requirements.txt

run:
	@$(MAKE) -j2 backend frontend

backend:
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
	@cp $(VENV_DIR)/lib/python3.11/site-packages/satellite_control/cpp/*.so src/satellite_control/cpp/ || true

clean:
	@rm -rf $(VENV_DIR) build dist src/lib
	@rm -f src/satellite_control/cpp/*.so
	@rm -rf .venv311
