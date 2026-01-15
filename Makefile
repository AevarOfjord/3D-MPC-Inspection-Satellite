.PHONY: run run-backend run-frontend

run:
	@$(MAKE) -j2 run-backend run-frontend

run-backend:
	python3 run_dashboard.py

run-frontend:
	cd ui && npm install && npm run dev
