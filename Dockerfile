# syntax=docker/dockerfile:1
FROM python:3.11-slim AS base

# Install system deps for C++ build + FFmpeg
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake ninja-build git ffmpeg \
    libeigen3-dev libosqp-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python deps first (cache-friendly layer ordering)
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt \
    && pip install --no-cache-dir scikit-build-core pybind11 cmake ninja

# Copy source and build C++ extensions
COPY CMakeLists.txt pyproject.toml MANIFEST.in ./
COPY src/ src/
RUN pip install --no-build-isolation -e .

# Verify C++ extensions load
RUN python -c "from satellite_control.cpp import _cpp_mpc, _cpp_sim, _cpp_physics; print('C++ OK')"

# Copy remaining project files
COPY run_dashboard.py run_simulation.py ./
COPY missions/ missions/
COPY assets/ assets/

# Dashboard port
EXPOSE 8000

# Default: run the dashboard backend
ENV SATELLITE_HEADLESS=1
CMD ["python", "run_dashboard.py"]
