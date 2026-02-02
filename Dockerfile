# Dockerfile for Satellite Control System
# Multi-stage build for optimized image size

# Stage 1: Build stage
FROM python:3.11-slim as builder

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /build

# Copy source
COPY . .

# Build wheels (project + dependencies)
RUN pip install --no-cache-dir --upgrade pip && \
    pip wheel --no-cache-dir . -w /wheels

# Stage 2: Runtime stage
FROM python:3.11-slim

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgomp1 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
RUN useradd -m -u 1000 satellite && \
    mkdir -p /app /data && \
    chown -R satellite:satellite /app /data

# Set working directory
WORKDIR /app

# Copy wheels from builder
COPY --from=builder /wheels /wheels

# Copy application code
COPY --chown=satellite:satellite . .

# Install application + deps from local wheels
RUN pip install --no-cache-dir --no-index --find-links /wheels /wheels/*.whl && \
    python - <<'PY'
import glob
import os
import shutil
import site

site_pkgs = site.getsitepackages()[0]
src_dir = "/app/src/satellite_control/cpp"
os.makedirs(src_dir, exist_ok=True)
for so_path in glob.glob(os.path.join(site_pkgs, "satellite_control", "cpp", "*.so")):
    shutil.copy2(so_path, src_dir)
PY

# Switch to non-root user
USER satellite

# Set environment variables
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1

# Default command (headless simulation)
CMD ["python", "run_simulation.py", "run", "--headless"]
