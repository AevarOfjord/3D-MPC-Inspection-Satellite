#!/bin/bash
# Satellite Control System - Setup Script
# This script creates a clean virtual environment and installs all dependencies.
# Run: chmod +x setup.sh && ./setup.sh

set -e

echo "🛰️  Satellite Control System - Clean Install"
echo "============================================="

# Clean up any existing environment
if [ -d ".venv311" ]; then
    echo "Removing existing virtual environment..."
    rm -rf .venv311
fi

if [ -d "build" ]; then
    echo "Removing existing build directory..."
    rm -rf build
fi

if [ -d "ui/node_modules" ]; then
    echo "Removing ui/node_modules to speed up backend build..."
    rm -rf ui/node_modules
fi

# Create fresh virtual environment
echo "Creating virtual environment with Python 3.11..."
python3.11 -m venv .venv311

# Activate environment
source .venv311/bin/activate

# Prefer Makefiles to avoid missing ninja during build isolation
export CMAKE_GENERATOR="Unix Makefiles"
export SKBUILD_GENERATOR="Unix Makefiles"

# Upgrade pip
echo "Upgrading pip..."
python -m pip install --upgrade pip

# Install Python dependencies
echo "Installing Python dependencies..."
python -m pip install -r requirements-dev.txt

# Install build tools required for C++ extension
echo "Installing build tools (ninja)..."
python -m pip install ninja

# Install the package (builds C++ extension)
echo "Installing satellite_control package (showing build progress, no build isolation)..."
python -m pip install -v --no-build-isolation -e .

# Install frontend dependencies
echo "Installing frontend dependencies..."
cd ui
npm install --legacy-peer-deps
cd ..

echo ""
echo "✅ Installation complete!"
echo ""
echo "To run the system:"
echo "  1. Activate the environment: source .venv311/bin/activate"
echo "  2. Start backend + frontend:  make run"
echo "  3. Or run simulation only:    make sim"
echo ""
