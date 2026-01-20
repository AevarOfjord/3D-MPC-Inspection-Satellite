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

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Install the package (builds C++ extension)
echo "Installing satellite_control package (showing build progress)..."
pip install -v .

# Copy .so files to src for development
echo "Copying compiled extensions..."
cp .venv311/lib/python3.11/site-packages/satellite_control/cpp/*.so src/satellite_control/cpp/ 2>/dev/null || true

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
