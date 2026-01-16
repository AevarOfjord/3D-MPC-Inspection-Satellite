#!/usr/bin/env python3
"""
Satellite Control Simulation Entry Point
Delegates to the new CLI interface.
"""

import os
import sys

# Add the current directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.satellite_control.cli import app  # noqa: E402

if __name__ == "__main__":
    app()
