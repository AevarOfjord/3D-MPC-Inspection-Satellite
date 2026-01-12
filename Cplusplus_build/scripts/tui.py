#!/usr/bin/env python3
"""
Terminal User Interface for Satellite Control System

Usage:
    python3 scripts/tui.py
"""

import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tui.menu import MainMenu


def main():
    """Entry point for TUI."""
    try:
        menu = MainMenu()
        menu.run()
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)


if __name__ == "__main__":
    main()
