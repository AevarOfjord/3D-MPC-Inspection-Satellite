"""
Pytest configuration and shared fixtures.

This file provides common fixtures and configuration for all tests.
"""

import pathlib
import sys

# ---------------------------------------------------------------------------
# Ensure ``src/`` is importable even when the editable install is broken.
#
# macOS APFS marks every file inside a dot-prefixed directory (like .venv311)
# with UF_HIDDEN, causing Python's ``site.py`` to skip all ``.pth`` files.
# Prepending ``src/`` to ``sys.path`` and symlinking the C++ ``.so`` files
# into ``src/satellite_control/cpp/`` is the most reliable cross-platform
# workaround.
# ---------------------------------------------------------------------------
_SRC_DIR = str(pathlib.Path(__file__).resolve().parent.parent / "src" / "python")
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

import matplotlib.pyplot as plt
import pytest
from satellite_control.config.simulation_config import SimulationConfig

# ============================================================================
# Configuration Reset Fixture
# ============================================================================


@pytest.fixture(autouse=True)
def fresh_config():
    """
    Provide a fresh SimulationConfig (preferred in v3.0.0).

    This replaces resetting SatelliteConfig and encourages tests to
    depend on explicit configs.
    """
    sim_config = SimulationConfig.create_default()
    yield sim_config


# ============================================================================
# Pytest Configuration Hooks
# ============================================================================


def pytest_configure(config):
    """Configure pytest with custom settings."""
    config.addinivalue_line("markers", "unit: mark test as a unit test")
    config.addinivalue_line("markers", "integration: mark test as an integration test")
    config.addinivalue_line("markers", "hardware: mark test as requiring hardware")
    config.addinivalue_line("markers", "slow: mark test as slow running")


def pytest_collection_modifyitems(config, items):
    """Modify test collection to add markers automatically."""
    for item in items:
        # Add 'unit' marker to tests in test_unit_* files
        if "test_unit_" in item.nodeid:
            item.add_marker(pytest.mark.unit)

        # Add 'integration' marker to tests in test_integration_* files
        if "test_integration_" in item.nodeid:
            item.add_marker(pytest.mark.integration)

        # Add 'hardware' marker to hardware tests
        if "hardware" in item.nodeid.lower():
            item.add_marker(pytest.mark.hardware)


@pytest.fixture(autouse=True)
def cleanup_matplotlib():
    """Automatically close all matplotlib figures after each test."""
    yield
    plt.close("all")
