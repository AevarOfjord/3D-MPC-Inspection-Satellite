"""Shared command-vector and thruster-count utilities."""

from typing import Any

import numpy as np


def parse_command_vector_raw(command_str: Any) -> np.ndarray:
    """Parse command vector without shape assumptions."""
    try:
        if command_str is None:
            return np.array([])
        command_str = str(command_str).strip()
        if command_str == "":
            return np.array([])
        values = [float(x.strip()) for x in command_str.strip("[]").split(",")]
        return np.array(values, dtype=float)
    except Exception:
        return np.array([])


def get_thruster_count(data_accessor: Any, app_config: Any | None = None) -> int:
    """Infer thruster count from available data and configuration."""
    cols: list[str] = []

    if (
        hasattr(data_accessor, "_data_backend")
        and data_accessor._data_backend == "pandas"
        and hasattr(data_accessor, "data")
        and data_accessor.data is not None
    ):
        cols = list(data_accessor.data.columns)
    elif hasattr(data_accessor, "_col_data") and data_accessor._col_data is not None:
        cols = list(data_accessor._col_data.keys())

    max_id = 0
    if cols:
        for i in range(1, 33):
            if f"Thruster_{i}_Val" in cols or f"Thruster_{i}_Cmd" in cols:
                max_id = i
    if max_id > 0:
        return max_id

    control_data = getattr(data_accessor, "control_data", None)
    if control_data is not None and len(control_data) > 0:
        if "Command_Vector" in control_data.columns:
            sample = parse_command_vector_raw(control_data["Command_Vector"].iloc[0])
            if sample.size > 0:
                return int(sample.size)

    try:
        if app_config is not None and getattr(app_config, "physics", None) is not None:
            return len(app_config.physics.thruster_positions)
        from satellite_control.config.simulation_config import SimulationConfig

        default_config = SimulationConfig.create_default()
        return len(default_config.app_config.physics.thruster_positions)
    except Exception:
        return 8


def parse_command_vector(
    command_str: Any,
    data_accessor: Any,
    app_config: Any | None = None,
) -> np.ndarray:
    """Parse command vector with robust zero-filled fallback."""
    thruster_count = get_thruster_count(data_accessor, app_config)
    parsed = parse_command_vector_raw(command_str)
    if parsed.size == 0:
        return np.zeros(thruster_count, dtype=float)
    return parsed
