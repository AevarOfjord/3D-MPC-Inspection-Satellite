"""Shared data-context helpers for visualization plotting modules."""

from typing import Any, Optional, Sequence, Tuple

import numpy as np


def resolve_data_frame_and_columns(data_accessor: Any) -> Tuple[Optional[Any], Sequence[str]]:
    """Resolve primary dataframe-like source and available columns."""
    if hasattr(data_accessor, "control_data") and data_accessor.control_data is not None:
        df = data_accessor.control_data
        return df, df.columns

    if (
        hasattr(data_accessor, "_data_backend")
        and data_accessor._data_backend == "pandas"
        and hasattr(data_accessor, "data")
        and data_accessor.data is not None
    ):
        df = data_accessor.data
        return df, df.columns

    if hasattr(data_accessor, "_col_data") and data_accessor._col_data is not None:
        return None, list(data_accessor._col_data.keys())

    return None, []


def get_control_time_axis(
    *,
    df: Optional[Any],
    cols: Sequence[str],
    fallback_len: int,
    dt: float,
) -> np.ndarray:
    """Build a time axis from control logs when possible, otherwise fallback."""
    if df is not None and "Control_Time" in cols:
        return df["Control_Time"].values
    if df is not None and "CONTROL_DT" in cols:
        dt_val = df["CONTROL_DT"].iloc[0]
        return np.arange(len(df)) * float(dt_val)
    base_len = len(df) if df is not None else fallback_len
    return np.arange(base_len) * float(dt)


def get_series(plot_gen: Any, name: str, df: Optional[Any], cols: Sequence[str]) -> np.ndarray:
    """Read series from dataframe source when available, else fallback to plot accessor."""
    if df is not None and name in cols:
        return df[name].values
    return plot_gen._col(name)


def normalize_series(values: np.ndarray, base_len: int) -> np.ndarray:
    """Pad/truncate series to target length and coerce to float."""
    if values is None or len(values) == 0:
        return np.zeros(base_len, dtype=float)
    try:
        arr = np.array(values, dtype=float)
    except (ValueError, TypeError):
        return np.zeros(base_len, dtype=float)
    if arr.size < base_len:
        padded = np.zeros(base_len, dtype=float)
        padded[: arr.size] = arr
        return padded
    return arr[:base_len]


def has_valve_data(data_accessor: Any) -> bool:
    """Check whether actuator valve columns are available."""
    if (
        hasattr(data_accessor, "_data_backend")
        and data_accessor._data_backend == "pandas"
        and hasattr(data_accessor, "data")
        and data_accessor.data is not None
    ):
        return "Thruster_1_Val" in data_accessor.data.columns
    if hasattr(data_accessor, "_col_data") and data_accessor._col_data is not None:
        return "Thruster_1_Val" in data_accessor._col_data
    return False
