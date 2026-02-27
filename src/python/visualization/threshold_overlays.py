"""Shared threshold/limit overlays for post-run plots."""

from __future__ import annotations

import numpy as np
from matplotlib.axes import Axes
from visualization.plot_style import PlotStyle


def add_limit_overlay(
    ax: Axes,
    *,
    time_s: np.ndarray,
    values: np.ndarray,
    limit: float,
    label: str,
    symmetric: bool = True,
    color: str = PlotStyle.COLOR_THRESHOLD,
    breach_alpha: float = 0.16,
) -> None:
    """Draw threshold lines and translucent breach fill."""
    if limit <= 0.0:
        return

    ax.axhline(
        y=limit,
        color=color,
        linestyle="--",
        linewidth=1.2,
        label=label,
    )

    if symmetric:
        ax.axhline(
            y=-limit,
            color=color,
            linestyle="--",
            linewidth=1.2,
        )
        ax.fill_between(
            time_s,
            limit,
            values,
            where=values > limit,
            color=color,
            alpha=breach_alpha,
            interpolate=True,
        )
        ax.fill_between(
            time_s,
            -limit,
            values,
            where=values < -limit,
            color=color,
            alpha=breach_alpha,
            interpolate=True,
        )
        return

    ax.fill_between(
        time_s,
        limit,
        values,
        where=values > limit,
        color=color,
        alpha=breach_alpha,
        interpolate=True,
    )


def add_limit_band(
    ax: Axes,
    *,
    lower: float,
    upper: float,
    label: str,
    color: str = PlotStyle.COLOR_THRESHOLD,
    alpha: float = 0.08,
) -> None:
    """Draw a translucent threshold band."""
    if upper <= lower:
        return
    ax.axhline(y=lower, color=color, linestyle="--", linewidth=1.0)
    ax.axhline(y=upper, color=color, linestyle="--", linewidth=1.0, label=label)
    ax.axhspan(lower, upper, color=color, alpha=alpha)
