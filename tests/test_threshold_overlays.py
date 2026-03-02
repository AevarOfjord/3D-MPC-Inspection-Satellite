import matplotlib.pyplot as plt
import numpy as np

from controller.shared.python.visualization.threshold_overlays import (
    add_limit_band,
    add_limit_overlay,
)


def test_add_limit_overlay_draws_lines_and_fill() -> None:
    fig, ax = plt.subplots(1, 1)
    time = np.linspace(0.0, 4.0, 20)
    values = np.linspace(-2.0, 2.0, 20)
    add_limit_overlay(
        ax,
        time_s=time,
        values=values,
        limit=0.5,
        label="threshold",
        symmetric=True,
    )
    # +limit and -limit lines
    assert len(ax.lines) >= 2
    # At least one fill collection for breach region.
    assert len(ax.collections) >= 1
    plt.close(fig)


def test_add_limit_band_draws_expected_bounds() -> None:
    fig, ax = plt.subplots(1, 1)
    add_limit_band(ax, lower=-0.2, upper=0.2, label="band")
    assert len(ax.lines) == 2
    assert len(ax.patches) >= 1
    plt.close(fig)
