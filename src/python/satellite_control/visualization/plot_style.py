"""Shared plotting style constants and helpers for visualization modules."""

import matplotlib.pyplot as plt


class PlotStyle:
    """Centralized plot styling constants for consistent visualizations."""

    # Figure sizes
    FIGSIZE_SINGLE = (10, 6)  # Slightly smaller for papers
    FIGSIZE_SUBPLOTS = (10, 8)
    FIGSIZE_WIDE = (12, 5)
    FIGSIZE_TALL = (8, 8)

    # Resolution (High for print)
    DPI = 300

    # Line properties
    LINEWIDTH = 1.5
    LINEWIDTH_THICK = 2.5
    MARKER_SIZE = 8

    # Grid
    GRID_ALPHA = 0.15

    # Font sizes
    AXIS_LABEL_SIZE = 14
    LEGEND_SIZE = 12
    TITLE_SIZE = 16
    ANNOTATION_SIZE = 12

    # Text box style (Minimalist)
    TEXTBOX_STYLE = dict(
        boxstyle="round,pad=0.3",
        facecolor="white",
        edgecolor="black",  # High contrast border
        linewidth=0.5,
        alpha=1.0,  # Opaque
    )

    # Colors (4-Color Scheme: Black, Blue, Red, Green)
    COLOR_PRIMARY = "#000000"  # Black (Axes, Text)

    # Semantic Roles
    COLOR_SIGNAL_POS = "#1f77b4"  # Blue (Position Signals)
    COLOR_SIGNAL_ANG = "#2ca02c"  # Green (Angular Signals)

    COLOR_REFERENCE = "#d62728"  # Red (Reference)
    COLOR_THRESHOLD = "#d62728"  # Red (Limits/Tolerances)
    COLOR_TARGET = "#d62728"  # Red (Target/Reference alias)

    COLOR_BARS = "#1f77b4"  # Blue (Thruster Bars)
    COLOR_SUCCESS = "#2ca02c"  # Green
    COLOR_ERROR = "#d62728"  # Red

    # Legacy aliases for compatibility if needed
    COLOR_SIGNAL = "#000000"  # Default black
    COLOR_SECONDARY = "#d62728"  # Red

    @staticmethod
    def apply_axis_style(ax, xlabel: str = "", ylabel: str = "", title: str = ""):
        """Apply consistent styling to an axis."""
        if xlabel:
            ax.set_xlabel(xlabel, fontsize=PlotStyle.AXIS_LABEL_SIZE)
        if ylabel:
            ax.set_ylabel(ylabel, fontsize=PlotStyle.AXIS_LABEL_SIZE)
        if title:
            ax.set_title(title, fontsize=PlotStyle.TITLE_SIZE)
        ax.grid(True, alpha=PlotStyle.GRID_ALPHA)
        ax.legend(fontsize=PlotStyle.LEGEND_SIZE)

    @staticmethod
    def save_figure(fig, path, close: bool = True):
        """Save figure with consistent settings."""
        plt.tight_layout()
        fig.savefig(path, dpi=PlotStyle.DPI, bbox_inches="tight")
        if close:
            plt.close(fig)
