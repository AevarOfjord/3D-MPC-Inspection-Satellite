"""Shared plotting style constants and helpers for visualization modules."""

import matplotlib.pyplot as plt
from cycler import cycler


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
    FONT_FAMILY = "sans-serif"
    FONT_FALLBACKS = [
        "DejaVu Sans",
        "Arial",
        "Liberation Sans",
        "Helvetica",
        "Noto Sans",
    ]
    BASE_FONT_SIZE = 11
    TICK_LABEL_SIZE = 10
    AXIS_LABEL_SIZE = 14
    LEGEND_SIZE = 12
    TITLE_SIZE = 16
    SUPTITLE_SIZE = 17
    ANNOTATION_SIZE = 12

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
    COLOR_WARNING = "#f59e0b"
    COLOR_MUTED = "#6b7280"
    COLOR_BG = "#ffffff"
    COLOR_GRID = "#d1d5db"

    # Text box style (Minimalist)
    TEXTBOX_STYLE = dict(
        boxstyle="round,pad=0.3",
        facecolor=COLOR_BG,
        edgecolor=COLOR_PRIMARY,
        linewidth=0.5,
        alpha=1.0,  # Opaque
    )

    PALETTE = [
        COLOR_SIGNAL_POS,
        COLOR_SIGNAL_ANG,
        COLOR_TARGET,
        COLOR_WARNING,
        COLOR_MUTED,
        "#14b8a6",
        "#a855f7",
        "#0f172a",
    ]

    @staticmethod
    def apply_global_theme() -> None:
        """Apply consistent global plot theme for all generated figures."""
        plt.rcParams.update(
            {
                "font.family": PlotStyle.FONT_FAMILY,
                "font.sans-serif": PlotStyle.FONT_FALLBACKS,
                "font.size": PlotStyle.BASE_FONT_SIZE,
                "axes.titlesize": PlotStyle.TITLE_SIZE,
                "axes.labelsize": PlotStyle.AXIS_LABEL_SIZE,
                "xtick.labelsize": PlotStyle.TICK_LABEL_SIZE,
                "ytick.labelsize": PlotStyle.TICK_LABEL_SIZE,
                "legend.fontsize": PlotStyle.LEGEND_SIZE,
                "axes.prop_cycle": cycler(color=PlotStyle.PALETTE),
                "axes.facecolor": PlotStyle.COLOR_BG,
                "figure.facecolor": PlotStyle.COLOR_BG,
                "grid.color": PlotStyle.COLOR_GRID,
                "grid.alpha": PlotStyle.GRID_ALPHA,
                "savefig.facecolor": PlotStyle.COLOR_BG,
                "savefig.edgecolor": PlotStyle.COLOR_BG,
            }
        )

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
    def style_figure(fig) -> None:
        """Normalize typography and legend styling across all axes in a figure."""
        if fig._suptitle is not None:  # noqa: SLF001
            fig._suptitle.set_fontsize(PlotStyle.SUPTITLE_SIZE)  # noqa: SLF001
            fig._suptitle.set_fontweight("semibold")  # noqa: SLF001

        for ax in fig.axes:
            title = ax.get_title()
            if title:
                ax.set_title(
                    title, fontsize=PlotStyle.TITLE_SIZE, fontweight="semibold"
                )
            xlabel = ax.get_xlabel()
            if xlabel:
                ax.set_xlabel(xlabel, fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ylabel = ax.get_ylabel()
            if ylabel:
                ax.set_ylabel(ylabel, fontsize=PlotStyle.AXIS_LABEL_SIZE)
            ax.tick_params(labelsize=PlotStyle.TICK_LABEL_SIZE)
            legend = ax.get_legend()
            if legend is not None:
                for txt in legend.get_texts():
                    txt.set_fontsize(PlotStyle.LEGEND_SIZE)

    @staticmethod
    def save_figure(fig, path, close: bool = True):
        """Save figure with consistent settings."""
        PlotStyle.apply_global_theme()
        PlotStyle.style_figure(fig)
        path.parent.mkdir(parents=True, exist_ok=True)
        plt.tight_layout()
        fig.savefig(path, dpi=PlotStyle.DPI, bbox_inches="tight")
        if close:
            plt.close(fig)
