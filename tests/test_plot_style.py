import matplotlib.pyplot as plt
from visualization.plot_style import PlotStyle


def test_apply_global_theme_sets_core_rcparams() -> None:
    PlotStyle.apply_global_theme()
    assert str(plt.rcParams["font.family"][0]) == PlotStyle.FONT_FAMILY
    assert float(plt.rcParams["axes.labelsize"]) == float(PlotStyle.AXIS_LABEL_SIZE)


def test_style_figure_normalizes_font_sizes() -> None:
    fig, ax = plt.subplots(1, 1)
    ax.set_title("T")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.plot([0, 1], [0, 1], label="line")
    ax.legend()
    PlotStyle.style_figure(fig)
    assert int(ax.xaxis.label.get_size()) == PlotStyle.AXIS_LABEL_SIZE
    assert int(ax.yaxis.label.get_size()) == PlotStyle.AXIS_LABEL_SIZE
    assert int(ax.title.get_size()) == PlotStyle.TITLE_SIZE
    plt.close(fig)
