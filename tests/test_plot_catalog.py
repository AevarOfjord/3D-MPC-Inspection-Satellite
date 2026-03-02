from controller.shared.python.visualization.plot_catalog import (
    GROUP_BY_ID,
    PLOT_GROUPS,
    PLOT_SPECS,
)


def test_plot_catalog_integrity() -> None:
    group_ids = [group.id for group in PLOT_GROUPS]
    assert len(group_ids) == len(set(group_ids)), "Group IDs must be unique"
    assert tuple(sorted(group_ids)) == tuple(sorted(GROUP_BY_ID.keys()))

    group_orders = [group.order for group in PLOT_GROUPS]
    assert group_orders == sorted(group_orders), "Group ordering must be deterministic"

    plot_ids = [spec.plot_id for spec in PLOT_SPECS]
    assert len(plot_ids) == len(set(plot_ids)), "Plot IDs must be unique"

    spec_orders = [spec.order for spec in PLOT_SPECS]
    assert spec_orders == sorted(spec_orders), "Plot ordering must be deterministic"

    assert len(PLOT_SPECS) >= 30, "Full profile should include 25+ plots"

    for spec in PLOT_SPECS:
        assert spec.group_id in GROUP_BY_ID
        assert len(spec.filename) > 0
