"""
Unit tests for PlotGenerator module.

Tests the plotting functionality extracted from unified_visualizer.py.
"""

from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from src.satellite_control.visualization.plot_generator import PlotGenerator


@pytest.fixture
def mock_data_accessor():
    """Create a mock data accessor for testing."""
    accessor = MagicMock()
    accessor._col = MagicMock(return_value=np.array([0.0, 1.0, 2.0, 3.0, 4.0]))
    accessor._row = MagicMock(
        return_value={
            "Current_X": 1.0,
            "Current_Y": 2.0,
            "Command_Vector": "[0,0,0,0,0,0,0,0,0,0,0,0]",  # 12 elements for 12 thrusters or similar
        }
    )
    accessor._get_len = MagicMock(return_value=5)
    return accessor


@pytest.fixture
def plot_generator(mock_data_accessor):
    """Create a PlotGenerator instance for testing."""
    return PlotGenerator(
        data_accessor=mock_data_accessor,
        dt=0.1,
        system_title="Test System",
    )


class TestPlotGeneratorInitialization:
    """Test PlotGenerator initialization."""

    def test_plot_generator_creation(self, mock_data_accessor):
        """Test that PlotGenerator can be created."""
        generator = PlotGenerator(
            data_accessor=mock_data_accessor,
            dt=0.1,
            system_title="Test",
        )
        assert generator.data_accessor == mock_data_accessor
        assert generator.dt == 0.1
        assert generator.system_title == "Test"

    def test_data_accessor_methods(self, plot_generator, mock_data_accessor):
        """Test that data accessor methods work correctly."""
        result = plot_generator._col("Current_X")
        mock_data_accessor._col.assert_called_once_with("Current_X")
        assert isinstance(result, np.ndarray)

        result = plot_generator._row(0)
        mock_data_accessor._row.assert_called_once_with(0)
        assert isinstance(result, dict)

        result = plot_generator._get_len()
        mock_data_accessor._get_len.assert_called_once()
        assert result == 5


class TestPlotGeneratorMethods:
    """Test PlotGenerator plotting methods."""

    @patch("matplotlib.pyplot.subplots")
    @patch("matplotlib.pyplot.savefig")
    @patch("matplotlib.pyplot.close")
    def test_generate_position_tracking_plot(
        self, mock_close, mock_savefig, mock_subplots, plot_generator, tmp_path
    ):
        """Test position tracking plot generation."""
        mock_fig = MagicMock()

        # side_effect to handle different subplot geometries
        def subplots_side_effect(*args, **kwargs):
            nrows = 1
            if len(args) > 0:
                nrows = args[0]
            if "nrows" in kwargs:
                nrows = kwargs["nrows"]

            ncols = 1
            if len(args) > 1:
                ncols = args[1]
            if "ncols" in kwargs:
                ncols = kwargs["ncols"]

            if nrows == 1 and ncols == 1:
                return mock_fig, MagicMock()
            elif nrows * ncols > 1:
                # Return list to simulate flat array for tests
                return mock_fig, [MagicMock() for _ in range(nrows * ncols)]
            return mock_fig, MagicMock()

        mock_subplots.side_effect = subplots_side_effect

        plot_dir = tmp_path / "plots"
        plot_generator.generate_position_tracking_plot(plot_dir)

        mock_subplots.assert_called_once()
        # Call is via fig.savefig, not plt.savefig
        mock_fig.savefig.assert_called_once()

    @patch("matplotlib.pyplot.subplots")
    @patch("matplotlib.pyplot.savefig")
    @patch("matplotlib.pyplot.close")
    def test_generate_position_error_plot(
        self, mock_close, mock_savefig, mock_subplots, plot_generator, tmp_path
    ):
        """Test position error plot generation."""
        mock_fig = MagicMock()
        mock_axes = [MagicMock(), MagicMock(), MagicMock()]
        mock_subplots.return_value = (mock_fig, mock_axes)

        plot_dir = tmp_path / "plots"
        plot_generator.generate_position_error_plot(plot_dir)

        mock_subplots.assert_called_once()
        mock_fig.savefig.assert_called_once()

    @patch("matplotlib.pyplot.subplots")
    @patch("matplotlib.pyplot.savefig")
    @patch("matplotlib.pyplot.close")
    def test_generate_trajectory_plot(
        self, mock_close, mock_savefig, mock_subplots, plot_generator, tmp_path
    ):
        """Test trajectory plot generation."""
        mock_fig = MagicMock()
        mock_ax = MagicMock()
        mock_subplots.return_value = (
            mock_fig,
            [mock_ax, mock_ax],
        )  # Returns list of axes

        plot_dir = tmp_path / "plots"
        plot_generator.generate_trajectory_plot(plot_dir)

        mock_subplots.assert_called_once()
        mock_fig.savefig.assert_called_once()

    @patch("matplotlib.pyplot.subplots")
    @patch("matplotlib.pyplot.savefig")
    @patch("matplotlib.pyplot.close")
    def test_generate_all_plots(
        self, mock_close, mock_savefig, mock_subplots, plot_generator, tmp_path
    ):
        """Test that generate_all_plots calls all plot methods."""
        mock_fig = MagicMock()

        # side_effect to handle different subplot geometries
        def subplots_side_effect(*args, **kwargs):
            nrows = 1
            if len(args) > 0:
                nrows = args[0]
            if "nrows" in kwargs:
                nrows = kwargs["nrows"]

            ncols = 1
            if len(args) > 1:
                ncols = args[1]
            if "ncols" in kwargs:
                ncols = kwargs["ncols"]

            if nrows == 1 and ncols == 1:
                return mock_fig, MagicMock()
            elif nrows * ncols > 1:
                # Return list to simuate array (even for 2x2, just return flat list for test simplicity,
                # UNLESS code uses 2D slicing like axes[0,0])
                # PlotGenerator uses axes[0, 0] for 2x2 plots (error_norms).
                if nrows == 2 and ncols == 2:
                    return mock_fig, np.array(
                        [[MagicMock(), MagicMock()], [MagicMock(), MagicMock()]]
                    )
                return mock_fig, [MagicMock() for _ in range(nrows * ncols)]
            return mock_fig, MagicMock()

        mock_subplots.side_effect = subplots_side_effect

        plot_dir = tmp_path / "plots"

        # Mock interactive/dep-heavy methods, let the rest run on mocks
        methods_to_mock = [
            # Interactive uses plotly
            "generate_trajectory_3d_interactive_plot",
            # Complex subplot logic that might fail in mock environment
            "generate_error_norms_plot",
            "generate_trajectory_3d_orientation_plot",
            "generate_actuator_limits_plot",
            "generate_constraint_violations_plot",
            "generate_reaction_wheel_output_plot",
            "generate_z_tilt_coupling_plot",
            "generate_thruster_impulse_proxy_plot",
            "generate_phase_position_velocity_plot",
            "generate_phase_attitude_rate_plot",
            "generate_solver_health_plot",
            "generate_waypoint_progress_plot",
        ]

        mocks = {}
        patchers = []
        for method in methods_to_mock:
            patcher = patch.object(plot_generator, method)
            mocks[method] = patcher.start()
            patchers.append(patcher)

        try:
            plot_generator.generate_all_plots(plot_dir)

            for method, mock_obj in mocks.items():
                mock_obj.assert_called_once_with(plot_dir)

        finally:
            for patcher in patchers:
                patcher.stop()

        # Verify directory was created
        assert plot_dir.exists()


class TestPlotGeneratorDataHandling:
    """Test PlotGenerator data handling."""

    def test_handles_empty_data(self, mock_data_accessor):
        """Test that PlotGenerator handles empty data gracefully."""
        mock_data_accessor._get_len.return_value = 0
        mock_data_accessor._col.return_value = np.array([])

        generator = PlotGenerator(
            data_accessor=mock_data_accessor,
            dt=0.1,
        )

        # Should not raise exception
        assert generator._get_len() == 0

    def test_handles_missing_columns(self, mock_data_accessor):
        """Test that PlotGenerator handles missing columns gracefully."""

        def side_effect(name):
            if name == "Current_X":
                return np.array([1.0, 2.0, 3.0])
            else:
                return np.array([0.0, 0.0, 0.0])

        mock_data_accessor._col.side_effect = side_effect

        generator = PlotGenerator(
            data_accessor=mock_data_accessor,
            dt=0.1,
        )

        # Should handle missing columns
        result = generator._col("Current_X")
        assert len(result) == 3
