"""
Data Logger for Satellite Control System

Centralized data logging and CSV export for simulation runs.
Handles step-by-step data collection and exports to standardized CSV format.

Key features:
- Simulation mode logging
- Detailed log data with full state history per timestep
- Terminal message logging for debugging and analysis
- Automatic CSV export with standardized headers
- Configurable save paths with automatic directory creation
"""

import csv
import logging
from pathlib import Path
from typing import Any

from controller.shared.python.simulation.artifact_paths import artifact_relative_path

logger = logging.getLogger(__name__)


class DataLogger:
    """
    Centralized data logging and CSV export.

    Handles logging of simulation data and exports to CSV format
    for analysis. Used by SatelliteMPCLinearizedSimulation.
    """

    def __init__(
        self,
        mode: str = "simulation",
        buffer_size: int = 200,
        filename: str = "control_data.csv",
        max_terminal_entries: int = 0,
    ):
        """
        Initialize data logger.

        Args:
            mode: "simulation" or "physics"
            buffer_size: Number of entries to keep in memory before flushing to disk
            filename: Name of the CSV file to write
        """
        self.mode = mode.lower()
        if self.mode not in ["simulation", "physics"]:
            raise ValueError("Mode must be 'simulation' or 'physics'")
        self.filename = filename
        self.buffer_size = buffer_size
        self.max_terminal_entries = max(0, int(max_terminal_entries or 0))

        # Internal state
        self.detailed_log_data: list[dict[str, Any]] = []
        self.terminal_log_data: list[dict[str, Any]] = []
        self.data_save_path: Path | None = None
        self.current_step = 0
        self._headers_written = False

        # Cached headers (computed once per mode)
        self._cached_sim_headers: list[str] | None = None
        self._cached_physics_headers: list[str] | None = None

        # Pre-built format dispatch table for _format_value
        self._format_dispatch = self._build_format_dispatch()

        # Incremental stats tracking
        self.stats_solve_times: list[float] = []
        self.stats_timing_violations = 0
        self.stats_time_limit_exceeded = 0
        self.total_steps_recorded = 0

    def set_save_path(self, path: Path) -> None:
        """
        Set the directory path where data will be saved.

        Args:
            path: Directory path for saving data files
        """
        self.data_save_path = path
        if not path.exists():
            path.mkdir(parents=True, exist_ok=True)

        # Reset headers flag when path changes
        self._headers_written = False

    def log_entry(self, entry: dict[str, Any]) -> None:
        """
        Add a log entry to the detailed log data.
        Flushes to disk if buffer size is exceeded.

        Args:
            entry: Dictionary containing log data for one timestep
        """
        self.detailed_log_data.append(entry)
        self.current_step += 1
        self.total_steps_recorded += 1

        # Update incremental stats
        if "MPC_Solve_Time" in entry and entry["MPC_Solve_Time"] != "":
            try:
                self.stats_solve_times.append(float(entry["MPC_Solve_Time"]))
            except (ValueError, TypeError):
                pass

        if entry.get("Timing_Violation") == "YES":
            self.stats_timing_violations += 1

        if entry.get("MPC_Time_Limit_Exceeded") == "YES":
            self.stats_time_limit_exceeded += 1

        if len(self.detailed_log_data) >= self.buffer_size:
            self.flush()

    # ...
    def flush(self) -> bool:
        """
        Flush current buffer to disk.
        """
        if not self.data_save_path or not self.detailed_log_data:
            return False

        csv_file_path = self.data_save_path / self.filename
        csv_file_path.parent.mkdir(parents=True, exist_ok=True)
        headers = (
            self._get_physics_headers()
            if self.mode == "physics"
            else self._get_simulation_headers()
        )

        mode = "a" if self._headers_written else "w"

        try:
            with open(csv_file_path, mode, newline="") as csvfile:
                writer = csv.writer(csvfile)
                if not self._headers_written:
                    writer.writerow(headers)
                    self._headers_written = True

                for log_entry in self.detailed_log_data:
                    row = [
                        self._format_value(header, log_entry.get(header, ""))
                        for header in headers
                    ]
                    writer.writerow(row)

            # Clear buffer after successful write
            self.detailed_log_data = []
            return True
        except Exception as e:
            print(f" Error flushing CSV data: {e}")
            return False

    def log_terminal_message(self, message_data: dict[str, Any]) -> None:
        """
        Add a terminal output message to the terminal log.

        Args:
            message_data: Dictionary containing terminal message data
                Expected keys: time, status, stabilization_time (optional),
                pos_error, ang_error, thrusters, solve_time, next_update (optional)
        """
        self.terminal_log_data.append(message_data)
        if (
            self.max_terminal_entries
            and len(self.terminal_log_data) > self.max_terminal_entries
        ):
            overflow = len(self.terminal_log_data) - self.max_terminal_entries
            if overflow == 1:
                self.terminal_log_data.pop(0)
            else:
                del self.terminal_log_data[:overflow]

    def save_csv_data(self) -> bool:
        """
        Final export of any remaining data to CSV file.
        Also saves terminal log.

        Returns:
            True if save successful, False otherwise
        """
        if not self.data_save_path:
            return False

        # Flush any remaining data
        if self.detailed_log_data:
            success = self.flush()
            wrote_any_files = success
        else:
            success = True
            # Or True? If we assume "save completed" (nothing to save) is
            # success.
            wrote_any_files = False
            # If we want success = "data on disk", return True.
            # wrote_any_files implies we created a file.
            # But wrote_any_files implies we created a file.
            # If buffer was empty and no file existed, we didn't write.
            # But headers are written on first flush.
            # If headers were written previously, file exists.
            wrote_any_files = self._headers_written

        # Save terminal log (always overwrite/new file for terminal log as it's
        # small)
        if self.terminal_log_data:
            terminal_name = f"{self.mode}_terminal_log.csv"
            terminal_log_path = self.data_save_path / artifact_relative_path(
                terminal_name
            )
            terminal_log_path.parent.mkdir(parents=True, exist_ok=True)
            try:
                with open(terminal_log_path, "w", newline="") as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(self._get_terminal_log_headers())

                    for log_entry in self.terminal_log_data:
                        row = [
                            self._format_terminal_value(
                                header, log_entry.get(header, "")
                            )
                            for header in self._get_terminal_log_headers()
                        ]
                        writer.writerow(row)

                print(f" Terminal log saved to: {terminal_log_path}")
                wrote_any_files = True
            except Exception as e:
                print(f" Error saving terminal log: {e}")
                success = False

        return success and wrote_any_files

    def _get_physics_headers(self) -> list[str]:
        """Get CSV headers for physics mode (high-frequency valve tracking)."""
        headers = [
            "Time",
            "Current_X",
            "Current_Y",
            "Current_Z",
            "Current_QW",
            "Current_QX",
            "Current_QY",
            "Current_QZ",
            "Current_Roll",
            "Current_Pitch",
            "Current_Yaw",
            "Current_VX",
            "Current_VY",
            "Current_VZ",
            "Current_WX",
            "Current_WY",
            "Current_WZ",
            "Reference_X",
            "Reference_Y",
            "Reference_Z",
            "Reference_QW",
            "Reference_QX",
            "Reference_QY",
            "Reference_QZ",
            "Reference_Roll",
            "Reference_Pitch",
            "Reference_Yaw",
            "Error_X",
            "Error_Y",
            "Error_Z",
            "Error_Roll",
            "Error_Pitch",
            "Error_Yaw",
            "Frame_Origin_X",
            "Frame_Origin_Y",
            "Frame_Origin_Z",
            "Command_Vector",
            "RW_Torque_X",
            "RW_Torque_Y",
            "RW_Torque_Z",
            "Solve_Time",
        ]
        thruster_count = self._get_logged_thruster_count()
        for thruster_id in range(1, thruster_count + 1):
            headers.append(f"Thruster_{thruster_id}_Cmd")
            headers.append(f"Thruster_{thruster_id}_Val")
        return headers

    def _get_logged_thruster_count(self) -> int:
        """Infer thruster count from logged data or fall back to config."""
        max_id = 0
        for entry in self.detailed_log_data:
            for key in entry.keys():
                if not key.startswith("Thruster_"):
                    continue
                if not (key.endswith("_Cmd") or key.endswith("_Val")):
                    continue
                parts = key.split("_")
                if len(parts) < 2:
                    continue
                try:
                    thruster_id = int(parts[1])
                except (ValueError, TypeError):
                    continue
                if thruster_id > max_id:
                    max_id = thruster_id

        if max_id > 0:
            return max_id

        try:
            from controller.configs.simulation_config import SimulationConfig

            default_config = SimulationConfig.create_default()
            return len(default_config.app_config.physics.thruster_positions)
        except Exception:
            logger.debug(
                "Failed to detect thruster count from controller.configs. defaulting to 8"
            )
            return 8

    def _get_simulation_headers(self) -> list[str]:
        """Get CSV headers for simulation mode (historical format)."""
        return [
            "Step",
            "MPC_Start_Time",
            "Control_Time",
            "Actual_Time_Interval",
            "CONTROL_DT",
            "Mission_Phase",
            "Waypoint_Number",
            "Telemetry_X_mm",
            "Telemetry_Y_mm",
            "Telemetry_Z_mm",
            "Telemetry_Roll_deg",
            "Telemetry_Pitch_deg",
            "Telemetry_Yaw_deg",
            "Current_X",
            "Current_Y",
            "Current_Z",
            "Current_Roll",
            "Current_Pitch",
            "Current_Yaw",
            "Current_VX",
            "Current_VY",
            "Current_VZ",
            "Current_WX",
            "Current_WY",
            "Current_WZ",
            "Reference_X",
            "Reference_Y",
            "Reference_Z",
            "Reference_Roll",
            "Reference_Pitch",
            "Reference_Yaw",
            "Reference_VX",
            "Reference_VY",
            "Reference_VZ",
            "Reference_WX",
            "Reference_WY",
            "Reference_WZ",
            "Error_X",
            "Error_Y",
            "Error_Z",
            "Error_Roll",
            "Error_Pitch",
            "Error_Yaw",
            "Error_VX",
            "Error_VY",
            "Error_VZ",
            "Error_WX",
            "Error_WY",
            "Error_WZ",
            "MPC_Computation_Time",
            "MPC_T_Linearization",
            "MPC_T_Cost_Update",
            "MPC_T_Constraint_Update",
            "MPC_T_Matrix_Update",
            "MPC_T_Warmstart",
            "MPC_T_Solve_Only",
            "MPC_Status",
            "MPC_Solver",
            "MPC_Solver_Time_Limit",
            "MPC_Solve_Time",
            "MPC_Time_Limit_Exceeded",
            "MPC_Fallback_Used",
            "MPC_Objective",
            "MPC_Iterations",
            "MPC_Optimality_Gap",
            "Path_S",
            "Path_V_S",
            "Path_S_Proj",
            "Path_S_Pred",
            "Path_Progress",
            "Path_Remaining",
            "Path_Error",
            "Path_Endpoint_Error",
            "Mode_State",
            "Mode_Time_In_Mode_s",
            "Completion_Gate_Position_OK",
            "Completion_Gate_Angle_OK",
            "Completion_Gate_Velocity_OK",
            "Completion_Gate_Angular_Velocity_OK",
            "Completion_Gate_Hold_Elapsed_s",
            "Completion_Gate_Hold_Required_s",
            "Completion_Gate_Last_Breach_Reason",
            "Terminal_Gate_Fail_Reason",
            "Hold_Timer_s",
            "Hold_Reset_Count",
            "Solver_Health_Status",
            "Solver_Fallback_Count",
            "Solver_Hard_Limit_Breaches",
            "Solver_Last_Fallback_Reason",
            "Solver_Fallback_Active",
            "Solver_Fallback_Age_s",
            "Solver_Fallback_Scale",
            "Pointing_Policy_Active",
            "Pointing_Context_Source_Active",
            "Pointing_Context_Source",
            "Pointing_Axis_X",
            "Pointing_Axis_Y",
            "Pointing_Axis_Z",
            "Pointing_Z_Axis_Error_Deg",
            "Pointing_X_Axis_Error_Deg",
            "Pointing_Guardrail_Breached",
            "Pointing_Guardrail_Reason",
            "Object_Visible_Side",
            "Frame_Origin_X",
            "Frame_Origin_Y",
            "Frame_Origin_Z",
            "Command_Vector",
            "Command_Hex",
            "Command_Sent_Time",
            "Total_Active_Thrusters",
            "Thruster_Switches",
            "RW_Torque_X",
            "RW_Torque_Y",
            "RW_Torque_Z",
            "Total_MPC_Loop_Time",
            "Timing_Violation",
        ]

    def _get_terminal_log_headers(self) -> list[str]:
        """Get CSV headers for terminal log."""
        return [
            "Time",
            "Status",
            "Stabilization_Time",
            "Position_Error_m",
            "Angle_Error_deg",
            "Active_Thrusters",
            "Solve_Time_s",
            "Next_Update_s",
        ]

    def _build_format_dispatch(self) -> dict[str, str]:
        """Build a header -> format-spec lookup table (replaces if/elif chain)."""
        dispatch: dict[str, str] = {}

        # Integer columns
        for h in [
            "Step",
            "Waypoint_Number",
            "Total_Active_Thrusters",
            "Thruster_Switches",
            "MPC_Iterations",
            "Solver_Fallback_Count",
            "Solver_Hard_Limit_Breaches",
            "Hold_Reset_Count",
        ]:
            dispatch[h] = "int"

        # Time values — 4 decimals
        for h in [
            "MPC_Start_Time",
            "Control_Time",
            "Actual_Time_Interval",
            "Command_Sent_Time",
            "MPC_Computation_Time",
            "MPC_T_Linearization",
            "MPC_T_Cost_Update",
            "MPC_T_Constraint_Update",
            "MPC_T_Matrix_Update",
            "MPC_T_Warmstart",
            "MPC_T_Solve_Only",
            "MPC_Solve_Time",
            "Total_MPC_Loop_Time",
            "Mode_Time_In_Mode_s",
            "Completion_Gate_Hold_Elapsed_s",
            "Completion_Gate_Hold_Required_s",
            "Hold_Timer_s",
            "Solver_Fallback_Age_s",
        ]:
            dispatch[h] = ".4f"

        # Configuration values — 3 decimals
        for h in ["CONTROL_DT", "MPC_Solver_Time_Limit"]:
            dispatch[h] = ".3f"

        # Telemetry positions (mm) — 2 decimals
        for h in ["Telemetry_X_mm", "Telemetry_Y_mm", "Telemetry_Z_mm"]:
            dispatch[h] = ".2f"

        # Telemetry angles (degrees) — 2 decimals
        for h in ["Telemetry_Roll_deg", "Telemetry_Pitch_deg", "Telemetry_Yaw_deg"]:
            dispatch[h] = ".2f"

        # Position values (meters) — 5 decimals
        for h in [
            "Current_X",
            "Current_Y",
            "Current_Z",
            "Reference_X",
            "Reference_Y",
            "Reference_Z",
            "Error_X",
            "Error_Y",
            "Error_Z",
            "Frame_Origin_X",
            "Frame_Origin_Y",
            "Frame_Origin_Z",
            "RW_Torque_X",
            "RW_Torque_Y",
            "RW_Torque_Z",
        ]:
            dispatch[h] = ".5f"

        # Angle values (radians) — 5 decimals
        for h in [
            "Current_Roll",
            "Current_Pitch",
            "Current_Yaw",
            "Reference_Roll",
            "Reference_Pitch",
            "Reference_Yaw",
            "Error_Roll",
            "Error_Pitch",
            "Error_Yaw",
        ]:
            dispatch[h] = ".5f"

        # Quaternion values — 6 decimals
        for h in [
            "Current_QW",
            "Current_QX",
            "Current_QY",
            "Current_QZ",
            "Reference_QW",
            "Reference_QX",
            "Reference_QY",
            "Reference_QZ",
        ]:
            dispatch[h] = ".6f"

        # Velocity values — 5 decimals
        for h in [
            "Current_VX",
            "Current_VY",
            "Current_VZ",
            "Current_WX",
            "Current_WY",
            "Current_WZ",
            "Reference_VX",
            "Reference_VY",
            "Reference_VZ",
            "Reference_WX",
            "Reference_WY",
            "Reference_WZ",
            "Error_VX",
            "Error_VY",
            "Error_VZ",
            "Error_WX",
            "Error_WY",
            "Error_WZ",
        ]:
            dispatch[h] = ".5f"

        # Objective / gap — 3 decimals
        for h in [
            "MPC_Objective",
            "MPC_Optimality_Gap",
            "Solver_Fallback_Scale",
            "Pointing_Axis_X",
            "Pointing_Axis_Y",
            "Pointing_Axis_Z",
            "Pointing_Z_Axis_Error_Deg",
            "Pointing_X_Axis_Error_Deg",
        ]:
            dispatch[h] = ".3f"

        return dispatch

    def _format_value(self, header: str, value: Any) -> str:
        """
        Format numeric values with appropriate precision based on column type.
        Uses pre-built dispatch table for O(1) lookup instead of if/elif chain.

        Args:
            header: Column name
            value: Value to format

        Returns:
            Formatted string value
        """
        # Handle empty/None values
        if value is None or value == "":
            return ""

        # Boolean values
        if isinstance(value, bool):
            return str(value)

        # String values (includes Command_Vector, Command_Hex, Status)
        if isinstance(value, str):
            return value

        # Numeric formatting via dispatch table
        try:
            num_value = float(value)
            fmt = self._format_dispatch.get(header)
            if fmt is None:
                return f"{num_value:.6f}"
            if fmt == "int":
                return str(int(num_value))
            return format(num_value, fmt)
        except (ValueError, TypeError):
            return str(value)

    def _format_terminal_value(self, header: str, value: Any) -> str:
        """
        Format terminal log values with appropriate precision.

        Args:
            header: Column name
            value: Value to format

        Returns:
            Formatted string value
        """
        if value is None or value == "":
            return ""

        if isinstance(value, str):
            return value

        try:
            num_value = float(value)

            # Time values - 4 decimals (0.1ms precision)
            if header in [
                "Time",
                "Stabilization_Time",
                "Solve_Time_s",
                "Next_Update_s",
            ]:
                return f"{num_value:.4f}"

            # Position error - 5 decimals (0.01mm precision)
            elif header == "Position_Error_m":
                return f"{num_value:.5f}"

            # Angle error - 2 decimals (0.01 degree precision)
            elif header == "Angle_Error_deg":
                return f"{num_value:.2f}"

            else:
                return f"{num_value:.4f}"

        except (ValueError, TypeError):
            return str(value)

    def get_log_count(self) -> int:
        """Get the total number of logged entries recorded in this session."""
        return self.total_steps_recorded

    def clear_logs(self) -> None:
        """Clear all logged data."""
        self.detailed_log_data = []
        self.terminal_log_data = []
        self.current_step = 0
        self.stats_solve_times = []
        self.stats_timing_violations = 0
        self.stats_time_limit_exceeded = 0
        self.total_steps_recorded = 0
        self._headers_written = False

    def get_summary_stats(self) -> dict[str, Any]:
        """
        Calculate summary statistics from logged data.

        Returns:
            Dictionary containing summary statistics
        """
        if self.total_steps_recorded == 0:
            return {}

        stats = {
            "total_steps": self.total_steps_recorded,
            "mode": self.mode,
        }

        if self.stats_solve_times:
            import numpy as np

            stats["avg_solve_time"] = float(np.mean(self.stats_solve_times))
            stats["max_solve_time"] = float(np.max(self.stats_solve_times))
            stats["min_solve_time"] = float(np.min(self.stats_solve_times))
            stats["std_solve_time"] = float(np.std(self.stats_solve_times))

        stats["timing_violations"] = self.stats_timing_violations
        stats["time_limit_exceeded"] = self.stats_time_limit_exceeded

        return stats

    def print_summary(self) -> None:
        """Print summary of logged data."""
        stats = self.get_summary_stats()

        if not stats:
            print("No data logged")
            return

        print("\n" + "=" * 60)
        print(f"DATA LOGGER SUMMARY ({self.mode.upper()} MODE)")
        print("=" * 60)
        print(f"Total steps logged: {stats['total_steps']}")

        if "avg_solve_time" in stats:
            print("\nMPC Solve Time Statistics:")
            print(f"  Average: {stats['avg_solve_time'] * 1000:.2f} ms")
            print(f"  Min:     {stats['min_solve_time'] * 1000:.2f} ms")
            print(f"  Max:     {stats['max_solve_time'] * 1000:.2f} ms")
            print(f"  Std Dev: {stats['std_solve_time'] * 1000:.2f} ms")

        print("\nTiming Performance:")
        print(f"  Timing violations:     {stats['timing_violations']}")
        print(f"  Time limits exceeded:  {stats['time_limit_exceeded']}")

        print("=" * 60 + "\n")


def create_data_logger(
    mode: str = "simulation",
    filename: str = "control_data.csv",
    max_terminal_entries: int = 0,
) -> DataLogger:
    """
    Factory function to create a data logger.

    Args:
        mode: "simulation" or "physics"
        filename: Output CSV filename

    Returns:
        Configured DataLogger instance
    """
    return DataLogger(
        mode=mode,
        filename=filename,
        max_terminal_entries=max_terminal_entries,
    )
