"""Declarative post-run plot suite catalog (v2 full profile)."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlotGroup:
    """Logical group/folder in the post-run plot suite."""

    id: str
    order: int
    title: str
    folder: str


@dataclass(frozen=True)
class PlotSpec:
    """Single plot definition in deterministic output order."""

    plot_id: str
    group_id: str
    order: int
    title: str
    filename: str
    renderer: str
    format: str = "png"
    interactive: bool = False


PLOT_GROUPS: tuple[PlotGroup, ...] = (
    PlotGroup(
        id="overview",
        order=1,
        title="Overview",
        folder="01_overview",
    ),
    PlotGroup(
        id="trajectory",
        order=2,
        title="Trajectory",
        folder="02_trajectory",
    ),
    PlotGroup(
        id="tracking_error",
        order=3,
        title="Tracking & Error",
        folder="03_tracking_error",
    ),
    PlotGroup(
        id="actuators",
        order=4,
        title="Actuators",
        folder="04_actuators",
    ),
    PlotGroup(
        id="solver_timing",
        order=5,
        title="Solver & Timing",
        folder="05_solver_timing",
    ),
    PlotGroup(
        id="mission_progress",
        order=6,
        title="Mission Progress",
        folder="06_mission_progress",
    ),
)


PLOT_SPECS: tuple[PlotSpec, ...] = (
    PlotSpec(
        plot_id="overview.mission_overview",
        group_id="overview",
        order=101,
        title="Mission Overview",
        filename="01_mission_overview.png",
        renderer="_render_mission_overview",
    ),
    PlotSpec(
        plot_id="overview.constraints_overview",
        group_id="overview",
        order=102,
        title="Constraints Overview",
        filename="02_constraints_overview.png",
        renderer="_render_constraints_overview",
    ),
    PlotSpec(
        plot_id="overview.controller_health_overview",
        group_id="overview",
        order=103,
        title="Controller Health Overview",
        filename="03_controller_health_overview.png",
        renderer="_render_controller_health_overview",
    ),
    PlotSpec(
        plot_id="trajectory.xy_lvlh",
        group_id="trajectory",
        order=201,
        title="Trajectory XY (LVLH)",
        filename="01_trajectory_xy_lvlh.png",
        renderer="_render_trajectory_xy_lvlh",
    ),
    PlotSpec(
        plot_id="trajectory.xz_lvlh",
        group_id="trajectory",
        order=202,
        title="Trajectory XZ (LVLH)",
        filename="02_trajectory_xz_lvlh.png",
        renderer="_render_trajectory_xz_lvlh",
    ),
    PlotSpec(
        plot_id="trajectory.yz_lvlh",
        group_id="trajectory",
        order=203,
        title="Trajectory YZ (LVLH)",
        filename="03_trajectory_yz_lvlh.png",
        renderer="_render_trajectory_yz_lvlh",
    ),
    PlotSpec(
        plot_id="trajectory.3d_orientation",
        group_id="trajectory",
        order=204,
        title="Trajectory 3D Orientation",
        filename="04_trajectory_3d_orientation.png",
        renderer="_render_trajectory_3d_orientation",
    ),
    PlotSpec(
        plot_id="trajectory.3d_interactive",
        group_id="trajectory",
        order=205,
        title="Trajectory 3D Interactive",
        filename="05_trajectory_3d_interactive.html",
        renderer="_render_trajectory_3d_interactive",
        format="html",
        interactive=True,
    ),
    PlotSpec(
        plot_id="tracking.position_tracking_xyz",
        group_id="tracking_error",
        order=301,
        title="Position Tracking XYZ",
        filename="01_position_tracking_xyz.png",
        renderer="_render_position_tracking_xyz",
    ),
    PlotSpec(
        plot_id="tracking.position_error_xyz_limits",
        group_id="tracking_error",
        order=302,
        title="Position Error XYZ With Limits",
        filename="02_position_error_xyz_with_limits.png",
        renderer="_render_position_error_xyz_with_limits",
    ),
    PlotSpec(
        plot_id="tracking.position_error_norm_limit",
        group_id="tracking_error",
        order=303,
        title="Position Error Norm With Limit",
        filename="03_position_error_norm_with_limit.png",
        renderer="_render_position_error_norm_with_limit",
    ),
    PlotSpec(
        plot_id="tracking.velocity_tracking_xyz",
        group_id="tracking_error",
        order=304,
        title="Velocity Tracking XYZ",
        filename="04_velocity_tracking_xyz.png",
        renderer="_render_velocity_tracking_xyz",
    ),
    PlotSpec(
        plot_id="tracking.velocity_error_norm_limit",
        group_id="tracking_error",
        order=305,
        title="Velocity Error Norm With Limit",
        filename="05_velocity_error_norm_with_limit.png",
        renderer="_render_velocity_error_norm_with_limit",
    ),
    PlotSpec(
        plot_id="tracking.attitude_tracking_quaternion",
        group_id="tracking_error",
        order=306,
        title="Attitude Tracking Quaternion",
        filename="06_attitude_tracking_quaternion.png",
        renderer="_render_attitude_tracking_quaternion",
    ),
    PlotSpec(
        plot_id="tracking.attitude_error_quaternion_limit",
        group_id="tracking_error",
        order=307,
        title="Attitude Error Quaternion With Limit",
        filename="07_attitude_error_quaternion_with_limit.png",
        renderer="_render_attitude_error_quaternion_with_limit",
    ),
    PlotSpec(
        plot_id="tracking.angular_rate_error_limit",
        group_id="tracking_error",
        order=308,
        title="Angular Rate Error With Limit",
        filename="08_angular_rate_error_with_limit.png",
        renderer="_render_angular_rate_error_with_limit",
    ),
    PlotSpec(
        plot_id="actuators.thruster_usage_summary",
        group_id="actuators",
        order=401,
        title="Thruster Usage Summary",
        filename="01_thruster_usage_summary.png",
        renderer="_render_thruster_usage_summary",
    ),
    PlotSpec(
        plot_id="actuators.valve_activity_aggregate",
        group_id="actuators",
        order=402,
        title="Thruster Valve Activity Aggregate",
        filename="02_thruster_valve_activity_aggregate.png",
        renderer="_render_thruster_valve_activity_aggregate",
    ),
    PlotSpec(
        plot_id="actuators.command_vs_valve_tracking",
        group_id="actuators",
        order=403,
        title="Command Vs Valve Tracking",
        filename="03_command_vs_valve_tracking.png",
        renderer="_render_command_vs_valve_tracking",
    ),
    PlotSpec(
        plot_id="actuators.pwm_duty_cycles",
        group_id="actuators",
        order=404,
        title="PWM Duty Cycles",
        filename="04_pwm_duty_cycles.png",
        renderer="_render_pwm_duty_cycles",
    ),
    PlotSpec(
        plot_id="actuators.control_effort",
        group_id="actuators",
        order=405,
        title="Control Effort",
        filename="05_control_effort.png",
        renderer="_render_control_effort",
    ),
    PlotSpec(
        plot_id="actuators.reaction_wheel_output",
        group_id="actuators",
        order=406,
        title="Reaction Wheel Output",
        filename="06_reaction_wheel_output.png",
        renderer="_render_reaction_wheel_output",
    ),
    PlotSpec(
        plot_id="actuators.actuator_limits",
        group_id="actuators",
        order=407,
        title="Actuator Limits With Overlays",
        filename="07_actuator_limits_with_overlays.png",
        renderer="_render_actuator_limits_with_overlays",
    ),
    PlotSpec(
        plot_id="actuators.thruster_impulse_proxy",
        group_id="actuators",
        order=408,
        title="Thruster Impulse Proxy",
        filename="08_thruster_impulse_proxy.png",
        renderer="_render_thruster_impulse_proxy",
    ),
    PlotSpec(
        plot_id="actuators.cumulative_impulse_delta_v_proxy",
        group_id="actuators",
        order=409,
        title="Cumulative Impulse Delta-v Proxy",
        filename="09_cumulative_impulse_delta_v_proxy.png",
        renderer="_render_cumulative_impulse_delta_v_proxy",
    ),
    PlotSpec(
        plot_id="actuators.per_thruster_valve_activity",
        group_id="actuators",
        order=410,
        title="Per-Thruster Valve Activity",
        filename="10_thruster_NN_valve_activity.png",
        renderer="_render_per_thruster_valve_activity",
    ),
    PlotSpec(
        plot_id="solver.mpc_solve_time_limit",
        group_id="solver_timing",
        order=501,
        title="MPC Solve Time With Limit",
        filename="01_mpc_solve_time_with_limit.png",
        renderer="_render_mpc_solve_time_with_limit",
    ),
    PlotSpec(
        plot_id="solver.health_timeline",
        group_id="solver_timing",
        order=502,
        title="Solver Health Timeline",
        filename="02_solver_health_timeline.png",
        renderer="_render_solver_health_timeline",
    ),
    PlotSpec(
        plot_id="solver.iterations_status",
        group_id="solver_timing",
        order=503,
        title="Solver Iterations And Status",
        filename="03_solver_iterations_and_status.png",
        renderer="_render_solver_iterations_and_status",
    ),
    PlotSpec(
        plot_id="solver.timing_intervals",
        group_id="solver_timing",
        order=504,
        title="Timing Intervals",
        filename="04_timing_intervals.png",
        renderer="_render_timing_intervals",
    ),
    PlotSpec(
        plot_id="solver.error_vs_solve_time_scatter",
        group_id="solver_timing",
        order=505,
        title="Error Vs Solve Time Scatter",
        filename="05_error_vs_solve_time_scatter.png",
        renderer="_render_error_vs_solve_time_scatter",
    ),
    PlotSpec(
        plot_id="solver.fallback_breach_timeline",
        group_id="solver_timing",
        order=506,
        title="Fallback And Breach Timeline",
        filename="06_fallback_and_breach_timeline.png",
        renderer="_render_fallback_and_breach_timeline",
    ),
    PlotSpec(
        plot_id="mission.waypoint_progress",
        group_id="mission_progress",
        order=601,
        title="Waypoint Progress",
        filename="01_waypoint_progress.png",
        renderer="_render_waypoint_progress",
    ),
    PlotSpec(
        plot_id="mission.mode_timeline",
        group_id="mission_progress",
        order=602,
        title="Mode Timeline",
        filename="02_mode_timeline.png",
        renderer="_render_mode_timeline",
    ),
    PlotSpec(
        plot_id="mission.completion_gate_trace",
        group_id="mission_progress",
        order=603,
        title="Completion Gate Trace",
        filename="03_completion_gate_trace.png",
        renderer="_render_completion_gate_trace",
    ),
    PlotSpec(
        plot_id="mission.path_progress_remaining_distance",
        group_id="mission_progress",
        order=604,
        title="Path Progress Remaining Distance",
        filename="04_path_progress_remaining_distance.png",
        renderer="_render_path_progress_remaining_distance",
    ),
    PlotSpec(
        plot_id="mission.event_timeline_density",
        group_id="mission_progress",
        order=605,
        title="Event Timeline Density",
        filename="05_event_timeline_density.png",
        renderer="_render_event_timeline_density",
    ),
    PlotSpec(
        plot_id="mission.path_shaping_note",
        group_id="mission_progress",
        order=606,
        title="Path Shaping Note",
        filename="06_path_shaping_note.png",
        renderer="_render_path_shaping_note",
    ),
)


GROUP_BY_ID: dict[str, PlotGroup] = {group.id: group for group in PLOT_GROUPS}
