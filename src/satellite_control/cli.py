"""
Satellite Control CLI
=====================

Main entry point for the satellite control system.
Runs the MPC simulation with interactive mission selection.
"""

import math
from typing import Any, Dict, Optional, Tuple

import typer
from rich.console import Console
from rich.panel import Panel

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation

app = typer.Typer(
    help="Satellite Control System - MPC Simulation CLI",
    add_completion=False,
)
console = Console()


@app.command()
def run(
    auto: bool = typer.Option(
        False, "--auto", "-a", help="Run in auto mode with default parameters"
    ),
    duration: Optional[float] = typer.Option(
        None, "--duration", "-d", help="Override max simulation time in seconds"
    ),
    no_anim: bool = typer.Option(
        False, "--no-anim", help="Disable animation (headless mode)"
    ),
    headless: bool = typer.Option(
        False, "--headless", help="Alias for --no-anim (deprecated)", hidden=True
    ),
    mission_file: Optional[str] = typer.Option(
        None, "--mission", "-m", help="Path to mission file (JSON/YAML) to execute"
    ),
):
    """
    Run the Satellite MPC Simulation.
    """
    if headless:
        no_anim = True

    console.print(
        Panel.fit(
            "Satellite Control Simulation",
            style="bold blue",
            subtitle="MPC Control System",
        )
    )

    # Prepare simulation parameters
    sim_start_pos: Optional[Tuple[float, float, float]] = None
    sim_end_pos: Optional[Tuple[float, float, float]] = None
    sim_start_angle: Optional[Tuple[float, float, float]] = None
    sim_end_angle: Optional[Tuple[float, float, float]] = None
    config_overrides: Optional[Dict[str, Dict[str, Any]]] = None

    # Import SimulationConfig for Pydantic configuration
    from src.satellite_control.config.simulation_config import SimulationConfig

    simulation_config = None

    if auto:
        console.print(
            "[yellow]Running in AUTO mode with default parameters...[/yellow]"
        )
        sim_start_pos = (1.0, 1.0, 0.0)
        sim_end_pos = (0.0, 0.0, 0.0)
        sim_start_angle = (0.0, 0.0, 0.0)
        sim_end_angle = (0.0, 0.0, 0.0)
        # Use default Pydantic config for auto mode
        simulation_config = SimulationConfig.create_default()
        # Path-only default: straight line from start to end
        from src.satellite_control.mission.path_following import (
            build_point_to_point_path,
        )

        path = build_point_to_point_path(
            waypoints=[sim_start_pos, sim_end_pos],
            obstacles=None,
            step_size=0.1,
        )
        path_length = 0.0
        for start, end in zip(path, path[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            dz = end[2] - start[2]
            path_length += math.sqrt(dx * dx + dy * dy + dz * dz)

        mpc_cfg = simulation_config.app_config.mpc
        ms = simulation_config.mission_state
        ms.mpcc_path_waypoints = path
        ms.dxf_shape_path = path
        ms.dxf_path_length = path_length
        ms.dxf_path_speed = mpc_cfg.path_speed
        ms.trajectory_mode_active = False

    elif mission_file:
        from pathlib import Path
        import json

        m_path = Path(mission_file)
        if not m_path.exists():
            console.print(f"[red]Mission file not found: {m_path}[/red]")
            raise typer.Exit(code=1)

        console.print(f"[green]Loading mission from {m_path}[/green]")

        # Load JSON and detect format
        with open(m_path, "r") as f:
            mission_data = json.load(f)

        simulation_config = SimulationConfig.create_default()
        ms = simulation_config.mission_state

        if "path_following" in mission_data:
            console.print("[cyan]Detected path-following mission format...[/cyan]")

            path_cfg = mission_data.get("path_following", {}) or {}
            raw_waypoints = path_cfg.get("waypoints", [])
            if not raw_waypoints:
                console.print("[red]Path-following mission requires waypoints.[/red]")
                raise typer.Exit(code=1)

            sim_start_pos = tuple(mission_data.get("start_position", raw_waypoints[0]))
            sim_end_pos = tuple(mission_data.get("end_position", raw_waypoints[-1]))
            sim_end_angle = tuple(mission_data.get("end_orientation", [0, 0, 0]))

            # Obstacles
            obs_data = mission_data.get("obstacles", [])
            obstacles = []
            for o in obs_data:
                p = o.get("position", [0, 0, 0])
                r = o.get("radius", 0.5)
                obstacles.append((p[0], p[1], p[2], r))
            ms.obstacles = obstacles
            ms.obstacles_enabled = len(obstacles) > 0

            from src.satellite_control.mission.path_following import (
                build_point_to_point_path,
            )

            speed = float(
                path_cfg.get("speed", simulation_config.app_config.mpc.path_speed)
            )
            step_size = float(path_cfg.get("step_size", 0.1))

            path_points = [tuple(map(float, p)) for p in raw_waypoints]
            if path_points[0] != sim_start_pos:
                path_points = [tuple(map(float, sim_start_pos))] + path_points
            if len(path_points) < 2:
                console.print(
                    "[red]Path-following mission needs at least two points.[/red]"
                )
                raise typer.Exit(code=1)

            path = build_point_to_point_path(
                waypoints=path_points,
                obstacles=obstacles if ms.obstacles_enabled else None,
                step_size=step_size,
            )

            path_length = 0.0
            for start, end in zip(path, path[1:]):
                dx = end[0] - start[0]
                dy = end[1] - start[1]
                dz = end[2] - start[2]
                path_length += math.sqrt(dx * dx + dy * dy + dz * dz)

            mpc_cfg = simulation_config.app_config.mpc
            mpc_cfg.path_speed = speed

            ms.mpcc_path_waypoints = path
            ms.dxf_shape_path = path
            ms.dxf_path_length = path_length
            ms.dxf_path_speed = speed
            ms.trajectory_hold_end = float(path_cfg.get("hold_end", 0.0) or 0.0)
            ms.trajectory_mode_active = False
            ms.trajectory_type = "path"
            ms.dxf_shape_mode_active = False
            ms.mesh_scan_mode_active = False

            console.print(
                f"[green]Path loaded: {len(path)} points, {path_length:.2f}m[/green]"
            )

        # Detect Mission Control format (has mesh_scan field)
        elif "mesh_scan" in mission_data:
            console.print("[cyan]Detected Mission Control format (mesh scan)...[/cyan]")

            # Load Mission Control format
            sim_start_pos = tuple(mission_data.get("start_position", [10, 0, 0]))
            sim_end_pos = tuple(mission_data.get("end_position", [0, 0, 0]))
            sim_end_angle = tuple(mission_data.get("end_orientation", [0, 0, 0]))

            # Obstacles
            obs_data = mission_data.get("obstacles", [])
            obstacles = []
            for o in obs_data:
                p = o.get("position", [0, 0, 0])
                r = o.get("radius", 0.5)
                obstacles.append((p[0], p[1], p[2], r))
            ms.obstacles = obstacles
            ms.obstacles_enabled = len(obstacles) > 0

            # Mesh Scan
            mesh_scan = mission_data["mesh_scan"]
            from src.satellite_control.mission.mesh_scan import (
                build_mesh_scan_trajectory,
            )

            try:
                scan_speed = float(
                    mesh_scan.get(
                        "speed_max", simulation_config.app_config.mpc.path_speed
                    )
                )
                path, _, path_length = build_mesh_scan_trajectory(
                    obj_path=mesh_scan.get("obj_path"),
                    standoff=mesh_scan.get("standoff", 0.5),
                    levels=mesh_scan.get("levels", 8),
                    points_per_circle=mesh_scan.get("points_per_circle", 72),
                    v_max=scan_speed,
                    v_min=mesh_scan.get("speed_min", 0.05),
                    lateral_accel=mesh_scan.get("lateral_accel", 0.05),
                    dt=float(simulation_config.app_config.mpc.dt),
                    z_margin=mesh_scan.get("z_margin", 0.0),
                    scan_axis=mesh_scan.get("scan_axis", "Z"),
                    build_trajectory=False,
                )

                mpc_cfg = simulation_config.app_config.mpc
                mpc_cfg.path_speed = scan_speed

                ms.mesh_scan_mode_active = True
                ms.mesh_scan_obj_path = mesh_scan.get("obj_path")
                ms.dxf_shape_path = path
                ms.dxf_path_length = path_length
                ms.dxf_path_speed = scan_speed
                ms.mpcc_path_waypoints = path
                ms.trajectory_mode_active = False
                ms.trajectory_type = "scan"

                # Reset phase state for approach-stabilize-track
                ms.scan_phase = "APPROACH"
                ms.scan_approach_target = None
                ms.scan_stabilize_start_time = None

                console.print(
                    f"[green]Mesh scan loaded: {len(path)} points, {path_length:.1f}m path[/green]"
                )

            except Exception as e:
                console.print(f"[red]Failed to build scan trajectory: {e}[/red]")
                raise typer.Exit(code=1)
        else:
            # Original Mission format (with phases/waypoints)
            from src.satellite_control.mission.mission_types import Mission

            mission = Mission.from_dict(mission_data)
            sim_start_pos = mission.start_position.tolist()

            ms.obstacles_enabled = len(mission.obstacles) > 0
            ms.obstacles = mission.obstacles

            waypoints = mission.get_all_waypoints()
            if waypoints:
                from src.satellite_control.mission.path_following import (
                    build_point_to_point_path,
                )

                positions = [tuple(mission.start_position.tolist())] + [
                    tuple(wp.position.tolist()) for wp in waypoints
                ]
                speed = float(
                    mission.max_speed or simulation_config.app_config.mpc.path_speed
                )
                hold_end = float(waypoints[-1].hold_time) if waypoints else 0.0
                path = build_point_to_point_path(
                    waypoints=positions,
                    obstacles=None,
                    step_size=0.1,
                )
                path_length = 0.0
                for start, end in zip(path, path[1:]):
                    dx = end[0] - start[0]
                    dy = end[1] - start[1]
                    dz = end[2] - start[2]
                    path_length += math.sqrt(dx * dx + dy * dy + dz * dz)

                mpc_cfg = simulation_config.app_config.mpc
                mpc_cfg.path_speed = speed

                ms.mpcc_path_waypoints = path
                ms.dxf_shape_path = path
                ms.dxf_path_length = path_length
                ms.dxf_path_speed = speed
                ms.trajectory_hold_end = hold_end
                ms.trajectory_mode_active = False

    else:
        # Interactive mode (default)
        try:
            from src.satellite_control.mission.interactive_cli import (
                InteractiveMissionCLI,
            )

            interactive_cli = InteractiveMissionCLI()
            mode = interactive_cli.show_mission_menu()

            if mode == "path_following":
                mission_config = interactive_cli.run_path_following_mission()
                if not mission_config:
                    console.print("[red]Mission cancelled.[/red]")
                    raise typer.Exit()
                if "simulation_config" in mission_config:
                    simulation_config = mission_config["simulation_config"]
                if "start_pos" in mission_config:
                    sim_start_pos = mission_config.get("start_pos")
                if "start_angle" in mission_config:
                    sim_start_angle = mission_config.get("start_angle")
            elif mode == "scan_object":
                mission_config = interactive_cli.run_scan_mission()
                if not mission_config:
                    console.print("[red]Mission cancelled.[/red]")
                    raise typer.Exit()
                if "simulation_config" in mission_config:
                    simulation_config = mission_config["simulation_config"]
                if "start_pos" in mission_config:
                    sim_start_pos = mission_config.get("start_pos")
                if "start_angle" in mission_config:
                    sim_start_angle = mission_config.get("start_angle")
            elif mode == "starlink_orbit":
                mission_config = interactive_cli.run_starlink_orbit_mission()
                if not mission_config:
                    console.print("[red]Mission cancelled.[/red]")
                    raise typer.Exit()
                if "simulation_config" in mission_config:
                    simulation_config = mission_config["simulation_config"]
                if "start_pos" in mission_config:
                    sim_start_pos = mission_config.get("start_pos")
                if "start_angle" in mission_config:
                    sim_start_angle = mission_config.get("start_angle")
            elif mode == "load_saved":
                mission_config = interactive_cli.run_saved_mission()
                if not mission_config:
                    console.print("[red]Mission cancelled.[/red]")
                    raise typer.Exit()
                if "simulation_config" in mission_config:
                    simulation_config = mission_config["simulation_config"]
                if "start_pos" in mission_config:
                    sim_start_pos = mission_config.get("start_pos")
                if "start_angle" in mission_config:
                    sim_start_angle = mission_config.get("start_angle")
                if "end_pos" in mission_config:
                    sim_end_pos = mission_config.get("end_pos")
                if "end_angle" in mission_config:
                    sim_end_angle = mission_config.get("end_angle")
        except ImportError as e:
            console.print(
                f"[red]Interactive mission UI unavailable: {e}. Use --mission to run a path file.[/red]"
            )
            raise typer.Exit(code=1)

    # Validate configuration at startup
    try:
        from src.satellite_control.config.validator import validate_config_at_startup

        validate_config_at_startup()
    except ValueError as e:
        console.print(f"[bold red]Configuration validation failed:[/bold red] {e}")
        raise typer.Exit(code=1)

    # Apply CLI overrides
    console.print("\n[bold]Initializing Simulation...[/bold]")
    if duration:
        if config_overrides is None:
            config_overrides = {}
        if "simulation" not in config_overrides:
            config_overrides["simulation"] = {}
        config_overrides["simulation"]["max_duration"] = duration
        console.print(f"  Override: Duration = {duration}s")

    # Create default config if not set by mission
    if simulation_config is None:
        simulation_config = SimulationConfig.create_default()

    # Apply overrides to config
    if config_overrides:
        simulation_config = SimulationConfig.create_with_overrides(
            config_overrides, base_config=simulation_config
        )

    console.print("[green]Loaded Pydantic configuration[/green]")

    # Initialize Simulation (Pydantic config only - no Hydra)
    try:
        sim = SatelliteMPCLinearizedSimulation(
            start_pos=sim_start_pos,
            end_pos=sim_end_pos,
            start_angle=sim_start_angle,
            end_angle=sim_end_angle,
            simulation_config=simulation_config,
        )

        if duration and sim.max_simulation_time != duration:
            sim.max_simulation_time = duration

        console.print("[green]Simulation initialized successfully.[/green]")
        console.print("Starting Simulation loop...")

        sim.run_simulation(show_animation=not no_anim)

    except KeyboardInterrupt:
        console.print("\n[yellow]Simulation stopping (KeyboardInterrupt)...[/yellow]")
    except Exception as e:
        import traceback

        console.print(f"\n[bold red]Error running simulation:[/bold red] {e}")
        console.print("[dim]Full traceback:[/dim]")
        console.print(traceback.format_exc())
        raise typer.Exit(code=1)
    finally:
        if "sim" in locals():
            sim.close()


if __name__ == "__main__":
    app()
