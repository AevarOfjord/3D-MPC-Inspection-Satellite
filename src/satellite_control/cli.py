"""
Satellite Control CLI
=====================

Main entry point for the satellite control system.
Runs the MPC simulation with interactive mission selection.
"""

from typing import Any, Dict, Optional, Tuple

import typer
from rich.console import Console
from rich.panel import Panel

from src.satellite_control.core.simulation import SatelliteMPCLinearizedSimulation
from src.satellite_control.mission.mission_manager import MissionManager

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
    classic: bool = typer.Option(
        False, "--classic", help="Use classic text-based menu instead of interactive"
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
    sim_target_pos: Optional[Tuple[float, float, float]] = None
    sim_start_angle: Optional[Tuple[float, float, float]] = None
    sim_target_angle: Optional[Tuple[float, float, float]] = None
    config_overrides: Optional[Dict[str, Dict[str, Any]]] = None

    # Import SimulationConfig for Pydantic configuration
    from src.satellite_control.config.simulation_config import SimulationConfig

    simulation_config = None

    if auto:
        console.print(
            "[yellow]Running in AUTO mode with default parameters...[/yellow]"
        )
        sim_start_pos = (1.0, 1.0, 0.0)
        sim_target_pos = (0.0, 0.0, 0.0)
        sim_start_angle = (0.0, 0.0, 0.0)
        sim_target_angle = (0.0, 0.0, 0.0)
        # Use default Pydantic config for auto mode
        simulation_config = SimulationConfig.create_default()

    elif mission_file:
        from pathlib import Path
        from src.satellite_control.mission.mission_types import Mission

        m_path = Path(mission_file)
        if not m_path.exists():
            console.print(f"[red]Mission file not found: {m_path}[/red]")
            raise typer.Exit(code=1)

        console.print(f"[green]Loading mission from {m_path}[/green]")

        mission = Mission.load(m_path)
        sim_start_pos = mission.start_position.tolist()

        simulation_config = SimulationConfig.create_default()
        ms = simulation_config.mission_state

        ms.obstacles_enabled = len(mission.obstacles) > 0
        ms.obstacles = mission.obstacles

        waypoints = mission.get_all_waypoints()
        if waypoints:
            ms.enable_waypoint_mode = True
            ms.waypoint_targets = [tuple(wp.position.tolist()) for wp in waypoints]
            ms.waypoint_angles = [(0.0, 0.0, 0.0)] * len(waypoints)

    elif classic:
        mission_manager = MissionManager()
        mode = mission_manager.show_mission_menu()
        mission_result = mission_manager.run_selected_mission(mode)
        if not mission_result:
            console.print("[red]Mission configuration cancelled.[/red]")
            raise typer.Exit()
        if isinstance(mission_result, dict) and "simulation_config" in mission_result:
            simulation_config = mission_result["simulation_config"]
        if isinstance(mission_result, dict):
            if "start_pos" in mission_result:
                sim_start_pos = mission_result.get("start_pos")
            if "start_angle" in mission_result:
                sim_start_angle = mission_result.get("start_angle")

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
        except ImportError:
            console.print("[yellow]Falling back to classic menu...[/yellow]")
            mission_manager = MissionManager()
            mode = mission_manager.show_mission_menu()
            mission_result = mission_manager.run_selected_mission(mode)
            if not mission_result:
                console.print("[red]Mission cancelled.[/red]")
                raise typer.Exit()
            if (
                isinstance(mission_result, dict)
                and "simulation_config" in mission_result
            ):
                simulation_config = mission_result["simulation_config"]

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
            target_pos=sim_target_pos,
            start_angle=sim_start_angle,
            target_angle=sim_target_angle,
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
