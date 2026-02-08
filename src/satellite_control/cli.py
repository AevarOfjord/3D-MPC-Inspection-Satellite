"""
Satellite Control CLI
=====================

Main entry point for the satellite control system.
Runs the MPC simulation with interactive mission selection.
"""

import math
from typing import Any

import typer
from rich.console import Console
from rich.panel import Panel

from satellite_control.mission.repository import (
    MISSIONS_UNIFIED_DIR,
    list_mission_entries,
    load_mission_json,
)

app = typer.Typer(
    help="Satellite Control System - MPC Simulation CLI",
    add_completion=False,
)
console = Console()


def _prompt_saved_mission_file() -> str | None:
    """Prompt user to select a saved mission file for simulation."""
    entries = list_mission_entries(source_priority=("unified",))
    if not entries:
        console.print("[red]No saved missions found in missions_unified.[/red]")
        return None

    source_labels = {"unified": "unified"}

    try:
        import questionary

        choices = []
        for entry in entries:
            source = source_labels.get(entry.source, entry.source)
            choices.append(
                questionary.Choice(
                    title=f"💾 {entry.path.stem} [{source}]",
                    value=str(entry.path),
                )
            )
        choices.append(questionary.Separator())
        choices.append(questionary.Choice(title="×  Cancel", value="cancel"))
        selected = questionary.select(
            "Select mission to run:",
            choices=choices,
        ).ask()
        if selected in (None, "cancel"):
            return None
        return str(selected)
    except ImportError:
        console.print(
            "[yellow]questionary unavailable, using numbered selection.[/yellow]"
        )
        for idx, entry in enumerate(entries, start=1):
            source = source_labels.get(entry.source, entry.source)
            console.print(f"  {idx}. {entry.path.stem} [{source}]")
        raw = input("Select mission number (blank to cancel): ").strip()
        if not raw:
            return None
        try:
            selected_idx = int(raw)
        except ValueError:
            console.print("[red]Invalid mission selection.[/red]")
            return None
        if selected_idx < 1 or selected_idx > len(entries):
            console.print("[red]Mission selection out of range.[/red]")
            return None
        return str(entries[selected_idx - 1].path)


@app.command()
def run(
    auto: bool = typer.Option(
        False, "--auto", "-a", help="Run in auto mode with default parameters"
    ),
    duration: float | None = typer.Option(
        None, "--duration", "-d", help="Override max simulation time in seconds"
    ),
    no_anim: bool = typer.Option(
        False, "--no-anim", help="Disable animation (headless mode)"
    ),
    mission_file: str | None = typer.Option(
        None, "--mission", "-m", help="Path to mission file (JSON) to execute"
    ),
):
    """
    Run the Satellite MPC Simulation.
    """
    console.print(
        Panel.fit(
            "Satellite Control Simulation",
            style="bold blue",
            subtitle="MPC Control System",
        )
    )

    # Prepare simulation parameters
    sim_start_pos: tuple[float, float, float] | None = None
    sim_end_pos: tuple[float, float, float] | None = None
    sim_start_angle: tuple[float, float, float] | None = None
    sim_end_angle: tuple[float, float, float] | None = None
    config_overrides: dict[str, dict[str, Any]] | None = None

    # Import SimulationConfig for Pydantic configuration
    from satellite_control.config.simulation_config import SimulationConfig

    simulation_config = None
    if not auto and mission_file is None:
        mission_file = _prompt_saved_mission_file()
        if mission_file is None:
            console.print("[red]Mission cancelled.[/red]")
            raise typer.Exit()

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
        from satellite_control.mission.path_following import (
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
        ms.path_waypoints = path
        ms.path_length = path_length
        ms.path_speed = mpc_cfg.path_speed

    elif mission_file:
        import json
        from pathlib import Path

        m_path = Path(mission_file)
        if not m_path.exists():
            console.print(f"[red]Mission file not found: {m_path}[/red]")
            raise typer.Exit(code=1)

        console.print(f"[green]Loading mission from {m_path}[/green]")

        # Load JSON and detect format
        if MISSIONS_UNIFIED_DIR in m_path.parents:
            mission_data = load_mission_json(m_path.stem)
        else:
            mission_data = json.loads(m_path.read_text())

        simulation_config = SimulationConfig.create_default()
        from satellite_control.mission.runtime_loader import (
            compile_unified_mission_runtime,
            parse_unified_mission_payload,
        )

        try:
            mission_def = parse_unified_mission_payload(mission_data)
        except ValueError as exc:
            console.print(f"[red]{exc}[/red]")
            raise typer.Exit(code=1)

        console.print("[cyan]Detected unified mission format (v2)...[/cyan]")
        mission_runtime = compile_unified_mission_runtime(
            mission_def,
            simulation_config=simulation_config,
        )
        simulation_config = mission_runtime.simulation_config
        sim_start_pos = mission_runtime.start_pos
        sim_end_pos = mission_runtime.end_pos

        console.print(
            f"[green]Unified mission path: {len(mission_runtime.path)} points, "
            f"{mission_runtime.path_length:.2f}m[/green]"
        )

    else:
        console.print("[red]Mission file is required unless running with --auto.[/red]")
        raise typer.Exit(code=1)

    # Validate configuration at startup
    try:
        from satellite_control.config.validator import validate_config_at_startup

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
        from satellite_control.core.simulation import (
            SatelliteMPCLinearizedSimulation,
        )

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
