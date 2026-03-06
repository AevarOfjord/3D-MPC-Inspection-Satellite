"""
Satellite Control CLI
=====================

Main entry point for the satellite control system.
Runs the MPC simulation with interactive mission selection.
"""

import math
import os
from typing import Any

import typer
from rich.console import Console
from rich.panel import Panel

from controller.registry import (
    rewrite_profile_identifiers_in_payload,
)
from controller.shared.python.control_common.parameter_policy import (
    apply_profile_parameter_file_if_needed,
    normalize_shared_parameters_payload,
)
from controller.shared.python.mission.repository import (
    MISSIONS_DIR,
    list_mission_entries,
    load_mission_json,
)

app = typer.Typer(
    help="Satellite Control System - MPC Simulation CLI",
    add_completion=False,
)
console = Console()


def _set_run_mission_env(mission_path: str | None) -> None:
    """Populate mission context env vars used by SimulationIO naming/metadata."""
    if not mission_path:
        os.environ.pop("SATCTRL_RUNNER_MISSION_NAME", None)
        os.environ.pop("SATCTRL_RUNNER_MISSION_PATH", None)
        return

    from pathlib import Path

    mission_name = Path(mission_path).stem
    os.environ["SATCTRL_RUNNER_MISSION_NAME"] = mission_name
    os.environ["SATCTRL_RUNNER_MISSION_PATH"] = mission_path


def _prompt_saved_mission_file() -> str | None:
    """Prompt user to select a saved mission file for simulation."""
    entries = list_mission_entries(source_priority=("local",))
    if not entries:
        console.print("[red]No saved missions found in missions/ directory.[/red]")
        return None

    source_labels = {"local": "local"}

    try:
        import questionary

        select_style = questionary.Style(
            [
                ("qmark", ""),
                ("question", "bold"),
                ("pointer", "fg:#ffffff bg:#005f87 bold"),
                ("highlighted", "fg:#ffffff bg:#005f87 bold"),
                ("selected", "fg:#ffffff bg:#005f87"),
            ]
        )
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
            qmark="",
            style=select_style,
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
    config_file: str | None = typer.Option(
        None, "--config", "-c", help="Path to config overrides file (JSON)"
    ),
    controller_profile: str | None = typer.Option(
        None,
        "--controller-profile",
        help=(
            "Controller profile override: cpp_linearized_rti_osqp, "
            "cpp_hybrid_rti_osqp, cpp_nonlinear_rti_osqp, "
            "cpp_nonlinear_fullnlp_ipopt, cpp_nonlinear_rti_hpipm, or "
            "cpp_nonlinear_sqp_hpipm."
        ),
    ),
):
    """
    Run the Satellite MPC Simulation.
    """
    console.print(
        Panel.fit(
            "Run Simulation",
            style="bold blue",
        )
    )

    # Prepare simulation parameters
    sim_start_pos: tuple[float, float, float] | None = None
    sim_end_pos: tuple[float, float, float] | None = None
    sim_start_angle: tuple[float, float, float] | None = None
    sim_end_angle: tuple[float, float, float] | None = None
    config_overrides: dict[str, Any] | None = None
    required_duration_hint: float | None = None

    # Load config file if provided
    if config_file:
        import json
        from pathlib import Path

        cfg_path = Path(config_file)
        if not cfg_path.exists():
            console.print(f"[red]Config file not found: {cfg_path}[/red]")
            raise typer.Exit(code=1)

        try:
            loaded_overrides = json.loads(cfg_path.read_text())
            if not isinstance(loaded_overrides, dict):
                console.print("[red]Config file must contain a JSON object.[/red]")
                raise typer.Exit(code=1)
            normalize_shared_parameters_payload(loaded_overrides)
            rewrite_profile_identifiers_in_payload(loaded_overrides)
            config_overrides = loaded_overrides
            console.print(
                f"[green]Loaded configuration overrides from {cfg_path}[/green]"
            )
        except json.JSONDecodeError as e:
            console.print(f"[red]Invalid JSON in config file: {e}[/red]")
            raise typer.Exit(code=1)

    # Import SimulationConfig for Pydantic configuration
    from controller.configs.simulation_config import SimulationConfig

    simulation_config = None
    if not auto and mission_file is None:
        mission_file = _prompt_saved_mission_file()
        if mission_file is None:
            console.print("[red]Mission cancelled.[/red]")
            raise typer.Exit()

    if auto:
        _set_run_mission_env(None)
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
        from controller.shared.python.mission.path_following import (
            build_point_to_point_path,
        )

        path = build_point_to_point_path(
            waypoints=[sim_start_pos, sim_end_pos],
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
        _set_run_mission_env(str(m_path))

        # Load JSON and detect format
        if MISSIONS_DIR in m_path.parents:
            mission_data = load_mission_json(m_path.stem)
        else:
            mission_data = json.loads(m_path.read_text())

        simulation_config = SimulationConfig.create_default()
        simulation_config.app_config.input_file_path = str(m_path)
        from controller.shared.python.mission.runtime_loader import (
            compile_unified_mission_runtime,
            parse_unified_mission_payload,
        )

        try:
            mission_def = parse_unified_mission_payload(mission_data)
        except ValueError as exc:
            console.print(f"[red]{exc}[/red]")
            raise typer.Exit(code=1)

        console.print("[cyan]Detected unified mission format...[/cyan]")
        mission_runtime = compile_unified_mission_runtime(
            mission_def,
            simulation_config=simulation_config,
        )
        simulation_config = mission_runtime.simulation_config
        simulation_config.app_config.input_file_path = str(m_path)
        if mission_runtime.runtime_plan is not None:
            required_duration_hint = float(
                mission_runtime.runtime_plan.required_duration_s
            )
        sim_start_pos = mission_runtime.start_pos
        sim_end_pos = mission_runtime.end_pos

        console.print(
            f"[green]Unified mission path: {len(mission_runtime.path)} points, "
            f"{mission_runtime.path_length:.2f}m[/green]"
        )
        if mission_runtime.runtime_plan is not None:
            console.print(
                "[cyan]runtime plan: speed="
                f"{mission_runtime.runtime_plan.path_speed_mps:.3f} m/s, "
                f"ETA={mission_runtime.runtime_plan.estimated_eta_s:.1f}s, "
                f"required_duration={mission_runtime.runtime_plan.required_duration_s:.1f}s[/cyan]"
            )

    else:
        console.print("[red]Mission file is required unless running with --auto.[/red]")
        raise typer.Exit(code=1)

    # Validate configuration at startup
    try:
        from controller.configs.validator import validate_config_at_startup

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
        if required_duration_hint is not None and duration < required_duration_hint:
            is_contract_run = str(
                os.environ.get("SATCTRL_CONTRACT_SCENARIO", "0")
            ).strip() in {"1", "true", "TRUE"}
            if is_contract_run:
                config_overrides["simulation"]["max_duration"] = required_duration_hint
                duration = required_duration_hint
                console.print(
                    "[yellow]Duration override was below contract minimum; "
                    f"enforcing {required_duration_hint:.1f}s.[/yellow]"
                )
            else:
                console.print(
                    "[yellow]Warning: provided duration is below estimated required "
                    f"duration ({required_duration_hint:.1f}s).[/yellow]"
                )
    if controller_profile:
        profile = str(controller_profile).strip().lower()
        if profile not in {
            "cpp_linearized_rti_osqp",
            "cpp_hybrid_rti_osqp",
            "cpp_nonlinear_rti_osqp",
            "cpp_nonlinear_fullnlp_ipopt",
            "cpp_nonlinear_rti_hpipm",
            "cpp_nonlinear_sqp_hpipm",
        }:
            console.print(
                "[red]Invalid controller profile. Use one of: "
                "cpp_linearized_rti_osqp, cpp_hybrid_rti_osqp, "
                "cpp_nonlinear_rti_osqp, cpp_nonlinear_fullnlp_ipopt, "
                "cpp_nonlinear_rti_hpipm, cpp_nonlinear_sqp_hpipm.[/red]"
            )
            raise typer.Exit(code=1)
        if config_overrides is None:
            config_overrides = {}
        if "mpc_core" not in config_overrides:
            config_overrides["mpc_core"] = {}
        config_overrides["mpc_core"]["controller_profile"] = profile
        console.print(f"  Override: Controller profile = {profile}")

    # Create default config if not set by mission
    if simulation_config is None:
        simulation_config = SimulationConfig.create_default()

    # Apply shared/profile parameter policy before AppConfig validation.
    if config_overrides is not None:
        default_profile = simulation_config.app_config.mpc_core.controller_profile
        try:
            (
                config_overrides,
                applied_profile_file,
                shared_parameters_enabled,
                resolved_profile,
            ) = apply_profile_parameter_file_if_needed(
                config_overrides=config_overrides,
                default_profile=default_profile,
            )
        except ValueError as exc:
            console.print(f"[red]{exc}[/red]")
            raise typer.Exit(code=1)
        if shared_parameters_enabled:
            console.print(
                "[green]Shared parameter mode enabled (shared.parameters=true).[/green]"
            )
        elif applied_profile_file:
            console.print(
                "[cyan]Applied profile-specific parameter file: "
                f"{applied_profile_file}[/cyan]"
            )
        os.environ["SATCTRL_SHARED_PARAMETERS"] = (
            "1" if shared_parameters_enabled else "0"
        )
        os.environ["SATCTRL_ACTIVE_CONTROLLER_PROFILE"] = str(resolved_profile)
        if applied_profile_file:
            os.environ["SATCTRL_PROFILE_PARAMETER_FILE"] = applied_profile_file
        else:
            os.environ.pop("SATCTRL_PROFILE_PARAMETER_FILE", None)

    # Apply overrides to config
    if config_overrides:
        simulation_config = SimulationConfig.create_with_overrides(
            config_overrides, base_config=simulation_config
        )
    if mission_file:
        simulation_config.app_config.input_file_path = str(mission_file)

    console.print("[green]Loaded Pydantic configuration[/green]")

    # Initialize Simulation (Pydantic config only - no Hydra)
    try:
        from controller.shared.python.simulation.engine import (
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


@app.command()
def serve(
    host: str = typer.Option("127.0.0.1", help="Host to bind (default: 127.0.0.1)"),
    port: int = typer.Option(8000, help="Port to bind (default: 8000)"),
    dev: bool = typer.Option(
        False, "--dev", help="Enable auto-reload for local development"
    ),
):
    """
    Run Satellite Control dashboard backend.
    """
    import uvicorn

    uvicorn.run(
        "controller.shared.python.dashboard.app:app",
        host=host,
        port=port,
        reload=dev,
        reload_dirs=["controller"] if dev else None,
    )


if __name__ == "__main__":
    app()
