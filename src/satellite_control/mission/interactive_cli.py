"""
Interactive Mission CLI Module

Enhanced user interface using questionary for styled menus
and rich for formatted output.
"""

import sys
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import questionary
from questionary import Style
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
import json
from pathlib import Path

from src.satellite_control.config.simulation_config import SimulationConfig
from src.satellite_control.mission.mission_logic import MissionLogic

# Custom style for questionary
MISSION_STYLE = Style(
    [
        ("qmark", "fg:gray"),  # Subtle gray instead of bold cyan
        ("question", "fg:white"),
        ("answer", "fg:green bold"),
        ("pointer", "fg:cyan bold"),
        ("highlighted", "fg:cyan bold"),
        ("selected", "fg:green"),
        ("separator", "fg:gray"),
        ("instruction", "fg:gray italic"),
    ]
)

# Use a subtle arrow instead of "?" for prompts
QMARK = "›"

console = Console()


class InteractiveMissionCLI:
    """
    Interactive mission CLI using questionary for styled menus.

    Provides a modern terminal UI experience with:
    - Arrow key navigation
    - Visual mission presets
    - Real-time input validation
    - Parameter preview before confirmation
    """

    def __init__(self, logic: Optional[MissionLogic] = None):
        self.logic = logic or MissionLogic()
        self.system_title = "Satellite Control Simulation"

    def show_welcome_banner(self) -> None:
        """Display welcome banner with system info."""
        banner = Text()
        banner.append("◈  ", style="bold")
        banner.append("SATELLITE CONTROL SYSTEM", style="bold cyan")
        banner.append("  ◈", style="bold")

        console.print()
        console.print(
            Panel(
                banner,
                subtitle="MPC-Based Precision Control",
                style="cyan",
                padding=(0, 2),
            )
        )

    def show_mission_menu(self) -> str:
        """Show main mission selection menu with styling."""
        self.show_welcome_banner()

        choices = [
            questionary.Choice(
                title="›  Mission 1: Point-to-Point Path (MPCC)",
                value="path_following",
            ),
            questionary.Choice(
                title="◇  Mission 2: Scan Object (path-only)",
                value="scan_object",
            ),
            questionary.Choice(
                title="◆  Mission 3: Circle Starlink Satellite (path-only)",
                value="starlink_orbit",
            ),
            questionary.Choice(
                title="💾 Load Saved Mission (JSON)",
                value="load_saved",
            ),
            questionary.Separator(),
            questionary.Choice(
                title="×  Exit",
                value="exit",
            ),
        ]

        result = questionary.select(
            "Select Mission Type:",
            choices=choices,
            style=MISSION_STYLE,
            instruction="(Use arrow keys)",
            qmark=QMARK,
        ).ask()

        if result == "exit" or result is None:
            console.print("[yellow]Exiting...[/yellow]")
            sys.exit(0)

        return str(result)

    def select_mission_preset(
        self, return_simulation_config: bool = False
    ) -> Optional[Dict[str, Any]]:
        """Select a mission preset with visual preview.

        Args:
            return_simulation_config: If True, also create and return a SimulationConfig
                with MissionState updated from the preset (v2.0.0 path).
        """
        console.print()
        console.print(Panel("Mission Presets", style="blue"))

        choices = [
            questionary.Choice(
                title="○  Custom Mission (manual configuration)",
                value="custom",
            ),
            questionary.Separator("─── Quick Start Demos ───"),
            questionary.Choice(
                title="●  Simple: (0,0,0) → (1,1,1) [Rotated]",
                value="simple",
            ),
            questionary.Choice(
                title="◆  Obstacle Avoidance: diagonal 3D path",
                value="obstacle",
            ),
            questionary.Choice(
                title="◇  Multi-Waypoint: 3D square ramp",
                value="square",
            ),
            questionary.Choice(
                title="▫  Corridor: navigate through gap + Z",
                value="corridor",
            ),
            questionary.Choice(
                title="↝  Zig Zag: Reaction Wheel Test",
                value="zigzag",
            ),
        ]

        # Scan for saved missions
        saved_missions = []
        missions_dir = Path("missions")
        if missions_dir.exists():
            for f in sorted(missions_dir.glob("*.json")):
                saved_missions.append(
                    questionary.Choice(title=f"💾 {f.stem}", value=f"json:{f}")
                )

        if saved_missions:
            choices.append(questionary.Separator("─── Saved Missions ───"))
            choices.extend(saved_missions)

        result = questionary.select(
            "Select mission preset:",
            choices=choices,
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        if result is None:
            return None

        if result == "custom":
            return None  # Signal to use custom flow

        if result.startswith("json:"):
            return self._load_saved_mission_json(result[5:])

        # V3.0.0: Always create SimulationConfig and MissionState
        preset = self._get_preset_config(result)
        if not preset:
            return {}

        # Create SimulationConfig and update its MissionState using the same preset
        simulation_config = SimulationConfig.create_default()
        mission_state = simulation_config.mission_state
        self._apply_preset(preset, mission_state=mission_state)

        preset["simulation_config"] = simulation_config
        return preset

    def _get_preset_config(self, preset_name: str) -> Dict[str, Any]:
        """Get configuration for a preset mission."""
        presets = {
            "simple": {
                "name": "Simple Navigation",
                "start_pos": (0.0, 0.0, 0.0),
                "start_angle": (0.0, 0.0, 0.0),
                "waypoints": [
                    ((1.0, 1.0, 1.0), (np.radians(45), np.radians(45), np.radians(45)))
                ],
                "obstacles": [],
            },
            "obstacle": {
                "name": "Obstacle Avoidance",
                "start_pos": (1.0, 1.0, 1.0),
                "start_angle": (0.0, 0.0, np.radians(45)),
                "waypoints": [((-1.0, -1.0, 0.0), (0.0, 0.0, np.radians(-135)))],
                "obstacles": [(0.0, 0.0, 0.0, 0.3)],
            },
            "square": {
                "name": "Square Pattern",
                "start_pos": (0.0, 0.0, 0.5),
                "start_angle": (0.0, 0.0, 0.0),
                "waypoints": [
                    ((1.0, 0.0, 1.0), (0.0, 0.0, 0.0)),
                    ((1.0, 1.0, 0.5), (0.0, 0.0, np.radians(90))),
                    ((0.0, 1.0, 0.0), (0.0, 0.0, np.radians(180))),
                    ((0.0, 0.0, 0.5), (0.0, 0.0, np.radians(270))),
                ],
                "obstacles": [],
            },
            "corridor": {
                "name": "Corridor Navigation",
                "start_pos": (1.0, 0.0, 1.0),
                "start_angle": (0.0, 0.0, np.radians(180)),
                "waypoints": [((-1.0, 0.0, 0.5), (0.0, 0.0, np.radians(180)))],
                "obstacles": [(0.0, 0.5, 0.0, 0.3), (0.0, -0.5, 0.0, 0.3)],
            },
            "zigzag": {
                "name": "Zig Zag (RW Test)",
                "start_pos": (-2.0, -2.0, -2.0),
                "start_angle": (0.0, 0.0, 0.0),
                "waypoints": [
                    (
                        (-1.0, 2.0, 2.0),
                        (np.radians(45), np.radians(45), np.radians(45)),
                    ),
                    (
                        (0.0, -2.0, 2.0),
                        (np.radians(-45), np.radians(45), np.radians(-45)),
                    ),
                    (
                        (1.0, 2.0, -2.0),
                        (np.radians(45), np.radians(-45), np.radians(45)),
                    ),
                    (
                        (2.0, -2.0, -2.0),
                        (np.radians(90), np.radians(0), np.radians(90)),
                    ),
                    ((2.0, 0.0, 0.0), (np.radians(0), np.radians(0), np.radians(0))),
                ],
                "obstacles": [],
            },
        }

        preset = presets.get(preset_name, presets["simple"])

        # Show preview
        self._show_mission_preview(preset)

        if not self._confirm_mission(preset["name"]):
            return {}

        # Return full preset dict (includes obstacles, waypoints, etc.)
        # This allows _apply_preset to work correctly
        preset["mission_type"] = "path_following"
        preset["mode"] = "path_following"
        preset["preset_name"] = preset_name
        preset["preset_name"] = preset_name
        return preset

    def _load_saved_mission_json(self, path_str: str) -> Optional[Dict[str, Any]]:
        """Load a mission config from a JSON file."""
        path = Path(path_str)
        if not path.exists():
            console.print(f"[red]Error: Mission file not found: {path}[/red]")
            return None

        try:
            data = json.loads(path.read_text())
        except Exception as e:
            console.print(f"[red]Error loading mission file: {e}[/red]")
            return None

        # Create config
        simulation_config = SimulationConfig.create_default()
        ms = simulation_config.mission_state

        # Hydrate MissionState from data (MissionConfigModel structure)
        # start_position, end_position, end_orientation, obstacles
        start_pos = tuple(data.get("start_position", [10, 0, 0]))
        end_pos = tuple(data.get("end_position", [0, 0, 0]))
        end_angle = tuple(data.get("end_orientation", [0, 0, 0]))

        # Obstacles
        obs_data = data.get("obstacles", [])
        obstacles = []
        for o in obs_data:
            # ObstacleModel: position, radius
            p = o.get("position", [0, 0, 0])
            r = o.get("radius", 0.5)
            obstacles.append((p[0], p[1], p[2], r))

        ms.obstacles = obstacles
        ms.obstacles_enabled = len(obstacles) > 0

        # Mesh Scan / Trajectory
        mesh_scan = data.get("mesh_scan")
        if mesh_scan:
            from src.satellite_control.mission.mesh_scan import (
                build_mesh_scan_trajectory,
            )

            try:
                # MeshScanConfigModel fields (path-only)
                speed_max = float(mesh_scan.get("speed_max", 0.2))
                path, _, path_length = build_mesh_scan_trajectory(
                    obj_path=mesh_scan.get("obj_path"),
                    standoff=mesh_scan.get("standoff", 0.5),
                    levels=mesh_scan.get("levels", 8),
                    points_per_circle=mesh_scan.get("points_per_circle", 72),
                    v_max=speed_max,
                    v_min=mesh_scan.get("speed_min", 0.05),
                    lateral_accel=mesh_scan.get("lateral_accel", 0.05),
                    dt=float(simulation_config.app_config.mpc.dt),
                    z_margin=mesh_scan.get("z_margin", 0.0),
                    build_trajectory=False,
                )

                simulation_config.app_config.mpc.path_speed = speed_max

                ms.mesh_scan_mode_active = True
                ms.mesh_scan_obj_path = mesh_scan.get("obj_path")
                ms.trajectory_mode_active = False
                ms.mpcc_path_waypoints = path
            except Exception as e:
                console.print(
                    f"[yellow]Warning: Failed to rebuild scan path: {e}[/yellow]"
                )

        # Construct the return dict expected by run_simulation
        return {
            "mission_type": "saved_json",
            "start_pos": start_pos,
            "start_angle": (
                0,
                0,
                0,
            ),  # Assuming start orientation is 0 unless specified?
            "end_pos": end_pos,
            "end_angle": end_angle,
            "simulation_config": simulation_config,
            "preset_name": Path(path_str).stem,
        }

    @staticmethod
    def _format_euler_deg(euler: Tuple[float, float, float]) -> str:
        roll, pitch, yaw = np.degrees(euler)
        return f"roll={roll:.1f}°, pitch={pitch:.1f}°, yaw={yaw:.1f}°"

    def _show_mission_preview(self, preset: Dict[str, Any]) -> None:
        """Show a visual preview of the mission configuration."""
        table = Table(title=f"◇ {preset['name']}", style="cyan")
        table.add_column("Parameter", style="bold")
        table.add_column("Value", style="green")

        # Start position
        sp = preset["start_pos"]
        sa = self._format_euler_deg(preset["start_angle"])
        if len(sp) == 3:
            table.add_row(
                "Start Position", f"({sp[0]:.1f}, {sp[1]:.1f}, {sp[2]:.1f}) m"
            )
        else:
            table.add_row("Start Position", f"({sp[0]:.1f}, {sp[1]:.1f}) m")
        table.add_row("Start Angle", sa)

        # Waypoints
        for i, (pos, angle) in enumerate(preset["waypoints"], 1):
            a_deg = self._format_euler_deg(angle)
            if len(pos) == 3:
                table.add_row(
                    f"Waypoint {i}",
                    f"({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}) m @ {a_deg}",
                )
            else:
                table.add_row(
                    f"Waypoint {i}", f"({pos[0]:.1f}, {pos[1]:.1f}) m @ {a_deg}"
                )

        # Obstacles
        if preset["obstacles"]:
            for i, (x, y, z, r) in enumerate(preset["obstacles"], 1):
                table.add_row(
                    f"Obstacle {i}", f"({x:.1f}, {y:.1f}, {z:.1f}) r={r:.2f}m"
                )
        else:
            table.add_row("Obstacles", "None")

        console.print()
        console.print(table)

    def _confirm_mission(self, mission_name: str) -> bool:
        """Confirm mission start with styled prompt."""
        return (
            questionary.confirm(
                f"Proceed with {mission_name}?",
                default=True,
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or False
        )

    def _apply_preset(self, preset: Dict[str, Any], mission_state) -> None:
        """Apply preset configuration.

        Args:
            preset: Preset configuration dictionary
            mission_state: MissionState to update (required in V3.0.0).
        """
        obstacles = preset.get("obstacles", [])

        # V3.0.0: Always require mission_state (no legacy fallback)
        if mission_state is None:
            raise ValueError(
                "mission_state is required (V3.0.0: no SatelliteConfig fallback)"
            )

        # Update MissionState
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = len(obstacles) > 0

        waypoints = preset.get("waypoints", [])
        start_pos = preset.get("start_pos")
        if start_pos and waypoints:
            from src.satellite_control.mission.path_following import (
                build_point_to_point_path,
            )

            positions = [start_pos] + [t[0] for t in waypoints]
            path = build_point_to_point_path(
                waypoints=positions,
                obstacles=None,
                step_size=0.1,
            )
            # path_length calculation removed as it is unused
            mission_state.mpcc_path_waypoints = path

    def get_position_interactive(
        self,
        prompt: str,
        default: Tuple[float, ...] = (0.0, 0.0, 0.0),
        dim: int = 3,
    ) -> Tuple[float, ...]:
        """Get position with interactive validation."""
        console.print(f"\n[bold]{prompt}[/bold]")

        def_x = default[0] if len(default) > 0 else 0.0
        def_y = default[1] if len(default) > 1 else 0.0
        def_z = default[2] if len(default) > 2 else 0.0

        x = questionary.text(
            f"X position (meters) [{def_x:.2f}]:",
            default=str(def_x),
            validate=lambda x: self._validate_float(x),
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        y = questionary.text(
            f"Y position (meters) [{def_y:.2f}]:",
            default=str(def_y),
            validate=lambda x: self._validate_float(x),
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        if dim == 3:
            z = questionary.text(
                f"Z position (meters) [{def_z:.2f}]:",
                default=str(def_z),
                validate=lambda x: self._validate_float(x),
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            return (float(x or def_x), float(y or def_y), float(z or def_z))

        return (float(x or def_x), float(y or def_y))

    def get_angle_interactive(
        self,
        prompt: str,
        default_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> Tuple[float, float, float]:
        """Get 3D Euler angles with interactive validation."""
        roll_default, pitch_default, yaw_default = default_deg
        roll = questionary.text(
            f"{prompt} roll (degrees) [{roll_default:.1f}]:",
            default=str(roll_default),
            validate=lambda x: self._validate_float(x),
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()
        pitch = questionary.text(
            f"{prompt} pitch (degrees) [{pitch_default:.1f}]:",
            default=str(pitch_default),
            validate=lambda x: self._validate_float(x),
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()
        yaw = questionary.text(
            f"{prompt} yaw (degrees) [{yaw_default:.1f}]:",
            default=str(yaw_default),
            validate=lambda x: self._validate_float(x),
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        return (
            float(np.radians(float(roll or roll_default))),
            float(np.radians(float(pitch or pitch_default))),
            float(np.radians(float(yaw or yaw_default))),
        )

    def configure_sphere_obstacles_interactive(
        self,
    ) -> Tuple[List[Tuple[float, float, float, float]], bool]:
        """Configure spherical obstacles with fixed 0.5m radius."""
        console.print()
        console.print(Panel("Obstacle Configuration", style="yellow"))
        add_obs = questionary.confirm(
            "Add spherical obstacles (radius 0.5m)?",
            default=False,
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        obstacles: List[Tuple[float, float, float, float]] = []
        if not add_obs:
            return obstacles, False

        count = questionary.text(
            "How many obstacles?",
            default="1",
            validate=lambda x: x.isdigit() and int(x) > 0,
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        num_obs = int(count or "1")
        for idx in range(num_obs):
            pos = self.get_position_interactive(
                f"Obstacle {idx + 1} position", default=(0.0, 0.0, 0.0)
            )
            obstacles.append((pos[0], pos[1], pos[2], 0.5))

        return obstacles, True

    def run_path_following_mission(self) -> Dict[str, Any]:
        """Mission 1: Point-to-point path with trajectory tracking."""
        from src.satellite_control.config.simulation_config import SimulationConfig
        from src.satellite_control.mission.path_following import (
            build_point_to_point_path,
        )

        simulation_config = SimulationConfig.create_default()
        mission_state = simulation_config.mission_state

        console.print()
        console.print(Panel("Mission 1: Point-to-Point Path", style="green"))

        start_pos = self.get_position_interactive(
            "Starting Position", default=(0.0, 0.0, 0.0)
        )
        start_angle = self.get_angle_interactive(
            "Starting Orientation", (0.0, 0.0, 0.0)
        )

        waypoints: List[
            Tuple[Tuple[float, float, float], Tuple[float, float, float]]
        ] = []
        console.print("\n[bold]Define waypoints (at least 1 endpoint required)[/bold]")

        while True:
            wp_num = len(waypoints) + 1
            pos = self.get_position_interactive(
                f"Waypoint {wp_num} position", default=(1.0, 0.0, 0.0)
            )
            ang = self.get_angle_interactive(
                f"Waypoint {wp_num} orientation", (0.0, 0.0, 0.0)
            )
            waypoints.append((pos, ang))

            add_more = questionary.confirm(
                "Add another waypoint?",
                default=False,
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            if not add_more:
                break

        speed = float(
            questionary.text(
                "Travel speed (m/s) [0.1]:",
                default="0.1",
                validate=lambda x: self._validate_positive_float(x) or x == "0",
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.1
        )

        obstacles, obstacles_enabled = self.configure_sphere_obstacles_interactive()

        positions = [start_pos] + [wp[0] for wp in waypoints]
        path = build_point_to_point_path(
            waypoints=positions,
            obstacles=None,
            step_size=0.1,
        )

        mission_state.trajectory_mode_active = False
        mission_state.trajectory_type = "path"
        mission_state.trajectory_hold_end = 5.0
        mission_state.trajectory_mode_active = False
        mission_state.trajectory_type = "path"
        mission_state.trajectory_hold_end = 5.0
        mission_state.mesh_scan_mode_active = False
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = obstacles_enabled

        simulation_config.app_config.mpc.path_speed = speed
        # Store path waypoints for MPC initialization
        mission_state.mpcc_path_waypoints = path  # Pass to simulation for set_path()

        return {
            "mission_type": "path_following",
            "start_pos": start_pos,
            "start_angle": start_angle,
            "simulation_config": simulation_config,
            "mpcc_path": path,  # Include path for MPC initialization
        }

    def run_scan_mission(self) -> Dict[str, Any]:
        """Mission 2: Scan object with circular/square rings."""
        from pathlib import Path

        from src.satellite_control.config.simulation_config import SimulationConfig
        from src.satellite_control.mission.mesh_scan import (
            build_cylinder_scan_trajectory,
        )

        simulation_config = SimulationConfig.create_default()
        mission_state = simulation_config.mission_state

        console.print()
        console.print(Panel("Mission 2: Scan Object", style="blue"))

        start_pos = self.get_position_interactive(
            "Starting Position", default=(0.0, 0.0, 0.0)
        )
        start_angle = self.get_angle_interactive(
            "Starting Orientation", (0.0, 0.0, 0.0)
        )

        mesh_dir = Path("models/meshes")
        obj_files = sorted(mesh_dir.glob("*.obj")) if mesh_dir.exists() else []
        choices = [
            questionary.Choice("Built-in Cylinder (0.5m dia x 3m)", value="cylinder")
        ]
        choices.extend([questionary.Choice(f.name, value=str(f)) for f in obj_files])

        selection = questionary.select(
            "Select object to scan:",
            choices=choices,
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        obj_pose = self.get_position_interactive(
            "Object position", default=(0.0, 0.0, 0.0)
        )
        obj_angle = self.get_angle_interactive("Object orientation", (0.0, 0.0, 0.0))
        standoff = float(
            questionary.text(
                "Scan standoff distance (m) [0.5]:",
                default="0.5",
                validate=lambda x: self._validate_positive_float(x) or x == "0",
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.5
        )

        speed = float(
            questionary.text(
                "Scan travel speed (m/s) [0.1]:",
                default="0.1",
                validate=lambda x: self._validate_positive_float(x) or x == "0",
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.1
        )

        obstacles, obstacles_enabled = self.configure_sphere_obstacles_interactive()

        dt = float(simulation_config.app_config.mpc.dt)

        if selection == "cylinder":
            path, _, path_length = build_cylinder_scan_trajectory(
                center=obj_pose,
                rotation_xyz=obj_angle,
                radius=0.25,
                height=3.0,
                standoff=standoff,
                fov_deg=60.0,
                overlap=0.85,
                v_max=speed,
                v_min=0.05,
                lateral_accel=0.05,
                dt=dt,
                ring_shape="circle",
                hold_start=0.0,
                hold_end=0.0,
                build_trajectory=False,
            )
        else:
            # Placeholder: default to cylinder if OBJ is not yet supported
            path, _, path_length = build_cylinder_scan_trajectory(
                center=obj_pose,
                rotation_xyz=obj_angle,
                radius=0.25,
                height=3.0,
                standoff=standoff,
                fov_deg=60.0,
                overlap=0.85,
                v_max=speed,
                v_min=0.05,
                lateral_accel=0.05,
                dt=dt,
                ring_shape="circle",
                hold_start=0.0,
                hold_end=0.0,
                build_trajectory=False,
            )
            mission_state.mesh_scan_obj_path = selection

        simulation_config.app_config.mpc.path_speed = speed
        mission_state.trajectory_mode_active = False
        mission_state.trajectory_type = "scan"
        mission_state.mesh_scan_mode_active = True

        mission_state.mesh_scan_object_pose = (
            obj_pose[0],
            obj_pose[1],
            obj_pose[2],
            obj_angle[0],
            obj_angle[1],
            obj_angle[2],
        )
        mission_state.mesh_scan_standoff = standoff
        mission_state.mesh_scan_fov_deg = 60.0
        mission_state.mesh_scan_overlap = 0.85
        mission_state.mesh_scan_ring_shape = "circle"
        mission_state.mesh_scan_ring_shape = "circle"
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = obstacles_enabled
        mission_state.trajectory_hold_end = 5.0
        mission_state.mpcc_path_waypoints = path

        return {
            "mission_type": "scan_object",
            "start_pos": start_pos,
            "start_angle": start_angle,
            "simulation_config": simulation_config,
        }

    def run_starlink_orbit_mission(self) -> Dict[str, Any]:
        """Mission 3: Circle Starlink Satellite with camera facing object."""
        from pathlib import Path

        from src.satellite_control.config.simulation_config import SimulationConfig
        from src.satellite_control.mission.starlink_orbit import (
            build_starlink_orbit_trajectory,
            get_starlink_bounds,
        )

        simulation_config = SimulationConfig.create_default()
        mission_state = simulation_config.mission_state

        console.print()
        console.print(Panel("Mission 3: Circle Starlink Satellite", style="magenta"))

        # Find the starlink.obj file
        obj_path = Path("OBJ_files/starlink.obj")
        if not obj_path.exists():
            console.print(f"[red]Error: Starlink OBJ not found at {obj_path}[/red]")
            return {}

        # Get Starlink bounds info
        try:
            radius_xy, z_height, z_center = get_starlink_bounds(str(obj_path))
            console.print(
                f"[cyan]Starlink dimensions: radius={radius_xy:.2f}m, height={z_height:.2f}m[/cyan]"
            )
        except Exception as e:
            console.print(f"[red]Error loading Starlink: {e}[/red]")
            return {}

        start_pos = self.get_position_interactive(
            "Starting Position", default=(2.0, 0.0, 0.0)
        )
        start_angle = self.get_angle_interactive(
            "Starting Orientation", (0.0, 0.0, 0.0)
        )

        starlink_pos = self.get_position_interactive(
            "Starlink Position", default=(0.0, 0.0, 0.0)
        )
        # Default to no rotation - orbit will be around Z axis
        starlink_angle = self.get_angle_interactive(
            "Starlink Orientation", (0.0, 0.0, 0.0)
        )

        standoff = float(
            questionary.text(
                "Standoff distance from Starlink surface (m) [0.5]:",
                default="0.5",
                validate=lambda x: self._validate_positive_float(x) or x == "0",
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.5
        )

        z_step = float(
            questionary.text(
                "Z height increment between orbits (m) [0.5]:",
                default="0.5",
                validate=lambda x: self._validate_positive_float(x),
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.5
        )

        speed = float(
            questionary.text(
                "Orbit travel speed (m/s) [0.1]:",
                default="0.1",
                validate=lambda x: self._validate_positive_float(x) or x == "0",
                style=MISSION_STYLE,
                qmark=QMARK,
            ).ask()
            or 0.1
        )

        obstacles, obstacles_enabled = self.configure_sphere_obstacles_interactive()

        try:
            path, _, path_length, _ = build_starlink_orbit_trajectory(
                obj_path=str(obj_path),
                center=starlink_pos,
                rotation_xyz=starlink_angle,
                standoff=standoff,
                z_step=z_step,
                points_per_ring=36,
                v_max=speed,
                v_min=0.05,
                lateral_accel=0.05,
                camera_face="-Y",
                build_trajectory=False,
            )
        except Exception as e:
            console.print(f"[red]Error generating orbit trajectory: {e}[/red]")
            return {}

        console.print(
            f"[green]Generated orbit: {len(path)} waypoints, {path_length:.1f}m total[/green]"
        )

        mission_state.trajectory_mode_active = False
        mission_state.trajectory_type = "starlink_orbit"
        mission_state.trajectory_hold_end = 5.0
        mission_state.mesh_scan_mode_active = True
        mission_state.mesh_scan_obj_path = str(obj_path)
        mission_state.dxf_shape_path = path
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = obstacles_enabled
        mission_state.mpcc_path_waypoints = path

        simulation_config.app_config.mpc.path_speed = speed

        return {
            "mission_type": "starlink_orbit",
            "start_pos": start_pos,
            "start_angle": start_angle,
            "simulation_config": simulation_config,
        }

    def run_saved_mission(self) -> Dict[str, Any]:
        """Load a saved mission from JSON."""
        return self.select_mission_preset(return_simulation_config=True) or {}

    def configure_obstacles_interactive(
        self, mission_state
    ) -> Tuple[List[Tuple[float, float, float, float]], bool]:
        """Configure obstacles with interactive menu.

        Args:
            mission_state: MissionState to update (required in V3.0.0).

        Returns:
            Tuple of (obstacles list, obstacles_enabled bool).
        """
        # V3.0.0: Always require mission_state (no legacy fallback)
        if mission_state is None:
            raise ValueError(
                "mission_state is required (V3.0.0: no SatelliteConfig fallback)"
            )

        console.print()
        console.print(Panel("Obstacle Configuration", style="yellow"))

        choices = [
            questionary.Choice(title="○  No obstacles", value="none"),
            questionary.Choice(title="●  Central obstacle", value="central"),
            questionary.Choice(title="◇  Corridor (two obstacles)", value="corridor"),
            questionary.Choice(title="◆  Scattered (four corners)", value="scattered"),
            questionary.Choice(title="▫  Custom (manual entry)", value="custom"),
        ]

        result = questionary.select(
            "Select obstacle configuration:",
            choices=choices,
            style=MISSION_STYLE,
            qmark=QMARK,
        ).ask()

        obstacles: List[Tuple[float, float, float, float]] = []

        if result == "central":
            obstacles.append((0.0, 0.0, 0.0, 0.3))
            console.print("[green]+ Added central obstacle at (0, 0, 0)[/green]")

        elif result == "corridor":
            obstacles.extend([(0.0, 0.4, 0.0, 0.25), (0.0, -0.4, 0.0, 0.25)])
            console.print("[green]+ Added corridor obstacles[/green]")

        elif result == "scattered":
            scattered = [
                (0.5, 0.5, 0.0, 0.2),
                (-0.5, 0.5, 0.0, 0.2),
                (0.5, -0.5, 0.0, 0.2),
                (-0.5, -0.5, 0.0, 0.2),
            ]
            obstacles.extend(scattered)
            console.print("[green]+ Added 4 corner obstacles[/green]")

        elif result == "custom":
            obstacles = self._add_custom_obstacles(mission_state)

        obstacles_enabled = len(obstacles) > 0

        # Update MissionState
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = obstacles_enabled

        return obstacles, obstacles_enabled

    def _add_custom_obstacles(
        self, mission_state
    ) -> List[Tuple[float, float, float, float]]:
        """Add custom obstacles interactively.

        Args:
            mission_state: MissionState to update (required in V3.0.0).

        Returns:
            List of obstacles as (x, y, z, radius) tuples.
        """
        obstacles: List[Tuple[float, float, float, float]] = []
        while True:
            add_more = questionary.confirm(
                "Add an obstacle?",
                default=False,
                style=MISSION_STYLE,
            ).ask()

            if not add_more:
                break

            pos = self.get_position_interactive("Obstacle position", dim=3)
            radius = questionary.text(
                "Radius (meters) [0.3]:",
                default="0.3",
                validate=lambda x: self._validate_positive_float(x),
                style=MISSION_STYLE,
            ).ask()

            r = float(radius or 0.3)
            obstacles.append((pos[0], pos[1], pos[2] if len(pos) > 2 else 0.0, r))
            console.print(f"[green]+ Added obstacle at {pos}[/green]")

        # V3.0.0: Update mission_state directly
        mission_state.obstacles = obstacles
        mission_state.obstacles_enabled = len(obstacles) > 0

        return obstacles

    def run_custom_waypoint_mission(self) -> Dict[str, Any]:
        """Run custom waypoint mission configuration."""
        # Create SimulationConfig for v2.0.0 pattern
        simulation_config = SimulationConfig.create_default()
        mission_state = simulation_config.mission_state

        console.print()
        console.print(Panel("Custom Waypoint Mission", style="green"))

        # Get start position
        start_pos = self.get_position_interactive(
            "Starting Position",
            default=(1.0, 1.0, 0.0),
        )
        start_angle = self.get_angle_interactive("Starting Angle", (0.0, 0.0, 0.0))

        # Get waypoints
        waypoints: List[Tuple[Tuple[float, ...], Tuple[float, float, float]]] = []

        console.print("\n[bold]Define waypoints[/bold] (at least 1 required)")

        while True:
            wp_num = len(waypoints) + 1
            console.print(f"\n[cyan]Waypoint {wp_num}[/cyan]")

            pos = self.get_position_interactive(
                f"Waypoint {wp_num}", (0.0, 0.0, 0.0)
            )
            angle = self.get_angle_interactive(
                f"Waypoint {wp_num} angle", (0.0, 0.0, 0.0)
            )

            waypoints.append((pos, angle))

            if len(waypoints) >= 1:
                add_more = questionary.confirm(
                    "Add another waypoint?",
                    default=False,
                    style=MISSION_STYLE,
                ).ask()
                if not add_more:
                    break

        # Configure obstacles
        self.configure_obstacles_interactive(mission_state)

        # Show summary
        self._show_custom_mission_summary(
            start_pos, start_angle, waypoints, mission_state
        )

        if not self._confirm_mission("Custom Waypoint Mission"):
            return {}

        positions = [start_pos] + [wp[0] for wp in waypoints]
        from src.satellite_control.mission.path_following import (
            build_point_to_point_path,
        )

        path = build_point_to_point_path(
            waypoints=positions,
            obstacles=None,
            step_size=0.1,
        )

        # Update MissionState (path-following)
        mission_state.mpcc_path_waypoints = path

        return {
            "mission_type": "path_following",
            "mode": "path_following",
            "start_pos": start_pos,
            "start_angle": start_angle,
            "simulation_config": simulation_config,
        }

    def _show_custom_mission_summary(
        self,
        start_pos: Tuple[float, ...],
        start_angle: Tuple[float, float, float],
        waypoints: List[Tuple[Tuple[float, ...], Tuple[float, float, float]]],
        mission_state,
    ) -> None:
        """Show summary of custom mission configuration."""
        table = Table(title="◇ Mission Summary", style="green")
        table.add_column("Parameter", style="bold")
        table.add_column("Value", style="cyan")

        sp = start_pos
        sa = self._format_euler_deg(start_angle)
        if len(sp) == 3:
            table.add_row("Start", f"({sp[0]:.2f}, {sp[1]:.2f}, {sp[2]:.2f}) @ {sa}")
        else:
            table.add_row("Start", f"({sp[0]:.2f}, {sp[1]:.2f}) @ {sa}")

        for i, (pos, angle) in enumerate(waypoints, 1):
            a_deg = self._format_euler_deg(angle)
            if len(pos) == 3:
                table.add_row(
                    f"Waypoint {i}",
                    f"({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) @ {a_deg}",
                )
            else:
                table.add_row(
                    f"Waypoint {i}", f"({pos[0]:.2f}, {pos[1]:.2f}) @ {a_deg}"
                )

        # V3.0.0: Read from mission_state instead of SatelliteConfig
        obstacles = mission_state.obstacles if mission_state else []
        if obstacles:
            table.add_row("Obstacles", f"{len(obstacles)} configured")
        else:
            table.add_row("Obstacles", "None")

        console.print()
        console.print(table)

    @staticmethod
    def _validate_float(value: str) -> bool:
        """Validate float input."""
        if not value:
            return True  # Allow empty for defaults
        try:
            float(value)
            return True
        except ValueError:
            return False

    @staticmethod
    def _validate_positive_float(value: str) -> bool:
        """Validate positive float input."""
        if not value:
            return True
        try:
            return float(value) > 0
        except ValueError:
            return False
