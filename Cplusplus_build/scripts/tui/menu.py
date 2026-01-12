"""
Main menu system for TUI.
"""

import os
import subprocess
from pathlib import Path
from typing import Optional, List, Tuple

# Try to import rich for beautiful formatting, fall back to plain text
try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.prompt import Prompt, Confirm
    from rich.text import Text
    from rich import print as rprint
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Note: Install 'rich' for better formatting: pip install rich")


class MainMenu:
    """Main menu for the TUI."""
    
    def __init__(self):
        self.base_path = Path(__file__).parent.parent.parent
        self.build_path = self.base_path / "build"
        self.config_path = self.base_path / "config"
        self.data_path = self.base_path / "Data" / "Simulation"
        
        if RICH_AVAILABLE:
            self.console = Console()
        
    def clear_screen(self):
        """Clear terminal screen."""
        os.system('clear' if os.name == 'posix' else 'cls')
    
    def print_header(self):
        """Print the main header."""
        if RICH_AVAILABLE:
            self.console.print(Panel.fit(
                "[bold cyan]SATELLITE CONTROL SYSTEM[/bold cyan]\n"
                "[dim]C++ Backend with Python TUI[/dim]",
                border_style="cyan"
            ))
        else:
            print("=" * 50)
            print("    SATELLITE CONTROL SYSTEM")
            print("    C++ Backend with Python TUI")
            print("=" * 50)
    
    def print_menu(self):
        """Print main menu options."""
        options = [
            ("1", "🚀 Run Mission", "Execute a simulation"),
            ("2", "📝 Create Custom Mission", "Build waypoints interactively"),
            ("3", "📋 List Available Missions", "Show config files"),
            ("4", "📊 View Last Results", "Display simulation stats"),
            ("5", "📈 Generate Visualization", "Create plots from data"),
            ("6", "⚠️  Fault Injection Test", "Test fault tolerance"),
            ("7", "🔧 Vehicle Configuration", "View/edit vehicle params"),
            ("8", "💾 Export Flight Code", "Generate C99 code"),
            ("Q", "❌ Exit", "Quit the application"),
        ]
        
        if RICH_AVAILABLE:
            table = Table(show_header=False, box=None, padding=(0, 2))
            table.add_column("Key", style="bold yellow")
            table.add_column("Option", style="bold white")
            table.add_column("Description", style="dim")
            
            for key, opt, desc in options:
                table.add_row(f"[{key}]", opt, desc)
            
            self.console.print(table)
        else:
            for key, opt, desc in options:
                print(f"  [{key}] {opt} - {desc}")
    
    def get_choice(self) -> str:
        """Get user menu choice."""
        print()
        if RICH_AVAILABLE:
            choice = Prompt.ask("[bold]Select option[/bold]", default="1")
        else:
            choice = input("Select option: ").strip()
        return choice.upper()
    
    def run(self):
        """Main menu loop."""
        while True:
            self.clear_screen()
            self.print_header()
            print()
            self.print_menu()
            
            choice = self.get_choice()
            
            if choice == "1":
                self.run_mission()
            elif choice == "2":
                self.create_mission()
            elif choice == "3":
                self.list_missions()
            elif choice == "4":
                self.view_results()
            elif choice == "5":
                self.generate_visualization()
            elif choice == "6":
                self.fault_test()
            elif choice == "7":
                self.vehicle_config()
            elif choice == "8":
                self.export_code()
            elif choice == "Q":
                print("\nGoodbye!")
                break
            else:
                self.pause("Invalid option. Press Enter to continue...")
    
    def pause(self, msg: str = "Press Enter to continue..."):
        """Pause and wait for user input."""
        input(f"\n{msg}")
    
    # ==================== Feature Implementations ====================
    
    def run_mission(self):
        """Run a simulation mission."""
        self.clear_screen()
        self.print_section("Run Mission")
        
        # List available missions
        missions = self.get_mission_files()
        
        # Determine latest sim directory from previous runs if needed (optional)
        # But here we just list configs.
        
        if not missions:
            print("No mission files found in config/")
            self.pause()
            return
        
        print("Available missions:")
        for i, m in enumerate(missions, 1):
            print(f"  [{i}] {m.name}")
        print(f"  [0] Cancel")
        
        try:
            choice = int(input("\nSelect mission: "))
            if choice == 0:
                return
            if 1 <= choice <= len(missions):
                mission_path = missions[choice - 1]
                self.execute_simulation(mission_path)
        except ValueError:
            print("Invalid selection")
        
        self.pause()
    
    def execute_simulation(self, config_path: Path):
        """Execute C++ simulation."""
        exe_path = self.build_path / "sat_control"
        if not exe_path.exists():
            print(f"Error: Executable not found at {exe_path}")
            print("Run 'make sat_control' in build directory first.")
            return
        
        print(f"\n{'='*50}")
        print(f"Running: {config_path.name}")
        print(f"{'='*50}\n")
        
        # Run C++ executable and stream output
        process = subprocess.Popen(
            [str(exe_path), str(config_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            cwd=str(self.base_path)
        )
        
        sim_dir = None
        for line in process.stdout:
            print(line, end='')
            if "Data Logger: Created directory" in line:
                # Extract path
                parts = line.strip().split("Data Logger: Created directory")
                if len(parts) > 1:
                    sim_dir = self.base_path / parts[1].strip()
        
        process.wait()
        print(f"\n{'='*50}")
        print(f"Simulation complete (exit code: {process.returncode})")
        
        # Auto-generate visualization
        if process.returncode == 0:
            print("\n📊 Generating visualization...")
            self._auto_visualize()
    
    def create_mission(self):
        """Interactive mission builder."""
        self.clear_screen()
        self.print_section("Create Custom Mission")
        
        print("Mission Builder - Create waypoints interactively\n")
        
        # Get mission name
        name = input("Mission name: ").strip() or "Custom Mission"
        
        # Build waypoints
        waypoints = []
        print("\nEnter waypoints (empty position to finish):")
        
        while True:
            print(f"\nWaypoint {len(waypoints) + 1}:")
            pos_str = input("  Position [x,y,z]: ").strip()
            
            if not pos_str:
                break
            
            try:
                pos = [float(x.strip()) for x in pos_str.split(",")]
                if len(pos) != 3:
                    print("  Need 3 values (x,y,z)")
                    continue
                
                hold = float(input("  Hold time [s] (default 2): ") or "2")
                waypoints.append({"position": pos, "hold_time": hold})
                print(f"  ✓ Added waypoint at ({pos[0]}, {pos[1]}, {pos[2]})")
            except ValueError:
                print("  Invalid input, try again")
        
        if not waypoints:
            print("\nNo waypoints added, cancelling.")
            self.pause()
            return
        
        # Generate YAML
        filename = name.lower().replace(" ", "_") + ".yaml"
        filepath = self.config_path / "mission" / filename
        
        self.write_mission_yaml(filepath, name, waypoints)
        print(f"\n✓ Mission saved to: {filepath}")
        
        if input("\nRun this mission now? [y/N]: ").lower() == 'y':
            self.execute_simulation(filepath)
        
        self.pause()
    
    def write_mission_yaml(self, path: Path, name: str, waypoints: List[dict]):
        """Write mission YAML file."""
        path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(path, 'w') as f:
            f.write(f"# Auto-generated mission: {name}\n")
            f.write("vehicle:\n")
            f.write("  name: \"Satellite\"\n")
            f.write("  mass: 10.0\n")
            f.write("  inertia_diag: [1.0, 1.0, 1.0]\n")
            f.write("  center_of_mass: [0.0, 0.0, 0.0]\n")
            f.write("  reaction_wheels:\n")
            f.write("    max_torque: 0.1\n")
            f.write("    max_speed_rad_s: 600.0\n")
            f.write("    inertia: 0.001\n")
            f.write("    enabled: true\n")
            f.write("  thrusters:\n")
            for i, (pos, dir) in enumerate([
                ([-0.15, 0, 0], [1, 0, 0]),
                ([0.15, 0, 0], [-1, 0, 0]),
                ([0, -0.15, 0], [0, 1, 0]),
                ([0, 0.15, 0], [0, -1, 0]),
                ([0, 0, -0.15], [0, 0, 1]),
                ([0, 0, 0.15], [0, 0, -1]),
            ], 1):
                f.write(f"    - id: \"T{i}\"\n")
                f.write(f"      position: {pos}\n")
                f.write(f"      direction: {dir}\n")
                f.write(f"      max_thrust: 1.0\n")
            
            f.write("\nmission:\n")
            f.write(f"  name: \"{name}\"\n")
            f.write("  max_duration: 120.0\n")
            f.write("  loop: false\n")
            f.write("  waypoints:\n")
            for wp in waypoints:
                f.write(f"    - position: {wp['position']}\n")
                f.write("      attitude: [1.0, 0.0, 0.0, 0.0]\n")
                f.write(f"      hold_time: {wp['hold_time']}\n")
                f.write("      pos_tolerance: 0.1\n")
    
    def list_missions(self):
        """List available mission configs."""
        self.clear_screen()
        self.print_section("Available Missions")
        
        missions = self.get_mission_files()
        
        if RICH_AVAILABLE:
            table = Table(title="Mission Configurations")
            table.add_column("#", style="dim")
            table.add_column("Filename", style="cyan")
            table.add_column("Path", style="dim")
            
            for i, m in enumerate(missions, 1):
                table.add_row(str(i), m.name, str(m.parent.relative_to(self.base_path)))
            
            self.console.print(table)
        else:
            for i, m in enumerate(missions, 1):
                print(f"  [{i}] {m.name}")
        
        self.pause()
    
    def view_results(self):
        """View last simulation results."""
        self.clear_screen()
        self.print_section("Last Simulation Results")
        
        # Find latest simulation directory
        if not self.data_path.exists():
            print("No simulation data found.")
            self.pause()
            return
        
        sim_dirs = sorted(self.data_path.glob("*"), key=lambda x: x.stat().st_mtime, reverse=True)
        if not sim_dirs:
            print("No simulation directories found.")
            self.pause()
            return
        
        latest = sim_dirs[0]
        csv_path = latest / "control_data.csv"
        
        if not csv_path.exists():
            print(f"No data file in {latest.name}")
            self.pause()
            return
        
        print(f"Simulation: {latest.name}\n")
        
        # Parse CSV and show stats
        import pandas as pd
        df = pd.read_csv(csv_path)
        
        import numpy as np
        final_error = np.sqrt(
            df['Error_X'].iloc[-1]**2 + 
            df['Error_Y'].iloc[-1]**2 + 
            df['Error_Z'].iloc[-1]**2
        )
        
        print(f"  Duration:     {df['Control_Time'].iloc[-1]:.2f} s")
        print(f"  Steps:        {len(df)}")
        print(f"  Final Error:  {final_error:.4f} m")
        print(f"\n  Start:  ({df['Current_X'].iloc[0]:.2f}, {df['Current_Y'].iloc[0]:.2f}, {df['Current_Z'].iloc[0]:.2f})")
        print(f"  Target: ({df['Target_X'].iloc[-1]:.2f}, {df['Target_Y'].iloc[-1]:.2f}, {df['Target_Z'].iloc[-1]:.2f})")
        print(f"  Final:  ({df['Current_X'].iloc[-1]:.2f}, {df['Current_Y'].iloc[-1]:.2f}, {df['Current_Z'].iloc[-1]:.2f})")
        
        self.pause()
    
    def generate_visualization(self):
        """Generate visualization plots."""
        self.clear_screen()
        self.print_section("Generate Visualization")
        
        viz_script = self.base_path / "scripts" / "visualize.py"
        
        print("Generating plots...\n")
        result = subprocess.run(
            ["python3", str(viz_script)],
            cwd=str(self.base_path),
            capture_output=True,
            text=True
        )
        print(result.stdout)
        if result.stderr:
            print(result.stderr)
        
        # Try to open the image
        if "Saved plot to:" in result.stdout:
            png_path = result.stdout.split("Saved plot to:")[-1].strip()
            if os.path.exists(png_path):
                print(f"\nOpening: {png_path}")
                subprocess.run(["open", png_path], check=False)
        
        self.pause()
    
    def fault_test(self):
        """Run fault injection test."""
        self.clear_screen()
        self.print_section("Fault Injection Test")
        
        print("This runs a mission with fault injection at 20 seconds.\n")
        
        # Use default test mission
        test_mission = self.config_path / "mission_test.yaml"
        if test_mission.exists():
            self.execute_simulation(test_mission)
        else:
            print("Default test mission not found.")
        
        self.pause()
    
    def vehicle_config(self):
        """View vehicle configuration."""
        self.clear_screen()
        self.print_section("Vehicle Configuration")
        
        print("Available vehicle templates:\n")
        
        vehicle_path = self.config_path / "vehicle"
        if vehicle_path.exists():
            for f in vehicle_path.glob("*.yaml"):
                print(f"  • {f.stem}")
        else:
            print("  No vehicle configs found.")
        
        self.pause()
    
    def export_code(self):
        """Export flight code (placeholder)."""
        self.clear_screen()
        self.print_section("Export Flight Code")
        
        print("Flight code export generates standalone C99 code")
        print("for embedded deployment.\n")
        print("[Not yet integrated with TUI - run from C++ directly]")
        
        self.pause()
    
    # ==================== Helpers ====================
    
    def print_section(self, title: str):
        """Print section header."""
        if RICH_AVAILABLE:
            self.console.print(f"\n[bold cyan]═══ {title} ═══[/bold cyan]\n")
        else:
            print(f"\n{'═'*3} {title} {'═'*3}\n")
    
    def get_mission_files(self) -> List[Path]:
        """Get all mission YAML files."""
        files = []
        files.extend(self.config_path.glob("*.yaml"))
        files.extend((self.config_path / "mission").glob("*.yaml"))
        return sorted(files)
    
    def _auto_visualize(self, sim_dir: Optional[Path] = None):
        """Auto-generate comprehensive output after simulation."""
        # Use new post-processor for full legacy-style output
        post_script = self.base_path / "scripts" / "post_processor.py"
        
        cmd = ["python3", str(post_script)]
        if sim_dir:
            cmd.append(str(sim_dir))
            
        result = subprocess.run(
            cmd,
            cwd=str(self.base_path),
            capture_output=True,
            text=True
        )
        
        print(result.stdout)
        if result.stderr:
            print(result.stderr)
            
        print("Generating 3D Animation (animation.mp4)...")
        viz_script = self.base_path / "scripts" / "visualize.py"
        viz_cmd = ["python3", str(viz_script), "--save"]
        if sim_dir:
            viz_cmd.append(str(sim_dir))
            
        subprocess.run(
            viz_cmd,
            cwd=str(self.base_path),
            check=False
        )
            
        print("✓ Visualization data saved to Data/Simulation/latest/")

