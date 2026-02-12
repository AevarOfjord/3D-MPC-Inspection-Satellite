"""
Manager for running background simulation processes and streaming logs.
"""
import asyncio
import logging
import os
import sys
from pathlib import Path
from typing import Optional

from fastapi import WebSocket, WebSocketDisconnect

logger = logging.getLogger("dashboard.runner")

# Constants
PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPTS_DIR = PROJECT_ROOT / "scripts"
SIMULATION_SCRIPT = SCRIPTS_DIR / "run_simulation.py"

class RunnerManager:
    """
    Manages the execution of the simulation command and streams output
    to connected WebSocket clients.
    """
    def __init__(self):
        self.process: Optional[asyncio.subprocess.Process] = None
        self.active_websockets: list[WebSocket] = []
        self._log_history: list[str] = []
        self.max_history_lines = 1000
        self._custom_config: dict | None = None
        self._temp_config_path: str | None = None

    def get_config(self) -> dict:
        """Get the current configuration (default + overrides)."""
        from satellite_control.config.simulation_config import SimulationConfig
        
        # Start with default
        config = SimulationConfig.create_default()
        
        # Apply overrides if present
        if self._custom_config:
            config = SimulationConfig.create_with_overrides(
                self._custom_config, base_config=config
            )
            
        return config.to_dict()

    def update_config(self, overrides: dict):
        """Update the custom configuration overrides."""
        self._custom_config = overrides
        logger.info("Updated custom configuration overrides")

    async def connect(self, websocket: WebSocket):
        """Accept a new WebSocket connection and send history."""
        await websocket.accept()
        self.active_websockets.append(websocket)
        logger.info(f"WebSocket connected. Total clients: {len(self.active_websockets)}")
        
        # Send history upon connection
        if self._log_history:
             try:
                history_text = "".join(self._log_history)
                await websocket.send_text(history_text)
             except Exception as e:
                logger.error(f"Error sending history to websocket: {e}")

    def disconnect(self, websocket: WebSocket):
        """Remove a WebSocket connection."""
        if websocket in self.active_websockets:
            self.active_websockets.remove(websocket)
            logger.info(f"WebSocket disconnected. Remaining clients: {len(self.active_websockets)}")

    async def _broadcast(self, message: str):
        """Send a message to all connected clients."""
        # Add to history
        self._log_history.append(message)
        if len(self._log_history) > self.max_history_lines:
            self._log_history.pop(0)
            
        to_remove = []
        for connection in self.active_websockets:
            try:
                await connection.send_text(message)
            except Exception as e:
                logger.warning(f"Error sending to websocket, removing client: {e}")
                to_remove.append(connection)
        
        for conn in to_remove:
            self.disconnect(conn)

    async def start_simulation(self, mission_name: Optional[str] = None):
        """Start the simulation process."""
        if self.process and self.process.returncode is None:
            await self._broadcast("\n>>> Simulator is already running.\n")
            return

        self._log_history.clear()
        
        cmd_args = [str(SIMULATION_SCRIPT)]
        if mission_name:
            try:
                # Resolve mission path
                from satellite_control.mission.repository import (
                    resolve_mission_file,
                )
                mission_path = resolve_mission_file(mission_name, source_priority=("local",))
                cmd_args.extend(["--mission", str(mission_path)])
                await self._broadcast(f">>> Selected mission: {mission_name}\n")
            except Exception as e:
                await self._broadcast(f">>> Error resolving mission '{mission_name}': {e}\n")
                return

        # Inject custom config if present
        if self._custom_config:
            import json
            import tempfile
            
            try:
                # Create a temporary file to store the config overrides
                # We use a named temporary file that persists until we delete it
                # Note: On Windows, opening a temp file twice can be an issue, but we're passing path to subprocess
                with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as tmp:
                    json.dump(self._custom_config, tmp)
                    config_path = tmp.name
                
                cmd_args.extend(["--config", config_path])
                self._temp_config_path = config_path # Store to clean up later
                await self._broadcast(f">>> Using custom configuration overrides\n")
            except Exception as e:
                 logger.error(f"Failed to create config file: {e}")
                 await self._broadcast(f">>> Warning: Failed to apply custom config: {e}\n")

        await self._broadcast(f">>> Starting simulation: python {' '.join(cmd_args)}\n")

        try:
            # Setup environment
            # Inherit current env but ensure PYTHONPATH includes src/python
            env = os.environ.copy()
            python_path = env.get("PYTHONPATH", "")
            src_python = str(PROJECT_ROOT / "src" / "python")
            if src_python not in python_path:
                env["PYTHONPATH"] = f"{src_python}:{python_path}" if python_path else src_python

            # Use sys.executable to ensure we use the same virtualenv python
            cmd = [sys.executable] + cmd_args
            
            # Start process
            self.process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                limit=10*1024*1024, # 10MB limit per chunk
                env=env,
                cwd=str(PROJECT_ROOT) # Run from project root
            )
            
            await self._broadcast(f">>> Process started with PID: {self.process.pid}\n\n")

            # Background tasks for reading streams
            asyncio.create_task(self._monitor_stream(self.process.stdout, "STDOUT"))
            asyncio.create_task(self._monitor_stream(self.process.stderr, "STDERR"))
            
            # Background task to wait for completion
            asyncio.create_task(self._wait_for_completion())

        except Exception as e:
            logger.error(f"Failed to start simulation: {e}")
            await self._broadcast(f"\n>>> Error starting simulation: {e}\n")
            self.process = None
            if self._temp_config_path and os.path.exists(self._temp_config_path):
                try:
                    os.unlink(self._temp_config_path)
                except:
                    pass
            self._temp_config_path = None

    async def stop_simulation(self):
        """Stop the currently running simulation."""
        if self.process and self.process.returncode is None:
            await self._broadcast("\n>>> Stopping simulation...\n")
            try:
                self.process.terminate()
                # Wait briefly
                try:
                    await asyncio.wait_for(self.process.wait(), timeout=3.0)
                except asyncio.TimeoutError:
                    logger.warning("Process did not terminate gracefully, killing it.")
                    self.process.kill()
                    await self.process.wait()
                
                await self._broadcast(">>> Simulation stopped by user.\n")
            except Exception as e:
                logger.error(f"Error stopping process: {e}")
                await self._broadcast(f">>> Error stopping process: {e}\n")
        else:
            await self._broadcast("\n>>> No simulation is running to stop.\n")
            
        # Cleanup temp file if it exists
        if self._temp_config_path and os.path.exists(self._temp_config_path):
             try:
                 os.unlink(self._temp_config_path)
             except:
                 pass
        self._temp_config_path = None

    async def _monitor_stream(self, stream: asyncio.StreamReader, stream_name: str):
        """Read lines from a stream and broadcast them."""
        # Note: We need to be careful not to block
        while True:
            # readline() yields bytes ending in \n usually
            line = await stream.readline()
            if line:
                decoded = line.decode('utf-8', errors='replace')
                # We broadcast the raw line including newline chars usually, 
                # but let's ensure it handles buffering correctly on frontend.
                await self._broadcast(decoded)
            else:
                break

    async def _wait_for_completion(self):
        """Wait for the process to finish and broadcast the result."""
        if self.process:
            return_code = await self.process.wait()
            await self._broadcast(f"\n>>> Simulation finished with return code {return_code}.\n")

