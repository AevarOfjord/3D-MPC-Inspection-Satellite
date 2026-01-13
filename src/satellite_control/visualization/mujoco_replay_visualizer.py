"""
MuJoCo Replay Visualizer Logic

Encapsulates the rendering, HUD, overlay, and playback logic for the MuJoCo replay tool.
Moving logic out of the script allows for better organization and potential reuse.
"""

import math
import sys
import time
from typing import Dict, List, Optional, Set, Tuple, Any

import mujoco
from mujoco import viewer as mujoco_viewer
import numpy as np
import pandas as pd

from src.satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz

# Constants for Visualization
DEFAULT_THRUSTER_COLORS: Dict[int, Tuple[float, float, float, float]] = {
    i: (0.0, 0.45, 1.0, 0.35) for i in range(1, 7)
}
TARGET_COLOR = (0.2, 1.0, 0.4, 0.9)
TRAJ_COLOR = (0.0, 0.8, 1.0, 0.8)

# 5x7 Font for HUD
FONT_5X7 = {
    "0": ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "10000", "11110", "00001", "00001", "11110"],
    "6": ["01110", "10000", "10000", "11110", "10001", "10001", "01110"],
    "7": ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "9": ["01110", "10001", "10001", "01111", "00001", "00001", "01110"],
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10001", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "V": ["10001", "10001", "10001", "10001", "01010", "01010", "00100"],
    ":": ["00000", "00100", "00100", "00000", "00100", "00100", "00000"],
    ",": ["00000", "00000", "00000", "00000", "00100", "00100", "01000"],
    ".": ["00000", "00000", "00000", "00000", "00000", "00100", "00100"],
    "-": ["00000", "00000", "00000", "01110", "00000", "00000", "00000"],
    " ": ["00000", "00000", "00000", "00000", "00000", "00000", "00000"],
}


class MuJoCoReplayVisualizer:
    def __init__(self, model_path: str, df: pd.DataFrame):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.df = df
        
        # Prepare Data
        self._prepare_data()
        
        # State
        self.state = {
            "play": False,
            "idx": 0,
            "speed": 1.0,
            "running": True,
            "request_seek": False,
            "active_ids": set(),
        }
        self.viewer_handle = None
        self.hud_cache = {"size": None, "image": None}
        
        # Initialize Geometry Colors/Visibility
        self._init_geoms()

    def _prepare_data(self):
        df = self.df
        
        # Time
        if "Control_Time" in df.columns:
            self.time_vals = self._to_numeric(df["Control_Time"])
        elif "Time" in df.columns:
            self.time_vals = self._to_numeric(df["Time"])
        else:
            self.time_vals = np.arange(len(df), dtype=float) * 0.05

        # Calculate avg frame dt
        diffs = np.diff(self.time_vals)
        diffs = diffs[diffs > 0]
        self.frame_dt = float(np.median(diffs)) if diffs.size > 0 else 0.05

        # State
        self.x = self._to_numeric(df.get("Current_X", []))
        self.y = self._to_numeric(df.get("Current_Y", []))
        self.z = self._to_numeric(df.get("Current_Z", []))
        
        self.roll = self._to_numeric(df.get("Current_Roll", []))
        self.pitch = self._to_numeric(df.get("Current_Pitch", []))
        self.yaw = self._to_numeric(df.get("Current_Yaw", []))

        self.vx = self._to_numeric(df.get("Current_VX", []))
        self.vy = self._to_numeric(df.get("Current_VY", []))
        self.vz = self._to_numeric(df.get("Current_VZ", []))
        
        # Target
        self.target_x = float(df.get("Target_X", pd.Series([0.0])).iloc[0])
        self.target_y = float(df.get("Target_Y", pd.Series([0.0])).iloc[0])
        self.target_z = float(df.get("Target_Z", pd.Series([0.0])).iloc[0])
        
        self.traj = np.column_stack([self.x, self.y, self.z])

    def _to_numeric(self, series: Any) -> np.ndarray:
        if len(series) != len(self.df):
             return np.zeros(len(self.df), dtype=float)
        values = pd.to_numeric(series, errors="coerce")
        values = values.fillna(0.0)
        return values.to_numpy(dtype=float)

    def _init_geoms(self):
        # Hide floor if it exists
        for geom_idx in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_idx)
            if name and ("floor" in name.lower() or "ground" in name.lower()):
                self.model.geom_rgba[geom_idx, 3] = 0.0
            elif self.model.geom_type[geom_idx] == mujoco.mjtGeom.mjGEOM_PLANE:
                self.model.geom_rgba[geom_idx, 3] = 0.0

    def launch(self):
        try:
            self.viewer_handle = mujoco_viewer.launch_passive(
                self.model,
                self.data,
                key_callback=self.on_key,
                show_left_ui=True,
                show_right_ui=True,
            )
        except RuntimeError as exc:
            print(f"Failed to launch viewer: {exc}")
            raise

        # Initial Camera View
        self.viewer_handle.cam.lookat[:] = [0.0, 0.0, 0.0]
        self.viewer_handle.cam.distance = 6.0
        self.viewer_handle.cam.elevation = 30.0
        self.viewer_handle.cam.azimuth = 45.0

        # Run Loop
        self.set_frame(0)
        last_tick = time.time()
        
        while self.viewer_handle.is_running() and self.state["running"]:
            self._update_hud(self.state["idx"], self.state.get("active_ids", set()))
            
            if self.state["play"]:
                now = time.time()
                if now - last_tick >= self.frame_dt / max(self.state["speed"], 0.1):
                    if self.state["idx"] < len(self.df) - 1:
                        self.set_frame(self.state["idx"] + 1)
                    else:
                        self.state["play"] = False
                    last_tick = now
            
            self.viewer_handle.sync()
            time.sleep(0.001)

        self.viewer_handle.close()

    def set_frame(self, idx: int):
        idx = int(np.clip(idx, 0, len(self.df) - 1))
        self.state["idx"] = idx

        # Set Physics State
        quat = euler_xyz_to_quat_wxyz((self.roll[idx], self.pitch[idx], self.yaw[idx]))
        
        # We need lock? Passive viewer instructions say we should modify data then sync.
        # Lock is usually for preventing race conditions if the viewer thread reads while we write.
        with self.viewer_handle.lock():
            self.data.qpos[0:3] = [self.x[idx], self.y[idx], self.z[idx]]
            self.data.qpos[3:7] = quat
            self.data.qvel[0:3] = [self.vx[idx], self.vy[idx], self.vz[idx]]
            mujoco.mj_forward(self.model, self.data)

        # Handle Thruster Visuals
        self._update_thruster_sites(idx)
        
        # Geometry Overlay (Target + Trajectory)
        self._update_overlay(idx)

    def _parse_command_vector(self, value: Any) -> np.ndarray:
        if value is None:
            return np.zeros(0)
        if isinstance(value, (list, tuple, np.ndarray)):
            return np.array(value, dtype=float)
        text = str(value).strip().strip("[]")
        if not text:
            return np.zeros(0)
        try:
            return np.array([float(x) for x in text.replace(",", " ").split()], dtype=float)
        except Exception:
            return np.zeros(0)

    def _update_thruster_sites(self, idx: int):
        cmd_vec = self._parse_command_vector(
            self.df.get("Command_Vector", pd.Series([None])).iloc[idx]
        )
        active_ids = set()
        if cmd_vec.size > 0:
            thruster_count = max(6, cmd_vec.size)
            active_ids = {i + 1 for i, val in enumerate(cmd_vec) if val > 0.5}
            
            for thruster_id in range(1, thruster_count + 1):
                site_name = f"thruster{thruster_id}"
                site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                if site_id == -1:
                    continue
                if thruster_id in active_ids:
                    self.model.site_rgba[site_id] = [1.0, 0.2, 0.2, 1.0]
                else:
                    self.model.site_rgba[site_id] = list(
                        DEFAULT_THRUSTER_COLORS.get(thruster_id, (0.5, 0.5, 0.5, 0.3))
                    )
        self.state["active_ids"] = active_ids

    def _update_overlay(self, idx: int):
        scn = getattr(self.viewer_handle, "user_scn", None)
        if not scn: return

        # Using with lock() context on viewer usually protects user_scn too
        with self.viewer_handle.lock():
            scn.ngeom = 0
            
            def add_geom():
                if scn.ngeom >= scn.maxgeom: return None
                g = scn.geoms[scn.ngeom]
                scn.ngeom += 1
                return g

            # Target Sphere
            geom = add_geom()
            if geom:
                mujoco.mjv_initGeom(
                    geom,
                    mujoco.mjtGeom.mjGEOM_SPHERE,
                    np.array([0.05, 0.0, 0.0]),
                    np.array([self.target_x, self.target_y, self.target_z]),
                    np.eye(3).reshape(9),
                    np.array(TARGET_COLOR, dtype=np.float32)
                )

            # Trajectory (Downsampled)
            if idx > 0:
                max_segments = 200
                step = max(1, int(np.ceil(idx / max_segments)))
                for j in range(0, idx, step):
                    p1 = self.traj[j]
                    p2 = self.traj[min(j + step, idx)]
                    
                    g = add_geom()
                    if not g: break
                    
                    mujoco.mjv_initGeom(
                        g,
                        mujoco.mjtGeom.mjGEOM_LINE,
                        np.zeros(3), np.zeros(3), np.eye(3).reshape(9),
                        np.array(TRAJ_COLOR, dtype=np.float32)
                    )
                    mujoco.mjv_connector(
                        g,
                        mujoco.mjtGeom.mjGEOM_LINE,
                        2.0,
                        p1.astype(float),
                        p2.astype(float)
                    )

    def _get_viewport(self):
        if self.viewer_handle is None:
            return None
        # Try direct access
        viewport = getattr(self.viewer_handle, "viewport", None)
        if viewport is not None:
            return viewport
            
        # Try accessing via internal simulation object if available
        sim_ref = getattr(self.viewer_handle, "_sim", None)
        if sim_ref:
            sim = None
            if callable(sim_ref):
                try: 
                    sim = sim_ref()
                except Exception: 
                    sim = None
            else:
                sim = sim_ref
            if sim:
                return getattr(sim, "viewport", None)
        return None

    def _draw_text(self, img: np.ndarray, text: str, x: int, y: int, scale: int):
        height, width, _ = img.shape
        color = np.array([235, 240, 248], dtype=np.uint8)
        line_height = (7 + 2) * scale
        char_width = (5 + 1) * scale
        cursor_y = y
        for line in text.splitlines():
            cursor_x = x
            for ch in line:
                pattern = FONT_5X7.get(ch.upper())
                if pattern is not None:
                    for row, bits in enumerate(pattern):
                        for col, bit in enumerate(bits):
                            if bit != "1": continue
                            px, py = cursor_x + col * scale, cursor_y + row * scale
                            if 0 <= px < width and 0 <= py < height:
                                img[py : py + scale, px : px + scale] = color
                cursor_x += char_width
            cursor_y += line_height

    def _update_hud(self, idx: int, active_ids: set):
        # Safely access viewport, as it might not be available on all Handle types
        viewport = self._get_viewport()
        if viewport is None:
            return

        if viewport.width <= 0 or viewport.height <= 0: return

        panel_w = max(220, int(viewport.width * 0.24))
        panel_h = max(130, int(viewport.height * 0.18))
        margin = max(10, int(viewport.width * 0.01))
        
        # Bottom-Left HUD Logic
        left = max(0, viewport.width - panel_w - margin)
        bottom = max(0, viewport.height - panel_h - margin)

        if self.hud_cache["size"] != (panel_w, panel_h):
            panel = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
            panel[:] = (12, 14, 18)
            # Borders
            border = max(1, int(panel_h * 0.04))
            panel[:border] = panel[-border:] = panel[:, :border] = panel[:, -border:] = (28, 32, 40)
            self.hud_cache["size"] = (panel_w, panel_h)
            self.hud_cache["image"] = panel
        
        panel = self.hud_cache["image"].copy()
        
        label_lines = [
            f"Time: {self.time_vals[idx]:.2f}s",
            f"Pos: {self.x[idx]:.2f}, {self.y[idx]:.2f}, {self.z[idx]:.2f}",
            f"Vel: {self.vx[idx]:.2f}, {self.vy[idx]:.2f}, {self.vz[idx]:.2f}",
            f"Dist: {np.linalg.norm(self.traj[idx] - np.array([self.target_x, self.target_y, self.target_z])):.2f} m",
            f"Active: {len(active_ids)}",
        ]
        
        pad = max(6, int(panel_w * 0.05))
        self._draw_text(panel, "\n".join(label_lines), pad, pad, scale=2)

        rect = mujoco.MjrRect(left, bottom, panel_w, panel_h)
        try:
            self.viewer_handle.set_images([(rect, panel)])
        except Exception:
            pass

    def on_key(self, key_code: int):
        # Arrow keys often come in as specific proprietary codes depending on GLFW backend
        # Standard GLFW/MuJoCo key codes:
        # RIGHT: 262, LEFT: 263, DOWN: 264, UP: 265
        # Standard ASCII: Space: 32
        
        # Basic Controls
        if key_code == 32: # Space
            self.state["play"] = not self.state["play"]
        
        # Navigation
        elif key_code == 262: # Right Arrow
            self.state["play"] = False
            self.set_frame(self.state["idx"] + 1)
        
        elif key_code == 263: # Left Arrow
            self.state["play"] = False
            self.set_frame(self.state["idx"] - 1)
            
        elif key_code == 265: # Up Arrow -> Speed Up
            self.state["speed"] = min(5.0, self.state["speed"] + 0.1)
            print(f"Speed: {self.state['speed']:.1f}x")
            
        elif key_code == 264: # Down Arrow -> Speed Down
            self.state["speed"] = max(0.1, self.state["speed"] - 0.1)
            print(f"Speed: {self.state['speed']:.1f}x")

        # Legacy Keys Support
        elif key_code in (ord('r'), ord('R')):
            self.state["play"] = False
            self.set_frame(0)

