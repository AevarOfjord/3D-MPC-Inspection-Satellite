#!/usr/bin/env python3
"""
MuJoCo replay viewer with timeline slider for saved simulation data.

Usage:
  python3 scripts/replay_mujoco.py --run-dir Data/Simulation/07-01-2026_14-25-34
  python3 scripts/replay_mujoco.py --csv Data/Simulation/07-01-2026_14-25-34/control_data.csv
  python3 scripts/replay_mujoco.py  # uses latest run, control_data.csv by default
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Dict, Tuple

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco
from mujoco import viewer as mujoco_viewer
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import pandas as pd

from src.satellite_control.utils.orientation_utils import euler_xyz_to_quat_wxyz


DEFAULT_THRUSTER_COLORS: Dict[int, Tuple[float, float, float, float]] = {
    1: (1.0, 0.0, 0.0, 0.3),
    2: (1.0, 0.2, 0.0, 0.3),
    3: (0.0, 1.0, 0.0, 0.3),
    4: (0.2, 1.0, 0.0, 0.3),
    5: (0.0, 0.0, 1.0, 0.3),
    6: (0.0, 0.2, 1.0, 0.3),
    7: (1.0, 1.0, 0.0, 0.3),
    8: (1.0, 0.8, 0.0, 0.3),
    9: (0.0, 1.0, 1.0, 0.3),
    10: (0.0, 1.0, 1.0, 0.3),
    11: (1.0, 0.0, 1.0, 0.3),
    12: (1.0, 0.0, 1.0, 0.3),
}


def _latest_run_dir(base: Path) -> Path:
    if not base.exists():
        raise FileNotFoundError(f"Base path not found: {base}")
    dirs = [p for p in base.iterdir() if p.is_dir()]
    if not dirs:
        raise FileNotFoundError(f"No run folders in {base}")
    return max(dirs, key=lambda p: p.stat().st_mtime)


def _resolve_csv(run_dir: Path, source: str) -> Path:
    if source == "physics":
        csv_path = run_dir / "physics_data.csv"
    else:
        csv_path = run_dir / "control_data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")
    return csv_path


def _to_numeric(series: pd.Series, default: float = 0.0) -> np.ndarray:
    values = pd.to_numeric(series, errors="coerce")
    values = values.fillna(default)
    return values.to_numpy(dtype=float)


def _parse_command_vector(value: object) -> np.ndarray:
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay MuJoCo simulation with slider.")
    parser.add_argument("--run-dir", type=str, help="Simulation run directory.")
    parser.add_argument("--csv", type=str, help="Path to control_data.csv or physics_data.csv.")
    parser.add_argument(
        "--source",
        choices=["control", "physics"],
        default="control",
        help="Which CSV to use when --run-dir is provided.",
    )
    parser.add_argument(
        "--model",
        type=str,
        default="models/satellite_3d.xml",
        help="MuJoCo model XML to render.",
    )
    args = parser.parse_args()

    if args.csv:
        csv_path = Path(args.csv)
        if not csv_path.exists():
            raise FileNotFoundError(f"CSV not found: {csv_path}")
        run_dir = csv_path.parent
    else:
        base = Path("Data") / "Simulation"
        if args.run_dir:
            run_dir = Path(args.run_dir)
        else:
            run_dir = _latest_run_dir(base)
        csv_path = _resolve_csv(run_dir, args.source)

    df = pd.read_csv(csv_path)
    if len(df) == 0:
        raise RuntimeError(f"Empty CSV: {csv_path}")

    time_col = None
    if "Control_Time" in df.columns:
        time_col = "Control_Time"
    elif "Time" in df.columns:
        time_col = "Time"

    if time_col:
        time_vals = _to_numeric(df[time_col], default=0.0)
    elif "Actual_Time_Interval" in df.columns:
        dt_guess = float(pd.to_numeric(df["Actual_Time_Interval"], errors="coerce").fillna(0.0).mean())
        if dt_guess <= 0:
            dt_guess = 0.05
        time_vals = np.arange(len(df)) * dt_guess
    else:
        time_vals = np.arange(len(df), dtype=float)

    diffs = np.diff(time_vals)
    diffs = diffs[diffs > 0]
    frame_dt = float(np.median(diffs)) if diffs.size > 0 else 0.05

    x = _to_numeric(df.get("Current_X", pd.Series([0.0] * len(df))))
    y = _to_numeric(df.get("Current_Y", pd.Series([0.0] * len(df))))
    z = _to_numeric(df.get("Current_Z", pd.Series([0.0] * len(df))))

    roll = _to_numeric(df.get("Current_Roll", pd.Series([0.0] * len(df))))
    pitch = _to_numeric(df.get("Current_Pitch", pd.Series([0.0] * len(df))))
    yaw = _to_numeric(df.get("Current_Yaw", pd.Series([0.0] * len(df))))

    vx = _to_numeric(df.get("Current_VX", pd.Series([0.0] * len(df))))
    vy = _to_numeric(df.get("Current_VY", pd.Series([0.0] * len(df))))
    vz = _to_numeric(df.get("Current_VZ", pd.Series([0.0] * len(df))))
    wx = _to_numeric(df.get("Current_WX", pd.Series([0.0] * len(df))))
    wy = _to_numeric(df.get("Current_WY", pd.Series([0.0] * len(df))))
    wz = _to_numeric(df.get("Current_WZ", pd.Series([0.0] * len(df))))

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)

    state = {"play": False, "idx": 0, "speed": 1.0, "running": True}
    slider_updating = {"active": False}
    viewer_handle = None

    def set_frame(idx: int, update_slider: bool = False) -> None:
        idx = int(np.clip(idx, 0, len(df) - 1))
        state["idx"] = idx

        quat = euler_xyz_to_quat_wxyz((roll[idx], pitch[idx], yaw[idx]))
        if viewer_handle is not None:
            with viewer_handle.lock():
                data.qpos[0:3] = [x[idx], y[idx], z[idx]]
                data.qpos[3:7] = quat
                data.qvel[0:3] = [vx[idx], vy[idx], vz[idx]]
                data.qvel[3:6] = [wx[idx], wy[idx], wz[idx]]
                mujoco.mj_forward(model, data)
        else:
            data.qpos[0:3] = [x[idx], y[idx], z[idx]]
            data.qpos[3:7] = quat
            data.qvel[0:3] = [vx[idx], vy[idx], vz[idx]]
            data.qvel[3:6] = [wx[idx], wy[idx], wz[idx]]
            mujoco.mj_forward(model, data)

        cmd_vec = _parse_command_vector(df.get("Command_Vector", pd.Series([None])).iloc[idx])
        if cmd_vec.size > 0:
            thruster_count = max(12, cmd_vec.size)
            active_ids = {i + 1 for i, val in enumerate(cmd_vec) if val > 0.5}
            for thruster_id in range(1, thruster_count + 1):
                site_name = f"thruster{thruster_id}"
                site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
                if site_id == -1:
                    continue
                if thruster_id in active_ids:
                    model.site_rgba[site_id] = [1.0, 0.2, 0.2, 1.0]
                else:
                    model.site_rgba[site_id] = list(
                        DEFAULT_THRUSTER_COLORS.get(thruster_id, (0.5, 0.5, 0.5, 0.3))
                    )

        if update_slider:
            slider_updating["active"] = True
            frame_slider.set_val(idx)
            slider_updating["active"] = False

        if viewer_handle is not None:
            viewer_handle.sync()

    def on_slider_change(val: float) -> None:
        if slider_updating["active"]:
            return
        state["play"] = False
        set_frame(int(val), update_slider=False)

    def on_play(_event) -> None:
        state["play"] = not state["play"]

    def on_step(_event) -> None:
        state["play"] = False
        set_frame(state["idx"] + 1, update_slider=True)

    def on_speed_change(val: float) -> None:
        state["speed"] = float(val)

    def on_close(_event) -> None:
        state["running"] = False

    def on_key(key: int) -> None:
        if key in (ord(" "), ord("p"), ord("P")):
            state["play"] = not state["play"]
        elif key in (ord("."), ord("s"), ord("S")):
            state["play"] = False
            set_frame(state["idx"] + 1, update_slider=True)
        elif key in (ord(","), ord("a"), ord("A")):
            state["play"] = False
            set_frame(state["idx"] - 1, update_slider=True)

    try:
        viewer_handle = mujoco_viewer.launch_passive(model, data, key_callback=on_key)
    except RuntimeError as exc:
        msg = str(exc).lower()
        if "mjpython" in msg:
            print(
                "MuJoCo viewer on macOS requires mjpython.\n"
                "Run: venv/bin/mjpython scripts/replay_mujoco.py --run-dir <path>\n"
                "If the viewer does not open, try: MUJOCO_GL=glfw venv/bin/mjpython "
                "scripts/replay_mujoco.py --run-dir <path>"
            )
            return 1
        raise
    viewer_handle.cam.lookat[:] = [0.0, 0.0, 0.0]
    viewer_handle.cam.distance = 6.0
    viewer_handle.cam.elevation = 30.0
    viewer_handle.cam.azimuth = 45.0

    plt.ion()
    fig = plt.figure(figsize=(8, 3))
    fig.canvas.manager.set_window_title("MuJoCo Replay Controls")
    fig.subplots_adjust(left=0.08, right=0.98, bottom=0.25, top=0.95)

    ax_slider = fig.add_axes([0.08, 0.15, 0.84, 0.05])
    ax_speed = fig.add_axes([0.08, 0.07, 0.55, 0.03])
    ax_play = fig.add_axes([0.68, 0.03, 0.12, 0.08])
    ax_step = fig.add_axes([0.82, 0.03, 0.12, 0.08])

    frame_slider = Slider(
        ax_slider,
        "Frame",
        0,
        len(df) - 1,
        valinit=0,
        valstep=1,
        valfmt="%0.0f",
    )
    speed_slider = Slider(ax_speed, "Speed", 0.1, 4.0, valinit=1.0, valstep=0.1)
    play_btn = Button(ax_play, "Play/Pause")
    step_btn = Button(ax_step, "Step")

    frame_slider.on_changed(on_slider_change)
    speed_slider.on_changed(on_speed_change)
    play_btn.on_clicked(on_play)
    step_btn.on_clicked(on_step)
    fig.canvas.mpl_connect("close_event", on_close)

    set_frame(0, update_slider=True)

    last_tick = time.time()
    while viewer_handle.is_running() and state["running"]:
        if state["play"]:
            now = time.time()
            if now - last_tick >= frame_dt / max(state["speed"], 0.1):
                if state["idx"] < len(df) - 1:
                    set_frame(state["idx"] + 1, update_slider=True)
                else:
                    state["play"] = False
                last_tick = now
        plt.pause(0.001)
        viewer_handle.sync()

    viewer_handle.close()
    plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
