import sys

sys.path.insert(0, ".")
from controller.configs.simulation_config import SimulationConfig

cfg = SimulationConfig.create_default()
mpc = cfg.app_config.mpc
import math

N = mpc.prediction_horizon
Q_att = mpc.Q_attitude
Q_omg = mpc.q_angular_velocity
print(f"N            = {N}")
print(f"Q_attitude   = {Q_att}")
print(f"Q_angvel     = {Q_omg}")
print(f"W_term_att   = {Q_att:.1f}  (sqrt={math.sqrt(Q_att):.2f})")
print(f"W_term_omg   = {Q_omg * N:.1f}  (sqrt={math.sqrt(Q_omg * N):.2f})")
print(f"Ratio att/omg= {Q_att / (Q_omg * N):.0f}:1")
print()
print(f"Q_smooth     = {mpc.Q_smooth}")
print(f"R_rw_torque  = {mpc.r_rw_torque}")
print(f"Q_contour    = {mpc.Q_contour}")
print(f"Q_terminal_pos={mpc.Q_terminal_pos}")
stage_att = Q_att * 0.02
print(f"\nStage att weight (x0.02 factor): {stage_att}")
print(f"Sum over N stages: {stage_att * N}")
print(f"Terminal att weight:             {Q_att}")
print(f"Ratio terminal_att / stage_sum:  {Q_att / (stage_att * N):.2f}")
