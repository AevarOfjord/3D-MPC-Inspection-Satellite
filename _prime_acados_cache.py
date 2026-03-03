"""
Computes and writes the OCP hash files for the acados controllers
so the existing compiled .dylib files are reused immediately.

Run from repo root:
    .venv311/bin/python _prime_acados_cache.py
"""

import hashlib
import os
import sys

sys.path.insert(0, ".")

from controller.configs.simulation_config import SimulationConfig
from controller.shared.python.control_common.profile_params import (
    resolve_effective_mpc_profile_contract,
)

for profile in ("acados_rti", "acados_sqp"):
    print(f"\n--- {profile} ---")
    cfg = SimulationConfig.create_default()
    # app_config uses cfg.app_config which is a proper AppConfig
    app_cfg = cfg.app_config

    # Replicate AcadosBaseController._extract_params exactly (uses cfg.mpc and cfg.physics)
    physics = app_cfg.physics
    mpc = app_cfg.mpc  # MPCParams

    sorted_ids = sorted(physics.thruster_positions.keys())
    num_thrusters = len(sorted_ids)
    if num_thrusters == 6:
        thruster_pairs = [(0, 1), (2, 3), (4, 5)]
    else:
        thruster_pairs = [(i, i + 1) for i in range(0, num_thrusters - 1, 2)]
    num_rw_axes = len(physics.reaction_wheels) if physics.reaction_wheels else 0

    N = int(mpc.prediction_horizon)
    dt = float(mpc.dt)
    Q_contour = float(mpc.Q_contour)
    _q_lag = float(mpc.Q_lag)
    if _q_lag <= 0.0:
        _q_lag = float(getattr(mpc, "Q_lag_default", 0.0) or 0.0)
    Q_lag = _q_lag

    # Profile-specific params
    eff = resolve_effective_mpc_profile_contract(app_cfg, profile)
    profile_specific = dict(eff.profile_specific)

    nlp_solver_type = "SQP_RTI" if profile == "acados_rti" else "SQP"
    acados_max_iter = int(profile_specific.get("acados_max_iter", 50))
    acados_tol_stat = float(profile_specific.get("acados_tol_stat", 1e-2))
    acados_tol_eq = float(profile_specific.get("acados_tol_eq", 1e-2))
    acados_tol_ineq = float(profile_specific.get("acados_tol_ineq", 1e-2))

    # Cost weights — mirror _extract_params exactly (all from cfg.mpc)
    Q_progress = float(mpc.Q_progress)
    Q_velocity_align = float(mpc.Q_velocity_align)
    Q_terminal_pos = float(mpc.Q_terminal_pos)
    Q_angvel = float(mpc.q_angular_velocity)
    Q_attitude = float(mpc.Q_attitude)
    R_thrust = float(mpc.r_thrust)
    R_rw_torque = float(mpc.r_rw_torque)
    thrust_pair_weight = float(mpc.thrust_pair_weight)
    path_speed = float(mpc.path_speed)
    path_speed_min = float(mpc.path_speed_min)
    path_speed_max = float(mpc.path_speed_max)

    nlp_solver_type = "SQP_RTI" if profile == "acados_rti" else "SQP"
    acados_max_iter = int(
        profile_specific.get("acados_max_iter", 1 if profile == "acados_rti" else 50)
    )
    acados_tol_stat = float(profile_specific.get("acados_tol_stat", 1e-2))
    acados_tol_eq = float(profile_specific.get("acados_tol_eq", 1e-2))
    acados_tol_ineq = float(profile_specific.get("acados_tol_ineq", 1e-2))

    hash_params = {
        "N": N,
        "dt": dt,
        "num_thrusters": num_thrusters,
        "num_rw_axes": num_rw_axes,
        "nlp_solver_type": nlp_solver_type,
        "max_iter": acados_max_iter,
        "tol_stat": acados_tol_stat,
        "tol_eq": acados_tol_eq,
        "tol_ineq": acados_tol_ineq,
        "Q_contour": Q_contour,
        "Q_lag": Q_lag,
        "Q_progress": Q_progress,
        "Q_velocity_align": Q_velocity_align,
        "Q_attitude": Q_attitude,
        "Q_angvel": Q_angvel,
        "Q_terminal_pos": Q_terminal_pos,
        "R_rw_torque": R_rw_torque,
        "R_thrust": R_thrust,
        "thrust_pair_weight": thrust_pair_weight,
        "path_speed": path_speed,
        "path_speed_min": path_speed_min,
        "path_speed_max": path_speed_max,
        "thruster_pairs": str(sorted(thruster_pairs)),
    }
    for k, v in sorted(hash_params.items()):
        print(f"    {k}: {v}")

    current_hash = hashlib.sha256(
        str(sorted(hash_params.items())).encode()
    ).hexdigest()[:16]
    print(f"  OCP hash: {current_hash}")

    lib_ext = ".dylib" if sys.platform == "darwin" else ".so"
    lib_path = os.path.join(
        "c_generated_code", f"libacados_ocp_solver_satellite_{profile}{lib_ext}"
    )
    build_dir = os.path.join("codegen_cache", profile)
    os.makedirs(build_dir, exist_ok=True)
    json_file = os.path.join(build_dir, "acados_ocp.json")
    hash_file = os.path.join(build_dir, "ocp_hash.txt")

    lib_ok = os.path.isfile(lib_path)
    json_ok = os.path.isfile(json_file)
    print(f"  Library: {'OK' if lib_ok else 'MISSING'} ({lib_path})")
    print(f"  JSON:    {'OK' if json_ok else 'MISSING'}")

    if lib_ok and json_ok:
        with open(hash_file, "w") as f:
            f.write(current_hash)
        print(f"  => Wrote {hash_file}")
        print("     Next run will reuse existing library (skip codegen+compile).")
    else:
        print(
            "  => Cannot prime cache: next run will compile (~1-2 min), then be cached."
        )

print("\nDone.")
