import time
import cProfile
import pstats
import numpy as np
from src.satellite_control.control.mpc_controller import MPCController
from omegaconf import OmegaConf


def profile_mpc():
    """Profile the MPC controller's expensive methods."""

    # Create a dummy config
    cfg = OmegaConf.create(
        {
            "control": {
                "mpc": {
                    "prediction_horizon": 50,
                    "solver_time_limit": 0.05,
                    "weights": {
                        "position": 10.0,
                        "velocity": 1.0,
                        "angle": 10.0,
                        "angular_velocity": 1.0,
                        "thrust": 0.1,
                        "rw_torque": 0.1,
                        "switch": 0.0,
                    },
                    "constraints": {
                        "max_velocity": 1.0,
                        "max_angular_velocity": 1.0,
                        "position_bounds": 10.0,
                    },
                    "settings": {
                        "dt": 0.05,
                        "thruster_type": "PWM",
                        "enable_rw_yaw": True,
                        "enable_z_tilt": True,
                        "z_tilt_gain": 0.35,
                        "z_tilt_max_deg": 20.0,
                        "verbose_mpc": False,
                    },
                }
            },
            "vehicle": {
                "mass": 10.0,
                "inertia": [0.15, 0.15, 0.15],
                "center_of_mass": [0.0, 0.0, 0.0],
                "thrusters": [],  # Populate with dummy if needed
                "reaction_wheels": [],
            },
        }
    )

    # Add dummy thrusters
    thruster_list = []
    for i in range(6):
        thruster_list.append(
            {
                "position": [0.1, 0.0, 0.0],
                "direction": [1.0, 0.0, 0.0],
                "max_thrust": 1.0,
            }
        )
    cfg.vehicle.thrusters = thruster_list

    # Initialize Controller
    print("Initializing MPC...")
    controller = MPCController(cfg)

    # Dummy State and Target
    x_current = np.zeros(13)
    x_current[3] = 1.0  # Valid Quat
    x_target = np.zeros(13)
    x_target[0] = 1.0  # Offset position
    x_target[3] = 1.0

    # Force a dynamics update by changing quaternion significantly
    # Only the first call does initialization, subsequent calls in loop are what we care about
    # But to trigger _update_A_data, we need to change orientation.

    print("Running Profiler...")
    profiler = cProfile.Profile()
    profiler.enable()

    for i in range(50):
        # Rotating quaternion to force relinearization
        angle = i * 0.1
        x_current[3] = np.cos(angle / 2)
        x_current[6] = np.sin(angle / 2)  # Z correlation

        controller.get_control_action(x_current, x_target)

    profiler.disable()

    stats = pstats.Stats(profiler).sort_stats("cumtime")
    stats.print_stats(20)


if __name__ == "__main__":
    profile_mpc()
