"""Timing constants for simulation and control loops."""


# DEFAULT TIMING PARAMETERS
# ============================================================================

# Simulation and control intervals
# SIMULATION_DT is the SINGLE SOURCE OF TRUTH for physics timestep
SIMULATION_DT = 0.001  # 1ms / 1000Hz physics
CONTROL_DT = 0.050  # 50 ms (MPC update rate)
MAX_SIMULATION_TIME = 0.0  # seconds (0 disables time limit)

# Stabilization timers
USE_FINAL_STABILIZATION_IN_SIMULATION = False

DEFAULT_PATH_SPEED = 0.1  # m/s - default speed for path following missions
