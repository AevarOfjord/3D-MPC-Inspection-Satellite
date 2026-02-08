"""
Core simulation subpackage.

Re-exports key classes for backward compatibility so that
``from satellite_control.core.simulation import X`` keeps working.
"""

from satellite_control.core.simulation.context import SimulationContext
from satellite_control.core.simulation.initialization import SimulationInitializer
from satellite_control.core.simulation.io import SimulationIO
from satellite_control.core.simulation.logger import SimulationLogger
from satellite_control.core.simulation.loop import SimulationLoop
from satellite_control.core.simulation.reference import (
    update_path_reference_state,
)
from satellite_control.core.simulation.runner import (
    SatelliteMPCLinearizedSimulation,
)
from satellite_control.core.simulation.step_logging import (
    log_simulation_step,
)

__all__ = [
    "SatelliteMPCLinearizedSimulation",
    "SimulationContext",
    "SimulationInitializer",
    "SimulationIO",
    "SimulationLogger",
    "SimulationLoop",
    "update_path_reference_state",
    "log_simulation_step",
]
