Satellite Thruster Control System
=================================

Model Predictive Control system for satellite thruster control with a custom C++ physics engine.

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   ARCHITECTURE
   DEVELOPMENT_GUIDE
   TESTING
   TROUBLESHOOTING

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/modules

.. toctree::
   :maxdepth: 1
   :caption: Additional Documentation

   SIMULATION
   VISUALIZATION
   MATHEMATICS


Quick Start
-----------

Installation
^^^^^^^^^^^^

.. code-block:: bash

   # Clone repository
   git clone https://github.com/AevarOfjord/SatelliteProject
   cd SatelliteProject

   # Create virtual environment
   python3.11 -m venv .venv311
   source .venv311/bin/activate

   # Install dependencies
   pip install -e ".[dev]"

Running a Simulation
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   python run_simulation.py


Key Components
--------------

**MPC Controllers**
   - ``MPCController``: Path-following MPC with OSQP solver

**Simulation**
   - ``SatelliteMPCLinearizedSimulation``: Main simulation class
   - ``ThrusterManager``: Thruster valve physics


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
