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

   V2_MIGRATION
   SIMULATION
   VISUALIZATION
   MATHEMATICS


Quick Start
-----------

Installation
^^^^^^^^^^^^

.. code-block:: bash

   # Clone repository
   git clone https://github.com/AevarOfjord/Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel.git
   cd Satellite_3D_PWM-Continuous_Thrusters_ReactionWheel

   # Create virtual environment
   python3.11 -m venv .venv311
   source .venv311/bin/activate

   # Install dependencies + build C++ extension
   make install

Running a Simulation
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   .venv311/bin/python scripts/run_simulation.py run


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
