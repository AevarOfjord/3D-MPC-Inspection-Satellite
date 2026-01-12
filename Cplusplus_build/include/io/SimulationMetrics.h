#pragma once

namespace sat_sim::io {
    struct SimulationMetrics {
        // MPC
        int mpc_count = 0;
        double total_mpc_time_ms = 0.0;
        double max_mpc_time_ms = 0.0;
        
        // Physics
        int physics_steps = 0;
        double total_physics_time_ms = 0.0;
        double max_physics_time_ms = 0.0;
        
        // Simulation
        double total_sim_time_s = 0.0; // Wall clock duration
        int total_cycles = 0;
    };
}
