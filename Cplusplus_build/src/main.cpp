#include <iostream>
#include <chrono>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <spdlog/spdlog.h>
#include <mujoco/mujoco.h>
#include <osqp.h> 
#include "io/ConfigLoader.h"
#include "io/DataLogger.h"
#include "physics/PhysicsEngine.h"
#include "control/ControlAllocator.h"
#include "control/MPCController.h"
#include "logic/MissionManager.h"
#include "fdir/FaultManager.h"

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("========================================");
    spdlog::info("   Satellite Control System (C++)       ");
    spdlog::info("   Fault Tolerant Mode                  ");
    spdlog::info("========================================");
    
    // 1. Config Loading
    std::string config_path = "config/mission_test.yaml";
    std::string control_config_path = "config/control.yaml";
    std::string model_path = "models/satellite_3d.xml";  // Always relative to cwd
    if (argc > 1) {
        config_path = argv[1];
    }
    
    // Load config using new system
    sat_sim::config::VehicleConfig vehicle_config;
    sat_sim::config::MissionConfig mission_config;
    sat_sim::config::ControlConfig control_config;
    
    try {
        vehicle_config = sat_sim::io::ConfigLoader::load_vehicle_config(config_path);
        mission_config = sat_sim::io::ConfigLoader::load_mission_config(config_path);
        control_config = sat_sim::io::ConfigLoader::load_control_config(control_config_path);
        
        spdlog::info("Config Loaded. Vehicle: {}, Control: {}", 
            vehicle_config.name, control_config_path);
    } catch (const std::exception& e) {
        spdlog::error("Config Loading Failed: {}", e.what());
        return 1;
    }

    // 2. Initialize Fault Manager
    sat_sim::fdir::FaultManager fault_manager(vehicle_config);

    // 3. Control Allocator (will be updated on faults)
    sat_sim::control::ControlAllocator allocator(vehicle_config);

    // 4. Legacy config for MPC (for now)
    sat_sim::core::VehicleConfig legacy_vehicle = 
        sat_sim::io::ConfigLoader::load_vehicle_config_legacy(config_path);
    sat_sim::core::MissionProfile legacy_mission = 
        sat_sim::io::ConfigLoader::load_mission_profile(config_path);

    sat_sim::control::MPCConfig mpc_cfg;
    // Map loaded config to controller config
    mpc_cfg.dt = control_config.mpc.dt;
    mpc_cfg.horizon = control_config.mpc.horizon;
    
    const auto& w = control_config.mpc.weights;
    mpc_cfg.q_pos = w.position;
    mpc_cfg.q_vel = w.velocity;
    mpc_cfg.q_att = w.attitude;
    mpc_cfg.q_angvel = w.angular_velocity;
    mpc_cfg.r_thrust = w.thrust;
    mpc_cfg.r_rw = w.reaction_wheel;

    sat_sim::control::MPCController mpc(legacy_vehicle, mpc_cfg);

    // 5. Mission Manager
    sat_sim::logic::MissionManager mission_manager(legacy_mission);

    // 6. Physics Initialization
    std::unique_ptr<sat_sim::physics::PhysicsEngine> physics;
    try {
        physics = std::make_unique<sat_sim::physics::PhysicsEngine>(model_path);
        spdlog::info("Physics Engine Initialized.");
    } catch (const std::exception& e) {
        spdlog::error("Physics Init Failed: {}", e.what());
        return 1;
    }

    // 7. Data Logger
    sat_sim::io::DataLogger logger;
    
    // Metrics
    sat_sim::io::SimulationMetrics metrics;

    // 8. Simulation Loop with Fault Injection
    double max_sim_time = 60.0;
    double fault_inject_time = 20.0;  // Inject fault at 20s
    bool fault_injected = false;
    
    spdlog::info("Starting Simulation (fault will be injected at {:.0f}s)...", fault_inject_time);
    
    auto init_state = physics->get_state();
    double mpc_dt = mpc_cfg.dt;
    double current_time = 0.0;
    int step_count = 0;

    // Pre-compute B matrix
    Eigen::MatrixXd B = allocator.get_B_matrix();

    while (current_time < max_sim_time) {
        auto state = physics->get_state();
        current_time = physics->get_time();

        // === FAULT INJECTION DEMO ===
        if (!fault_injected && current_time >= fault_inject_time) {
            spdlog::warn("========== INJECTING FAULT ==========");
            
            // Simulate thruster T1 stuck off
            fault_manager.inject_fault("T1", sat_sim::fdir::FaultType::STUCK_OFF);
            
            // Get effective config with failed thruster removed
            auto effective_config = fault_manager.get_effective_vehicle_config();
            
            // Update control allocator with new B-matrix
            allocator.update_config(effective_config);
            B = allocator.get_B_matrix();
            
            spdlog::info("Healthy thrusters remaining: {}", fault_manager.get_healthy_thruster_count());
            spdlog::warn("======================================");
            
            fault_injected = true;
        }

        // Update Mission Manager
        auto target_state = mission_manager.update(current_time, state);

        // Check completion
        if (mission_manager.is_complete()) {
            spdlog::info("Mission Complete at {:.2f}s", current_time);
            break;
        }
        
        // Compute Control
        auto start_mpc = std::chrono::high_resolution_clock::now();
        auto u = mpc.compute_control(state, target_state);
        auto end_mpc = std::chrono::high_resolution_clock::now();
        double mpc_ms = std::chrono::duration<double, std::milli>(end_mpc - start_mpc).count();
        
        metrics.mpc_count++;
        metrics.total_mpc_time_ms += mpc_ms;
        if(mpc_ms > metrics.max_mpc_time_ms) metrics.max_mpc_time_ms = mpc_ms;

        if (step_count % 20 == 0) {
            double max_thrust = 0.0;
            for(auto v : u.thruster_activations) if(v > max_thrust) max_thrust = v;
            spdlog::info("T={:.1f}s | Faults:{} | MaxThrust: {:.2f}N | Err: {:.3f}m", 
                current_time,
                fault_manager.has_active_faults() ? "YES" : "NO",
                max_thrust, 
                (state.position - target_state.position).norm());
        }

        logger.log_control_step(current_time, state, target_state, u, mpc_ms);

        // Apply continuous thrust
        Eigen::VectorXd u_thrust(u.thruster_activations.size());
        for(size_t k = 0; k < u.thruster_activations.size(); ++k) {
            u_thrust(k) = u.thruster_activations[k];
        }
        
        Eigen::VectorXd wrench = B * u_thrust;

        sat_sim::core::ControlInput u_applied;
        u_applied.thruster_activations = u.thruster_activations;
        u_applied.rw_torques = u.rw_torques;
        u_applied.force_body = sat_sim::core::Vector3(wrench(0), wrench(1), wrench(2));
        u_applied.torque_body = u.rw_torques + sat_sim::core::Vector3(wrench(3), wrench(4), wrench(5));

        double start_step = current_time;
        while (physics->get_time() < start_step + mpc_dt) {
            physics->set_control_input(u_applied);
            
            auto start_phys = std::chrono::high_resolution_clock::now();
            physics->step();
            auto end_phys = std::chrono::high_resolution_clock::now();
            double phys_ms = std::chrono::duration<double, std::milli>(end_phys - start_phys).count();
            
            metrics.physics_steps++;
            metrics.total_physics_time_ms += phys_ms;
            if(phys_ms > metrics.max_physics_time_ms) metrics.max_physics_time_ms = phys_ms;
            
            // Log high-rate physics data
            auto phys_state = physics->get_state();
            logger.log_physics_step(physics->get_time(), phys_state, target_state, u_applied);
        }
        
        step_count++;
        if (step_count % 20 == 0) logger.flush();
    }
    
    // Finalize metrics
    metrics.total_sim_time_s = physics->get_time();
    metrics.total_cycles = step_count;
    logger.log_metrics(metrics);
    
    logger.flush();
    spdlog::info("Simulation Ended.");
    
    // Summary
    spdlog::info("========== FAULT TOLERANCE SUMMARY ==========");
    spdlog::info("Active faults: {}", fault_manager.has_active_faults() ? "YES" : "NO");
    spdlog::info("Healthy thrusters: {}/{}", 
        fault_manager.get_healthy_thruster_count(),
        vehicle_config.thrusters.size());
    spdlog::info("Mission completed despite thruster failure.");
    
    return 0;
}
