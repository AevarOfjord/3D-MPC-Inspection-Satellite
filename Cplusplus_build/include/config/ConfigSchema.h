#pragma once
/**
 * @file ConfigSchema.h
 * @brief Structured configuration schemas for the satellite control system.
 * 
 * This header defines all configuration structures used for dependency injection.
 * Mirrors the Hydra/OmegaConf pattern from Python - YAML configs are loaded into
 * these strongly-typed structs at runtime.
 */

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>

namespace sat_sim::config {

    using Vector3 = Eigen::Vector3d;
    using Quaternion = Eigen::Vector4d;

    // ============================================================================
    // VEHICLE CONFIGURATION
    // ============================================================================

    /**
     * @brief Individual thruster definition.
     * 
     * Each thruster has a position relative to CoM, a thrust direction,
     * and physical limits.
     */
    struct ThrusterConfig {
        std::string id;                     // Unique identifier (e.g., "T1", "RCS_PX")
        Vector3 position;                   // Position relative to CoM [m]
        Vector3 direction;                  // Thrust direction unit vector (body frame)
        double max_thrust = 1.0;            // Maximum thrust [N]
        double min_thrust = 0.0;            // Minimum thrust (usually 0) [N]
        std::string group = "main";         // Grouping (e.g., "main", "rcs", "attitude")
        
        // Defaults
        ThrusterConfig() : position(Vector3::Zero()), direction(Vector3::UnitX()) {}
    };

    /**
     * @brief Reaction wheel configuration.
     * 
     * Assumes 3-axis orthogonal wheel assembly aligned with body frame.
     */
    struct ReactionWheelConfig {
        double max_torque = 0.1;            // Maximum torque per axis [Nm]
        double max_speed_rad_s = 600.0;     // Maximum wheel speed [rad/s]
        double inertia = 0.001;             // Wheel inertia [kg*m^2]
        bool enabled = true;                // Whether RW is available
    };

    /**
     * @brief Complete vehicle physical definition.
     */
    struct VehicleConfig {
        std::string name = "Satellite";
        double mass = 10.0;                 // [kg]
        Vector3 inertia_diag;               // Principal moments [Ixx, Iyy, Izz] [kg*m^2]
        Vector3 center_of_mass;             // CoM offset from body origin [m]
        
        std::vector<ThrusterConfig> thrusters;
        ReactionWheelConfig reaction_wheels;
        
        // Optional: mesh file for visualization/inspection
        std::string mesh_file = "";
        
        VehicleConfig() : inertia_diag(Vector3::Ones()), center_of_mass(Vector3::Zero()) {}
    };

    // ============================================================================
    // CONTROL CONFIGURATION
    // ============================================================================

    /**
     * @brief MPC weight parameters.
     */
    struct MPCWeights {
        double position = 100.0;            // Position tracking weight
        double velocity = 1.0;              // Velocity tracking weight
        double attitude = 10.0;             // Attitude tracking weight
        double angular_velocity = 1.0;      // Angular velocity tracking weight
        double thrust = 0.1;                // Thruster usage penalty
        double reaction_wheel = 1.0;        // Reaction wheel usage penalty
    };

    /**
     * @brief MPC solver configuration.
     */
    struct MPCConfig {
        int horizon = 20;                   // Prediction horizon steps
        double dt = 0.05;                   // Control timestep [s]
        MPCWeights weights;
        
        // Solver tuning
        int max_iterations = 1000;
        double tolerance = 1e-4;
        bool verbose = false;
    };

    /**
     * @brief Top-level control configuration.
     */
    struct ControlConfig {
        MPCConfig mpc;
        // Future: PIDConfig pid;
    };

    // ============================================================================
    // MISSION CONFIGURATION
    // ============================================================================

    /**
     * @brief Single waypoint definition.
     */
    struct Waypoint {
        Vector3 position;                   // Target position [m]
        Quaternion attitude;                // Target attitude [w, x, y, z]
        double hold_time = 0.0;             // Time to hold at waypoint [s]
        double pos_tolerance = 0.05;        // Position tolerance [m]
        double att_tolerance = 0.1;         // Attitude tolerance [rad]
        double vel_tolerance = 0.05;        // Linear velocity tolerance [m/s]
        double ang_vel_tolerance = 0.05;    // Angular velocity tolerance [rad/s]
        
        Waypoint() : position(Vector3::Zero()), attitude(Quaternion(1, 0, 0, 0)) {}
    };

    /**
     * @brief Mission profile definition.
     */
    struct MissionConfig {
        std::string name = "Mission";
        std::vector<Waypoint> waypoints;
        double max_duration = 300.0;        // Maximum mission time [s]
        bool loop = false;                  // Whether to repeat waypoints
    };

    // ============================================================================
    // ENVIRONMENT CONFIGURATION
    // ============================================================================

    /**
     * @brief Simulation environment settings.
     */
    struct EnvironmentConfig {
        std::string name = "default";
        double gravity = 0.0;               // Gravity magnitude [m/s^2] (0 for space)
        
        // Orbital parameters (for HCW dynamics)
        double orbital_altitude_km = 400.0; // Altitude for mean motion calc
        double mean_motion = 0.0;           // n = sqrt(mu/a^3) [rad/s], computed if 0
        
        // Perturbations
        bool enable_drag = false;
        bool enable_j2 = false;
        bool enable_gravity_gradient = false;
    };

    // ============================================================================
    // SIMULATION CONFIGURATION
    // ============================================================================

    /**
     * @brief Physics simulation settings.
     */
    struct SimulationConfig {
        std::string model_file = "";        // MuJoCo XML path
        double physics_dt = 0.001;          // Physics timestep [s]
        double control_dt = 0.05;           // Control update rate [s]
        int substeps = 50;                  // Physics substeps per control step
        bool headless = true;               // No visualization
    };

    // ============================================================================
    // ROOT CONFIGURATION
    // ============================================================================

    /**
     * @brief Top-level configuration aggregating all subsystems.
     * 
     * This is the "main.yaml" equivalent - the root of the config tree.
     */
    struct RootConfig {
        VehicleConfig vehicle;
        ControlConfig control;
        MissionConfig mission;
        EnvironmentConfig environment;
        SimulationConfig simulation;
        
        // Metadata
        std::string config_version = "1.0";
    };

} // namespace sat_sim::config
