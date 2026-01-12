#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace sat_sim::core {

    // --- Math Constants ---
    constexpr double PI = 3.14159265358979323846;

    // --- State Vector Components ---
    using Vector3 = Eigen::Vector3d;
    using Quaternion = Eigen::Vector4d; // [w, x, y, z] convention
    using Matrix3 = Eigen::Matrix3d;

    // --- System State ---
    struct StateVector {
        Vector3 position;       // Inertial [m]
        Quaternion attitude;    // Body to Inertial [w, x, y, z]
        Vector3 velocity;       // Inertial [m/s]
        Vector3 angular_velocity; // Body frame [rad/s]
        Vector3 rw_speeds;      // Reaction Wheel speeds [rad/s]

        static constexpr int DIM = 16; // 13 + 3
    };

    // --- Control Data ---
    struct ControlInput {
        Vector3 force_body;     // Total force in body frame [N] (Not typically used directly if we map thruster pulses, but useful for abstract control)
        Vector3 torque_body;    // Total torque in body frame [Nm]
        
        // For individual actuator access later:
        std::vector<double> thruster_activations; // [0, 1] usually or PWM duration
        Vector3 rw_torques; // Reaction wheel torques
    };

    // --- Configuration Structures ---
    
    struct ThrusterConfig {
        std::string id;
        Vector3 position;   // Relative to CoM [m]
        Vector3 direction;  // Thrust unit vector (Body Frame)
        double max_thrust;  // [N]
    };

    struct ReactionWheelConfig {
        double max_torque;      // [Nm]
        double max_speed_rad_s; // [rad/s]
        // Assuming orthogonal axes aligned with Body Frame for simplicity in V1
    };

    struct VehicleConfig {
        std::string name;
        double mass;            // [kg]
        Vector3 inertia_diag;   // Principal moments [kg*m^2]
        
        ReactionWheelConfig rw_config;
        std::vector<ThrusterConfig> thrusters;
    };

    struct MissionConfig {
        double dt;              // Control step [s]
        int horizon;            // MPC horizon Steps
    };

    struct Waypoint {
        Vector3 position;
        Quaternion attitude;
        double hold_time;       // [s] Time to wait after reaching
        double pos_tolerance;   // [m] Tolerance for "reached"
        double att_tolerance;   // [rad] Tolerance for "reached"
        double vel_tolerance;   // [m/s] Linear velocity tolerance
        double ang_vel_tolerance; // [rad/s] Angular velocity tolerance
    };

    struct MissionProfile {
        std::string name;
        std::vector<Waypoint> waypoints;
    };

} // namespace sat_sim::core
