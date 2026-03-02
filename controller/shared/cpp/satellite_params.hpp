#pragma once
#include <Eigen/Dense>
#include <vector>

namespace satellite_control {

using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

struct SatelliteParams {
    double dt;
    double mass;
    Vector3d inertia;
    int num_thrusters;
    int num_rw;
    std::vector<Vector3d> thruster_positions;
    std::vector<Vector3d> thruster_directions;
    std::vector<double> thruster_forces;
    std::vector<double> rw_torque_limits;
    std::vector<double> rw_inertia;
    std::vector<double> rw_speed_limits;
    std::vector<Vector3d> rw_axes;
    Vector3d com_offset;

    // Orbital dynamics parameters (for MPC linearization)
    double orbital_mean_motion = 0.0; // rad/s (CW)
    double orbital_mu = 3.986004418e14; // m^3/s^2
    double orbital_radius = 6.778e6; // m
    bool use_two_body = true;
};

} // namespace satellite_control
