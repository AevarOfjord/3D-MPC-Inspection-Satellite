#pragma once
#include <Eigen/Dense>
#include <vector>

namespace satellite_dt {

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
    Vector3d com_offset;
};

} // namespace satellite_dt
