
#pragma once
#include <Eigen/Dense>
#include <vector>
#include <tuple>

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

class Linearizer {
public:
    Linearizer(const SatelliteParams& params);

    // Returns {A, B}
    std::pair<MatrixXd, MatrixXd> linearize(const VectorXd& x_current);

private:
    SatelliteParams params_;
    
    // Precomputed thruster data in body frame
    std::vector<Vector3d> body_forces_;
    std::vector<Vector3d> body_torques_;

    void precompute_thrusters();
    Eigen::Matrix3d compute_rotation_matrix(const Eigen::Vector4d& q);
};

} // namespace satellite_dt
