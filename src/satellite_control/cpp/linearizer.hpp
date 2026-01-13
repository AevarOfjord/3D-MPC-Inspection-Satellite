#pragma once
#include <Eigen/Dense>
#include <vector>
#include <tuple>
#include "satellite_params.hpp"

namespace satellite_dt {

// Vector3d, VectorXd, MatrixXd already defined in satellite_params.hpp
// but they are inside the namespace, so they are available here.


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
