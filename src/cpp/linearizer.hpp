#pragma once
#include <Eigen/Dense>
#include <vector>
#include "satellite_params.hpp"
#include "orbital_dynamics.hpp"

namespace satellite_control {

// Vector3d, VectorXd, MatrixXd already defined in satellite_params.hpp
// but they are inside the namespace, so they are available here.


class Linearizer {
public:
    Linearizer(const SatelliteParams& params, bool enable_gyro_jacobian = true);

    // Returns {A, B}
    std::pair<MatrixXd, MatrixXd> linearize(const VectorXd& x_current);
    const VectorXd& affine() const { return affine_; }
    void set_freeze_target(bool freeze) { freeze_target_ = freeze; }
    void set_enable_gyro_jacobian(bool enable) { enable_gyro_jacobian_ = enable; }

private:
    SatelliteParams params_;
    CWDynamics cw_dynamics_;
    TwoBodyDynamics two_body_dynamics_;
    bool use_two_body_ = true;
    bool freeze_target_ = false;
    bool enable_gyro_jacobian_ = true;

    // Precomputed thruster data in body frame
    std::vector<Vector3d> body_forces_;
    std::vector<Vector3d> body_torques_;
    VectorXd affine_;

    void precompute_thrusters();
    Eigen::Matrix3d compute_rotation_matrix(const Eigen::Vector4d& q);
};

} // namespace satellite_control
