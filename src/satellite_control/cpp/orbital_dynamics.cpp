#include "orbital_dynamics.hpp"

namespace satellite_control {

CWDynamics::CWDynamics(double mean_motion) : n_(mean_motion), n_sq_(mean_motion * mean_motion) {}

Eigen::Vector3d CWDynamics::compute_acceleration(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) const {
    double x = position.x();
    // double y = position.y(); // y position doesn't affect acceleration directly in linearized CW
    double z = position.z();
    double vx = velocity.x();
    double vy = velocity.y();
    // double vz = velocity.z();

    // CW equations
    // ax = 3n^2x + 2nvy
    // ay = -2nvx
    // az = -n^2z
    double ax = 3.0 * n_sq_ * x + 2.0 * n_ * vy;
    double ay = -2.0 * n_ * vx;
    double az = -n_sq_ * z;

    return Eigen::Vector3d(ax, ay, az);
}

std::pair<Eigen::Matrix<double, 6, 6>, Eigen::Matrix<double, 6, 3>> CWDynamics::get_state_matrices(double dt) const {
    /*
     * Continuous-time A matrix (6x6)
     * State: [x, y, z, vx, vy, vz]
     * 
     * x_dot = vx
     * y_dot = vy
     * z_dot = vz
     * vx_dot = 3n^2x + 2nvy
     * vy_dot = -2nvx
     * vz_dot = -n^2z
     */
    Eigen::Matrix<double, 6, 6> A_cont = Eigen::Matrix<double, 6, 6>::Zero();
    
    // Identity blocks for position derivatives
    A_cont(0, 3) = 1.0;
    A_cont(1, 4) = 1.0;
    A_cont(2, 5) = 1.0;

    // Dynamics blocks
    A_cont(3, 0) = 3.0 * n_sq_;
    A_cont(3, 4) = 2.0 * n_;
    A_cont(4, 3) = -2.0 * n_;
    A_cont(5, 2) = -n_sq_;

    // B matrix (6x3) - input is acceleration [ax, ay, az]
    Eigen::Matrix<double, 6, 3> B_cont = Eigen::Matrix<double, 6, 3>::Zero();
    B_cont(3, 0) = 1.0;
    B_cont(4, 1) = 1.0;
    B_cont(5, 2) = 1.0;

    // Discretize using forward Euler: A_d = I + A*dt
    Eigen::Matrix<double, 6, 6> A_disc = Eigen::Matrix<double, 6, 6>::Identity() + A_cont * dt;
    Eigen::Matrix<double, 6, 3> B_disc = B_cont * dt;

    return {A_disc, B_disc};
}

std::pair<Eigen::Matrix<double, 16, 16>, Eigen::Matrix<double, 16, 9>> CWDynamics::get_mpc_dynamics_matrices(double dt) const {
    /*
     * Full 16-state dynamics, adding just the CW terms to an empty matrix.
     * The MPC controller will add this to the standard rigid body dynamics.
     * 
     * State indices:
     * 0-2: pos (x,y,z)
     * 7-9: vel (vx,vy,vz)
     */
    Eigen::Matrix<double, 16, 16> A_cw = Eigen::Matrix<double, 16, 16>::Zero();

    // Position -> Velocity coupling (Gravity Gradient)
    // vx_dot += 3n^2x + 2nvy
    A_cw(7, 0) = 3.0 * n_sq_ * dt;  // dvx/dx
    A_cw(7, 8) = 2.0 * n_ * dt;     // dvx/dvy (Coriolis, note: vy is index 8)

    // vy_dot += -2nvx
    A_cw(8, 7) = -2.0 * n_ * dt;    // dvy/dvx

    // vz_dot += -n^2z
    A_cw(9, 2) = -n_sq_ * dt;       // dvz/dz

    // B matrix is zero for CW gravity effects (it's passive dynamics)
    Eigen::Matrix<double, 16, 9> B_cw = Eigen::Matrix<double, 16, 9>::Zero();

    return {A_cw, B_cw};
}

} // namespace satellite_control
