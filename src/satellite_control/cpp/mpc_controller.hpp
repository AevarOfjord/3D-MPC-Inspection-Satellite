
#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <memory>
#include "osqp.h"
#include "linearizer.hpp"

namespace satellite_mpc {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::SparseMatrix;

struct MPCParams {
    // Dimensions
    int prediction_horizon = 50;
    double dt = 0.05;
    double solver_time_limit = 0.05;
    
    // Weights
    double Q_pos = 10.0;
    double Q_vel = 1.0;
    double Q_ang = 10.0;
    double Q_angvel = 1.0;
    double R_thrust = 0.1;
    double R_rw_torque = 0.1;
    
    // Constraints
    double max_velocity = 1.0;
    double max_angular_velocity = 1.0;
    double position_bounds = 10.0;
    
    // Z-tilt
    bool enable_z_tilt = true;
    double z_tilt_gain = 0.35;
    double z_tilt_max_rad = 0.35;  // ~20 deg
};

struct ControlResult {
    VectorXd u;
    int status;
    double solve_time;
    bool timeout;
};

class MPCControllerCpp {
public:
    MPCControllerCpp(const satellite_dt::SatelliteParams& sat_params, const MPCParams& mpc_params);
    ~MPCControllerCpp();

    // Main interface
    ControlResult get_control_action(const VectorXd& x_current, const VectorXd& x_target);
    
    // Accessors
    int num_controls() const { return nu_; }
    int prediction_horizon() const { return N_; }
    double dt() const { return dt_; }

private:
    // Dimensions
    int nx_ = 13;  // State dimension
    int nu_;       // Control dimension (RW + thrusters)
    int N_;        // Prediction horizon
    double dt_;
    
    // Parameters
    MPCParams mpc_params_;
    satellite_dt::SatelliteParams sat_params_;
    
    // Linearizer
    std::unique_ptr<satellite_dt::Linearizer> linearizer_;
    
    // OSQP workspace
    OSQPWorkspace* work_ = nullptr;
    OSQPSettings* settings_ = nullptr;
    OSQPData* data_ = nullptr;
    
    // Problem matrices (stored for updates)
    SparseMatrix<double> P_;  // Cost
    SparseMatrix<double> A_;  // Constraints
    VectorXd q_;              // Linear cost
    VectorXd l_, u_;          // Bounds
    
    // Precomputed
    VectorXd Q_diag_;
    VectorXd R_diag_;
    VectorXd control_lower_;
    VectorXd control_upper_;
    
    // State tracking
    Eigen::Vector4d prev_quat_;
    
    // Internal methods
    void init_solver();
    void update_dynamics(const VectorXd& x_current);
    void update_cost(const VectorXd& x_target);
    void update_constraints(const VectorXd& x_current);
    VectorXd apply_z_tilt(const VectorXd& x_current, const VectorXd& x_target);
    
    // CSC matrix helpers
    std::vector<c_float> P_data_;
    std::vector<c_int> P_indices_;
    std::vector<c_int> P_indptr_;
    std::vector<c_float> A_data_;
    std::vector<c_int> A_indices_;
    std::vector<c_int> A_indptr_;
    
    // B matrix index map for fast updates
    std::vector<std::vector<int>> B_idx_map_;
    
    // A matrix index map for dynamics matrix updates (only non-identity entries)
    std::vector<std::vector<int>> A_idx_map_;
};

} // namespace satellite_mpc
