#pragma once
#include "core/Types.h"
#include "control/ControlAllocator.h"
#include <osqp.h>
#include <Eigen/Sparse>
#include <vector>
#include <memory>

namespace sat_sim::control {

    struct MPCConfig {
        int horizon = 10;
        double dt = 0.1;

        // Weights
        double q_pos = 10.0;
        double q_att = 10.0;
        double q_vel = 1.0;
        double q_angvel = 1.0;
        double r_thrust = 0.1; // Penalty on thruster usage
        double r_rw = 0.1;     // Penalty on RW usage
    };

    class MPCController {
    public:
        MPCController(const core::VehicleConfig& vehicle_config, const MPCConfig& mpc_config);
        ~MPCController();

        /**
         * @brief Compute control input for the current state to track target.
         */
        core::ControlInput compute_control(const core::StateVector& current_state, const core::StateVector& target_state);

    private:
        core::VehicleConfig m_vehicle_config;
        MPCConfig m_mpc_config;
        ControlAllocator m_allocator;

        // Dimensions
        int m_nx = 13; // State dim
        int m_nu;      // Control dim (RW + Thrusters)
        int m_n_vars;  // Total decision variables
        int m_n_constr;// Total constraints

        // OSQP Workspace
        OSQPSettings* m_settings = nullptr;
        OSQPWorkspace* m_work = nullptr;
        OSQPData* m_data = nullptr;

        // Cached Matrices (Eigen Sparse for OSQP)
        Eigen::SparseMatrix<double> m_P_sparse;
        Eigen::VectorXd m_q_vec;
        Eigen::SparseMatrix<double> m_A_constr_sparse;
        Eigen::VectorXd m_l_vec;
        Eigen::VectorXd m_u_vec;

        bool m_solver_initialized = false;

        // Internal helper to linearize dynamics
        // Returns A (nx x nx), B (nx x nu)
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearize_dynamics(const core::StateVector& x_k);
        
        // Helper to update OSQP vectors/matrices
        void update_qp_matrices(const core::StateVector& current_state, const core::StateVector& target_state);
    };

} // namespace sat_sim::control
