#include "control/MPCController.h"
#include <spdlog/spdlog.h>
#include <iostream>

namespace sat_sim::control {

    using namespace Eigen;
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;

    MPCController::MPCController(const core::VehicleConfig& vehicle_config, const MPCConfig& mpc_config)
        : m_vehicle_config(vehicle_config), m_mpc_config(mpc_config), m_allocator(vehicle_config) 
    {
        m_nu = 3 + vehicle_config.thrusters.size();
        
        // Z = [x0, u0, x1, u1 ... xN]
        m_n_vars = (m_mpc_config.horizon + 1) * m_nx + m_mpc_config.horizon * m_nu;
        
        // Constraints: Init (nx) + Dynamics (N*nx) + Bounds (N*nu) 
        // A has rows for:
        // 1. Initial State: x0 = x_init (nx rows)
        // 2. Dynamics: x_{k+1} - A x_k - B u_k = 0 (N * nx rows)
        // 3. Input Bounds: u_min <= u_k <= u_max (N * nu rows)
        m_n_constr = m_nx + m_mpc_config.horizon * m_nx + m_mpc_config.horizon * m_nu;

        m_settings = (OSQPSettings*)c_malloc(sizeof(OSQPSettings));
        if (m_settings) {
            osqp_set_default_settings(m_settings);
            m_settings->alpha = 1.0; 
            m_settings->verbose = 0; // Silent for now
        }
    }

    MPCController::~MPCController() {
        if (m_work) osqp_cleanup(m_work);
        if (m_data) {
            if (m_data->A) c_free(m_data->A);
            if (m_data->P) c_free(m_data->P);
            c_free(m_data);
        }
        if (m_settings) c_free(m_settings);
    }

    std::pair<MatrixXd, MatrixXd> MPCController::linearize_dynamics(const core::StateVector& x_k) {
        double dt = m_mpc_config.dt;
        MatrixXd A = MatrixXd::Identity(13, 13);
        MatrixXd B = MatrixXd::Zero(13, m_nu);

        // Position prop via Velocity
        // p_{k+1} = p_k + v_k * dt
        A(0, 7) = dt;
        A(1, 8) = dt;
        A(2, 9) = dt;

        // Attitude prop via Quaternion Kinematics
        // q_{k+1} = q_k + 0.5 * G(q) * w_k * dt
        // q = [w, x, y, z] is stored in Vector4d as indices [0, 1, 2, 3]
        double qw = x_k.attitude(0);
        double qx = x_k.attitude(1);
        double qy = x_k.attitude(2);
        double qz = x_k.attitude(3);

        // G(q) matrix (4x3) matches q dot = 0.5 * G * omega
        MatrixXd G(4, 3);
        G << -qx, -qy, -qz,
              qw, -qz,  qy,
              qz,  qw, -qx,
             -qy,  qx,  qw;
        
        A.block<4, 3>(3, 10) = 0.5 * G * dt;

        // B Matrix Construction
        Vector3 J = m_vehicle_config.inertia_diag;
        
        // 1. Reaction Wheels (Columns 0-2) -> Torque
        // w dot = J^-1 * tau
        for (int i = 0; i < 3; ++i) {
            B(10 + i, i) = (1.0 / J[i]) * dt;
        }

        // 2. Thrusters (Columns 3..N) -> Force & Torque
        int n_thr = m_vehicle_config.thrusters.size();
        const auto& B_alloc = m_allocator.get_B_matrix(); // 6xN (Fx,Fy,Fz, Tx,Ty,Tz)

        Eigen::Quaterniond q_att(x_k.attitude(0), x_k.attitude(1), x_k.attitude(2), x_k.attitude(3));
        Matrix3 R = q_att.toRotationMatrix();

        for (int i = 0; i < n_thr; ++i) {
            int col_idx = 3 + i;
            Vector3 F_body; F_body = B_alloc.col(i).head<3>();
            Vector3 T_body; T_body = B_alloc.col(i).tail<3>();

            // Linear: v += (R * F_body / m) * dt
            Vector3 F_world = R * F_body;
            B.block<3, 1>(7, col_idx) = (F_world / m_vehicle_config.mass) * dt;

            // Angular: w += (T_body / J) * dt
            Vector3 dw = T_body.cwiseQuotient(J) * dt;
            B(10, col_idx) = dw.x();
            B(11, col_idx) = dw.y();
            B(12, col_idx) = dw.z();
        }

        return {A, B};
    }

    void MPCController::update_qp_matrices(const core::StateVector& current_state, const core::StateVector& target_state) {
        // Linearize around current state (Zero-Order Hold for whole horizon approx)
        auto [Ad, Bd] = linearize_dynamics(current_state);

        int N = m_mpc_config.horizon;
        
        // --- Build P (Cost) ---
        m_P_sparse.resize(m_n_vars, m_n_vars);
        m_P_sparse.setZero(); // Clear

        std::vector<Triplet<double>> p_triplets;

        // Create Q diagonal vector
        VectorXd Q_diag(13);
        Q_diag << m_mpc_config.q_pos, m_mpc_config.q_pos, m_mpc_config.q_pos,   // Pos
                  m_mpc_config.q_att, m_mpc_config.q_att, m_mpc_config.q_att, m_mpc_config.q_att, // Quat
                  m_mpc_config.q_vel, m_mpc_config.q_vel, m_mpc_config.q_vel,   // Vel
                  m_mpc_config.q_angvel, m_mpc_config.q_angvel, m_mpc_config.q_angvel; // AngVel

        // Create R diagonal vector
        VectorXd R_diag(m_nu);
        R_diag.head(3).setConstant(m_mpc_config.r_rw);        // RW
        R_diag.tail(m_nu - 3).setConstant(m_mpc_config.r_thrust); // Thrusters

        for (int k = 0; k <= N; ++k) {
            // State Cost Q
            int x_idx = k * (m_nx + m_nu); 
            // Only add Q if not pure terminal? Or all stages.
            for (int i = 0; i < m_nx; ++i) {
                p_triplets.emplace_back(x_idx + i, x_idx + i, Q_diag(i));
            }

            // Input Cost R (only up to N-1)
            if (k < N) {
                int u_idx = x_idx + m_nx;
                for (int i = 0; i < m_nu; ++i) {
                    p_triplets.emplace_back(u_idx + i, u_idx + i, R_diag(i));
                }
            }
        }
        m_P_sparse.setFromTriplets(p_triplets.begin(), p_triplets.end());

        // --- Build q (Linear Cost) ---
        // q = -Q * x_ref
        m_q_vec.resize(m_n_vars);
        m_q_vec.setZero();

        // Create ref vector (assuming constant reference over horizon)
        VectorXd x_ref(13);
        // Map attitude to [w, x, y, z] explicitly using indices
        // w=0, x=1, y=2, z=3
        x_ref.segment<3>(0) = target_state.position;
        x_ref(3) = target_state.attitude(0);
        x_ref(4) = target_state.attitude(1);
        x_ref(5) = target_state.attitude(2);
        x_ref(6) = target_state.attitude(3);
        x_ref.segment<3>(7) = target_state.velocity;
        x_ref.segment<3>(10) = target_state.angular_velocity;

        for (int k = 0; k <= N; ++k) {
            int x_idx = k * (m_nx + m_nu);
            // q block = -Q_diag * x_ref
            for (int i = 0; i < m_nx; ++i) {
                 m_q_vec(x_idx + i) = -Q_diag(i) * x_ref(i);
            }
        }

        // --- Build A_constr and l, u ---
        m_A_constr_sparse.resize(m_n_constr, m_n_vars);
        m_l_vec.resize(m_n_constr);
        m_u_vec.resize(m_n_constr);
        
        std::vector<Triplet<double>> a_triplets;
        int row_offset = 0;

        // 1. Initial State Constraint: x0 = current_state
        // Row 0..12: Identity(13) on x0
        for (int i = 0; i < m_nx; ++i) {
            a_triplets.emplace_back(row_offset + i, i, 1.0);
            
            // Current state values
            double val = 0;
            if (i < 3) val = current_state.position(i);
            else if (i < 7) { 
                // Map attitude to [w, x, y, z] using indices 0..3
                // i=3 -> w(0), i=4 -> x(1), i=5 -> y(2), i=6 -> z(3)
                val = current_state.attitude(i - 3);
            } else if (i < 10) val = current_state.velocity(i-7);
            else val = current_state.angular_velocity(i-10);

            m_l_vec(row_offset + i) = val;
            m_u_vec(row_offset + i) = val; // Equality using bounds
        }
        row_offset += m_nx;

        // 2. Dynamics Constraints: x_{k+1} - A x_k - B u_k = 0
        // for k=0 to N-1
        for (int k = 0; k < N; ++k) {
            int x_k_idx = k * (m_nx + m_nu);
            int u_k_idx = x_k_idx + m_nx;
            int x_kp1_idx = (k + 1) * (m_nx + m_nu);

            // Coeffs: -A on x_k, -B on u_k, +I on x_{k+1}
            
            // -A
            for (int r = 0; r < m_nx; ++r) {
                for (int c = 0; c < m_nx; ++c) {
                    if (Ad(r,c) != 0) a_triplets.emplace_back(row_offset + r, x_k_idx + c, -Ad(r,c));
                }
            }
            // -B
            for (int r = 0; r < m_nx; ++r) {
                for (int c = 0; c < m_nu; ++c) {
                    if (Bd(r,c) != 0) a_triplets.emplace_back(row_offset + r, u_k_idx + c, -Bd(r,c));
                }
            }
            // +I
            for (int r = 0; r < m_nx; ++r) {
                a_triplets.emplace_back(row_offset + r, x_kp1_idx + r, 1.0);
            }

            // RHS = 0
            m_l_vec.segment(row_offset, m_nx).setZero();
            m_u_vec.segment(row_offset, m_nx).setZero();
            
            row_offset += m_nx;
        }

        // 3. Input Bounds
        double max_rw_torque = m_vehicle_config.rw_config.max_torque;
        if (max_rw_torque <= 0) max_rw_torque = 0.1; // Default fallback

        for (int k = 0; k < N; ++k) {
             int u_k_idx = k * (m_nx + m_nu) + m_nx;
             
             // RW Inputs (0,1,2) -> Torque limits
             for (int i = 0; i < 3; ++i) {
                 a_triplets.emplace_back(row_offset + i, u_k_idx + i, 1.0);
                 m_l_vec(row_offset + i) = -max_rw_torque;
                 m_u_vec(row_offset + i) =  max_rw_torque;
             }
             
             // Thruster Inputs (3..nu-1) -> Continuous thrust [0, max_thrust]
             // Per-thruster bounds from vehicle config
             int n_thr = m_vehicle_config.thrusters.size();
             for (int i = 0; i < n_thr; ++i) {
                 int u_idx = 3 + i;
                 a_triplets.emplace_back(row_offset + u_idx, u_k_idx + u_idx, 1.0);
                 m_l_vec(row_offset + u_idx) = 0.0;
                 // Use per-thruster max_thrust for upper bound
                 m_u_vec(row_offset + u_idx) = m_vehicle_config.thrusters[i].max_thrust;
             }
             
             row_offset += m_nu;
        }

        m_A_constr_sparse.setFromTriplets(a_triplets.begin(), a_triplets.end());
    }

    core::ControlInput MPCController::compute_control(const core::StateVector& current_state, const core::StateVector& target_state) {
        // 1. Build Matrices
        update_qp_matrices(current_state, target_state);

        // 2. Prepare OSQP data
        if (m_solver_initialized && m_work) {
            osqp_cleanup(m_work);
            m_work = nullptr;
            if (m_data) {
                 if (m_data->A) c_free(m_data->A);
                 if (m_data->P) c_free(m_data->P);
                 c_free(m_data);
            }
        }
 
        // Convert Eigen Sparse to CSC arrays
        // Note: OSQP takes c_int (usually long long). Eigen uses int. 
        // We must copy indices.
        
        m_data = (OSQPData*)c_malloc(sizeof(OSQPData));
        m_data->n = m_n_vars;
        m_data->m = m_n_constr;
        
        // P
        m_P_sparse.makeCompressed();
        c_int nnz_P = m_P_sparse.nonZeros();
        c_float* P_x = (c_float*)c_malloc(nnz_P * sizeof(c_float));
        c_int* P_i = (c_int*)c_malloc(nnz_P * sizeof(c_int));
        c_int* P_p = (c_int*)c_malloc((m_n_vars + 1) * sizeof(c_int));

        Eigen::Map<VectorXd>(P_x, nnz_P) = Map<const VectorXd>(m_P_sparse.valuePtr(), nnz_P);
        std::copy(m_P_sparse.innerIndexPtr(), m_P_sparse.innerIndexPtr() + nnz_P, P_i);
        std::copy(m_P_sparse.outerIndexPtr(), m_P_sparse.outerIndexPtr() + m_n_vars + 1, P_p);
        
        m_data->P = csc_matrix(m_data->n, m_data->n, nnz_P, P_x, P_i, P_p);

        // A
        m_A_constr_sparse.makeCompressed();
        c_int nnz_A = m_A_constr_sparse.nonZeros();
        c_float* A_x = (c_float*)c_malloc(nnz_A * sizeof(c_float));
        c_int* A_i = (c_int*)c_malloc(nnz_A * sizeof(c_int));
        c_int* A_p = (c_int*)c_malloc((m_n_vars + 1) * sizeof(c_int));

        Eigen::Map<VectorXd>(A_x, nnz_A) = Map<const VectorXd>(m_A_constr_sparse.valuePtr(), nnz_A);
        std::copy(m_A_constr_sparse.innerIndexPtr(), m_A_constr_sparse.innerIndexPtr() + nnz_A, A_i);
        std::copy(m_A_constr_sparse.outerIndexPtr(), m_A_constr_sparse.outerIndexPtr() + m_n_vars + 1, A_p);

        m_data->A = csc_matrix(m_data->m, m_data->n, nnz_A, A_x, A_i, A_p);

        // Vectors
        m_data->q = (c_float*)c_malloc(m_n_vars * sizeof(c_float));
        Eigen::Map<VectorXd>(m_data->q, m_n_vars) = m_q_vec;

        m_data->l = (c_float*)c_malloc(m_n_constr * sizeof(c_float));
        Eigen::Map<VectorXd>(m_data->l, m_n_constr) = m_l_vec;

        m_data->u = (c_float*)c_malloc(m_n_constr * sizeof(c_float));
        Eigen::Map<VectorXd>(m_data->u, m_n_constr) = m_u_vec;

        // 3. Setup and Solve
        osqp_setup(&m_work, m_data, m_settings);
        osqp_solve(m_work);

        // 4. Extract Solution
        core::ControlInput u_out;
        if (m_work->info->status_val == OSQP_SOLVED || m_work->info->status_val == OSQP_SOLVED_INACCURATE || m_work->info->status_val == OSQP_MAX_ITER_REACHED) {
            // u0 is at index nx in solution vector x
            // u_out.rw_torques = u0[0..2]
            // u_out.thrusters = u0[3..]
            
            // OSQP solution is in m_work->solution->x
            int u0_idx = m_nx;
            
            u_out.rw_torques[0] = m_work->solution->x[u0_idx + 0];
            u_out.rw_torques[1] = m_work->solution->x[u0_idx + 1];
            u_out.rw_torques[2] = m_work->solution->x[u0_idx + 2];

            int n_thr = m_vehicle_config.thrusters.size();
            u_out.thruster_activations.resize(n_thr);
            for(int i=0; i<n_thr; ++i) {
                u_out.thruster_activations[i] = m_work->solution->x[u0_idx + 3 + i];
            }
        } else {
            spdlog::warn("MPC Solver Failed: {}", m_work->info->status_val);
            u_out.rw_torques.setZero();
            u_out.thruster_activations.assign(m_vehicle_config.thrusters.size(), 0.0);
        }

        m_solver_initialized = true;
        
        // Clean up temporary arrays (OSQP setup copies them?)
        // Standard OSQP (c_state) copies data in setup.
        // We can free the arrays we malloced for OSQPData, but NOT the arrays inside OSQPData? 
        // No, we passed pointers to OSQPData. 
        // osqp_setup allocates its own workspace.
        // We can safely free our helper arrays.
        c_free(P_x); c_free(P_i); c_free(P_p);
        c_free(A_x); c_free(A_i); c_free(A_p);
        c_free(m_data->q);
        c_free(m_data->l);
        c_free(m_data->u);
        // And the structs
        c_free(m_data->P);
        c_free(m_data->A);
        c_free(m_data);
        m_data = nullptr;

        return u_out;
    }

} // namespace sat_sim::control
