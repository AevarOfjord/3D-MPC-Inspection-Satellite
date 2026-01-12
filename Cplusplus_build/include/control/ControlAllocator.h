#pragma once
#include "core/Types.h"
#include "config/ConfigSchema.h"
#include <Eigen/Dense>

namespace sat_sim::control {

    class ControlAllocator {
    public:
        using Matrix6X = Eigen::Matrix<double, 6, Eigen::Dynamic>;

        /**
         * @brief Construct from legacy core::VehicleConfig.
         */
        explicit ControlAllocator(const core::VehicleConfig& config);

        /**
         * @brief Construct from new config::VehicleConfig.
         */
        explicit ControlAllocator(const config::VehicleConfig& config);

        /**
         * @brief Get the Control Allocation Matrix B.
         * Dimensions: 6 x N_thrusters.
         * Maps [u1, ... un]^T -> [Fx, Fy, Fz, Tx, Ty, Tz]^T (Body Frame)
         */
        const Matrix6X& get_B_matrix() const { return m_B; }

        /**
         * @brief Recompute B-matrix with new configuration.
         * 
         * Used for fault adaptation - pass effective config with failed
         * thrusters removed.
         */
        void update_config(const config::VehicleConfig& config);

        /**
         * @brief Map thruster forces to body wrench.
         * @param thrust_values Vector of size N_thrusters (actual force values).
         * @return Vector6 [Force_3D, Torque_3D]
         */
        Eigen::Vector<double, 6> map_thrust_to_wrench(const std::vector<double>& thrust_values) const;

        /**
         * @brief Get number of thrusters in current config.
         */
        int get_num_thrusters() const { return m_num_thrusters; }

    private:
        Matrix6X m_B;
        int m_num_thrusters;
        
        void compute_B_matrix(const std::vector<config::ThrusterConfig>& thrusters);
    };

} // namespace sat_sim::control
