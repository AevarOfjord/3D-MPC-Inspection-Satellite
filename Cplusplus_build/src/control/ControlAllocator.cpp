#include "control/ControlAllocator.h"
#include <spdlog/spdlog.h>
#include <iostream>

namespace sat_sim::control {

    // Constructor from legacy config
    ControlAllocator::ControlAllocator(const core::VehicleConfig& config) {
        m_num_thrusters = config.thrusters.size();
        
        // Convert to config::ThrusterConfig format
        std::vector<config::ThrusterConfig> thrusters;
        for (const auto& t : config.thrusters) {
            config::ThrusterConfig tc;
            tc.id = t.id;
            tc.position = t.position;
            tc.direction = t.direction;
            tc.max_thrust = t.max_thrust;
            thrusters.push_back(tc);
        }
        
        compute_B_matrix(thrusters);
    }

    // Constructor from new config
    ControlAllocator::ControlAllocator(const config::VehicleConfig& config) {
        m_num_thrusters = config.thrusters.size();
        compute_B_matrix(config.thrusters);
    }

    void ControlAllocator::update_config(const config::VehicleConfig& config) {
        m_num_thrusters = config.thrusters.size();
        compute_B_matrix(config.thrusters);
        spdlog::info("ControlAllocator: B-matrix updated for {} thrusters (fault adaptation)", m_num_thrusters);
    }

    void ControlAllocator::compute_B_matrix(const std::vector<config::ThrusterConfig>& thrusters) {
        if (thrusters.empty()) {
            spdlog::warn("ControlAllocator: No thrusters defined!");
            m_B.resize(6, 0);
            return;
        }

        m_B.resize(6, thrusters.size());
        m_B.setZero();

        for (size_t i = 0; i < thrusters.size(); ++i) {
            const auto& t = thrusters[i];
            
            // Force Vector (Body Frame)
            // For CONTINUOUS thrust: u is actual force [0, max_thrust]
            // B maps u -> [Force, Torque], so we use unit direction
            config::Vector3 dir = t.direction.normalized();

            // Torque Vector (Body Frame)
            // T = r x (u * dir) = u * (r x dir)
            config::Vector3 cross_prod = t.position.cross(dir);

            // Fill B Matrix Column with unit mappings
            m_B(0, i) = dir.x();
            m_B(1, i) = dir.y();
            m_B(2, i) = dir.z();
            m_B(3, i) = cross_prod.x();
            m_B(4, i) = cross_prod.y();
            m_B(5, i) = cross_prod.z();
        }

        spdlog::info("Control Allocation Matrix B Computed (6x{})", thrusters.size());
    }

    Eigen::Vector<double, 6> ControlAllocator::map_thrust_to_wrench(const std::vector<double>& thrust_values) const {
        if (thrust_values.size() != static_cast<size_t>(m_num_thrusters)) {
            spdlog::error("ControlAllocator: Thrust size mismatch ({} != {})", thrust_values.size(), m_num_thrusters);
            return Eigen::Vector<double, 6>::Zero();
        }

        Eigen::Map<const Eigen::VectorXd> u(thrust_values.data(), thrust_values.size());
        return m_B * u;
    }

} // namespace sat_sim::control
