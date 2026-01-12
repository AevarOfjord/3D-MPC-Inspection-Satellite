#include "fdir/FaultManager.h"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace sat_sim::fdir {

    FaultManager::FaultManager(const config::VehicleConfig& vehicle)
        : m_original_config(vehicle)
    {
        // Initialize status for all thrusters
        for (const auto& t : vehicle.thrusters) {
            ActuatorStatus status;
            status.id = t.id;
            status.type = ActuatorType::THRUSTER;
            status.fault = FaultType::NONE;
            status.effectiveness = 1.0;
            m_status.push_back(status);
        }

        // Add reaction wheel axes (X, Y, Z)
        if (vehicle.reaction_wheels.enabled) {
            for (const char* axis : {"RW_X", "RW_Y", "RW_Z"}) {
                ActuatorStatus status;
                status.id = axis;
                status.type = ActuatorType::REACTION_WHEEL;
                status.fault = FaultType::NONE;
                status.effectiveness = 1.0;
                m_status.push_back(status);
            }
        }

        spdlog::info("FaultManager: Initialized with {} actuators ({} thrusters, {} RW axes)",
            m_status.size(), vehicle.thrusters.size(), vehicle.reaction_wheels.enabled ? 3 : 0);
    }

    int FaultManager::find_actuator_index(const std::string& id) const {
        for (size_t i = 0; i < m_status.size(); ++i) {
            if (m_status[i].id == id) return static_cast<int>(i);
        }
        return -1;
    }

    void FaultManager::inject_fault(const std::string& id, FaultType fault, double effectiveness) {
        int idx = find_actuator_index(id);
        if (idx < 0) {
            spdlog::warn("FaultManager: Unknown actuator '{}'", id);
            return;
        }

        m_status[idx].fault = fault;
        
        switch (fault) {
            case FaultType::STUCK_OFF:
                m_status[idx].effectiveness = 0.0;
                break;
            case FaultType::STUCK_ON:
                m_status[idx].effectiveness = 0.0; // Can't control it
                break;
            case FaultType::DEGRADED:
                m_status[idx].effectiveness = std::clamp(effectiveness, 0.0, 1.0);
                break;
            case FaultType::INTERMITTENT:
                m_status[idx].effectiveness = 0.5; // Unreliable
                break;
            default:
                m_status[idx].effectiveness = 1.0;
        }

        spdlog::warn("FAULT INJECTED: {} -> {} (effectiveness: {:.1f}%)", 
            id, 
            fault == FaultType::STUCK_OFF ? "STUCK_OFF" :
            fault == FaultType::STUCK_ON ? "STUCK_ON" :
            fault == FaultType::DEGRADED ? "DEGRADED" : "INTERMITTENT",
            m_status[idx].effectiveness * 100);

        notify_change();
    }

    void FaultManager::clear_fault(const std::string& id) {
        int idx = find_actuator_index(id);
        if (idx >= 0) {
            m_status[idx].fault = FaultType::NONE;
            m_status[idx].effectiveness = 1.0;
            spdlog::info("FAULT CLEARED: {}", id);
            notify_change();
        }
    }

    void FaultManager::clear_all_faults() {
        for (auto& s : m_status) {
            s.fault = FaultType::NONE;
            s.effectiveness = 1.0;
        }
        spdlog::info("All faults cleared");
        notify_change();
    }

    std::vector<int> FaultManager::get_healthy_thruster_indices() const {
        std::vector<int> healthy;
        int thruster_idx = 0;
        for (const auto& s : m_status) {
            if (s.type == ActuatorType::THRUSTER) {
                if (s.is_healthy()) {
                    healthy.push_back(thruster_idx);
                }
                thruster_idx++;
            }
        }
        return healthy;
    }

    config::VehicleConfig FaultManager::get_effective_vehicle_config() const {
        config::VehicleConfig effective = m_original_config;
        
        // Filter to only healthy thrusters
        std::vector<config::ThrusterConfig> healthy_thrusters;
        for (size_t i = 0; i < m_original_config.thrusters.size(); ++i) {
            const auto& status = m_status[i];
            if (status.is_healthy()) {
                config::ThrusterConfig t = m_original_config.thrusters[i];
                // Apply degradation to max_thrust if partially failed
                t.max_thrust *= status.effectiveness;
                healthy_thrusters.push_back(t);
            }
        }
        effective.thrusters = healthy_thrusters;

        // Check RW status
        bool any_rw_fault = false;
        for (const auto& s : m_status) {
            if (s.type == ActuatorType::REACTION_WHEEL && !s.is_healthy()) {
                any_rw_fault = true;
                // In a more complete implementation, we'd track per-axis RW health
            }
        }
        if (any_rw_fault) {
            effective.reaction_wheels.enabled = false;
        }

        spdlog::info("Effective config: {} of {} thrusters healthy, RW {}",
            healthy_thrusters.size(), m_original_config.thrusters.size(),
            effective.reaction_wheels.enabled ? "enabled" : "DISABLED");

        return effective;
    }

    bool FaultManager::has_active_faults() const {
        return std::any_of(m_status.begin(), m_status.end(), 
            [](const ActuatorStatus& s) { return s.fault != FaultType::NONE; });
    }

    int FaultManager::get_healthy_thruster_count() const {
        return static_cast<int>(std::count_if(m_status.begin(), m_status.end(),
            [](const ActuatorStatus& s) { 
                return s.type == ActuatorType::THRUSTER && s.is_healthy(); 
            }));
    }

    int FaultManager::get_healthy_rw_count() const {
        return static_cast<int>(std::count_if(m_status.begin(), m_status.end(),
            [](const ActuatorStatus& s) { 
                return s.type == ActuatorType::REACTION_WHEEL && s.is_healthy(); 
            }));
    }

    void FaultManager::notify_change() {
        if (m_callback) {
            m_callback(m_status);
        }
    }

} // namespace sat_sim::fdir
