#pragma once
/**
 * @file FaultManager.h
 * @brief Fault Detection, Isolation, and Recovery (FDIR) system.
 * 
 * Manages actuator health status and triggers B-matrix recomputation
 * when faults are detected.
 */

#include "config/ConfigSchema.h"
#include "core/Types.h"
#include <vector>
#include <string>
#include <functional>

namespace sat_sim::fdir {

    enum class ActuatorType {
        THRUSTER,
        REACTION_WHEEL
    };

    enum class FaultType {
        NONE,
        STUCK_OFF,          // Actuator cannot produce output
        STUCK_ON,           // Actuator stuck at max output
        DEGRADED,           // Reduced performance (partial failure)
        INTERMITTENT        // Random failures
    };

    struct ActuatorStatus {
        std::string id;
        ActuatorType type;
        FaultType fault = FaultType::NONE;
        double effectiveness = 1.0;     // 1.0 = healthy, 0.0 = failed
        bool is_healthy() const { return fault == FaultType::NONE && effectiveness > 0.5; }
    };

    /**
     * @brief Callback invoked when actuator status changes.
     * 
     * The control system should recompute the B-matrix when this fires.
     */
    using FaultCallback = std::function<void(const std::vector<ActuatorStatus>&)>;

    class FaultManager {
    public:
        explicit FaultManager(const config::VehicleConfig& vehicle);

        /**
         * @brief Set fault on a specific actuator.
         * @param id Actuator ID (from config)
         * @param fault Type of fault
         * @param effectiveness For DEGRADED faults, 0.0-1.0 remaining capability
         */
        void inject_fault(const std::string& id, FaultType fault, double effectiveness = 0.0);

        /**
         * @brief Clear fault and restore actuator to healthy.
         */
        void clear_fault(const std::string& id);

        /**
         * @brief Clear all faults.
         */
        void clear_all_faults();

        /**
         * @brief Get current status of all actuators.
         */
        const std::vector<ActuatorStatus>& get_status() const { return m_status; }

        /**
         * @brief Get list of healthy thruster indices.
         */
        std::vector<int> get_healthy_thruster_indices() const;

        /**
         * @brief Build a modified vehicle config with failed actuators removed.
         * 
         * This creates a new VehicleConfig that only contains healthy thrusters,
         * allowing the MPC to recompute with reduced actuator set.
         */
        config::VehicleConfig get_effective_vehicle_config() const;

        /**
         * @brief Check if any faults are active.
         */
        bool has_active_faults() const;

        /**
         * @brief Register callback for fault status changes.
         */
        void set_fault_callback(FaultCallback cb) { m_callback = cb; }

        /**
         * @brief Get number of healthy thrusters.
         */
        int get_healthy_thruster_count() const;

        /**
         * @brief Get number of healthy reaction wheel axes.
         */
        int get_healthy_rw_count() const;

    private:
        config::VehicleConfig m_original_config;
        std::vector<ActuatorStatus> m_status;
        FaultCallback m_callback;

        void notify_change();
        int find_actuator_index(const std::string& id) const;
    };

} // namespace sat_sim::fdir
