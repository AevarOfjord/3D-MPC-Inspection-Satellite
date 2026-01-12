#pragma once
#include "core/Types.h"
#include <vector>

namespace sat_sim::logic {

    class MissionManager {
    public:
        MissionManager(const core::MissionProfile& profile);

        /**
         * @brief Updates the mission state machine.
         * @param current_time Simulation time [s]
         * @param current_state Current vehicle state
         * @return Target state for the controller
         */
        core::StateVector update(double current_time, const core::StateVector& current_state);

        bool is_complete() const;
        int get_current_waypoint_index() const;

    private:
        core::MissionProfile m_profile;
        int m_current_wp_index = 0;
        bool m_mission_complete = false;
        
        // State tracking
        bool m_is_holding = false;
        double m_hold_start_time = 0.0;

        bool check_waypoint_reached(const core::Waypoint& wp, const core::StateVector& state);
    };

} // namespace sat_sim::logic
