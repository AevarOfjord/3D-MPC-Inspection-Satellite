#include "logic/MissionManager.h"
#include <spdlog/spdlog.h>
#include <cmath>

namespace sat_sim::logic {

    MissionManager::MissionManager(const core::MissionProfile& profile) 
        : m_profile(profile) {
        if (m_profile.waypoints.empty()) {
            spdlog::warn("MissionManager: No waypoints provided in mission profile!");
            m_mission_complete = true;
        } else {
            spdlog::info("MissionManager: Initialized with {} waypoints", m_profile.waypoints.size());
        }
    }

    core::StateVector MissionManager::update(double current_time, const core::StateVector& current_state) {
        if (m_mission_complete || m_profile.waypoints.empty()) {
            // Return last waypoint or safe hold if complete
            if (!m_profile.waypoints.empty()) {
                const auto& wp = m_profile.waypoints.back();
                core::StateVector target;
                target.position = wp.position;
                target.attitude = wp.attitude;
                target.velocity.setZero();
                target.angular_velocity.setZero();
                return target;
            }
            // Fallback safe state
            core::StateVector safe;
            safe.position = current_state.position; // Hold here
            safe.velocity.setZero();
            safe.angular_velocity.setZero();
            safe.attitude = core::Quaternion(1,0,0,0);
            return safe;
        }

        const auto& wp = m_profile.waypoints[m_current_wp_index];

        // Check completion criteria
        if (!m_is_holding) {
            if (check_waypoint_reached(wp, current_state)) {
                spdlog::info("Transition: Reached Waypoint {} ({:.2f}, {:.2f}, {:.2f}). Holding for {:.1f}s", 
                    m_current_wp_index, wp.position.x(), wp.position.y(), wp.position.z(), wp.hold_time);
                m_is_holding = true;
                m_hold_start_time = current_time;
            }
        } else {
            // Check hold time
            if (current_time - m_hold_start_time >= wp.hold_time) {
                spdlog::info("Transition: Hold Complete for Waypoint {}.", m_current_wp_index);
                m_current_wp_index++;
                m_is_holding = false;

                if (m_current_wp_index >= static_cast<int>(m_profile.waypoints.size())) {
                    spdlog::info("Mission Complete!");
                    m_mission_complete = true;
                    // Keep using the last waypoint
                    m_current_wp_index = m_profile.waypoints.size() - 1;
                }
            }
        }

        // Construct Target (always aim for current active waypoint)
        core::StateVector target;
        // Careful: if completed, m_current_wp_index is reset to last one above
        const auto& active_wp = m_profile.waypoints[m_current_wp_index];
        target.position = active_wp.position;
        target.attitude = active_wp.attitude;
        target.velocity.setZero();
        target.angular_velocity.setZero();

        return target;
    }

    bool MissionManager::check_waypoint_reached(const core::Waypoint& wp, const core::StateVector& state) {
        double pos_err = (state.position - wp.position).norm();
        
        Eigen::Quaterniond q_curr(state.attitude(0), state.attitude(1), state.attitude(2), state.attitude(3));
        Eigen::Quaterniond q_target(wp.attitude(0), wp.attitude(1), wp.attitude(2), wp.attitude(3));
        
        // angularDistance returns [0, pi]
        double angle_diff = q_curr.angularDistance(q_target);

        double vel_norm = state.velocity.norm();
        double ang_vel_norm = state.angular_velocity.norm();

        return (pos_err <= wp.pos_tolerance) && (angle_diff <= wp.att_tolerance) &&
               (vel_norm <= wp.vel_tolerance) && (ang_vel_norm <= wp.ang_vel_tolerance);
    }
    
    bool MissionManager::is_complete() const { return m_mission_complete; }
    int MissionManager::get_current_waypoint_index() const { return m_current_wp_index; }

} // namespace sat_sim::logic
