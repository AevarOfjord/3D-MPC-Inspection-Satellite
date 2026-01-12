#include "planner/InspectionPlanner.h"
#include <spdlog/spdlog.h>
#include <cmath>

namespace sat_sim::planner {

    config::MissionConfig InspectionPlanner::plan_mission(
            const std::string& mesh_file,
            const Vector3& start_position,
            const InspectionParams& params) {
        
        config::MissionConfig mission;
        mission.name = "Auto-Generated Inspection";
        mission.loop = false;

        // 1. Load mesh
        if (!m_mesh.load(mesh_file)) {
            spdlog::error("InspectionPlanner: Failed to load mesh: {}", mesh_file);
            return mission;
        }

        // 2. Create inspection zones
        m_zones = m_mesh.create_zones(params.zone_size);
        if (m_zones.empty()) {
            spdlog::error("InspectionPlanner: No zones created from mesh");
            return mission;
        }

        // 3. Generate viewpoints
        m_viewpoints = m_mesh.generate_viewpoints(m_zones, params.standoff_distance);

        // 4. Solve TSP for optimal ordering
        m_tour_order = m_mesh.solve_tsp(m_viewpoints, start_position);

        // 5. Convert to mission waypoints
        mission.waypoints.reserve(m_tour_order.size());

        for (int idx : m_tour_order) {
            const auto& vp = m_viewpoints[idx];
            
            config::Waypoint wp;
            wp.position = vp.position;
            wp.attitude = compute_look_at_quaternion(vp.position, vp.look_at);
            wp.hold_time = params.hold_time;
            wp.pos_tolerance = params.pos_tolerance;
            wp.att_tolerance = params.att_tolerance;
            
            mission.waypoints.push_back(wp);
        }

        // Estimate mission duration
        double total_dist = 0;
        Vector3 prev = start_position;
        for (int idx : m_tour_order) {
            total_dist += (m_viewpoints[idx].position - prev).norm();
            prev = m_viewpoints[idx].position;
        }
        double travel_time = total_dist / 0.1; // Assume ~0.1 m/s average
        double hold_time = params.hold_time * m_tour_order.size();
        mission.max_duration = travel_time + hold_time + 60.0; // Margin

        spdlog::info("InspectionPlanner: Mission planned with {} waypoints", mission.waypoints.size());
        spdlog::info("  - Estimated distance: {:.1f}m", total_dist);
        spdlog::info("  - Estimated duration: {:.0f}s", mission.max_duration);

        return mission;
    }

    config::Quaternion InspectionPlanner::compute_look_at_quaternion(
            const Vector3& from, const Vector3& to) const {
        
        // Compute rotation from +X axis to look direction
        Vector3 look_dir = (to - from).normalized();
        
        // Default forward is +X
        Vector3 forward(1, 0, 0);
        
        // Compute rotation axis and angle
        Vector3 axis = forward.cross(look_dir);
        double cos_angle = forward.dot(look_dir);
        
        if (axis.norm() < 1e-6) {
            // Parallel or anti-parallel
            if (cos_angle > 0) {
                return config::Quaternion(1, 0, 0, 0); // Identity
            } else {
                return config::Quaternion(0, 0, 1, 0); // 180° around Y
            }
        }
        
        axis.normalize();
        double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));
        
        // Quaternion from axis-angle
        double s = std::sin(angle / 2);
        double c = std::cos(angle / 2);
        
        return config::Quaternion(c, axis.x() * s, axis.y() * s, axis.z() * s);
    }

} // namespace sat_sim::planner
