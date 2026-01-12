#pragma once
/**
 * @file InspectionPlanner.h
 * @brief Complete inspection mission planning from mesh to waypoints.
 */

#include "planner/MeshAnalyzer.h"
#include "config/ConfigSchema.h"
#include <vector>
#include <string>

namespace sat_sim::planner {

    /**
     * @brief Inspection mission parameters.
     */
    struct InspectionParams {
        double standoff_distance = 3.0;     // Distance from surface [m]
        double zone_size = 2.0;             // Approx area per zone [m²]
        double hold_time = 2.0;             // Time at each viewpoint [s]
        double pos_tolerance = 0.1;         // Position tolerance [m]
        double att_tolerance = 0.1;         // Attitude tolerance [rad]
    };

    /**
     * @brief High-level inspection mission planner.
     */
    class InspectionPlanner {
    public:
        InspectionPlanner() = default;

        /**
         * @brief Plan complete inspection mission from mesh file.
         * @param mesh_file Path to STL/OBJ file.
         * @param start_position Inspector starting position.
         * @param params Inspection parameters.
         * @return MissionConfig with ordered waypoints.
         */
        config::MissionConfig plan_mission(
            const std::string& mesh_file,
            const Vector3& start_position,
            const InspectionParams& params = InspectionParams());

        /**
         * @brief Get the generated viewpoints (after plan_mission).
         */
        const std::vector<Viewpoint>& get_viewpoints() const { return m_viewpoints; }

        /**
         * @brief Get mesh analyzer (after plan_mission).
         */
        const MeshAnalyzer& get_mesh() const { return m_mesh; }

    private:
        MeshAnalyzer m_mesh;
        std::vector<InspectionZone> m_zones;
        std::vector<Viewpoint> m_viewpoints;
        std::vector<int> m_tour_order;

        config::Quaternion compute_look_at_quaternion(
            const Vector3& from, const Vector3& to) const;
    };

} // namespace sat_sim::planner
