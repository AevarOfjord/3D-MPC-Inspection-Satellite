#pragma once
/**
 * @file MeshAnalyzer.h
 * @brief Mesh loading and surface analysis for inspection planning.
 * 
 * Loads STL/OBJ files and discretizes the surface into inspection zones.
 */

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace sat_sim::planner {

    using Vector3 = Eigen::Vector3d;

    /**
     * @brief A triangle face with position and normal.
     */
    struct Triangle {
        Vector3 v0, v1, v2;     // Vertices
        Vector3 normal;         // Face normal (outward)
        Vector3 centroid;       // Center point
        double area;            // Surface area
    };

    /**
     * @brief An inspection zone - a region of the surface to be observed.
     */
    struct InspectionZone {
        int id;
        Vector3 centroid;       // Zone center
        Vector3 normal;         // Average surface normal
        double area;            // Total area of zone
        std::vector<int> triangle_indices;  // Triangles in this zone
    };

    /**
     * @brief A viewpoint for observing a zone.
     */
    struct Viewpoint {
        int zone_id;            // Which zone this observes
        Vector3 position;       // Camera position
        Vector3 look_at;        // Target point (zone centroid)
        double distance;        // Distance from surface
    };

    /**
     * @brief Mesh loader and analyzer for inspection planning.
     */
    class MeshAnalyzer {
    public:
        MeshAnalyzer() = default;

        /**
         * @brief Load mesh from file (STL or OBJ).
         * @param filepath Path to mesh file.
         * @return True if successful.
         */
        bool load(const std::string& filepath);

        /**
         * @brief Get all triangles in the mesh.
         */
        const std::vector<Triangle>& get_triangles() const { return m_triangles; }

        /**
         * @brief Get mesh bounding box.
         */
        void get_bounds(Vector3& min_pt, Vector3& max_pt) const;

        /**
         * @brief Get total surface area.
         */
        double get_total_area() const { return m_total_area; }

        /**
         * @brief Discretize surface into inspection zones.
         * @param zone_size Approximate area per zone.
         * @return List of inspection zones.
         */
        std::vector<InspectionZone> create_zones(double zone_size = 1.0) const;

        /**
         * @brief Generate optimal viewpoints for each zone.
         * @param zones List of inspection zones.
         * @param standoff_distance Distance from surface for viewing.
         * @return List of viewpoints.
         */
        std::vector<Viewpoint> generate_viewpoints(
            const std::vector<InspectionZone>& zones,
            double standoff_distance = 2.0) const;

        /**
         * @brief Solve TSP to order viewpoints for minimal travel.
         * @param viewpoints Unordered viewpoints.
         * @param start_position Starting position for the tour.
         * @return Ordered indices for visiting viewpoints.
         */
        std::vector<int> solve_tsp(
            const std::vector<Viewpoint>& viewpoints,
            const Vector3& start_position) const;

    private:
        std::vector<Triangle> m_triangles;
        Vector3 m_min_bound, m_max_bound;
        double m_total_area = 0.0;

        bool load_stl(const std::string& filepath);
        bool load_obj(const std::string& filepath);
        void compute_bounds();
    };

} // namespace sat_sim::planner
