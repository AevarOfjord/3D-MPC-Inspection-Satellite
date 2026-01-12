#include "planner/MeshAnalyzer.h"
#include <spdlog/spdlog.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <limits>

namespace sat_sim::planner {

    // ============================================================================
    // MESH LOADING
    // ============================================================================

    bool MeshAnalyzer::load(const std::string& filepath) {
        m_triangles.clear();
        m_total_area = 0.0;

        // Determine file type from extension
        std::string ext = filepath.substr(filepath.find_last_of('.') + 1);
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        bool success = false;
        if (ext == "stl") {
            success = load_stl(filepath);
        } else if (ext == "obj") {
            success = load_obj(filepath);
        } else {
            spdlog::error("MeshAnalyzer: Unsupported file format: {}", ext);
            return false;
        }

        if (success) {
            compute_bounds();
            spdlog::info("MeshAnalyzer: Loaded {} triangles, area={:.2f}m²", 
                m_triangles.size(), m_total_area);
        }

        return success;
    }

    bool MeshAnalyzer::load_stl(const std::string& filepath) {
        std::ifstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            spdlog::error("MeshAnalyzer: Cannot open file: {}", filepath);
            return false;
        }

        // Try ASCII first
        file.seekg(0);
        std::string line;
        std::getline(file, line);
        
        bool is_ascii = (line.find("solid") != std::string::npos && 
                         line.find("facet") == std::string::npos);

        file.seekg(0);

        if (is_ascii) {
            // ASCII STL format
            while (std::getline(file, line)) {
                if (line.find("facet normal") != std::string::npos) {
                    Triangle tri;
                    
                    // Parse normal
                    std::istringstream ss(line);
                    std::string token;
                    ss >> token >> token; // "facet" "normal"
                    ss >> tri.normal.x() >> tri.normal.y() >> tri.normal.z();

                    // Skip "outer loop"
                    std::getline(file, line);

                    // Read 3 vertices
                    for (int i = 0; i < 3; ++i) {
                        std::getline(file, line);
                        std::istringstream vss(line);
                        vss >> token; // "vertex"
                        Vector3& v = (i == 0) ? tri.v0 : (i == 1) ? tri.v1 : tri.v2;
                        vss >> v.x() >> v.y() >> v.z();
                    }

                    // Compute centroid and area
                    tri.centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0;
                    Vector3 e1 = tri.v1 - tri.v0;
                    Vector3 e2 = tri.v2 - tri.v0;
                    tri.area = 0.5 * e1.cross(e2).norm();
                    m_total_area += tri.area;

                    m_triangles.push_back(tri);
                }
            }
        } else {
            // Binary STL format
            file.seekg(80); // Skip header
            
            uint32_t num_triangles;
            file.read(reinterpret_cast<char*>(&num_triangles), 4);

            for (uint32_t i = 0; i < num_triangles; ++i) {
                Triangle tri;
                float data[12]; // normal(3) + v0(3) + v1(3) + v2(3)
                file.read(reinterpret_cast<char*>(data), 48);
                
                tri.normal = Vector3(data[0], data[1], data[2]);
                tri.v0 = Vector3(data[3], data[4], data[5]);
                tri.v1 = Vector3(data[6], data[7], data[8]);
                tri.v2 = Vector3(data[9], data[10], data[11]);

                tri.centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0;
                Vector3 e1 = tri.v1 - tri.v0;
                Vector3 e2 = tri.v2 - tri.v0;
                tri.area = 0.5 * e1.cross(e2).norm();
                m_total_area += tri.area;

                m_triangles.push_back(tri);

                uint16_t attr;
                file.read(reinterpret_cast<char*>(&attr), 2); // Attribute byte count
            }
        }

        return !m_triangles.empty();
    }

    bool MeshAnalyzer::load_obj(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            spdlog::error("MeshAnalyzer: Cannot open file: {}", filepath);
            return false;
        }

        std::vector<Vector3> vertices;
        std::string line;

        while (std::getline(file, line)) {
            std::istringstream ss(line);
            std::string prefix;
            ss >> prefix;

            if (prefix == "v") {
                // Vertex
                Vector3 v;
                ss >> v.x() >> v.y() >> v.z();
                vertices.push_back(v);
            } else if (prefix == "f") {
                // Face (triangle or quad)
                std::vector<int> indices;
                std::string token;
                while (ss >> token) {
                    // Handle "v", "v/vt", "v/vt/vn", "v//vn" formats
                    int idx = std::stoi(token.substr(0, token.find('/')));
                    indices.push_back(idx - 1); // OBJ is 1-indexed
                }

                // Triangulate if needed
                for (size_t i = 2; i < indices.size(); ++i) {
                    Triangle tri;
                    tri.v0 = vertices[indices[0]];
                    tri.v1 = vertices[indices[i-1]];
                    tri.v2 = vertices[indices[i]];

                    Vector3 e1 = tri.v1 - tri.v0;
                    Vector3 e2 = tri.v2 - tri.v0;
                    tri.normal = e1.cross(e2).normalized();
                    tri.centroid = (tri.v0 + tri.v1 + tri.v2) / 3.0;
                    tri.area = 0.5 * e1.cross(e2).norm();
                    m_total_area += tri.area;

                    m_triangles.push_back(tri);
                }
            }
        }

        return !m_triangles.empty();
    }

    void MeshAnalyzer::compute_bounds() {
        if (m_triangles.empty()) return;

        m_min_bound = Vector3::Constant(std::numeric_limits<double>::max());
        m_max_bound = Vector3::Constant(std::numeric_limits<double>::lowest());

        for (const auto& tri : m_triangles) {
            for (const Vector3* v : {&tri.v0, &tri.v1, &tri.v2}) {
                m_min_bound = m_min_bound.cwiseMin(*v);
                m_max_bound = m_max_bound.cwiseMax(*v);
            }
        }
    }

    void MeshAnalyzer::get_bounds(Vector3& min_pt, Vector3& max_pt) const {
        min_pt = m_min_bound;
        max_pt = m_max_bound;
    }

    // ============================================================================
    // ZONE GENERATION
    // ============================================================================

    std::vector<InspectionZone> MeshAnalyzer::create_zones(double zone_size) const {
        if (m_triangles.empty()) return {};

        // Simple spatial clustering: divide bounding box into grid cells
        Vector3 extent = m_max_bound - m_min_bound;
        double cell_size = std::sqrt(zone_size);
        
        int nx = std::max(1, static_cast<int>(std::ceil(extent.x() / cell_size)));
        int ny = std::max(1, static_cast<int>(std::ceil(extent.y() / cell_size)));
        int nz = std::max(1, static_cast<int>(std::ceil(extent.z() / cell_size)));

        // Grid of zones
        std::vector<InspectionZone> zones(nx * ny * nz);
        for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
            zones[i].id = i;
            zones[i].area = 0;
        }

        // Assign triangles to zones
        for (size_t ti = 0; ti < m_triangles.size(); ++ti) {
            const auto& tri = m_triangles[ti];
            
            int ix = std::min(nx-1, static_cast<int>((tri.centroid.x() - m_min_bound.x()) / cell_size));
            int iy = std::min(ny-1, static_cast<int>((tri.centroid.y() - m_min_bound.y()) / cell_size));
            int iz = std::min(nz-1, static_cast<int>((tri.centroid.z() - m_min_bound.z()) / cell_size));

            int zone_idx = ix + iy * nx + iz * nx * ny;
            zones[zone_idx].triangle_indices.push_back(static_cast<int>(ti));
            zones[zone_idx].area += tri.area;
        }

        // Compute zone centroids and normals
        std::vector<InspectionZone> valid_zones;
        for (auto& zone : zones) {
            if (zone.triangle_indices.empty()) continue;

            zone.centroid = Vector3::Zero();
            zone.normal = Vector3::Zero();
            double total_weight = 0;

            for (int ti : zone.triangle_indices) {
                const auto& tri = m_triangles[ti];
                zone.centroid += tri.centroid * tri.area;
                zone.normal += tri.normal * tri.area;
                total_weight += tri.area;
            }

            zone.centroid /= total_weight;
            zone.normal.normalize();
            zone.id = static_cast<int>(valid_zones.size());
            valid_zones.push_back(zone);
        }

        spdlog::info("MeshAnalyzer: Created {} inspection zones", valid_zones.size());
        return valid_zones;
    }

    // ============================================================================
    // VIEWPOINT GENERATION
    // ============================================================================

    std::vector<Viewpoint> MeshAnalyzer::generate_viewpoints(
            const std::vector<InspectionZone>& zones,
            double standoff_distance) const {
        
        std::vector<Viewpoint> viewpoints;
        viewpoints.reserve(zones.size());

        for (const auto& zone : zones) {
            Viewpoint vp;
            vp.zone_id = zone.id;
            vp.look_at = zone.centroid;
            vp.distance = standoff_distance;
            
            // Position camera along surface normal at standoff distance
            vp.position = zone.centroid + zone.normal * standoff_distance;

            viewpoints.push_back(vp);
        }

        spdlog::info("MeshAnalyzer: Generated {} viewpoints at {:.1f}m standoff", 
            viewpoints.size(), standoff_distance);
        return viewpoints;
    }

    // ============================================================================
    // TSP SOLVER (Greedy Nearest Neighbor)
    // ============================================================================

    std::vector<int> MeshAnalyzer::solve_tsp(
            const std::vector<Viewpoint>& viewpoints,
            const Vector3& start_position) const {
        
        if (viewpoints.empty()) return {};

        std::vector<int> order;
        std::vector<bool> visited(viewpoints.size(), false);
        order.reserve(viewpoints.size());

        // Start from nearest viewpoint to start_position
        Vector3 current = start_position;

        for (size_t step = 0; step < viewpoints.size(); ++step) {
            double min_dist = std::numeric_limits<double>::max();
            int nearest = -1;

            for (size_t i = 0; i < viewpoints.size(); ++i) {
                if (visited[i]) continue;
                double dist = (viewpoints[i].position - current).norm();
                if (dist < min_dist) {
                    min_dist = dist;
                    nearest = static_cast<int>(i);
                }
            }

            if (nearest >= 0) {
                order.push_back(nearest);
                visited[nearest] = true;
                current = viewpoints[nearest].position;
            }
        }

        // Compute total tour length
        double total_dist = (viewpoints[order[0]].position - start_position).norm();
        for (size_t i = 1; i < order.size(); ++i) {
            total_dist += (viewpoints[order[i]].position - viewpoints[order[i-1]].position).norm();
        }

        spdlog::info("MeshAnalyzer: TSP tour computed, total distance: {:.2f}m", total_dist);
        return order;
    }

} // namespace sat_sim::planner
