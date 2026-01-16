#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>

namespace satellite_control {

using Eigen::Vector3d;

/**
 * @brief Obstacle types for collision avoidance
 */
enum class ObstacleType {
    SPHERE,      // Spherical keep-out zone
    CYLINDER,    // Cylindrical (e.g., solar panel axis)
    BOX          // Box-shaped region
};

/**
 * @brief Single obstacle definition
 */
struct Obstacle {
    ObstacleType type = ObstacleType::SPHERE;
    Vector3d position = Vector3d::Zero();  // Center position [m]
    double radius = 0.5;                    // For sphere/cylinder [m]
    Vector3d size = Vector3d::Ones();       // For box: half-extents [m]
    Vector3d axis = Vector3d::UnitZ();      // For cylinder: axis direction
    std::string name = "obstacle";
    
    /**
     * @brief Compute signed distance from point to obstacle surface
     * @param point Query point [m]
     * @return Signed distance (negative = inside, positive = outside)
     */
    double signed_distance(const Vector3d& point) const;
    
    /**
     * @brief Compute gradient of distance function (points away from obstacle)
     * @param point Query point [m]
     * @return Normalized gradient vector
     */
    Vector3d distance_gradient(const Vector3d& point) const;
};

/**
 * @brief Collection of obstacles for collision checking
 */
class ObstacleSet {
public:
    ObstacleSet() = default;
    
    /**
     * @brief Add an obstacle to the set
     */
    void add(const Obstacle& obs);
    
    /**
     * @brief Clear all obstacles
     */
    void clear();
    
    /**
     * @brief Get minimum signed distance to any obstacle
     * @param point Query point [m]
     * @return Minimum signed distance
     */
    double min_distance(const Vector3d& point) const;
    
    /**
     * @brief Get constraint data for MPC linearization
     * @param point Current position
     * @param margin Safety margin [m]
     * @return Vector of (normal, offset) pairs for linear constraints
     */
    std::vector<std::pair<Vector3d, double>> get_linear_constraints(
        const Vector3d& point, double margin) const;
    
    /**
     * @brief Number of obstacles
     */
    size_t size() const { return obstacles_.size(); }
    
    /**
     * @brief Access individual obstacle
     */
    const Obstacle& operator[](size_t i) const { return obstacles_[i]; }

private:
    std::vector<Obstacle> obstacles_;
};

} // namespace satellite_control
