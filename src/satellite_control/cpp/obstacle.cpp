#include "obstacle.hpp"
#include <cmath>
#include <limits>

namespace satellite_control {

double Obstacle::signed_distance(const Vector3d& point) const {
    switch (type) {
        case ObstacleType::SPHERE: {
            // Distance from center minus radius
            return (point - position).norm() - radius;
        }
        
        case ObstacleType::CYLINDER: {
            // Project point onto axis, compute distance to axis, subtract radius
            Vector3d to_point = point - position;
            double along_axis = to_point.dot(axis);
            Vector3d perpendicular = to_point - along_axis * axis;
            return perpendicular.norm() - radius;
        }
        
        case ObstacleType::BOX: {
            // Axis-aligned box signed distance
            Vector3d d = (point - position).cwiseAbs() - size;
            Vector3d d_clamped = d.cwiseMax(0.0);
            double outside = d_clamped.norm();
            double inside = std::min(d.maxCoeff(), 0.0);
            return outside + inside;
        }
        
        default:
            return std::numeric_limits<double>::max();
    }
}

Vector3d Obstacle::distance_gradient(const Vector3d& point) const {
    switch (type) {
        case ObstacleType::SPHERE: {
            // Gradient points from obstacle center to query point
            Vector3d diff = point - position;
            double norm = diff.norm();
            if (norm < 1e-10) {
                return Vector3d::UnitX();  // Arbitrary direction if at center
            }
            return diff / norm;
        }
        
        case ObstacleType::CYLINDER: {
            // Gradient is perpendicular to axis
            Vector3d to_point = point - position;
            double along_axis = to_point.dot(axis);
            Vector3d perpendicular = to_point - along_axis * axis;
            double norm = perpendicular.norm();
            if (norm < 1e-10) {
                // On the axis, return arbitrary perpendicular
                Vector3d arbitrary = (std::abs(axis.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
                return axis.cross(arbitrary).normalized();
            }
            return perpendicular / norm;
        }
        
        case ObstacleType::BOX: {
            // Gradient for axis-aligned box
            Vector3d to_center = point - position;
            Vector3d abs_to_center = to_center.cwiseAbs();
            
            // Find which face is closest
            Vector3d extent_diff = abs_to_center - size;
            int max_axis = 0;
            double max_val = extent_diff[0];
            for (int i = 1; i < 3; ++i) {
                if (extent_diff[i] > max_val) {
                    max_val = extent_diff[i];
                    max_axis = i;
                }
            }
            
            Vector3d grad = Vector3d::Zero();
            grad[max_axis] = (to_center[max_axis] >= 0) ? 1.0 : -1.0;
            return grad;
        }
        
        default:
            return Vector3d::UnitX();
    }
}

// ObstacleSet implementation

void ObstacleSet::add(const Obstacle& obs) {
    obstacles_.push_back(obs);
}

void ObstacleSet::clear() {
    obstacles_.clear();
}

double ObstacleSet::min_distance(const Vector3d& point) const {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& obs : obstacles_) {
        double dist = obs.signed_distance(point);
        if (dist < min_dist) {
            min_dist = dist;
        }
    }
    return min_dist;
}

std::vector<std::pair<Vector3d, double>> ObstacleSet::get_linear_constraints(
    const Vector3d& point, double margin) const {
    
    std::vector<std::pair<Vector3d, double>> constraints;
    constraints.reserve(obstacles_.size());
    
    for (const auto& obs : obstacles_) {
        // Linearize: n^T * x >= offset
        // where n is the gradient (pointing away from obstacle)
        // and offset ensures we stay at least (radius + margin) away
        
        Vector3d n = obs.distance_gradient(point);
        
        // The constraint is: n^T * (x - obs.position) >= radius + margin
        // Rearranged: n^T * x >= n^T * obs.position + radius + margin
        double offset = n.dot(obs.position) + obs.radius + margin;
        
        constraints.emplace_back(n, offset);
    }
    
    return constraints;
}

} // namespace satellite_control
