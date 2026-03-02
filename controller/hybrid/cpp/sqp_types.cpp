/**
 * @file sqp_types.cpp
 * @brief Implementation of path data utilities for V2 MPC.
 */

#include "sqp_types.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace satellite_control {
namespace v2 {

namespace {

Vector3d robust_tangent_from_index(
    const PathData& path,
    int idx,
    double s_query,
    bool* used_fallback
) {
    if (used_fallback) {
        *used_fallback = false;
    }
    const int n = static_cast<int>(path.points.size());
    if (n < 2) {
        return Vector3d::UnitX();
    }
    idx = std::max(0, std::min(idx, n - 2));

    Vector3d seg = path.points[idx + 1] - path.points[idx];
    double seg_norm = seg.norm();
    if (seg_norm > 1e-12) {
        return seg / seg_norm;
    }

    if (used_fallback) {
        *used_fallback = true;
    }
    for (int j = idx + 1; j + 1 < n; ++j) {
        seg = path.points[j + 1] - path.points[j];
        seg_norm = seg.norm();
        if (seg_norm > 1e-12) {
            return seg / seg_norm;
        }
    }
    for (int j = idx - 1; j >= 0; --j) {
        seg = path.points[j + 1] - path.points[j];
        seg_norm = seg.norm();
        if (seg_norm > 1e-12) {
            return seg / seg_norm;
        }
    }

    std::cerr << "[PathData] WARNING: all path segments near s="
              << s_query << " are degenerate (<1e-12 m); "
              << "falling back to UnitX tangent. Check path waypoints.\n";
    return Vector3d::UnitX();
}

}  // namespace

// ---------------------------------------------------------------------------
// PathData implementation
// ---------------------------------------------------------------------------

Vector3d PathData::get_point(double s_query) const {
    if (!valid || points.empty()) return Vector3d::Zero();
    s_query = clamp_s(s_query);

    // Binary search for segment
    auto it = std::upper_bound(s.begin(), s.end(), s_query);
    int idx = static_cast<int>(it - s.begin()) - 1;
    idx = std::max(0, std::min(idx, static_cast<int>(points.size()) - 2));

    double s0 = s[idx];
    double s1 = s[idx + 1];
    double seg_len = s1 - s0;
    if (seg_len < 1e-12) return points[idx];

    double alpha = (s_query - s0) / seg_len;
    return points[idx] + alpha * (points[idx + 1] - points[idx]);
}

Vector3d PathData::get_tangent(double s_query) const {
    if (!valid || points.size() < 2) return Vector3d::UnitX();
    s_query = clamp_s(s_query);

    auto it = std::upper_bound(s.begin(), s.end(), s_query);
    int idx = static_cast<int>(it - s.begin()) - 1;
    idx = std::max(0, std::min(idx, static_cast<int>(points.size()) - 2));
    bool used_fallback = false;
    Vector3d tangent = robust_tangent_from_index(*this, idx, s_query, &used_fallback);
    if (used_fallback) {
        ++degenerate_fallback_count;
    }
    return tangent;
}

Vector3d PathData::get_tangent_lookahead(
    double s_query,
    double lookahead_m,
    double lookback_m
) const {
    if (!valid || points.size() < 2) return Vector3d::UnitX();
    s_query = clamp_s(s_query);
    const double lookahead = std::max(0.0, lookahead_m);
    const double lookback = std::max(0.0, lookback_m);
    double s_lo = clamp_s(s_query - lookback);
    double s_hi = clamp_s(s_query + lookahead);
    if (s_hi < s_lo) {
        std::swap(s_hi, s_lo);
    }

    if (std::abs(s_hi - s_lo) > 1e-12) {
        Vector3d sec = get_point(s_hi) - get_point(s_lo);
        double sec_norm = sec.norm();
        if (sec_norm > 1e-9) {
            return sec / sec_norm;
        }
    }
    return get_tangent(s_query);
}

double PathData::clamp_s(double s_query) const {
    return std::max(0.0, std::min(s_query, total_length));
}

std::tuple<double, Vector3d, double, double> PathData::project(
    const Vector3d& pos
) const {
    if (!valid || points.size() < 2) {
        return {0.0, Vector3d::Zero(), std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()};
    }

    double min_dist = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    Vector3d best_point = points[0];

    for (size_t i = 0; i + 1 < points.size(); ++i) {
        Vector3d seg = points[i + 1] - points[i];
        double seg_len2 = seg.squaredNorm();
        double t;
        Vector3d proj;

        if (seg_len2 < 1e-24) {
            t = 0.0;
            proj = points[i];
        } else {
            t = (pos - points[i]).dot(seg) / seg_len2;
            t = std::max(0.0, std::min(1.0, t));
            proj = points[i] + t * seg;
        }

        double dist = (pos - proj).norm();
        if (dist < min_dist) {
            min_dist = dist;
            best_point = proj;
            best_s = s[i] + t * (s[i + 1] - s[i]);
        }
    }

    best_s = std::max(0.0, std::min(best_s, total_length));
    double endpoint_error = (pos - points.back()).norm();

    return {best_s, best_point, min_dist, endpoint_error};
}

}  // namespace v2
}  // namespace satellite_control
