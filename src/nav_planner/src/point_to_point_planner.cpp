// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/planning/point_to_point_planner.h"
#include "nav_planner/common/math_utils.h"

#include <cmath>
#include <algorithm>

namespace slam_nav {

void PointToPointPlanner::SetParams(const PlannerParams& params) {
    params_ = params;
}

bool PointToPointPlanner::Plan(const Pose2D& current, const Pose2D& goal, bool is_final) {
    path_.clear();
    valid_ = false;

    double total_dist = Distance(current.x, current.y, goal.x, goal.y);

    if (total_dist < params_.path_resolution) {
        Waypoint wp;
        wp.x = goal.x;
        wp.y = goal.y;
        wp.yaw = goal.yaw;
        wp.target_speed = 0.0;
        path_.push_back(wp);
        valid_ = true;
        return true;
    }

    double bearing = Azimuth(current.x, current.y, goal.x, goal.y);

    int num_points = static_cast<int>(std::ceil(total_dist / params_.path_resolution));
    num_points = std::max(num_points, 2);

    path_.reserve(num_points + 1);

    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(num_points);
        Waypoint wp;
        wp.x = Lerp(current.x, goal.x, t);
        wp.y = Lerp(current.y, goal.y, t);

        wp.yaw = (i < num_points) ? bearing : goal.yaw;

        if (is_final) {
            double dist_to_end = total_dist * (1.0 - t);
            if (dist_to_end < params_.slow_down_dist && params_.slow_down_dist > kEpsilon) {
                double ratio = dist_to_end / params_.slow_down_dist;
                wp.target_speed = params_.min_speed +
                                  ratio * (params_.target_speed - params_.min_speed);
            } else {
                wp.target_speed = params_.target_speed;
            }
        } else {
            wp.target_speed = params_.target_speed;
        }

        path_.push_back(wp);
    }

    if (!path_.empty()) {
        path_.back().yaw = goal.yaw;
    }

    valid_ = true;
    return true;
}

const std::vector<Waypoint>& PointToPointPlanner::GetPath() const {
    return path_;
}

bool PointToPointPlanner::HasValidPath() const {
    return valid_ && !path_.empty();
}

void PointToPointPlanner::ClearPath() {
    path_.clear();
    valid_ = false;
}

bool PointToPointPlanner::PlanMulti(const Pose2D& current,
                                     const std::vector<Pose2D>& waypoints) {
    path_.clear();
    valid_ = false;

    if (waypoints.empty()) return false;

    if (waypoints.size() == 1) {
        return Plan(current, waypoints[0], true);
    }

    double total_dist = 0.0;
    {
        double px = current.x, py = current.y;
        for (const auto& wp : waypoints) {
            total_dist += Distance(px, py, wp.x, wp.y);
            px = wp.x;
            py = wp.y;
        }
    }

    double accumulated_dist = 0.0;
    Pose2D seg_start = current;

    for (size_t seg = 0; seg < waypoints.size(); ++seg) {
        const Pose2D& seg_end = waypoints[seg];
        bool is_last_seg = (seg + 1 == waypoints.size());

        double seg_dist = Distance(seg_start.x, seg_start.y, seg_end.x, seg_end.y);

        if (seg_dist < params_.path_resolution) {
            Waypoint wp;
            wp.x = seg_end.x;
            wp.y = seg_end.y;
            wp.yaw = seg_end.yaw;
            wp.target_speed = is_last_seg ? 0.0 : params_.target_speed;
            path_.push_back(wp);
            accumulated_dist += seg_dist;
            seg_start = seg_end;
            continue;
        }

        double bearing = Azimuth(seg_start.x, seg_start.y, seg_end.x, seg_end.y);

        int num_points = static_cast<int>(std::ceil(seg_dist / params_.path_resolution));
        num_points = std::max(num_points, 2);

        int start_i = (seg == 0) ? 0 : 1;

        for (int i = start_i; i <= num_points; ++i) {
            double t = static_cast<double>(i) / static_cast<double>(num_points);
            Waypoint wp;
            wp.x = Lerp(seg_start.x, seg_end.x, t);
            wp.y = Lerp(seg_start.y, seg_end.y, t);

            wp.yaw = (i < num_points) ? bearing : seg_end.yaw;

            double dist_from_start = accumulated_dist + seg_dist * t;
            double dist_to_end = total_dist - dist_from_start;

            if (is_last_seg && dist_to_end < params_.slow_down_dist &&
                params_.slow_down_dist > kEpsilon) {
                double ratio = dist_to_end / params_.slow_down_dist;
                wp.target_speed = params_.min_speed +
                                  ratio * (params_.target_speed - params_.min_speed);
            } else {
                wp.target_speed = params_.target_speed;
            }

            path_.push_back(wp);
        }

        accumulated_dist += seg_dist;
        seg_start = seg_end;
    }

    valid_ = !path_.empty();
    return valid_;
}

}  // namespace slam_nav
