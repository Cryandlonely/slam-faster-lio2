// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/planning/line_planner.h"
#include <cmath>
#include <algorithm>

namespace slam_nav {

LinePlanner::LinePlanner(const Config& config) : config_(config) {}

std::vector<Waypoint> LinePlanner::Plan(const Pose2D& start, const Pose2D& goal,
                                         std::optional<double> target_heading)
{
    std::vector<Waypoint> path;
    
    double dx = goal.x - start.x;
    double dy = goal.y - start.y;
    double distance = Distance(0, 0, dx, dy);
    
    if (distance < 1e-6) {
        // 起点和终点重合
        Waypoint wp;
        wp.x = goal.x;
        wp.y = goal.y;
        wp.yaw = target_heading.value_or(goal.yaw);
        wp.target_speed = 0.0;
        path.push_back(wp);
        return path;
    }
    
    // 计算方向
    double heading = GetHeading(dx, dy);
    
    // 生成路径点
    int num_points = static_cast<int>(std::ceil(distance / config_.waypoint_spacing));
    num_points = std::min(num_points + 1, config_.max_waypoints);
    
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        
        Waypoint wp;
        wp.x = start.x + t * dx;
        wp.y = start.y + t * dy;
        wp.yaw = heading;  // 所有点指向目标方向
        wp.target_speed = 0.5;  // 默认速度（可配置）
        
        path.push_back(wp);
    }
    
    // 末尾点覆盖到目标
    if (!path.empty()) {
        path.back().x = goal.x;
        path.back().y = goal.y;
        path.back().yaw = target_heading.value_or(heading);
        path.back().target_speed = 0.0;  // 到达目标减速
    }
    
    return path;
}

std::vector<Waypoint> LinePlanner::PlanToMultipleGoals(const Pose2D& start,
                                                       const std::vector<Pose2D>& goals)
{
    std::vector<Waypoint> full_path;
    
    if (goals.empty()) return full_path;
    
    Pose2D current = start;
    
    for (const auto& goal : goals) {
        auto segment = Plan(current, goal, goal.yaw_specified ? std::optional(goal.yaw) : std::nullopt);
        
        // 去除重复的起点
        if (!full_path.empty() && !segment.empty()) {
            segment.erase(segment.begin());
        }
        
        full_path.insert(full_path.end(), segment.begin(), segment.end());
        current = goal;
    }
    
    return full_path;
}

double LinePlanner::Distance(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

double LinePlanner::GetHeading(double dx, double dy)
{
    return std::atan2(dy, dx);
}

}  // namespace slam_nav
