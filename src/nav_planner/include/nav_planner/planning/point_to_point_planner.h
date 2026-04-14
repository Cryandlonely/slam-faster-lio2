// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_PLANNING_POINT_TO_POINT_PLANNER_H_
#define NAV_PLANNER_PLANNING_POINT_TO_POINT_PLANNER_H_

#include "nav_planner/common/types.h"

#include <vector>

namespace slam_nav {

/// 规划参数 (默认值针对室内SLAM导航优化)
struct PlannerParams {
    double path_resolution   = 0.05;  // 路径点间距 (m), 室内精度更高可更密
    double target_speed      = 0.5;   // 默认目标速度 (m/s), 室内安全速度
    double slow_down_dist    = 0.5;   // 减速区距离 (m)
    double min_speed         = 0.05;  // 减速区最低速度 (m/s)
};

/// 点到点路径规划器
///
/// 输入: 当前 Pose2D (从 SLAM Odometry) + 目标 Pose2D (从 goal_pose)
/// 输出: 均匀插值路径点序列 (std::vector<Waypoint>)
class PointToPointPlanner {
public:
    PointToPointPlanner() = default;
    ~PointToPointPlanner() = default;

    void SetParams(const PlannerParams& params);
    const PlannerParams& GetParams() const { return params_; }

    /// 执行规划
    bool Plan(const Pose2D& current, const Pose2D& goal, bool is_final = true);

    /// 多航点一次性规划
    bool PlanMulti(const Pose2D& current, const std::vector<Pose2D>& waypoints);

    const std::vector<Waypoint>& GetPath() const;
    bool HasValidPath() const;
    void ClearPath();

private:
    PlannerParams params_;
    std::vector<Waypoint> path_;
    bool valid_ = false;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_PLANNING_POINT_TO_POINT_PLANNER_H_
