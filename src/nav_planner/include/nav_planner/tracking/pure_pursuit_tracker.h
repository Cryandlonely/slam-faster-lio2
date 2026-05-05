// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_TRACKING_PURE_PURSUIT_TRACKER_H_
#define NAV_PLANNER_TRACKING_PURE_PURSUIT_TRACKER_H_

#include "nav_planner/common/types.h"

#include <vector>

namespace slam_nav {

/// 跟踪参数 (默认值针对 GPS/RTK 精度优化，源自 207_ws)
struct TrackerParams {
    double lookahead_distance   = 1.2;   // 前视距离 (m)
    double min_lookahead        = 0.3;   // 最小前视距离 (m)
    double max_lookahead        = 5.0;   // 最大前视距离 (m)
    double lookahead_gain       = 1.2;   // 前视距离速度增益
    double goal_tolerance       = 0.5;   // 到达判定阈值 (m), GPS精度适配
    double heading_tolerance    = 0.15;  // 航向到达阈值 (rad, ~8.6°), yaml 以度输入由节点换算
    double max_linear_x         = 1.5;   // 最大前进速度 (m/s)
    double max_linear_y         = 1.5;   // 最大横移速度 (m/s)
    double max_angular_velocity = 1.0;   // 最大角速度 (rad/s)
    double heading_kp           = 1.5;   // 航向 P 控制增益
    double cte_kp               = 0.5;   // 横向纠偏增益
    double cte_dead_zone        = 0.3;   // CTE 死区 (m); 应 ≥ 定位精度, GPS 建议 0.3m
    double cmd_filter_alpha     = 0.3;   // 低通滤波系数
    double heading_align_threshold = 0.785;  // 航向对齐阈值 (rad, ~45°), yaml 以度输入由节点换算
};

/// Pure Pursuit 轨迹跟踪器 (万向轮底盘适配)
///
/// 输入: 当前 Pose2D (SLAM里程计) + 规划路径 (vector<Waypoint>)
/// 输出: OmniControlCmd (vx, vy, yaw_rate)
class PurePursuitTracker {
public:
    PurePursuitTracker() = default;
    ~PurePursuitTracker() = default;

    void SetParams(const TrackerParams& params);
    void SetPath(const std::vector<Waypoint>& path, bool is_final = true);
    bool ComputeControl(const Pose2D& current, OmniControlCmd& cmd);
    bool IsGoalReached() const;
    TrackingDebugInfo GetDebugInfo() const;
    void Reset();

private:
    int FindLookaheadPoint(const Pose2D& current, double lookahead_dist) const;

    TrackerParams params_;
    std::vector<Waypoint> path_;
    int nearest_index_ = 0;
    bool goal_reached_ = false;
    bool position_reached_ = false;
    bool is_final_segment_ = true;
    TrackingDebugInfo debug_info_;

    double prev_vx_ = 0.0;
    double prev_vy_ = 0.0;
    double prev_yaw_rate_ = 0.0;
    double prev_speed_ = 0.0;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_TRACKING_PURE_PURSUIT_TRACKER_H_
