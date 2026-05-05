// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 改造自 207_ws pure_pursuit_tracker.cpp
// 命名空间: slam_nav (原 nav_planner)
// 适用于履带车 (差速驱动): vx 前进, vy=0, yaw_rate 转向

#include "nav_planner/tracking/pure_pursuit_tracker.h"
#include "nav_planner/common/math_utils.h"

#include <algorithm>
#include <cmath>

namespace slam_nav {

void PurePursuitTracker::SetParams(const TrackerParams& params) {
    params_ = params;
}

void PurePursuitTracker::SetPath(const std::vector<Waypoint>& path, bool is_final) {
    path_ = path;
    nearest_index_ = 0;
    goal_reached_ = false;
    position_reached_ = false;
    is_final_segment_ = is_final;
    debug_info_ = TrackingDebugInfo{};
    // 重置低通滤波状态, 防止旧速度命令残留导致新路径起步时方向错误
    prev_vx_ = 0.0;
    prev_vy_ = 0.0;
    prev_yaw_rate_ = 0.0;
    prev_speed_ = 0.0;
}

bool PurePursuitTracker::ComputeControl(const Pose2D& current,
                                         OmniControlCmd& cmd) {
    cmd = OmniControlCmd{};

    if (path_.empty()) {
        return false;
    }

    // ---- 1. 计算到终点距离 ----
    const auto& goal = path_.back();
    double dist_to_goal = Distance(current.x, current.y, goal.x, goal.y);
    debug_info_.distance_to_goal = dist_to_goal;

    // ---- 1b. 位置已到达 ----
    if (position_reached_ || dist_to_goal < params_.goal_tolerance) {
        position_reached_ = true;

        // 中间航点: 位置到达即完成, 不停车不旋转
        if (!is_final_segment_) {
            goal_reached_ = true;
            return false;
        }

        // 最终目标点: 原地旋转到目标 yaw
        double heading_error_to_goal = AngleDiff(goal.yaw, current.yaw);
        debug_info_.heading_error = heading_error_to_goal;

        if (std::abs(heading_error_to_goal) < params_.heading_tolerance) {
            goal_reached_ = true;
            return false;
        }

        cmd.vx = 0.0;
        cmd.vy = 0.0;
        cmd.yaw_rate = params_.heading_kp * heading_error_to_goal;
        cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);
        return true;
    }

    // ---- 2. 动态前视距离 ----
    double speed = prev_speed_;
    double lookahead_dist = params_.lookahead_distance +
                            params_.lookahead_gain * speed;
    lookahead_dist = Clamp(lookahead_dist,
                           params_.min_lookahead,
                           params_.max_lookahead);
    lookahead_dist = std::min(lookahead_dist, dist_to_goal + 0.1);
    debug_info_.lookahead_dist = lookahead_dist;

    // ---- 3. 搜索前视点 ----
    int target_idx = FindLookaheadPoint(current, lookahead_dist);
    if (target_idx < 0) {
        target_idx = static_cast<int>(path_.size()) - 1;
    }
    debug_info_.target_index = target_idx;

    const auto& target = path_[target_idx];

    // ---- 4. 前视点方向 ----
    double dx_world = target.x - current.x;
    double dy_world = target.y - current.y;
    double dist_to_target = std::sqrt(dx_world * dx_world + dy_world * dy_world);

    if (dist_to_target < kEpsilon) {
        return true;
    }

    // ---- 4b. 航向对齐阶段 ----
    // 航向偏差超过阈值时, 先原地旋转对齐方向, 再开始平移运动
    {
        double desired_yaw_pre = std::atan2(dy_world, dx_world);
        double heading_err_pre = AngleDiff(desired_yaw_pre, current.yaw);

        if (std::abs(heading_err_pre) > params_.heading_align_threshold) {
            debug_info_.heading_error = heading_err_pre;

            cmd.vx = 0.0;
            cmd.vy = 0.0;
            cmd.yaw_rate = params_.heading_kp * heading_err_pre;
            cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

            // 低通滤波平滑 (确保平移速度平稳衰减到零)
            double alpha = params_.cmd_filter_alpha;
            cmd.vx       = alpha * cmd.vx       + (1.0 - alpha) * prev_vx_;
            cmd.vy       = alpha * cmd.vy       + (1.0 - alpha) * prev_vy_;
            cmd.yaw_rate = alpha * cmd.yaw_rate + (1.0 - alpha) * prev_yaw_rate_;

            prev_vx_ = cmd.vx;
            prev_vy_ = cmd.vy;
            prev_yaw_rate_ = cmd.yaw_rate;
            prev_speed_ = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy);

            return true;
        }
    }

    // ---- 5. 横向跟踪误差 (CTE) ----
    double cte = 0.0;
    {
        int nn = nearest_index_;
        int nn_next = std::min(nn + 1, static_cast<int>(path_.size()) - 1);
        if (nn < nn_next) {
            double seg_dx = path_[nn_next].x - path_[nn].x;
            double seg_dy = path_[nn_next].y - path_[nn].y;
            double pt_dx = current.x - path_[nn].x;
            double pt_dy = current.y - path_[nn].y;
            double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
            if (seg_len > kEpsilon) {
                cte = (seg_dx * pt_dy - seg_dy * pt_dx) / seg_len;
            }
        }
    }
    debug_info_.cross_track_error = cte;

    // ---- 6. 履带车速度解算 (前进速度 + 偏航角速度) ----
    double target_speed = target.target_speed;
    if (target_speed < kEpsilon) {
        target_speed = params_.max_linear_x;
    }
    cmd.vx = target_speed;  // 前进速度 = 路径点预烘焙速度
    cmd.vy = 0.0;           // 履带车无横向移动

    // ---- 7. 航向控制 + CTE 纠偏叠加到 yaw_rate ----
    double desired_yaw = std::atan2(dy_world, dx_world);
    double heading_error = AngleDiff(desired_yaw, current.yaw);
    debug_info_.heading_error = heading_error;

    double adaptive_kp = params_.heading_kp / (1.0 + 0.5 * speed);
    // CTE → yaw_rate: cte>0 在路径左侧 → 需右转 → 负 yaw_rate
    double cte_yaw = (std::abs(cte) > params_.cte_dead_zone) ? -params_.cte_kp * cte : 0.0;
    cmd.yaw_rate = adaptive_kp * heading_error + cte_yaw;

    // ---- 8. 限幅 ----
    cmd.vx = Clamp(cmd.vx, params_.max_linear_x);
    // cmd.vy 恒为 0, 无需限幅
    cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

    // ---- 9. 一阶低通滤波平滑输出 ----
    double alpha = params_.cmd_filter_alpha;
    cmd.vx       = alpha * cmd.vx       + (1.0 - alpha) * prev_vx_;
    cmd.vy       = 0.0;  // 恒为 0
    cmd.yaw_rate = alpha * cmd.yaw_rate + (1.0 - alpha) * prev_yaw_rate_;

    prev_vx_ = cmd.vx;
    prev_vy_ = 0.0;
    prev_yaw_rate_ = cmd.yaw_rate;
    prev_speed_ = cmd.vx;

    return true;
}

bool PurePursuitTracker::IsGoalReached() const {
    return goal_reached_;
}

TrackingDebugInfo PurePursuitTracker::GetDebugInfo() const {
    return debug_info_;
}

void PurePursuitTracker::Reset() {
    path_.clear();
    nearest_index_ = 0;
    goal_reached_ = false;
    position_reached_ = false;
    debug_info_ = TrackingDebugInfo{};
    prev_vx_ = 0.0;
    prev_vy_ = 0.0;
    prev_yaw_rate_ = 0.0;
    prev_speed_ = 0.0;
}

int PurePursuitTracker::FindLookaheadPoint(const Pose2D& current,
                                            double lookahead_dist) const {
    if (path_.empty()) return -1;

    int best_idx = -1;
    double best_dist_diff = 1e9;

    int start_idx = std::max(0, nearest_index_);

    for (int i = start_idx; i < static_cast<int>(path_.size()); ++i) {
        double d = Distance(current.x, current.y,
                            path_[i].x, path_[i].y);
        double diff = std::abs(d - lookahead_dist);

        if (d >= lookahead_dist * 0.5 && diff < best_dist_diff) {
            best_dist_diff = diff;
            best_idx = i;
        }
    }

    // 更新最近点 (向前不回退)
    if (best_idx >= 0) {
        double min_dist = 1e9;
        for (int i = nearest_index_;
             i < static_cast<int>(path_.size()); ++i) {
            double d = Distance(current.x, current.y,
                                path_[i].x, path_[i].y);
            if (d < min_dist) {
                min_dist = d;
                const_cast<PurePursuitTracker*>(this)->nearest_index_ = i;
            } else if (d > min_dist + 1.0) {
                break;
            }
        }
    }

    return best_idx;
}

}  // namespace slam_nav
