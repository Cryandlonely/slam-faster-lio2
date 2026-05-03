// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 改造自 207_ws pure_pursuit_tracker.cpp
// 命名空间: slam_nav (原 nav_planner)
// 功能完全一致, 适用于万向轮底盘的 Pure Pursuit 跟踪

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

        if (!is_final_segment_) {
            goal_reached_ = true;
            return false;
        }

        double heading_error_to_goal = AngleDiff(goal.yaw, current.yaw);
        debug_info_.heading_error = heading_error_to_goal;

        if (std::abs(heading_error_to_goal) < params_.heading_tolerance) {
            goal_reached_ = true;
            return false;
        }

        // 原地旋转到目标 yaw
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
    {
        double desired_yaw_pre = std::atan2(dy_world, dx_world);
        double heading_err_pre = AngleDiff(desired_yaw_pre, current.yaw);

        if (std::abs(heading_err_pre) > params_.heading_align_threshold) {
            debug_info_.heading_error = heading_err_pre;

            cmd.vx = 0.0;
            cmd.vy = 0.0;
            cmd.yaw_rate = params_.heading_kp * heading_err_pre;
            cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

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

    // 世界→机器人坐标系旋转
    double cos_yaw = std::cos(current.yaw);
    double sin_yaw = std::sin(current.yaw);
    double dx_body =  cos_yaw * dx_world + sin_yaw * dy_world;
    double dy_body = -sin_yaw * dx_world + cos_yaw * dy_world;

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

    // ---- 6. 速度解算 ----
    double target_speed = target.target_speed;
    if (target_speed < kEpsilon) {
        target_speed = params_.max_linear_x;
    }

    double dir_norm = std::sqrt(dx_body * dx_body + dy_body * dy_body);
    if (dir_norm > kEpsilon) {
        cmd.vx = target_speed * (dx_body / dir_norm);
        cmd.vy = target_speed * (dy_body / dir_norm);
    }

    // ---- 6b. 横向纠偏 ----
    if (std::abs(cte) > 0.03) {  // 室内死区 3cm (SLAM精度通常cm级)
        int nn = nearest_index_;
        int nn_next = std::min(nn + 1, static_cast<int>(path_.size()) - 1);
        double seg_dx = path_[nn_next].x - path_[nn].x;
        double seg_dy = path_[nn_next].y - path_[nn].y;
        double seg_len = std::sqrt(seg_dx * seg_dx + seg_dy * seg_dy);
        if (seg_len > kEpsilon) {
            double normal_wx = seg_dy / seg_len;
            double normal_wy = -seg_dx / seg_len;
            double corr_wx = params_.cte_kp * cte * normal_wx;
            double corr_wy = params_.cte_kp * cte * normal_wy;
            double corr_bx =  cos_yaw * corr_wx + sin_yaw * corr_wy;
            double corr_by = -sin_yaw * corr_wx + cos_yaw * corr_wy;
            cmd.vx += corr_bx;
            cmd.vy += corr_by;
        }
    }

    // ---- 7. 航向控制 ----
    double desired_yaw = std::atan2(dy_world, dx_world);
    double heading_error = AngleDiff(desired_yaw, current.yaw);
    debug_info_.heading_error = heading_error;

    double adaptive_kp = params_.heading_kp / (1.0 + 0.5 * speed);
    cmd.yaw_rate = adaptive_kp * heading_error;

    // ---- 8. 限幅 ----
    cmd.vx = Clamp(cmd.vx, params_.max_linear_x);
    cmd.vy = Clamp(cmd.vy, params_.max_linear_y);
    cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

    // ---- 9. 低通滤波 ----
    double alpha = params_.cmd_filter_alpha;
    cmd.vx       = alpha * cmd.vx       + (1.0 - alpha) * prev_vx_;
    cmd.vy       = alpha * cmd.vy       + (1.0 - alpha) * prev_vy_;
    cmd.yaw_rate = alpha * cmd.yaw_rate + (1.0 - alpha) * prev_yaw_rate_;

    // ---- 10. 加速度限幅 (防止起步/路径切换时速度突变) ----
    if (params_.max_accel > 0.0 && params_.control_dt > 0.0) {
        double max_dv = params_.max_accel * params_.control_dt;
        double dvx = cmd.vx - prev_vx_;
        double dvy = cmd.vy - prev_vy_;
        double dv  = std::sqrt(dvx * dvx + dvy * dvy);
        if (dv > max_dv) {
            double scale = max_dv / dv;
            cmd.vx = prev_vx_ + dvx * scale;
            cmd.vy = prev_vy_ + dvy * scale;
        }
    }

    prev_vx_ = cmd.vx;
    prev_vy_ = cmd.vy;
    prev_yaw_rate_ = cmd.yaw_rate;
    prev_speed_ = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy);

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

    auto* self = const_cast<PurePursuitTracker*>(this);

    // ---- Step 1: 从当前 nearest_index_ 起找最近路径点 ----
    {
        double min_dist = 1e9;
        for (int i = nearest_index_; i < static_cast<int>(path_.size()); ++i) {
            double d = Distance(current.x, current.y, path_[i].x, path_[i].y);
            if (d < min_dist) {
                min_dist = d;
                self->nearest_index_ = i;
            } else if (d > min_dist + 1.0) {
                break;
            }
        }
    }

    // ---- Step 2: 利用路径段投影推进 nearest_index_ ----
    // 若机器人已冲过某路径点（到下一点的向量与机器人方向同侧），则跳过该点，
    // 避免把身后的路径点当作前视目标，导致掉头行为。
    while (nearest_index_ < static_cast<int>(path_.size()) - 1) {
        double seg_dx = path_[nearest_index_ + 1].x - path_[nearest_index_].x;
        double seg_dy = path_[nearest_index_ + 1].y - path_[nearest_index_].y;
        double to_robot_x = current.x - path_[nearest_index_].x;
        double to_robot_y = current.y - path_[nearest_index_].y;
        // 点积 > 0 表示机器人已投影过该点，向前推进
        if (seg_dx * to_robot_x + seg_dy * to_robot_y > 0.0) {
            self->nearest_index_++;
        } else {
            break;
        }
    }

    // ---- Step 3: 从推进后的 nearest_index_ 起搜索前视点 ----
    int best_idx = -1;
    double best_dist_diff = 1e9;

    for (int i = nearest_index_; i < static_cast<int>(path_.size()); ++i) {
        double d = Distance(current.x, current.y, path_[i].x, path_[i].y);
        double diff = std::abs(d - lookahead_dist);

        if (d >= lookahead_dist * 0.5 && diff < best_dist_diff) {
            best_dist_diff = diff;
            best_idx = i;
        }
    }

    return best_idx;
}

}  // namespace slam_nav
