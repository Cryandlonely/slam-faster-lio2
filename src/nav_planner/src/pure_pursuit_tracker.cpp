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
        goal_reached_ = true;
        return false;
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

    // ---- 4b. 航向对齐阶段 (仅在起步/静止时触发, 行进中不停车转向) ----
    {
        double desired_yaw_pre = std::atan2(dy_world, dx_world);
        double heading_err_pre = AngleDiff(desired_yaw_pre, current.yaw);

        if (std::abs(heading_err_pre) > params_.heading_align_threshold
            && prev_speed_ < 0.1)  // 只在静止时原地旋转, 运动中靠 yaw_rate 连续转向
        {
            debug_info_.heading_error = heading_err_pre;

            // 履带车原地转向: vx/vy 强制为 0, 不走低通滤波 (防止前冲)
            cmd.vx = 0.0;
            cmd.vy = 0.0;
            cmd.yaw_rate = params_.heading_kp * heading_err_pre;
            cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

            double alpha = params_.cmd_filter_alpha;
            cmd.yaw_rate = alpha * cmd.yaw_rate + (1.0 - alpha) * prev_yaw_rate_;

            prev_vx_ = 0.0;
            prev_vy_ = 0.0;
            prev_yaw_rate_ = cmd.yaw_rate;
            prev_speed_ = 0.0;
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

    // ---- 6. 速度解算 (履带车只用 vx, vy 强制为 0) ----
    // 提前计算前视方向误差, 供转弯减速使用
    double desired_yaw = std::atan2(dy_world, dx_world);
    double heading_error = AngleDiff(desired_yaw, current.yaw);

    // 6a. 末段减速: 用实时 dist_to_goal 计算, 避免依赖路径点预烘焙的 target_speed
    double target_speed = params_.max_linear_x;
    if (params_.slow_down_dist > kEpsilon && dist_to_goal < params_.slow_down_dist) {
        double ratio = dist_to_goal / params_.slow_down_dist;
        target_speed = params_.min_speed + ratio * (params_.max_linear_x - params_.min_speed);
        target_speed = std::max(target_speed, params_.min_speed);
    }

    // 6b. 转弯减速: heading_error 越大速度越低, 防止履带车高速过弯跑偏
    // 线性插值: 0° → target_speed,  90° → corner_min_speed
    if (params_.corner_min_speed > kEpsilon) {
        double herr_abs = std::abs(heading_error);
        double turn_factor = std::max(0.0, 1.0 - herr_abs / (M_PI / 2.0));
        double corner_speed = params_.corner_min_speed
                              + turn_factor * (target_speed - params_.corner_min_speed);
        target_speed = std::min(target_speed, std::max(corner_speed, params_.corner_min_speed));
    }

    // 6c. 路径预烘焙转角减速上限 (PlanMulti 在拐角前已设置较低的 target_speed)
    // 比反应式 corner_min_speed 更早起效: 车辆尚未"看到"转角方向时就已开始减速
    if (nearest_index_ < static_cast<int>(path_.size())) {
        double baked = path_[nearest_index_].target_speed;
        if (baked > kEpsilon) {
            target_speed = std::min(target_speed, baked);
        }
    }

    // 履带车: 只取前向分量, 横向分量通过 yaw_rate 纠偏
    cmd.vx = target_speed;
    cmd.vy = 0.0;

    // ---- 6c. CTE 纠偏 → 叠加到 yaw_rate (履带车无 vy) ----
    // cte > 0: 机器人在路径左侧 → 需右转 → yaw_rate 为负 → 取反
    double cte_yaw_correction = 0.0;
    if (std::abs(cte) > 0.20) {  // 20cm 死区: GPS/RTK水平精度±0.1~0.5m
        cte_yaw_correction = -params_.cte_kp * cte;
    }

    // ---- 7. 航向控制 + CTE 纠偏 (履带车: 两者都叠加到 yaw_rate) ----
    debug_info_.heading_error = heading_error;

    double adaptive_kp = params_.heading_kp / (1.0 + 0.5 * speed);
    cmd.yaw_rate = adaptive_kp * heading_error + cte_yaw_correction;

    // ---- 8. 限幅 ----
    cmd.vx = Clamp(cmd.vx, params_.max_linear_x);
    cmd.vy = Clamp(cmd.vy, params_.max_linear_y);
    cmd.yaw_rate = Clamp(cmd.yaw_rate, params_.max_angular_velocity);

    // ---- 9. 低通滤波 ----
    double alpha = params_.cmd_filter_alpha;
    cmd.vx       = alpha * cmd.vx       + (1.0 - alpha) * prev_vx_;
    cmd.vy       = alpha * cmd.vy       + (1.0 - alpha) * prev_vy_;
    cmd.yaw_rate = alpha * cmd.yaw_rate + (1.0 - alpha) * prev_yaw_rate_;

    // ---- 10. 加速度限幅 (只限加速, 不限减速 → 允许转弯前瞬间降速) ----
    if (params_.max_accel > 0.0 && params_.control_dt > 0.0) {
        double max_dv = params_.max_accel * params_.control_dt;
        double dvx = cmd.vx - prev_vx_;
        if (dvx > max_dv) {  // 只限制加速, 减速不受限
            cmd.vx = prev_vx_ + max_dv;
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
    // 条件1: 路径方向投影 (原有逻辑)
    // 条件2: 路径点在车身后方 (基于车辆朝向) —— 解决大幅冲过转角时掉头问题
    {
        double cos_h = std::cos(current.yaw);
        double sin_h = std::sin(current.yaw);
        while (nearest_index_ < static_cast<int>(path_.size()) - 1) {
            // 条件1: 路径方向投影 > 0 (机器人已投影过该点)
            double seg_dx = path_[nearest_index_ + 1].x - path_[nearest_index_].x;
            double seg_dy = path_[nearest_index_ + 1].y - path_[nearest_index_].y;
            double to_robot_x = current.x - path_[nearest_index_].x;
            double to_robot_y = current.y - path_[nearest_index_].y;
            bool cond1 = (seg_dx * to_robot_x + seg_dy * to_robot_y > 0.0);

            // 条件2: 路径点在车身后方 (点在车辆朝向的负方向)
            double to_pt_x = path_[nearest_index_].x - current.x;
            double to_pt_y = path_[nearest_index_].y - current.y;
            bool cond2 = (cos_h * to_pt_x + sin_h * to_pt_y < 0.0);

            if (cond1 || cond2) {
                self->nearest_index_++;
            } else {
                break;
            }
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
