// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
// 改造自 207_ws nav_planner — 室内SLAM导航版

#ifndef NAV_PLANNER_COMMON_TYPES_H_
#define NAV_PLANNER_COMMON_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

namespace slam_nav {

// ==================== 基础几何类型 ====================

/// 2D 位置点 (局部坐标, 米)
struct Point2D {
    double x = 0.0;  // x轴 (m)
    double y = 0.0;  // y轴 (m)
};

/// 2D 位姿 (位置 + 航向)
struct Pose2D {
    double x   = 0.0;   // x轴 (m)
    double y   = 0.0;   // y轴 (m)
    double yaw = 0.0;   // 航向角 (rad), ENU约定: 正东=0, 逆时针为正
};

/// 路径点 (带航向和速度约束)
struct Waypoint {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;            // 期望航向 (rad)
    double target_speed = 0.0;   // 期望速度 (m/s)
};

// ==================== 控制量 ====================

/// 万向轮底盘中间控制量
struct OmniControlCmd {
    double vx       = 0.0;  // 前进速度 (m/s), 正值前进
    double vy       = 0.0;  // 横移速度 (m/s), 正值左移
    double yaw_rate = 0.0;  // 偏航角速度 (rad/s), 正值逆时针
};

/// SDK 控制消息 (完整字段)
struct SdkControlMsg {
    double  vx          = 0.0;   // m/s, 正值前进
    double  vy          = 0.0;   // m/s, 正值左移
    double  yaw_rate    = 0.0;   // rad/s, 正值逆时针
    int8_t  mode        = 3;     // 默认站立模式 (3=stand)
    double  roll        = 0.0;
    double  pitch       = 0.0;
    double  yaw         = 0.0;
    double  height_rate = 0.0;
};

// ==================== 导航状态 ====================

/// 导航状态枚举
enum class NavState : uint8_t {
    IDLE     = 0,  // 空闲, 等待目标
    PLANNING = 1,  // 正在规划路径
    TRACKING = 2,  // 正在跟踪轨迹
    REACHED  = 3,  // 已到达目标
    ERROR    = 4,  // 异常
};

/// NavState → 字符串
inline std::string NavStateToString(NavState state) {
    switch (state) {
        case NavState::IDLE:     return "IDLE";
        case NavState::PLANNING: return "PLANNING";
        case NavState::TRACKING: return "TRACKING";
        case NavState::REACHED:  return "REACHED";
        case NavState::ERROR:    return "ERROR";
        default:                 return "UNKNOWN";
    }
}

// ==================== 跟踪调试信息 ====================

/// Pure Pursuit 调试信息
struct TrackingDebugInfo {
    double cross_track_error = 0.0;    // 横向跟踪误差 (m)
    double heading_error     = 0.0;    // 航向误差 (rad)
    double lookahead_dist    = 0.0;    // 实际前视距离 (m)
    int    target_index      = -1;     // 当前目标路径点索引
    double distance_to_goal  = 0.0;    // 到终点距离 (m)
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_COMMON_TYPES_H_
