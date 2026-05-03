// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_COMMON_MATH_UTILS_H_
#define NAV_PLANNER_COMMON_MATH_UTILS_H_

#include <cmath>
#include <algorithm>

namespace slam_nav {

// ==================== 常量 ====================

constexpr double kPi      = M_PI;
constexpr double k2Pi     = 2.0 * M_PI;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kEpsilon  = 1e-9;

// ==================== 角度工具 ====================

/// 将角度归一化到 [-π, π]
inline double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

/// 计算两个角度之间的最短角度差 (结果在 [-π, π))
inline double AngleDiff(double target, double current) {
    return NormalizeAngle(target - current);
}

// ==================== 距离工具 ====================

/// 两点间欧氏距离
inline double Distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/// 两点间方位角 (从 (x1,y1) 到 (x2,y2) 的航向)
inline double Azimuth(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

// ==================== 限幅工具 ====================

/// 对称限幅 [-limit, limit]
inline double Clamp(double value, double limit) {
    return std::clamp(value, -std::abs(limit), std::abs(limit));
}

/// 非对称限幅 [min_val, max_val]
inline double Clamp(double value, double min_val, double max_val) {
    return std::clamp(value, min_val, max_val);
}

// ==================== 四元数工具 ====================

/// yaw → 四元数 (仅绕 Z 轴旋转)
struct Quaternion {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Quaternion YawToQuaternion(double yaw) {
    Quaternion q;
    q.w = std::cos(yaw * 0.5);
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    return q;
}

/// 四元数 → yaw
inline double QuaternionToYaw(double w, double x, double y, double z) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// ==================== 线性插值 ====================

/// 线性插值 t ∈ [0, 1]
inline double Lerp(double a, double b, double t) {
    return a + t * (b - a);
}

}  // namespace slam_nav

#endif  // NAV_PLANNER_COMMON_MATH_UTILS_H_
