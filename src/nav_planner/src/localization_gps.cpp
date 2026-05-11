// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/common/localization.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace slam_nav {

constexpr double EARTH_RADIUS = 6371000.0;  // 地球半径 (米)
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

GpsLocalizationProvider::GpsLocalizationProvider(rclcpp::Node* node,
                                                   double ref_lat,
                                                   double ref_lon)
    : ref_lat_(ref_lat), ref_lon_(ref_lon),
      current_lat_(0), current_lon_(0), current_alt_(0),
      current_heading_(0)
{
    (void)node;  // 避免未使用警告
}

std::optional<Pose2D> GpsLocalizationProvider::GetCurrentPose() const
{
    if (!is_ready_) return std::nullopt;
    
    double x, y;
    ConvertGpsToLocal(current_lat_, current_lon_, x, y);
    
    Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.yaw = current_heading_;
    pose.yaw_specified = true;
    
    return pose;
}

std::optional<double> GpsLocalizationProvider::GetCurrentHeading() const
{
    if (!is_ready_) return std::nullopt;
    return current_heading_;
}

bool GpsLocalizationProvider::IsReady() const
{
    return is_ready_;
}

double GpsLocalizationProvider::GetTimestamp() const
{
    return timestamp_;
}

void GpsLocalizationProvider::SetGpsData(double lat, double lon, double altitude,
                                          double heading, double timestamp)
{
    current_lat_ = lat;
    current_lon_ = lon;
    current_alt_ = altitude;
    current_heading_ = heading;
    timestamp_ = timestamp;
    is_ready_ = true;
}

void GpsLocalizationProvider::ConvertGpsToLocal(double lat, double lon,
                                                 double& x, double& y) const
{
    // 与 rtk_node 和 slam_nav_node 保持完全一致的简化平面投影公式
    // 使用 111320.0 m/deg (而非球面公式 EARTH_RADIUS * radian_diff)
    // 保证三处坐标系完全对齐
    constexpr double kMetersPerDegreeLat = 111320.0;
    constexpr double kDegToRad = M_PI / 180.0;
    const double meters_per_degree_lon =
        111320.0 * std::cos(ref_lat_ * kDegToRad);

    // X 轴（东西向）：经度差→米
    x = (lon - ref_lon_) * meters_per_degree_lon;

    // Y 轴（南北向）：纬度差→米
    y = (lat - ref_lat_) * kMetersPerDegreeLat;
}

}  // namespace slam_nav
