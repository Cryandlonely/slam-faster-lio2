// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/common/localization.h"
#include <rclcpp/rclcpp.hpp>

namespace slam_nav {

SlamLocalizationProvider::SlamLocalizationProvider(rclcpp::Node* node)
{
    // 基础初始化，具体与ROS2订阅集成在slam_nav_node.cpp中
    (void)node;  // 避免未使用警告
}

std::optional<Pose2D> SlamLocalizationProvider::GetCurrentPose() const
{
    if (!is_ready_) return std::nullopt;
    return current_pose_;
}

std::optional<double> SlamLocalizationProvider::GetCurrentHeading() const
{
    if (!is_ready_) return std::nullopt;
    return current_pose_.yaw;
}

bool SlamLocalizationProvider::IsReady() const
{
    return is_ready_;
}

double SlamLocalizationProvider::GetTimestamp() const
{
    return timestamp_;
}

void SlamLocalizationProvider::SetPose(const Pose2D& pose, double timestamp)
{
    current_pose_ = pose;
    timestamp_ = timestamp;
    is_ready_ = true;
}

}  // namespace slam_nav
