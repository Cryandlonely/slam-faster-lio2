// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/slam_nav_node.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<slam_nav::SlamNavNode>();
    RCLCPP_INFO(node->get_logger(), "SLAM 导航节点已启动");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
