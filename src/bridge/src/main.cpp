#include <rclcpp/rclcpp.hpp>
#include "bridge/bridge_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bridge::BridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
