#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "chassis/chassis_serial.h"

#include <memory>
#include <mutex>

namespace chassis {

class ChassisNode : public rclcpp::Node {
public:
    explicit ChassisNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ChassisNode() override;

private:
    void DeclareAndLoadParams();

    // 订阅 /cmd_vel 回调: m/s → mm/s, rad/s → 0.001 rad/s
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 底盘反馈回调 (串口后台线程 → ROS2 发布)
    void OnChassisFeedback(const ChassisSerial::FeedbackData& fb);

    // 发布反馈数据
    void PublishFeedback(const ChassisSerial::FeedbackData& fb);

    // 串口底层
    std::unique_ptr<ChassisSerial> serial_;

    // 订阅
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // 发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_status_pub_;

    // 参数
    std::string serial_port_ = "/dev/ttyUSB0";
    double max_vx_     = 1.0;   // m/s
    double max_vy_     = 0.8;   // m/s
    double max_vz_     = 1.0;   // rad/s

    // 状态
    std::mutex mutex_;
};

}  // namespace chassis
