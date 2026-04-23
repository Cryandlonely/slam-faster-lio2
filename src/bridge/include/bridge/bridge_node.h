#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "bridge/tcp_server.h"

#include <memory>
#include <mutex>
#include <string>

namespace bridge {

/// BridgeNode: 对外通讯桥接 ROS2 节点
///
/// 功能:
///   1. TCP 服务器监听外部连接 (JSON Lines 协议)
///   2. 接收外部导航指令 → 转发到 ROS2 话题
///   3. 聚合系统状态 → 推送给外部客户端
///
/// 支持的入站命令 (TCP → ROS2):
///   {"cmd":"nav_goal","x":1.0,"y":2.0}                        → /goal_pose
///   {"cmd":"nav_waypoints","waypoints":[{"x":..,"y":..},...]}  → /nav_waypoints
///   {"cmd":"nav_cancel"}                                       → /nav_cancel
///   {"cmd":"query_status"}                                     → 立即推送状态
///
/// 出站状态推送 (ROS2 → TCP):
///   定时推送聚合后的系统状态 JSON (导航+底盘+SLAM)
class BridgeNode : public rclcpp::Node {
public:
    explicit BridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~BridgeNode() override;

private:
    void DeclareAndLoadParams();

    // TCP 消息处理
    void OnTcpMessage(int client_fd, const std::string& msg);
    void OnTcpConnect(int client_fd, bool connected);
    void OnPcdTcpMessage(int client_fd, const std::string& msg);
    void OnPcdTcpConnect(int client_fd, bool connected);

    // ROS2 回调: 聚合系统状态
    void NavStatusCallback(const std_msgs::msg::String::SharedPtr msg);
    void SlamOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg);
    void BatteryCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // 状态推送定时器
    void StatusBroadcastCallback();

    // 构造聚合状态 JSON
    std::string BuildStatusJson();
    std::string ResolveTransPcdPath() const;

    // TCP 服务器
    std::unique_ptr<TcpServer> tcp_server_;
    std::unique_ptr<TcpServer> pcd_server_;

    // ---- ROS2 订阅 (聚合状态源) ----
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chassis_fb_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;

    // ---- ROS2 发布 (转发外部指令) ----
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_waypoints_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr nav_cancel_pub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ---- 参数 ----
    uint16_t tcp_port_       = 9090;
    uint16_t pcd_tcp_port_   = 9091;
    double   status_rate_    = 5.0;    // Hz
    std::string slam_odom_topic_ = "/slam/odom";
    std::string trans_pcd_rel_path_ = "src/location/PCD/transPCD/trans.pcd";
    int trans_chunk_bytes_ = 65536;

    // ---- 缓存的状态数据 ----
    std::mutex status_mutex_;
    std::string nav_status_json_     = "{}";
    std::string chassis_fb_json_     = "{}";
    double slam_x_ = 0.0, slam_y_ = 0.0, slam_yaw_ = 0.0;
    bool   slam_valid_ = false;
    float  battery_voltage_ = 0.0f;
};

}  // namespace bridge
