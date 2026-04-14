#include "bridge/bridge_node.h"

#include <cmath>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

namespace bridge {

// 四元数 → yaw
static double quat_to_yaw(double w, double x, double y, double z) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

BridgeNode::BridgeNode(const rclcpp::NodeOptions& options)
    : Node("bridge_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== BridgeNode 初始化 (对外通讯桥接) ===");

    DeclareAndLoadParams();

    // ---- TCP 服务器 ----
    tcp_server_ = std::make_unique<TcpServer>(tcp_port_);
    tcp_server_->setMessageCallback(
        [this](int fd, const std::string& msg) { OnTcpMessage(fd, msg); });
    tcp_server_->setConnectCallback(
        [this](int fd, bool connected) { OnTcpConnect(fd, connected); });

    if (!tcp_server_->start()) {
        RCLCPP_ERROR(this->get_logger(), "TCP 服务器启动失败 (port=%d)", tcp_port_);
        throw std::runtime_error("TCP 服务器启动失败");
    }
    RCLCPP_INFO(this->get_logger(), "TCP 服务器已启动, 端口: %d", tcp_port_);

    // ---- ROS2 订阅 (聚合状态源) ----
    nav_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_status", 10,
        std::bind(&BridgeNode::NavStatusCallback, this, std::placeholders::_1));

    slam_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        slam_odom_topic_, 10,
        std::bind(&BridgeNode::SlamOdomCallback, this, std::placeholders::_1));

    chassis_fb_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chassis/feedback", 10,
        std::bind(&BridgeNode::ChassisFeedbackCallback, this, std::placeholders::_1));

    battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/chassis/battery", 10,
        std::bind(&BridgeNode::BatteryCallback, this, std::placeholders::_1));

    // ---- ROS2 发布 (转发外部指令) ----
    goal_pose_pub_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    nav_waypoints_pub_ = this->create_publisher<std_msgs::msg::String>("/nav_waypoints", 10);
    nav_cancel_pub_   = this->create_publisher<std_msgs::msg::Bool>("/nav_cancel", 10);

    // ---- 状态推送定时器 ----
    auto period = std::chrono::duration<double>(1.0 / status_rate_);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&BridgeNode::StatusBroadcastCallback, this));

    RCLCPP_INFO(this->get_logger(), "BridgeNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  TCP 端口: %d, 状态推送: %.0f Hz", tcp_port_, status_rate_);
    RCLCPP_INFO(this->get_logger(), "  订阅: /nav_status, %s, /chassis/feedback, /chassis/battery",
                slam_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  发布: goal_pose, /nav_waypoints, /nav_cancel");
}

BridgeNode::~BridgeNode() {
    if (tcp_server_) {
        tcp_server_->stop();
    }
}

void BridgeNode::DeclareAndLoadParams() {
    this->declare_parameter<int>("tcp_port", 9090);
    this->declare_parameter<double>("status_rate", 5.0);
    this->declare_parameter<std::string>("slam_odom_topic", "/slam/odom");

    tcp_port_        = static_cast<uint16_t>(this->get_parameter("tcp_port").as_int());
    status_rate_     = this->get_parameter("status_rate").as_double();
    slam_odom_topic_ = this->get_parameter("slam_odom_topic").as_string();
}

// ==================== TCP 消息处理 ====================

void BridgeNode::OnTcpMessage(int client_fd, const std::string& msg) {
    try {
        auto j = nlohmann::json::parse(msg);

        if (!j.contains("cmd") || !j["cmd"].is_string()) {
            std::string err = R"({"error":"missing 'cmd' field"})";
            tcp_server_->sendTo(client_fd, err);
            return;
        }

        std::string cmd = j["cmd"].get<std::string>();

        if (cmd == "nav_goal") {
            // 单点导航: {"cmd":"nav_goal","x":1.0,"y":2.0,"yaw":0.0}
            if (!j.contains("x") || !j.contains("y")) {
                tcp_server_->sendTo(client_fd, R"({"error":"nav_goal requires x,y"})");
                return;
            }

            auto pose_msg = geometry_msgs::msg::PoseStamped();
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.position.x = j["x"].get<double>();
            pose_msg.pose.position.y = j["y"].get<double>();
            pose_msg.pose.position.z = 0.0;

            double yaw = j.value("yaw", 0.0);
            pose_msg.pose.orientation.w = std::cos(yaw / 2.0);
            pose_msg.pose.orientation.z = std::sin(yaw / 2.0);

            goal_pose_pub_->publish(pose_msg);

            RCLCPP_INFO(this->get_logger(), "[TCP] 收到导航目标: x=%.2f y=%.2f",
                        pose_msg.pose.position.x, pose_msg.pose.position.y);
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_goal"})");

        } else if (cmd == "nav_waypoints") {
            // 多航点: {"cmd":"nav_waypoints","waypoints":[...],"target_vel":0.5}
            // 直接转发整个 JSON (去掉 cmd 字段)
            nlohmann::json fwd = j;
            fwd.erase("cmd");
            auto wp_msg = std_msgs::msg::String();
            wp_msg.data = fwd.dump();
            nav_waypoints_pub_->publish(wp_msg);

            RCLCPP_INFO(this->get_logger(), "[TCP] 收到多航点指令");
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_waypoints"})");

        } else if (cmd == "nav_cancel") {
            auto cancel_msg = std_msgs::msg::Bool();
            cancel_msg.data = true;
            nav_cancel_pub_->publish(cancel_msg);

            RCLCPP_INFO(this->get_logger(), "[TCP] 收到取消导航指令");
            tcp_server_->sendTo(client_fd, R"({"ack":"nav_cancel"})");

        } else if (cmd == "query_status") {
            std::string status = BuildStatusJson();
            tcp_server_->sendTo(client_fd, status);

        } else {
            tcp_server_->sendTo(client_fd,
                R"({"error":"unknown cmd","supported":["nav_goal","nav_waypoints","nav_cancel","query_status"]})");
        }

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[TCP] JSON 解析失败: %s", e.what());
        tcp_server_->sendTo(client_fd,
            std::string(R"({"error":"json_parse_error","detail":")") + e.what() + "\"}");
    }
}

void BridgeNode::OnTcpConnect(int client_fd, bool connected) {
    if (connected) {
        RCLCPP_INFO(this->get_logger(), "[TCP] 客户端连接 fd=%d (共 %zu)",
                    client_fd, tcp_server_->clientCount());
    } else {
        RCLCPP_INFO(this->get_logger(), "[TCP] 客户端断开 fd=%d", client_fd);
    }
}

// ==================== ROS2 状态回调 ====================

void BridgeNode::NavStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    nav_status_json_ = msg->data;
}

void BridgeNode::SlamOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    slam_x_   = msg->pose.pose.position.x;
    slam_y_   = msg->pose.pose.position.y;
    slam_yaw_ = quat_to_yaw(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    slam_valid_ = true;
}

void BridgeNode::ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    chassis_fb_json_ = msg->data;
}

void BridgeNode::BatteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    battery_voltage_ = msg->data;
}

// ==================== 状态推送 ====================

void BridgeNode::StatusBroadcastCallback() {
    if (tcp_server_->clientCount() == 0) return;

    std::string status = BuildStatusJson();
    tcp_server_->broadcast(status);
}

std::string BridgeNode::BuildStatusJson() {
    std::lock_guard<std::mutex> lock(status_mutex_);

    nlohmann::json j;
    j["type"] = "status";
    j["timestamp"] = this->now().seconds();

    // SLAM 定位
    j["slam"]["valid"] = slam_valid_;
    j["slam"]["x"]     = slam_x_;
    j["slam"]["y"]     = slam_y_;
    j["slam"]["yaw"]   = slam_yaw_;

    // 导航状态 (直接嵌入)
    try {
        j["nav"] = nlohmann::json::parse(nav_status_json_);
    } catch (...) {
        j["nav"] = nullptr;
    }

    // 底盘反馈
    try {
        j["chassis"] = nlohmann::json::parse(chassis_fb_json_);
    } catch (...) {
        j["chassis"] = nullptr;
    }

    j["battery"] = battery_voltage_;
    j["tcp_clients"] = tcp_server_->clientCount();

    return j.dump();
}

}  // namespace bridge
