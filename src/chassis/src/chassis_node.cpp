#include "chassis/chassis_node.h"

#include <algorithm>
#include <cmath>
#include <sstream>

using namespace std::chrono_literals;

namespace chassis {

ChassisNode::ChassisNode(const rclcpp::NodeOptions& options)
    : Node("chassis_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== ChassisNode 初始化 (履带车 CAN 控制) ===");

    DeclareAndLoadParams();

    // ---- CAN 初始化 ----
    can_ = std::make_unique<ChassisCAN>(can_interface_);
    if (!can_->open()) {
        RCLCPP_ERROR(this->get_logger(),
            "无法打开 CAN 接口 [%s], 请确认:\n"
            "  sudo ip link set %s type can bitrate 500000\n"
            "  sudo ip link set %s up",
            can_interface_.c_str(), can_interface_.c_str(), can_interface_.c_str());
        throw std::runtime_error("CAN 接口打开失败: " + can_interface_);
    }
    RCLCPP_INFO(this->get_logger(), "CAN 接口 [%s] 已打开", can_interface_.c_str());

    // 注册反馈回调
    can_->setFeedbackCallback(
        [this](const ChassisCAN::FeedbackData& fb) {
            OnChassisFeedback(fb);
        });

    // 启动后台接收
    if (!can_->startReceiving()) {
        RCLCPP_ERROR(this->get_logger(), "CAN 后台接收启动失败");
        throw std::runtime_error("CAN 后台接收启动失败");
    }

    // ---- 订阅 ----
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&ChassisNode::CmdVelCallback, this, std::placeholders::_1));

    // ---- 发布 ----
    imu_pub_          = this->create_publisher<sensor_msgs::msg::Imu>("/chassis/imu", 10);
    battery_pub_      = this->create_publisher<std_msgs::msg::Float32>("/chassis/battery", 10);
    feedback_pub_     = this->create_publisher<std_msgs::msg::String>("/chassis/feedback", 10);
    motor_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/chassis/motor_stopped", 10);

    // ---- 看门狗: cmd_vel 超时急停 ----
    last_cmd_time_ = this->now();
    auto wd_period = std::chrono::duration<double>(watchdog_timeout_ / 2.0);
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(wd_period),
        std::bind(&ChassisNode::WatchdogCallback, this));

    RCLCPP_INFO(this->get_logger(), "ChassisNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  CAN 接口: %s", can_interface_.c_str());
    RCLCPP_INFO(this->get_logger(), "  速度限幅: vx=%.1f vy=%.1f vz=%.1f", max_vx_, max_vy_, max_vz_);
    RCLCPP_INFO(this->get_logger(), "  看门狗超时: %.2f s", watchdog_timeout_);
    RCLCPP_INFO(this->get_logger(), "  订阅: /cmd_vel");
    RCLCPP_INFO(this->get_logger(), "  发布: /chassis/imu, /chassis/battery, /chassis/feedback, /chassis/motor_stopped");
}

ChassisNode::~ChassisNode() {
    if (can_) {
        can_->stop();
        can_->stopReceiving();
        can_->close();
    }
}

void ChassisNode::DeclareAndLoadParams() {
    this->declare_parameter<std::string>("can_interface", "can0");
    this->declare_parameter<double>("max_vx", 1.0);
    this->declare_parameter<double>("max_vy", 0.8);
    this->declare_parameter<double>("max_vz", 1.0);
    this->declare_parameter<double>("watchdog_timeout", 0.5);

    can_interface_    = this->get_parameter("can_interface").as_string();
    max_vx_           = this->get_parameter("max_vx").as_double();
    max_vy_           = this->get_parameter("max_vy").as_double();
    max_vz_           = this->get_parameter("max_vz").as_double();
    watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
}

void ChassisNode::CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    last_cmd_time_ = this->now();
    cmd_received_ = true;

    // 限幅 (m/s, rad/s)
    double vx = std::clamp(msg->linear.x,  -max_vx_, max_vx_);
    double vy = std::clamp(msg->linear.y,  -max_vy_, max_vy_);
    double vz = std::clamp(msg->angular.z, -max_vz_, max_vz_);

    // 单位转换: m/s → mm/s, rad/s → 0.001 rad/s
    auto vx_mm  = static_cast<int16_t>(std::round(vx * 1000.0));
    auto vy_mm  = static_cast<int16_t>(std::round(vy * 1000.0));
    auto vz_mrad = static_cast<int16_t>(std::round(vz * 1000.0));

    if (!can_->setVelocity(vx_mm, vy_mm, vz_mrad)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "CAN 发送失败: vx=%d vy=%d vz=%d", vx_mm, vy_mm, vz_mrad);
    }
}

void ChassisNode::OnChassisFeedback(const ChassisCAN::FeedbackData& fb) {
    // 此回调在 CAN 后台线程中调用, 需要线程安全地发布
    PublishFeedback(fb);
}

void ChassisNode::WatchdogCallback() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!cmd_received_) return;

    auto elapsed = (this->now() - last_cmd_time_).seconds();
    if (elapsed > watchdog_timeout_) {
        can_->stop();
        cmd_received_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "cmd_vel 超时 (%.2fs), 已急停", elapsed);
    }
}

void ChassisNode::PublishFeedback(const ChassisCAN::FeedbackData& fb) {
    // IMU 消息
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";
    imu_msg.linear_acceleration.x = fb.acc_x;
    imu_msg.linear_acceleration.y = fb.acc_y;
    imu_msg.linear_acceleration.z = fb.acc_z;
    imu_msg.angular_velocity.x = fb.gyro_x;
    imu_msg.angular_velocity.y = fb.gyro_y;
    imu_msg.angular_velocity.z = fb.gyro_z;
    // 无姿态估计, 设协方差为 -1 表示未知
    imu_msg.orientation_covariance[0] = -1.0;
    imu_pub_->publish(imu_msg);

    // 电池电压
    auto bat_msg = std_msgs::msg::Float32();
    bat_msg.data = fb.battery_voltage;
    battery_pub_->publish(bat_msg);

    // 电机状态
    auto motor_msg = std_msgs::msg::Bool();
    motor_msg.data = fb.motor_stopped;
    motor_status_pub_->publish(motor_msg);

    // 完整反馈 (JSON)
    auto fb_msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << "{\"vx\":" << fb.vx
        << ",\"vy\":" << fb.vy
        << ",\"vz\":" << fb.vz
        << ",\"acc_x\":" << fb.acc_x
        << ",\"acc_y\":" << fb.acc_y
        << ",\"acc_z\":" << fb.acc_z
        << ",\"gyro_x\":" << fb.gyro_x
        << ",\"gyro_y\":" << fb.gyro_y
        << ",\"gyro_z\":" << fb.gyro_z
        << ",\"battery\":" << fb.battery_voltage
        << ",\"motor_stopped\":" << (fb.motor_stopped ? "true" : "false")
        << "}";
    fb_msg.data = oss.str();
    feedback_pub_->publish(fb_msg);
}

}  // namespace chassis
