/**
 * @file chassis_driver_node.cpp
 * @brief ROS2 底盘驱动节点 - 订阅 /cmd_vel 话题控制机器狗底盘 (zsibot SDK)
 */

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "/usr/local/include/zsibot/include/zsibot_api.h"

using namespace std::chrono_literals;
using namespace zsibot;

// 机器狗控制模式定义
enum class RobotState {
    UNKNOWN = 0,
    PASSIVE = 1,      // 失能/软急停
    LIE_DOWN = 2,     // 趴下
    STAND_UP = 3,     // 站立
    WALKING = 4,      // 行走中
};

class ChassisDriverNode : public rclcpp::Node {
public:
    ChassisDriverNode() : Node("chassis_driver_node") {
        RCLCPP_INFO(this->get_logger(), "=== 底盘驱动节点初始化 ===");
        RCLCPP_INFO(this->get_logger(), "机器狗 IP: %s", robot_ip_.c_str());

        // 初始化机器狗连接
        try {
            executor_ = std::make_shared<ZsibotExecutor>(Role::ROLE_SDK, robot_ip_);
            // 进入手动遥控模式 (需要时通过 /robot_action 发 "sdk_mode" 切换到SDK模式)
            executor_->SetCmd(CmdCode::CMD_REMOTE_CONTROL_RIGHT);
            robot_connected_ = true;
            RCLCPP_INFO(this->get_logger(), "机器狗连接成功! 已进入手动遥控模式, 速度档位: %d",
                static_cast<int>(executor_->GetSpeedLevel()));
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "机器狗连接失败: %s", e.what());
            robot_connected_ = false;
        }

        // 订阅 /cmd_vel 话题
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ChassisDriverNode::CmdVelCallback, this, std::placeholders::_1));

        // 订阅运动方向控制话题 (前进/后退/左右平移/左右旋转/停止)
        move_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_move", 10,
            std::bind(&ChassisDriverNode::MoveCallback, this, std::placeholders::_1));

        // 订阅控制命令话题（用于站立、趴下等动作）
        action_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_action", 10,
            std::bind(&ChassisDriverNode::ActionCallback, this, std::placeholders::_1));

        // 订阅使能话题（用于启用/禁用跟随）
        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot_enable", 10,
            std::bind(&ChassisDriverNode::EnableCallback, this, std::placeholders::_1));

        // 定时器：检查命令超时
        last_cmd_time_ = this->now();
        standup_deadline_ = this->now();  // 初始化：无过渡期
        timeout_timer_ = this->create_wall_timer(
            100ms, std::bind(&ChassisDriverNode::TimeoutCheck, this));

        // 状态发布定时器
        status_timer_ = this->create_wall_timer(
            1000ms, std::bind(&ChassisDriverNode::PublishStatus, this));

        // 状态发布器
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        // 底盘 Odometry 发布器 (提供 yaw 方位 + 速度)
        chassis_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/chassis/odom", 10);

        // 底盘 Odometry 发布定时器 (50Hz)
        odom_timer_ = this->create_wall_timer(
            20ms, std::bind(&ChassisDriverNode::PublishOdom, this));




        // 底盘控制开关服务
        chassis_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/chassis_control",
            std::bind(&ChassisDriverNode::ChassisControlCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "底盘驱动节点启动完成");
        RCLCPP_INFO(this->get_logger(), "=====================================");
        RCLCPP_INFO(this->get_logger(), "使用说明:");
        RCLCPP_INFO(this->get_logger(), "  发送 'stand_up' 到 /robot_action 让机器狗站立");
        RCLCPP_INFO(this->get_logger(), "  发送 'sit_down' 到 /robot_action 让机器狗趴下");
        RCLCPP_INFO(this->get_logger(), "  发送 true 到 /robot_enable 启用运动控制");
        RCLCPP_INFO(this->get_logger(), "  调用 /chassis_control 服务 true/false 打开/关闭底盘连接");
        RCLCPP_INFO(this->get_logger(), "  发送 JSON 到 /robot_move 控制运动方向");
        RCLCPP_INFO(this->get_logger(), "    {\"action\":\"forward\",\"speed\":0.3}");
        RCLCPP_INFO(this->get_logger(), "    action: forward/backward/left/right/rotate_left/rotate_right/stop");
        RCLCPP_INFO(this->get_logger(), "=====================================");


    }

    ~ChassisDriverNode() {
        if (robot_connected_ && executor_) {
            executor_->SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
            RCLCPP_INFO(this->get_logger(), "底盘驱动节点关闭，机器狗已停止");
        }
    }

private:
    // /cmd_vel 回调
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!robot_connected_ || !executor_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "机器狗未连接，忽略速度命令");
            return;
        }

        // 检查是否已使能
        if (!motion_enabled_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "运动控制未使能，发送 true 到 /robot_enable 或 'stand_up' 到 /robot_action");
            return;
        }

        // 检查机器狗是否已站立 (从 SDK 实时读取)
        if (!isRobotStanding()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "机器狗未站立 (ControlMode=%d)，请先发送 'stand_up' 命令到 /robot_action",
                static_cast<int>(executor_->GetControlMode()));
            return;
        }

        // 站立过渡期内不接受速度命令
        if (isInStandupTransition()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "机器狗正在站立中，请稍候...");
            return;
        }

        last_cmd_time_ = this->now();

        // nav_planner 发来的是物理速度 (m/s, rad/s)
        // SetRemote 接收的是比例值 [-1, 1], 实际速度 = 比例值 × 档位最大速度
        // 档位1: max≈1.0m/s, 档位2: max≈2.0m/s, 档位3: max≈3.0m/s
        double scale = getSpeedLevelScale();    
        float vx = std::clamp(static_cast<float>(msg->linear.x / scale), -1.0f, 1.0f);
        float vy = std::clamp(static_cast<float>(msg->linear.y / scale), -1.0f, 1.0f);
        float yaw_rate = std::clamp(static_cast<float>(msg->angular.z / scale), -1.0f, 1.0f);

        // 使用新SDK SetRemote: {vx, vy, yaw_rate, height} (比例值)
        executor_->SetRemote({vx, vy, yaw_rate, 0}, std::array<float32_t, 14>{0});

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "运动命令: 目标vx=%.2f vy=%.2f yaw=%.2f → 比例vx=%.3f vy=%.3f yaw=%.3f (档位缩放:%.1f)",
            msg->linear.x, msg->linear.y, msg->angular.z, vx, vy, yaw_rate, scale);
    }

    // /robot_move 运动方向控制回调
    // JSON 格式: {"action": "forward", "speed": 0.3}
    // action: forward / backward / left / right / rotate_left / rotate_right / stop
    // speed: 可选，默认 0.3 m/s (旋转默认 0.8 rad/s)
    void MoveCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!robot_connected_ || !executor_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "机器狗未连接，忽略运动命令");
            return;
        }
        if (!motion_enabled_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "运动控制未使能");
            return;
        }
        if (!isRobotStanding()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "机器狗未站立 (ControlMode=%d)，请先发送 stand_up",
                static_cast<int>(executor_->GetControlMode()));
            return;
        }
        if (isInStandupTransition()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "机器狗正在站立中，请稍候...");
            return;
        }

        // 解析 JSON
        std::string raw = msg->data;
        std::string action;
        float speed = 0.3f;
        float rotate_speed = 0.8f;

        // 简易 JSON 解析 (避免引入第三方库)
        // 提取 action 字段
        auto extract_string = [&](const std::string& key) -> std::string {
            std::string search = "\"" + key + "\"";
            auto pos = raw.find(search);
            if (pos == std::string::npos) return "";
            pos = raw.find(':', pos);
            if (pos == std::string::npos) return "";
            auto q1 = raw.find('"', pos + 1);
            if (q1 == std::string::npos) return "";
            auto q2 = raw.find('"', q1 + 1);
            if (q2 == std::string::npos) return "";
            return raw.substr(q1 + 1, q2 - q1 - 1);
        };
        auto extract_float = [&](const std::string& key, float def) -> float {
            std::string search = "\"" + key + "\"";
            auto pos = raw.find(search);
            if (pos == std::string::npos) return def;
            pos = raw.find(':', pos);
            if (pos == std::string::npos) return def;
            try { return std::stof(raw.substr(pos + 1)); } catch (...) { return def; }
        };

        action = extract_string("action");
        speed = extract_float("speed", speed);
        rotate_speed = extract_float("rotate_speed", rotate_speed);

        // 速度限幅
        speed = std::clamp(speed, 0.0f, 1.0f);
        rotate_speed = std::clamp(rotate_speed, 0.0f, 1.0f);

        float vx = 0.0f, vy = 0.0f, yaw_rate = 0.0f;

        if (action == "forward") {
            vx = speed;
        } else if (action == "backward") {
            vx = -speed;
        } else if (action == "left") {
            vy = speed;
        } else if (action == "right") {
            vy = -speed;
        } else if (action == "rotate_left") {
            yaw_rate = rotate_speed;
        } else if (action == "rotate_right") {
            yaw_rate = -rotate_speed;
        } else if (action == "stop") {
            vx = 0.0f; vy = 0.0f; yaw_rate = 0.0f;
        } else {
            RCLCPP_WARN(this->get_logger(), "未知运动动作: %s", action.c_str());
            return;
        }

        last_cmd_time_ = this->now();
        executor_->SetRemote({vx, vy, yaw_rate, 0}, std::array<float32_t, 14>{0});

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "运动方向命令: %s, vx=%.2f, vy=%.2f, yaw=%.2f",
            action.c_str(), vx, vy, yaw_rate);
    }

    // 使能回调
    void EnableCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        motion_enabled_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "运动控制 %s", motion_enabled_ ? "已使能" : "已禁用");

    }

    // 动作命令回调
    void ActionCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!robot_connected_ || !executor_) {
            RCLCPP_WARN(this->get_logger(), "机器狗未连接，忽略动作命令");
            return;
        }

        std::string action = msg->data;
        RCLCPP_INFO(this->get_logger(), "收到动作命令: %s", action.c_str());

        if (action == "stand_up" || action == "standup") {
            executor_->SetCmd(CmdCode::CMD_STAND_UP);
            robot_standing_ = true;
            motion_enabled_ = true;
            standup_deadline_ = this->now() + rclcpp::Duration(3, 0);  // 3秒过渡期
            last_cmd_time_ = this->now() + rclcpp::Duration(5, 0);    // 延后超时起算
            RCLCPP_INFO(this->get_logger(), "机器狗站立中 (3秒过渡期)，运动控制已自动使能");
        } else if (action == "sit_down" || action == "sitdown" || action == "lie_down" || action == "liedown") {
            executor_->SetCmd(CmdCode::CMD_SIT_DOWN);
            robot_standing_ = false;
            motion_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "机器狗趴下，运动控制已禁用");
        } else if (action == "passive" || action == "stop" || action == "emergency_stop") {
            executor_->SetCmd(CmdCode::CMD_EMERGENCY_STOP);
            robot_standing_ = false;
            motion_enabled_ = false;
            RCLCPP_INFO(this->get_logger(), "机器狗软急停");
        } else if (action == "jump") {
            if (isRobotStanding()) executor_->SetCmd(CmdCode::CMD_JUMP);
        } else if (action == "front_jump" || action == "forward_jump") {
            if (isRobotStanding()) executor_->SetCmd(CmdCode::CMD_FORWARD_JUMP);
        } else if (action == "backflip" || action == "back_flip") {
            if (isRobotStanding()) executor_->SetCmd(CmdCode::CMD_BACK_FLIP);
        } else if (action == "shake_hand" || action == "greet") {
            if (isRobotStanding()) executor_->SetCmd(CmdCode::CMD_GREET);
        } else if (action == "two_leg_stand") {
            if (isRobotStanding()) executor_->SetCmd(CmdCode::CMD_TWO_LEG_STAND);
        } else if (action == "balance_stand") {
            executor_->SetCmd(CmdCode::CMD_BALANCE_STAND_MODE);
        } else if (action == "enter_lab") {
            executor_->SetCmd(CmdCode::CMD_ENTER_LAB_MODE);
        } else if (action == "exit_lab") {
            executor_->SetCmd(CmdCode::CMD_EXIT_LAB_MODE);
        } else if (action == "unload_squat") {
            executor_->SetCmd(CmdCode::CMD_UNLOAD_SQUAT);
        } else if (action == "sdk_mode") {
            // 1. 先发零速，确保机器狗静止
            executor_->SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
            // 2. 切换到SDK模式
            executor_->SetCmd(CmdCode::CMD_SDK_CONTROL_RIGHT);
            RCLCPP_INFO(this->get_logger(), "已切换到SDK模式, 当前速度档位: %d",
                static_cast<int>(executor_->GetSpeedLevel()));
            // 3. 延迟500ms后：设置行走步态（需零速）
            speed_level_timer_ = this->create_wall_timer(
                500ms, [this]() {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (robot_connected_ && executor_) {
                        executor_->SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
                        executor_->SetCmd(CmdCode::CMD_GENERAL_SDK_CONTROL_RIGHT);
                        RCLCPP_INFO(this->get_logger(),
                            "已设置行走步态, 500ms后切换速度档位...");
                    }
                    speed_level_timer_.reset();

                    // 4. 再延迟500ms后：设置速度档位 (SDK需要时间处理步态切换)
                    speed_level_timer2_ = this->create_wall_timer(
                        500ms, [this]() {
                            std::lock_guard<std::mutex> lock(mutex_);
                            if (robot_connected_ && executor_) {
                                executor_->SetCmd(CmdCode::CMD_NORMAL_SPEED);
                                RCLCPP_INFO(this->get_logger(),
                                    "已设置速度档位: NORMAL, 反馈档位: %d",
                                    static_cast<int>(executor_->GetSpeedLevel()));
                            }
                            speed_level_timer2_.reset();
                        });
                });
        } else if (action == "remote_mode") {
            //executor_->SetCmd(CmdCode::CMD_SLOW_SPEED);
            executor_->SetCmd(CmdCode::CMD_REMOTE_CONTROL_RIGHT);
        } else {
            RCLCPP_WARN(this->get_logger(), "未知动作命令: %s", action.c_str());
        }
    }

    // 底盘控制开关服务回调
    void ChassisControlCallback(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (request->data) {
            // 打开底盘控制：建立连接
            if (robot_connected_ && executor_) {
                response->success = true;
                response->message = "底盘已处于连接状态";
                RCLCPP_INFO(this->get_logger(), "底盘已处于连接状态，无需重复连接");
                return;
            }

            try {
                executor_ = std::make_shared<ZsibotExecutor>(Role::ROLE_SDK, robot_ip_);
                executor_->SetCmd(CmdCode::CMD_REMOTE_CONTROL_RIGHT);
                robot_connected_ = true;
                response->success = true;
                response->message = "底盘连接成功";
                RCLCPP_INFO(this->get_logger(), "底盘连接已建立，已进入手动遥控模式");

            } catch (const std::exception& e) {
                robot_connected_ = false;
                executor_.reset();
                response->success = false;
                response->message = std::string("底盘连接失败: ") + e.what();
                RCLCPP_ERROR(this->get_logger(), "底盘连接失败: %s", e.what());
            }
        } else {
            // 关闭底盘控制：断开连接
            if (!robot_connected_ || !executor_) {
                response->success = true;
                response->message = "底盘已处于断开状态";
                RCLCPP_INFO(this->get_logger(), "底盘已处于断开状态");
                return;
            }

            // 先停止运动
            executor_->SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
            // 趴下后断开
            executor_->SetCmd(CmdCode::CMD_SIT_DOWN);

            executor_.reset();
            robot_connected_ = false;
            robot_standing_ = false;
            motion_enabled_ = false;

            response->success = true;
            response->message = "底盘连接已断开";
            RCLCPP_INFO(this->get_logger(), "底盘连接已断开");
        }
    }

    // 从 SDK 实时查询机器狗是否站立 (非纯软件标记)
    bool isRobotStanding() {
        if (!robot_connected_ || !executor_) return false;
        auto mode = executor_->GetControlMode();
        // CM_STAND_UP(1), CM_MOVE_MODE(5), CM_BALANCE_STAND_MODE(6) 都属于站立状态
        return (mode == ControlMode::CM_STAND_UP ||
                mode == ControlMode::CM_MOVE_MODE ||
                mode == ControlMode::CM_BALANCE_STAND_MODE);
    }

    // 获取当前速度档位的缩放因子
    // SetRemote 接收比例值 [-1,1], 实际物理速度 = 比例值 × 档位最大速度
    // 将物理速度(m/s)转换为比例值时: 比例值 = 物理速度 / 缩放因子
    double getSpeedLevelScale() {
        if (!robot_connected_ || !executor_) return 1.0;
        int level = static_cast<int>(executor_->GetSpeedLevel());
        switch (level) {
            case 1: return 1.0;   // 慢速档: max ≈ 1.0 m/s
            case 2: return 2.0;   // 正常档: max ≈ 2.0 m/s
            case 3: return 3.0;   // 高速档: max ≈ 3.0 m/s
            default: return 1.0;
        }
    }

    // 检查是否在站立过渡期 (standUp 后需要等待机器狗完成动作)
    bool isInStandupTransition() {
        return this->now() < standup_deadline_;
    }

    // 超时检查
    void TimeoutCheck() {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!robot_connected_ || !executor_ || !motion_enabled_ || !isRobotStanding()) return;
        if (isInStandupTransition()) return;  // 站立过渡期，不发move命令

        auto elapsed = this->now() - last_cmd_time_;
        if (elapsed.seconds() > cmd_timeout_) {
            executor_->SetRemote({0, 0, 0, 0}, std::array<float32_t, 14>{0});
        }
    }


    // 发布底盘 Odometry (yaw 方位 + 速度)
    // yaw 约定: 正东=0, 逆时针为正(0~π), 顺时针为负(0~-π)
    void PublishOdom() {
        if (!robot_connected_ || !executor_) return;

        auto rpy = executor_->GetRPY();
        SpeedInfo speed = executor_->GetSpeed();
        double yaw = static_cast<double>(rpy[2]);

        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id  = "base_link";

        // 位置不由底盘提供 (由 RTK 提供)
        odom_msg.pose.pose.position.x = 0.0;
        odom_msg.pose.pose.position.y = 0.0;
        odom_msg.pose.pose.position.z = 0.0;

        // yaw → 四元数 (仅绕 Z 轴旋转)
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        odom_msg.pose.pose.orientation.w = cy;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = sy;

        // 速度
        odom_msg.twist.twist.linear.x  = static_cast<double>(speed.speed);
        odom_msg.twist.twist.angular.z = static_cast<double>(speed.angle_speed);

        chassis_odom_pub_->publish(odom_msg);
    }

    // 发布状态
    void PublishStatus() {
        if (!robot_connected_ || !executor_) {
            auto status_msg = std_msgs::msg::String();
            status_msg.data = "{\"status\":\"disconnected\"}";
            status_pub_->publish(status_msg);
            return;
        }

        uint32_t battery = executor_->GetPower();
        int ctrl_mode = static_cast<int>(executor_->GetControlMode());
        int speed_level = static_cast<int>(executor_->GetSpeedLevel());
        SpeedInfo speed = executor_->GetSpeed();
        auto rpy = executor_->GetRPY();

        char buf[512];
        std::snprintf(buf, sizeof(buf),
            "{\"status\":\"connected\""
            ",\"battery\":%u"
            ",\"speed\":%.3f,\"angle_speed\":%.3f"
            ",\"roll\":%.4f,\"pitch\":%.4f,\"yaw\":%.4f"
            ",\"mode\":%d,\"speed_level\":%d,\"standing\":%s,\"enabled\":%s}",
            battery,
            speed.speed, speed.angle_speed,
            rpy[0], rpy[1], rpy[2],
            ctrl_mode, speed_level,
            isRobotStanding() ? "true" : "false",
            motion_enabled_ ? "true" : "false");

        auto status_msg = std_msgs::msg::String();
        status_msg.data = buf;
        status_pub_->publish(status_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
            "电量:%u%% 速度:%.2fm/s 角速度:%.2frad/s RPY:(%.2f,%.2f,%.2f) 模式:%d 档位:%d 站立:%s 使能:%s",
            battery, speed.speed, speed.angle_speed,
            rpy[0], rpy[1], rpy[2], ctrl_mode,
            static_cast<int>(executor_->GetSpeedLevel()),
            isRobotStanding() ? "是" : "否",
            motion_enabled_ ? "是" : "否");
    }

private:
    const std::string robot_ip_ = "192.168.168.168";
    static constexpr double cmd_timeout_ = 0.5;

    std::shared_ptr<ZsibotExecutor> executor_;
    bool robot_connected_ = false;
    bool robot_standing_ = false;   // 机器狗是否站立
    bool motion_enabled_ = false;   // 运动控制是否使能
    rclcpp::Time standup_deadline_;  // 站立过渡期截止时间

    rclcpp::Time last_cmd_time_;
    std::mutex mutex_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr move_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr chassis_control_srv_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr speed_level_timer_;  // 延迟设置速度档位
    rclcpp::TimerBase::SharedPtr speed_level_timer2_; // 延迟设置速度档位(第二步)
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChassisDriverNode>();
    RCLCPP_INFO(node->get_logger(), "底盘驱动节点已启动");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
