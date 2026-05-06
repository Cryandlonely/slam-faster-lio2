// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 基于 207_ws nav_planner_node.cpp 重写
// 核心变更:
//   1. 定位源: 单一 /outdoor/odom (RTK+INS 融合, 包含 position + orientation)
//   2. 去除 A*/livox/动态障碍, 仅保留 PointToPointPlanner + PurePursuitTracker
//   3. 状态发布格式适配 bridge_node: {"type":"pose","state":"...","lat":...,"lon":...,"yaw":...}
//   4. 增加 nav_pause 支持
//   5. 航点格式: {coord_mode:"gps", waypoints:[{lat,lon}], target_vel:...}

#include "nav_planner/slam_nav_node.h"
#include "nav_planner/common/math_utils.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace std::chrono_literals;

namespace slam_nav {

SlamNavNode::SlamNavNode(const rclcpp::NodeOptions& options)
    : Node("slam_nav_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== SlamNavNode 初始化 ===");

    DeclareAndLoadParams();

    state_machine_.SetCallback(
        [this](NavState old_s, NavState new_s) {
            OnStateChange(old_s, new_s);
        });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---- 订阅 ----
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SlamNavNode::OdomCallback, this, std::placeholders::_1));

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&SlamNavNode::GoalPoseCallback, this, std::placeholders::_1));

    nav_waypoints_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_waypoints", 10,
        std::bind(&SlamNavNode::NavWaypointsCallback, this, std::placeholders::_1));

    nav_cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_cancel", 10,
        std::bind(&SlamNavNode::NavCancelCallback, this, std::placeholders::_1));

    nav_pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_pause", 10,
        std::bind(&SlamNavNode::NavPauseCallback, this, std::placeholders::_1));

    chassis_feedback_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/chassis/feedback", 10,
        std::bind(&SlamNavNode::ChassisFeedbackCallback, this, std::placeholders::_1));

    // ---- 发布 ----
    path_pub_           = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
    status_pub_         = this->create_publisher<std_msgs::msg::String>("/nav_status", 10);
    current_pose_pub_   = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);
    tracking_debug_pub_ = this->create_publisher<std_msgs::msg::String>("/tracking_debug", 10);
    cmd_vel_pub_        = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    robot_action_pub_   = this->create_publisher<std_msgs::msg::String>("/robot_action", 10);
    robot_enable_pub_   = this->create_publisher<std_msgs::msg::Bool>("/robot_enable", 10);
    actual_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/actual_trajectory", 10);
    goal_marker_pub_    = this->create_publisher<visualization_msgs::msg::Marker>("/goal_marker", 10);
    planned_route_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planned_route_marker", 10);

    actual_trajectory_.header.frame_id = map_frame_;

    // ---- 控制循环定时器 ----
    auto control_period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
        std::bind(&SlamNavNode::ControlLoop, this));

    // ---- 状态发布定时器 ----
    auto status_period = std::chrono::duration<double>(1.0 / status_rate_);
    status_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(status_period),
        [this]() {
            PublishStatus();
            PublishCurrentPose();
            PublishTF();
            PublishActualTrajectory();
            PublishPath();
            PublishGoalMarker();
            PublishPlannedRouteMarker();
        });

    RCLCPP_INFO(this->get_logger(), "SlamNavNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  定位话题: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  参考点: lat=%.6f lon=%.6f", ref_latitude_, ref_longitude_);
    RCLCPP_INFO(this->get_logger(), "  控制频率: %.1f Hz", control_rate_);
}

// ==================== 参数声明与加载 ====================

void SlamNavNode::DeclareAndLoadParams() {
    // -- 定位话题 --
    this->declare_parameter<std::string>("odom_topic", "/outdoor/odom");
    odom_topic_ = this->get_parameter("odom_topic").as_string();

    // -- 参考坐标原点 (用于 GPS ↔ 局部坐标转换) --
    this->declare_parameter<double>("reference.latitude", 36.66111);
    this->declare_parameter<double>("reference.longitude", 117.01665);
    ref_latitude_  = this->get_parameter("reference.latitude").as_double();
    ref_longitude_ = this->get_parameter("reference.longitude").as_double();

    // -- 控制频率 --
    this->declare_parameter<double>("control.rate", 10.0);
    this->declare_parameter<double>("debug.status_rate", 2.0);
    control_rate_ = this->get_parameter("control.rate").as_double();
    status_rate_  = this->get_parameter("debug.status_rate").as_double();

    // -- 坐标系 --
    this->declare_parameter<std::string>("frame.map", "map");
    this->declare_parameter<std::string>("frame.base", "base_link");
    map_frame_  = this->get_parameter("frame.map").as_string();
    base_frame_ = this->get_parameter("frame.base").as_string();

    // -- 规划参数 --
    PlannerParams pp;
    this->declare_parameter<double>("planning.path_resolution", pp.path_resolution);
    this->declare_parameter<double>("planning.target_speed", pp.target_speed);
    this->declare_parameter<double>("planning.slow_down_dist", pp.slow_down_dist);
    this->declare_parameter<double>("planning.min_speed", pp.min_speed);
    pp.path_resolution = this->get_parameter("planning.path_resolution").as_double();
    pp.target_speed    = this->get_parameter("planning.target_speed").as_double();
    pp.slow_down_dist  = this->get_parameter("planning.slow_down_dist").as_double();
    pp.min_speed       = this->get_parameter("planning.min_speed").as_double();
    planner_.SetParams(pp);

    // -- 跟踪参数 --
    TrackerParams tp;
    this->declare_parameter<double>("tracking.lookahead_distance", tp.lookahead_distance);
    this->declare_parameter<double>("tracking.min_lookahead", tp.min_lookahead);
    this->declare_parameter<double>("tracking.max_lookahead", tp.max_lookahead);
    this->declare_parameter<double>("tracking.lookahead_gain", tp.lookahead_gain);
    this->declare_parameter<double>("tracking.goal_tolerance", tp.goal_tolerance);
    this->declare_parameter<double>("tracking.heading_tolerance", tp.heading_tolerance);
    this->declare_parameter<double>("tracking.max_linear_x", tp.max_linear_x);
    this->declare_parameter<double>("tracking.max_linear_y", tp.max_linear_y);
    this->declare_parameter<double>("tracking.max_angular_velocity", tp.max_angular_velocity);
    this->declare_parameter<double>("tracking.heading_kp", tp.heading_kp);
    this->declare_parameter<double>("tracking.cte_kp", tp.cte_kp);
    this->declare_parameter<double>("tracking.cte_dead_zone", tp.cte_dead_zone);
    this->declare_parameter<double>("tracking.cmd_filter_alpha", tp.cmd_filter_alpha);
    this->declare_parameter<double>("tracking.heading_align_threshold", tp.heading_align_threshold);
    tp.lookahead_distance      = this->get_parameter("tracking.lookahead_distance").as_double();
    tp.min_lookahead           = this->get_parameter("tracking.min_lookahead").as_double();
    tp.max_lookahead           = this->get_parameter("tracking.max_lookahead").as_double();
    tp.lookahead_gain          = this->get_parameter("tracking.lookahead_gain").as_double();
    tp.goal_tolerance          = this->get_parameter("tracking.goal_tolerance").as_double();
    tp.heading_tolerance       = this->get_parameter("tracking.heading_tolerance").as_double();
    tp.max_linear_x            = this->get_parameter("tracking.max_linear_x").as_double();
    tp.max_linear_y            = this->get_parameter("tracking.max_linear_y").as_double();
    tp.max_angular_velocity    = this->get_parameter("tracking.max_angular_velocity").as_double();
    tp.heading_kp              = this->get_parameter("tracking.heading_kp").as_double();
    tp.cte_kp                  = this->get_parameter("tracking.cte_kp").as_double();
    tp.cte_dead_zone           = this->get_parameter("tracking.cte_dead_zone").as_double();
    tp.cmd_filter_alpha        = this->get_parameter("tracking.cmd_filter_alpha").as_double();
    tp.heading_align_threshold = this->get_parameter("tracking.heading_align_threshold").as_double();
    // heading_align_threshold 和 heading_tolerance 配置文件以度输入, 此处转为弧度
    tp.heading_align_threshold = tp.heading_align_threshold * kDegToRad;
    tp.heading_tolerance       = tp.heading_tolerance * kDegToRad;
    tracker_.SetParams(tp);

    // -- SDK 接口参数 --
    SdkInterfaceParams sp;
    this->declare_parameter<double>("sdk.max_vx", sp.max_vx);
    this->declare_parameter<double>("sdk.max_vy", sp.max_vy);
    this->declare_parameter<double>("sdk.max_yaw_rate", sp.max_yaw_rate);
    sp.max_vx        = this->get_parameter("sdk.max_vx").as_double();
    sp.max_vy        = this->get_parameter("sdk.max_vy").as_double();
    sp.max_yaw_rate  = this->get_parameter("sdk.max_yaw_rate").as_double();
    sdk_interface_.SetParams(sp);

    RCLCPP_INFO(this->get_logger(),
                "参数加载完成: ref(%.6f, %.6f) rate=%.0f Hz goal_tol=%.2f m",
                ref_latitude_, ref_longitude_, control_rate_, tp.goal_tolerance);
}

// ==================== 坐标转换 ====================

void SlamNavNode::GpsToLocal(double lat, double lon, double& x, double& y) const {
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_lon =
        111320.0 * std::cos(ref_latitude_ * kDegToRad);
    x = (lon - ref_longitude_) * meters_per_lon;
    y = (lat - ref_latitude_)  * kMetersPerDegreeLat;
}

void SlamNavNode::LocalToGps(double x, double y, double& lat, double& lon) const {
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_lon =
        111320.0 * std::cos(ref_latitude_ * kDegToRad);
    lat = ref_latitude_  + y / kMetersPerDegreeLat;
    lon = ref_longitude_ + x / meters_per_lon;
}

// ==================== 回调函数 ====================

void SlamNavNode::OdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_z_      = msg->pose.pose.position.z;

    current_pose_.yaw = QuaternionToYaw(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    odom_received_ = true;
}

void SlamNavNode::GoalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    CancelMultiNav();

    Pose2D goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    if (odom_received_) {
        goal.yaw = Azimuth(current_pose_.x, current_pose_.y, goal.x, goal.y);
    } else {
        goal.yaw = QuaternionToYaw(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
    }

    goal_pose_ = goal;

    RCLCPP_INFO(this->get_logger(),
                "收到目标: x=%.3f y=%.3f yaw=%.2f°",
                goal.x, goal.y, goal.yaw * kRadToDeg);

    state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);
    PublishGoalMarker();
}

void SlamNavNode::NavCancelCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data) return;

    std::lock_guard<std::mutex> lock(data_mutex_);

    RCLCPP_INFO(this->get_logger(), "收到取消导航命令");

    CancelMultiNav();
    goal_pose_.reset();
    state_machine_.HandleEvent(NavEvent::CANCEL);
    tracker_.Reset();
    planner_.ClearPath();
    PublishStopCmd();
}

void SlamNavNode::NavPauseCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    nav_paused_ = msg->data;
    if (nav_paused_) {
        PublishStopCmd();
        RCLCPP_INFO(this->get_logger(), "导航已暂停");
    } else {
        RCLCPP_INFO(this->get_logger(), "导航已继续");
    }
}

void SlamNavNode::ChassisFeedbackCallback(
    const std_msgs::msg::String::SharedPtr msg) {
    // 解析 JSON: {"vx": 0.5, "vy": 0.0, ...}
    // 只取 |vx| 作为实际线速, 无锁直接写入 (double 写入是原子的)
    try {
        auto j = nlohmann::json::parse(msg->data);
        double vx = j.value("vx", 0.0);
        actual_chassis_speed_ = std::abs(vx);
    } catch (...) {
        // 解析失败时保持上一个平切値
    }
}

// ==================== 多航点队列 ====================

void SlamNavNode::NavWaypointsCallback(
    const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    try {
        auto j = nlohmann::json::parse(msg->data);

        if (!j.contains("waypoints") || !j["waypoints"].is_array()) {
            RCLCPP_WARN(this->get_logger(), "收到 /nav_waypoints 但缺少 waypoints 数组");
            return;
        }

        // 读取目标速度
        if (j.contains("target_vel") && j["target_vel"].is_number()) {
            double cmd_speed = j["target_vel"].get<double>();
            if (cmd_speed > 0.0) {
                auto pp = planner_.GetParams();
                pp.target_speed = cmd_speed;
                planner_.SetParams(pp);
                RCLCPP_INFO(this->get_logger(), "使用命令目标速度: %.2f m/s", cmd_speed);
            }
        }

        waypoint_queue_.clear();
        waypoint_index_ = 0;

        for (const auto& wp : j["waypoints"]) {
            Pose2D pose;
            // 支持 lat/lon 格式 (GPS) 和 x/y 格式 (局部坐标)
            if (wp.contains("lat") && wp.contains("lon")) {
                double lat = wp["lat"].get<double>();
                double lon = wp["lon"].get<double>();
                GpsToLocal(lat, lon, pose.x, pose.y);
            } else if (wp.contains("latitude") && wp.contains("longitude")) {
                double lat = wp["latitude"].get<double>();
                double lon = wp["longitude"].get<double>();
                GpsToLocal(lat, lon, pose.x, pose.y);
            } else if (wp.contains("x") && wp.contains("y")) {
                pose.x = wp["x"].get<double>();
                pose.y = wp["y"].get<double>();
            } else {
                RCLCPP_WARN(this->get_logger(), "航点格式未识别, 跳过");
                continue;
            }
            pose.yaw = wp.value("yaw", 0.0);
            waypoint_queue_.push_back(pose);
        }

        if (waypoint_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到 /nav_waypoints 但无有效航点");
            return;
        }

        // 计算航向: 非末尾朝向下一段, 末尾跟上一段
        for (size_t i = 0; i < waypoint_queue_.size(); ++i) {
            if (i + 1 < waypoint_queue_.size()) {
                waypoint_queue_[i].yaw = Azimuth(
                    waypoint_queue_[i].x, waypoint_queue_[i].y,
                    waypoint_queue_[i+1].x, waypoint_queue_[i+1].y);
            } else if (i > 0) {
                waypoint_queue_[i].yaw = Azimuth(
                    waypoint_queue_[i-1].x, waypoint_queue_[i-1].y,
                    waypoint_queue_[i].x, waypoint_queue_[i].y);
            }
        }

        if (waypoint_queue_.size() == 1 && odom_received_) {
            waypoint_queue_[0].yaw = Azimuth(
                current_pose_.x, current_pose_.y,
                waypoint_queue_[0].x, waypoint_queue_[0].y);
        }

        multi_nav_active_ = true;
        waypoint_index_ = 0;
        goal_pose_ = waypoint_queue_.back();

        RCLCPP_INFO(this->get_logger(),
                    "收到多航点任务: %zu 个航点", waypoint_queue_.size());

        state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);
        PublishGoalMarker();

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "/nav_waypoints JSON 解析失败: %s", e.what());
    }
}

// ==================== 控制循环 ====================

void SlamNavNode::ControlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 暂停状态: 不发送控制命令
    if (nav_paused_) return;

    NavState state = state_machine_.GetState();

    switch (state) {
        case NavState::IDLE:
        case NavState::REACHED:
            break;

        case NavState::PLANNING: {
            if (!odom_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "等待 Odometry 数据...");
                return;
            }
            if (!goal_pose_.has_value()) {
                state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                return;
            }

            bool ok = false;
            if (multi_nav_active_ && waypoint_queue_.size() > 1) {
                ok = planner_.PlanMulti(current_pose_, waypoint_queue_);
            } else {
                ok = planner_.Plan(current_pose_, goal_pose_.value(), true);
            }

            if (ok && planner_.HasValidPath()) {
                tracker_.SetPath(planner_.GetPath(), true);
                state_machine_.HandleEvent(NavEvent::PLAN_SUCCESS);
                PublishPath();
                actual_trajectory_.poses.clear();
                last_traj_x_ = current_pose_.x;
                last_traj_y_ = current_pose_.y;
                RCLCPP_INFO(this->get_logger(), "路径规划成功, %zu 个路径点",
                            planner_.GetPath().size());
            } else {
                state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                RCLCPP_ERROR(this->get_logger(), "路径规划失败");
            }
            break;
        }

        case NavState::TRACKING: {
            OmniControlCmd cmd;
            tracker_.SetActualSpeed(actual_chassis_speed_);  // 底盘实际速度优先于指令速度
            bool tracking = tracker_.ComputeControl(current_pose_, cmd);

            if (!tracking) {
                if (tracker_.IsGoalReached()) {
                    if (multi_nav_active_) {
                        multi_nav_active_ = false;
                        RCLCPP_INFO(this->get_logger(),
                            "多航点导航: 全部 %zu 个航点已完成!",
                            waypoint_queue_.size());
                    }
                    state_machine_.HandleEvent(NavEvent::GOAL_REACHED);
                    PublishStopCmd();
                    RCLCPP_INFO(this->get_logger(), "已到达目标!");
                } else {
                    state_machine_.HandleEvent(NavEvent::TRACKING_LOST);
                    PublishStopCmd();
                    RCLCPP_WARN(this->get_logger(), "跟踪丢失");
                }
                return;
            }

            SdkControlMsg sdk_msg;
            sdk_interface_.MapToSdkMsg(cmd, sdk_msg);
            PublishControlCmd(sdk_msg);
            PublishTrackingDebug();
            break;
        }

        case NavState::ERROR:
            PublishStopCmd();
            break;
    }
}

// ==================== 发布函数 ====================

void SlamNavNode::PublishPath() {
    if (!planner_.HasValidPath()) return;

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_frame_;

    for (const auto& wp : planner_.GetPath()) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position.x = wp.x;
        ps.pose.position.y = wp.y;
        ps.pose.position.z = 0.0;

        auto q = YawToQuaternion(wp.yaw);
        ps.pose.orientation.w = q.w;
        ps.pose.orientation.x = q.x;
        ps.pose.orientation.y = q.y;
        ps.pose.orientation.z = q.z;

        path_msg.poses.push_back(ps);
    }

    path_pub_->publish(path_msg);
}

void SlamNavNode::PublishStatus() {
    // 将当前局部坐标 (x, y) 逆转换为 GPS (lat, lon)
    // 供 bridge_node 读取并转发给 TCP 客户端
    double cur_lat = 0.0, cur_lon = 0.0;
    LocalToGps(current_pose_.x, current_pose_.y, cur_lat, cur_lon);

    auto msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "{\"type\":\"pose\""
        << ",\"state\":\"" << state_machine_.GetStateString() << "\""
        << std::setprecision(9)
        << ",\"lat\":" << cur_lat
        << ",\"lon\":" << cur_lon
        << std::setprecision(4)
        << ",\"yaw\":" << current_pose_.yaw;

    if (goal_pose_.has_value()) {
        double gl = 0.0, gn = 0.0;
        LocalToGps(goal_pose_->x, goal_pose_->y, gl, gn);
        oss << std::setprecision(9)
            << ",\"goal_lat\":" << gl
            << ",\"goal_lon\":" << gn;
    }

    auto debug = tracker_.GetDebugInfo();
    oss << std::setprecision(4)
        << ",\"dist_to_goal\":" << debug.distance_to_goal
        << ",\"cross_track_error\":" << debug.cross_track_error;

    if (multi_nav_active_) {
        oss << ",\"multi_nav\":true"
            << ",\"waypoint_index\":" << waypoint_index_
            << ",\"waypoint_total\":" << waypoint_queue_.size();
    }

    if (nav_paused_) {
        oss << ",\"paused\":true";
    }

    oss << "}";
    msg.data = oss.str();
    status_pub_->publish(msg);
}

void SlamNavNode::PublishCurrentPose() {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.pose.position.x = current_pose_.x;
    msg.pose.position.y = current_pose_.y;
    msg.pose.position.z = current_z_;

    auto q = YawToQuaternion(current_pose_.yaw);
    msg.pose.orientation.w = q.w;
    msg.pose.orientation.x = q.x;
    msg.pose.orientation.y = q.y;
    msg.pose.orientation.z = q.z;

    current_pose_pub_->publish(msg);
}

void SlamNavNode::PublishTrackingDebug() {
    auto debug = tracker_.GetDebugInfo();
    auto msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << "{\"cross_track_error\":" << debug.cross_track_error
        << ",\"heading_error\":" << debug.heading_error
        << ",\"lookahead_dist\":" << debug.lookahead_dist
        << ",\"target_index\":" << debug.target_index
        << ",\"distance_to_goal\":" << debug.distance_to_goal
        << "}";
    msg.data = oss.str();
    tracking_debug_pub_->publish(msg);
}

void SlamNavNode::PublishControlCmd(const SdkControlMsg& sdk_msg) {
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x  = sdk_msg.vx;
    twist.linear.y  = sdk_msg.vy;
    twist.angular.z = sdk_msg.yaw_rate;
    cmd_vel_pub_->publish(twist);
}

void SlamNavNode::PublishStopCmd() {
    auto stop = sdk_interface_.MakeStopMsg();
    PublishControlCmd(stop);
}

void SlamNavNode::PublishTF() {
    if (!odom_received_) return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = this->now();
    t.header.frame_id = map_frame_;
    t.child_frame_id  = base_frame_;

    t.transform.translation.x = current_pose_.x;
    t.transform.translation.y = current_pose_.y;
    t.transform.translation.z = current_z_;

    auto q = YawToQuaternion(current_pose_.yaw);
    t.transform.rotation.w = q.w;
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;

    tf_broadcaster_->sendTransform(t);
}

void SlamNavNode::PublishActualTrajectory() {
    if (!odom_received_) return;

    // 仅在导航跟踪阶段记录轨迹，避免静止等待时的定位微抖动画出回环。
    if (state_machine_.GetState() != NavState::TRACKING) {
        actual_trajectory_.header.stamp = this->now();
        actual_trajectory_pub_->publish(actual_trajectory_);
        return;
    }

    double dx = current_pose_.x - last_traj_x_;
    double dy = current_pose_.y - last_traj_y_;
    if (dx * dx + dy * dy > 0.01) {  // 10cm 门限, 降低轨迹点密度减轻 RViz 负载
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = map_frame_;
        ps.pose.position.x = current_pose_.x;
        ps.pose.position.y = current_pose_.y;
        ps.pose.position.z = current_z_;

        auto q = YawToQuaternion(current_pose_.yaw);
        ps.pose.orientation.w = q.w;
        ps.pose.orientation.x = q.x;
        ps.pose.orientation.y = q.y;
        ps.pose.orientation.z = q.z;

        actual_trajectory_.poses.push_back(ps);
        last_traj_x_ = current_pose_.x;
        last_traj_y_ = current_pose_.y;

        if (actual_trajectory_.poses.size() > 2000) {
            actual_trajectory_.poses.erase(
                actual_trajectory_.poses.begin(),
            actual_trajectory_.poses.begin() + 500);
        }
    }

    actual_trajectory_.header.stamp = this->now();
    actual_trajectory_pub_->publish(actual_trajectory_);
}

void SlamNavNode::PublishGoalMarker() {
    if (!goal_pose_.has_value()) return;

    auto marker = visualization_msgs::msg::Marker();
    marker.header.stamp    = this->now();
    marker.header.frame_id = map_frame_;
    marker.ns      = "nav_goal";
    marker.id      = 0;
    marker.type    = visualization_msgs::msg::Marker::CYLINDER;
    marker.action  = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal_pose_->x;
    marker.pose.position.y = goal_pose_->y;
    marker.pose.position.z = 0.3;

    auto q = YawToQuaternion(goal_pose_->yaw);
    marker.pose.orientation.w = q.w;
    marker.pose.orientation.x = q.x;
    marker.pose.orientation.y = q.y;
    marker.pose.orientation.z = q.z;

    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.6;

    marker.color.r = 1.0f;
    marker.color.g = 0.2f;
    marker.color.b = 0.2f;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration(0, 0);

    goal_marker_pub_->publish(marker);
}

void SlamNavNode::PublishPlannedRouteMarker() {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.stamp = this->now();
    marker.header.frame_id = map_frame_;
    marker.ns = "planned_route";
    marker.id = 0;

    if (!planner_.HasValidPath()) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
        planned_route_marker_pub_->publish(marker);
        return;
    }

    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.12;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.2f;
    marker.color.a = 1.0f;
    marker.lifetime = rclcpp::Duration(0, 0);

    marker.points.reserve(planner_.GetPath().size());
    for (const auto& wp : planner_.GetPath()) {
        geometry_msgs::msg::Point p;
        p.x = wp.x;
        p.y = wp.y;
        p.z = 0.08;
        marker.points.push_back(p);
    }

    planned_route_marker_pub_->publish(marker);
}

// ==================== 状态机回调 ====================

void SlamNavNode::OnStateChange(NavState old_state, NavState new_state) {
    RCLCPP_INFO(this->get_logger(), "状态切换: %s → %s",
                NavStateToString(old_state).c_str(),
                NavStateToString(new_state).c_str());

    if (new_state == NavState::TRACKING) {
        auto enable_msg = std_msgs::msg::Bool();
        enable_msg.data = true;
        robot_enable_pub_->publish(enable_msg);
    }

    if (new_state == NavState::IDLE ||
        new_state == NavState::REACHED ||
        new_state == NavState::ERROR) {

        if (new_state == NavState::REACHED) {
            tracker_.Reset();
            planner_.ClearPath();
            multi_nav_active_ = false;
        }

        PublishStopCmd();
    }
}

// ==================== 多航点管理 ====================

void SlamNavNode::CancelMultiNav() {
    if (multi_nav_active_) {
        RCLCPP_INFO(this->get_logger(), "多航点导航已取消");
    }
    multi_nav_active_ = false;
    waypoint_queue_.clear();
    waypoint_index_ = 0;
}

}  // namespace slam_nav
