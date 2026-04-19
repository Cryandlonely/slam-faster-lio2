// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 改造自 207_ws nav_planner_node.cpp
// 核心变更:
//   1. 定位源: SLAM odom (单一话题提供 position + orientation)
//   2. 去除 GPS 相关逻辑 (GpsToLocal, ref_lat/lon, NavSatFix)
//   3. 航点使用 map 坐标系 (x, y, yaw)
//   4. 新增 tf2 监听器作为备用定位通道
//   5. 参数适配室内环境

#include "nav_planner/slam_nav_node.h"
#include "nav_planner/common/math_utils.h"

#include <chrono>
#include <sstream>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

namespace slam_nav {

SlamNavNode::SlamNavNode(const rclcpp::NodeOptions& options)
    : Node("slam_nav_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== SlamNavNode 初始化 (室内SLAM导航) ===");

    // ---- 参数声明与加载 ----
    DeclareAndLoadParams();

    // ---- 状态机回调 ----
    state_machine_.SetCallback(
        [this](NavState old_s, NavState new_s) {
            OnStateChange(old_s, new_s);
        });

    // ---- TF2 ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ---- 订阅 ----
    // SLAM 里程计 (来自 Faster-LIO2)
    slam_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        slam_odom_topic_, 10,
        std::bind(&SlamNavNode::SlamOdomCallback, this, std::placeholders::_1));

    // RViz2 目标点 (map 坐标系)
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&SlamNavNode::GoalPoseCallback, this, std::placeholders::_1));

    // 本地坐标航点列表 (JSON)
    nav_waypoints_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_waypoints", 10,
        std::bind(&SlamNavNode::NavWaypointsCallback, this, std::placeholders::_1));

    // 取消导航
    nav_cancel_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_cancel", 10,
        std::bind(&SlamNavNode::NavCancelCallback, this, std::placeholders::_1));

    // 初始位姿 (RViz2 的 2D Pose Estimate)
    initial_pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&SlamNavNode::InitialPoseCallback, this, std::placeholders::_1));

    // OccupancyGrid 地图 (A* 规划用)
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, rclcpp::QoS(1).transient_local().reliable(),
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            astar_planner_.UpdateMap(msg);
            RCLCPP_INFO_ONCE(this->get_logger(),
                "已接收 OccupancyGrid 地图: %dx%d, 分辨率=%.3fm",
                msg->info.width, msg->info.height, msg->info.resolution);
            // 发布膨胀后的代价地图用于调试
            if (costmap_pub_) {
                auto costmap_msg = astar_planner_.GetInflatedCostmapMsg();
                costmap_msg.header.stamp = this->now();
                costmap_pub_->publish(costmap_msg);
            }
        });

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
    costmap_pub_         = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/inflated_costmap", rclcpp::QoS(1).transient_local().reliable());

    // 初始化实际轨迹消息
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
            PublishActualTrajectory();
            PublishPath();
            PublishGoalMarker();
        });

    RCLCPP_INFO(this->get_logger(), "SlamNavNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  SLAM里程计话题: %s", slam_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  订阅: %s, goal_pose, /nav_waypoints, /nav_cancel, /initialpose, %s",
                slam_odom_topic_.c_str(), map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  发布: /planned_path, /actual_trajectory, /nav_status, "
                "/current_pose, /tracking_debug, /cmd_vel, /goal_marker");
    RCLCPP_INFO(this->get_logger(), "  规划器: %s", use_astar_ ? "A* 避障规划" : "直线规划");
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s → %s", map_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  控制频率: %.1f Hz", control_rate_);
    RCLCPP_INFO(this->get_logger(), "  TF位姿模式: %s", use_tf_pose_ ? "开启" : "关闭");
}

// ==================== 参数声明与加载 ====================

void SlamNavNode::DeclareAndLoadParams() {
    // -- SLAM 配置 --
    this->declare_parameter<std::string>("slam.odom_topic", "/slam/odom");
    this->declare_parameter<std::string>("slam.map_frame", "map");
    this->declare_parameter<std::string>("slam.base_frame", "base_link");
    this->declare_parameter<bool>("slam.use_tf_pose", false);
    slam_odom_topic_ = this->get_parameter("slam.odom_topic").as_string();
    map_frame_  = this->get_parameter("slam.map_frame").as_string();
    base_frame_ = this->get_parameter("slam.base_frame").as_string();
    use_tf_pose_ = this->get_parameter("slam.use_tf_pose").as_bool();

    // -- 地图配置 --
    this->declare_parameter<std::string>("planning.map_topic", "map");
    this->declare_parameter<bool>("planning.use_astar", true);
    map_topic_  = this->get_parameter("planning.map_topic").as_string();
    use_astar_  = this->get_parameter("planning.use_astar").as_bool();

    // -- 控制频率 --
    this->declare_parameter<double>("control.rate", 20.0);
    this->declare_parameter<double>("debug.status_rate", 2.0);
    control_rate_ = this->get_parameter("control.rate").as_double();
    status_rate_  = this->get_parameter("debug.status_rate").as_double();

    // -- 规划参数 (室内默认值) --
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

    // -- A* 规划参数 --
    AStarParams ap;
    ap.path_resolution   = pp.path_resolution;
    ap.target_speed      = pp.target_speed;
    ap.slow_down_dist    = pp.slow_down_dist;
    ap.min_speed         = pp.min_speed;
    this->declare_parameter<int>("planning.obstacle_inflate", ap.obstacle_inflate);
    this->declare_parameter<int>("planning.occupied_thresh", ap.occupied_thresh);
    ap.obstacle_inflate  = this->get_parameter("planning.obstacle_inflate").as_int();
    ap.occupied_thresh   = this->get_parameter("planning.occupied_thresh").as_int();
    astar_planner_.SetParams(ap);

    // -- 跟踪参数 (室内默认值) --
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
    this->declare_parameter<double>("tracking.cmd_filter_alpha", tp.cmd_filter_alpha);
    this->declare_parameter<double>("tracking.heading_align_threshold", tp.heading_align_threshold);
    tp.lookahead_distance   = this->get_parameter("tracking.lookahead_distance").as_double();
    tp.min_lookahead        = this->get_parameter("tracking.min_lookahead").as_double();
    tp.max_lookahead        = this->get_parameter("tracking.max_lookahead").as_double();
    tp.lookahead_gain       = this->get_parameter("tracking.lookahead_gain").as_double();
    tp.goal_tolerance       = this->get_parameter("tracking.goal_tolerance").as_double();
    tp.heading_tolerance    = this->get_parameter("tracking.heading_tolerance").as_double();
    tp.max_linear_x         = this->get_parameter("tracking.max_linear_x").as_double();
    tp.max_linear_y         = this->get_parameter("tracking.max_linear_y").as_double();
    tp.max_angular_velocity = this->get_parameter("tracking.max_angular_velocity").as_double();
    tp.heading_kp           = this->get_parameter("tracking.heading_kp").as_double();
    tp.cte_kp               = this->get_parameter("tracking.cte_kp").as_double();
    tp.cmd_filter_alpha     = this->get_parameter("tracking.cmd_filter_alpha").as_double();
    tp.heading_align_threshold = this->get_parameter("tracking.heading_align_threshold").as_double();
    tracker_.SetParams(tp);

    // -- SDK 接口参数 --
    SdkInterfaceParams sp;
    this->declare_parameter<double>("sdk.max_vx", sp.max_vx);
    this->declare_parameter<double>("sdk.max_vy", sp.max_vy);
    this->declare_parameter<double>("sdk.max_yaw_rate", sp.max_yaw_rate);
    this->declare_parameter<int>("sdk.default_mode", static_cast<int>(sp.default_mode));
    this->declare_parameter<bool>("sdk.enable_posture_fields", sp.enable_posture);
    sp.max_vx        = this->get_parameter("sdk.max_vx").as_double();
    sp.max_vy        = this->get_parameter("sdk.max_vy").as_double();
    sp.max_yaw_rate  = this->get_parameter("sdk.max_yaw_rate").as_double();
    sp.default_mode  = static_cast<int8_t>(this->get_parameter("sdk.default_mode").as_int());
    sp.enable_posture = this->get_parameter("sdk.enable_posture_fields").as_bool();
    sdk_interface_.SetParams(sp);

    RCLCPP_INFO(this->get_logger(), "参数加载完成: slam_odom=%s rate=%.0fHz",
                slam_odom_topic_.c_str(), control_rate_);
}

// ==================== 回调函数 ====================

void SlamNavNode::SlamOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // SLAM 里程计同时提供位置和朝向 (与 RTK 的双话题不同)
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    slam_z_ = msg->pose.pose.position.z;

    // 从四元数提取 yaw
    current_pose_.yaw = QuaternionToYaw(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    slam_odom_received_ = true;
}

void SlamNavNode::GoalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 单点目标取消多航点队列
    CancelMultiNav();

    Pose2D goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;

    // 到点朝向: 使用消息中四元数指定的朝向
    goal.yaw = QuaternionToYaw(
        msg->pose.orientation.w,
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z);

    goal_pose_ = goal;

    RCLCPP_INFO(this->get_logger(),
                "收到目标 (map坐标系): x=%.3f y=%.3f yaw=%.2f°",
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

void SlamNavNode::InitialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 手动设置初始位姿 (适用于 SLAM 漂移校正或重定位)
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = QuaternionToYaw(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    RCLCPP_INFO(this->get_logger(),
                "手动设置初始位姿: x=%.3f y=%.3f yaw=%.2f°",
                current_pose_.x, current_pose_.y,
                current_pose_.yaw * kRadToDeg);
}

// ==================== 多航点 (本地坐标) ====================

void SlamNavNode::NavWaypointsCallback(
    const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    try {
        auto j = nlohmann::json::parse(msg->data);

        if (!j.contains("waypoints") || !j["waypoints"].is_array()) {
            RCLCPP_WARN(this->get_logger(),
                        "收到 /nav_waypoints 但缺少 waypoints 数组");
            return;
        }

        waypoint_queue_.clear();
        waypoint_index_ = 0;

        // 读取命令中的目标速度
        if (j.contains("target_vel") && j["target_vel"].is_number()) {
            double cmd_speed = j["target_vel"].get<double>();
            if (cmd_speed > 0.0) {
                auto pp = planner_.GetParams();
                pp.target_speed = cmd_speed;
                planner_.SetParams(pp);
                RCLCPP_INFO(this->get_logger(),
                    "使用命令目标速度: %.2f m/s", cmd_speed);
            }
        }

        for (const auto& wp : j["waypoints"]) {
            // 室内SLAM导航: 航点使用 map 坐标系本地坐标
            if (!wp.contains("x") || !wp.contains("y")) {
                RCLCPP_WARN(this->get_logger(),
                    "跳过无效航点: 缺少 x 或 y 字段");
                continue;
            }
            Pose2D pose;
            pose.x = wp["x"].get<double>();
            pose.y = wp["y"].get<double>();
            pose.yaw = wp.value("yaw", 0.0);
            pose.yaw_specified = wp.contains("yaw");  // 标记用户是否指定了 yaw
            waypoint_queue_.push_back(pose);
        }

        if (waypoint_queue_.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "收到 /nav_waypoints 但无有效航点");
            return;
        }

        // 对未指定 yaw 的中间航点, 自动计算行驶方向作为朝向
        // 用户明确指定了 yaw 的航点保持不变
        for (size_t i = 0; i < waypoint_queue_.size(); ++i) {
            if (waypoint_queue_[i].yaw_specified) {
                continue;  // 用户指定了 yaw, 不覆盖
            }
            if (i + 1 < waypoint_queue_.size()) {
                waypoint_queue_[i].yaw = Azimuth(
                    waypoint_queue_[i].x, waypoint_queue_[i].y,
                    waypoint_queue_[i+1].x, waypoint_queue_[i+1].y);
            } else if (i > 0) {
                waypoint_queue_[i].yaw = Azimuth(
                    waypoint_queue_[i-1].x, waypoint_queue_[i-1].y,
                    waypoint_queue_[i].x, waypoint_queue_[i].y);
            } else if (slam_odom_received_) {
                // 单航点且未指定 yaw: 使用行驶方向
                waypoint_queue_[i].yaw = Azimuth(
                    current_pose_.x, current_pose_.y,
                    waypoint_queue_[i].x, waypoint_queue_[i].y);
            }
        }

        multi_nav_active_ = true;
        waypoint_index_ = 0;
        goal_pose_ = waypoint_queue_.back();

        RCLCPP_INFO(this->get_logger(),
                    "收到多航点任务: %zu 个航点 (map坐标系)",
                    waypoint_queue_.size());

        state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "/nav_waypoints JSON 解析失败: %s", e.what());
    }
}

// ==================== TF 位姿获取 ====================

bool SlamNavNode::GetPoseFromTF(Pose2D& pose) {
    try {
        auto transform = tf_buffer_->lookupTransform(
            map_frame_, base_frame_, tf2::TimePointZero);

        pose.x = transform.transform.translation.x;
        pose.y = transform.transform.translation.y;
        pose.yaw = QuaternionToYaw(
            transform.transform.rotation.w,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z);
        return true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
            "TF 查询失败 (%s → %s): %s",
            map_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
    }
}

// ==================== 控制循环 ====================

void SlamNavNode::ControlLoop() {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 如果使用 TF 模式, 每个控制周期从 TF 更新位姿
    if (use_tf_pose_) {
        Pose2D tf_pose;
        if (GetPoseFromTF(tf_pose)) {
            current_pose_ = tf_pose;
            slam_odom_received_ = true;
        }
    }

    NavState state = state_machine_.GetState();

    switch (state) {
        case NavState::IDLE:
        case NavState::REACHED:
            break;

        case NavState::PLANNING: {
            if (!slam_odom_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "等待 SLAM 里程计数据 (%s)...", slam_odom_topic_.c_str());
                return;
            }

            if (!goal_pose_.has_value()) {
                state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                return;
            }

            bool ok = false;
            if (use_astar_ && astar_planner_.HasMap()) {
                // A* 避障规划
                if (multi_nav_active_ && waypoint_queue_.size() > 1) {
                    ok = astar_planner_.PlanMulti(current_pose_, waypoint_queue_);
                } else {
                    ok = astar_planner_.Plan(current_pose_, goal_pose_.value(), true);
                }

                if (ok && astar_planner_.HasValidPath()) {
                    tracker_.SetPath(astar_planner_.GetPath(), true);
                    state_machine_.HandleEvent(NavEvent::PLAN_SUCCESS);
                    // 同步路径到 planner_ 用于发布
                    planner_.ClearPath();
                    PublishPath();
                    actual_trajectory_.poses.clear();
                    last_traj_x_ = current_pose_.x;
                    last_traj_y_ = current_pose_.y;
                    RCLCPP_INFO(this->get_logger(), "A* 路径规划成功, %zu 个路径点",
                                astar_planner_.GetPath().size());
                } else {
                    RCLCPP_WARN(this->get_logger(), "A* 规划失败, 回退到直线规划");
                    // 回退到直线规划
                    ok = false;
                }
            }

            if (!ok) {
                // A* 规划失败, 室内环境禁止直线规划(会撞墙)
                state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                RCLCPP_ERROR(this->get_logger(),
                    "A* 路径规划失败 (目标可能不可达), 已取消导航");
            }
            break;
        }

        case NavState::TRACKING: {
            OmniControlCmd cmd;
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
    // 优先使用 A* 路径
    const std::vector<Waypoint>* path_ptr = nullptr;
    if (use_astar_ && astar_planner_.HasValidPath()) {
        path_ptr = &astar_planner_.GetPath();
    } else if (planner_.HasValidPath()) {
        path_ptr = &planner_.GetPath();
    }
    if (!path_ptr) return;

    auto path_msg = nav_msgs::msg::Path();
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_frame_;

    for (const auto& wp : *path_ptr) {
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
    auto msg = std_msgs::msg::String();
    std::ostringstream oss;
    oss << "{\"state\":\"" << state_machine_.GetStateString() << "\""
        << ",\"x\":" << current_pose_.x
        << ",\"y\":" << current_pose_.y
        << ",\"yaw\":" << current_pose_.yaw
        << ",\"slam_valid\":" << (slam_odom_received_ ? "true" : "false");

    if (goal_pose_.has_value()) {
        oss << ",\"goal_x\":" << goal_pose_->x
            << ",\"goal_y\":" << goal_pose_->y;
    }

    auto debug = tracker_.GetDebugInfo();
    oss << ",\"dist_to_goal\":" << debug.distance_to_goal
        << ",\"cross_track_error\":" << debug.cross_track_error;

    if (multi_nav_active_) {
        oss << ",\"multi_nav\":true"
            << ",\"waypoint_index\":" << waypoint_index_
            << ",\"waypoint_total\":" << waypoint_queue_.size();
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
    msg.pose.position.z = slam_z_;

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
    if (!slam_odom_received_) return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = map_frame_;
    t.child_frame_id  = base_frame_;

    t.transform.translation.x = current_pose_.x;
    t.transform.translation.y = current_pose_.y;
    t.transform.translation.z = slam_z_;

    auto q = YawToQuaternion(current_pose_.yaw);
    t.transform.rotation.w = q.w;
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;

    tf_broadcaster_->sendTransform(t);
}

void SlamNavNode::PublishActualTrajectory() {
    if (!slam_odom_received_) return;

    double dx = current_pose_.x - last_traj_x_;
    double dy = current_pose_.y - last_traj_y_;
    if (dx * dx + dy * dy > 0.0025) {  // 移动超过 5cm
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = map_frame_;
        ps.pose.position.x = current_pose_.x;
        ps.pose.position.y = current_pose_.y;
        ps.pose.position.z = 0.0;

        auto q = YawToQuaternion(current_pose_.yaw);
        ps.pose.orientation.w = q.w;
        ps.pose.orientation.x = q.x;
        ps.pose.orientation.y = q.y;
        ps.pose.orientation.z = q.z;

        actual_trajectory_.poses.push_back(ps);
        last_traj_x_ = current_pose_.x;
        last_traj_y_ = current_pose_.y;

        if (actual_trajectory_.poses.size() > 5000) {
            actual_trajectory_.poses.erase(
                actual_trajectory_.poses.begin(),
                actual_trajectory_.poses.begin() + 1000);
        }
    }

    actual_trajectory_.header.stamp = this->now();
    actual_trajectory_pub_->publish(actual_trajectory_);
}

void SlamNavNode::PublishGoalMarker() {
    if (!goal_pose_.has_value()) return;

    auto marker = visualization_msgs::msg::Marker();
    marker.header.stamp = this->now();
    marker.header.frame_id = map_frame_;
    marker.ns = "nav_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = goal_pose_->x;
    marker.pose.position.y = goal_pose_->y;
    marker.pose.position.z = 0.3;

    auto q = YawToQuaternion(goal_pose_->yaw);
    marker.pose.orientation.w = q.w;
    marker.pose.orientation.x = q.x;
    marker.pose.orientation.y = q.y;
    marker.pose.orientation.z = q.z;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.5;

    marker.color.r = 0.2f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.8f;

    marker.lifetime = rclcpp::Duration(0, 0);

    goal_marker_pub_->publish(marker);
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
            astar_planner_.ClearPath();
            multi_nav_active_ = false;
        }

        PublishStopCmd();
    }
}

// ==================== 多航点队列管理 ====================

void SlamNavNode::CancelMultiNav() {
    if (multi_nav_active_) {
        RCLCPP_INFO(this->get_logger(),
                    "多航点导航已取消");
    }
    multi_nav_active_ = false;
    waypoint_queue_.clear();
    waypoint_index_ = 0;
}

}  // namespace slam_nav
