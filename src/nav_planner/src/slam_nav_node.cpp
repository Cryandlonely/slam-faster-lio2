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
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

namespace slam_nav {

namespace {


std::string ToLowerCopy(std::string value) {
    for (char& ch : value) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return value;
}

std::string ResolveCoordMode(const nlohmann::json& j) {
    if (j.contains("coord_mode") && j["coord_mode"].is_string()) {
        return ToLowerCopy(j["coord_mode"].get<std::string>());
    }
    if (j.contains("input_mode") && j["input_mode"].is_string()) {
        auto mode = ToLowerCopy(j["input_mode"].get<std::string>());
        if (mode == "indoor") return "local";
        if (mode == "outdoor") return "gps";
    }
    return "local";
}

double HeadingDegToEnuYaw(double heading_deg) {
    double yaw_enu = kPi / 2.0 - heading_deg * kDegToRad;
    return NormalizeAngle(yaw_enu);
}

}  // namespace

SlamNavNode::SlamNavNode(const rclcpp::NodeOptions& options)
    : Node("slam_nav_node", options)
{
    RCLCPP_INFO(this->get_logger(), "=== SlamNavNode 初始化 ===");

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
    // 统一定位里程计: slam 模式来自 Faster-LIO2, gps 模式来自 /outdoor/odom
    localization_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        localization_odom_topic_, 10,
        std::bind(&SlamNavNode::LocalizationOdomCallback, this, std::placeholders::_1));

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

    // 暂停/继续导航
    nav_pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/nav_pause", 10,
        std::bind(&SlamNavNode::NavPauseCallback, this, std::placeholders::_1));

    // 运行时定位源切换 (来自 bridge)
    nav_mode_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/nav_mode_cmd", 10,
        std::bind(&SlamNavNode::NavModeCmdCallback, this, std::placeholders::_1));

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

    // Livox 点云 (动态障碍物检测)
    livox_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SlamNavNode::LivoxCallback, this, std::placeholders::_1));

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
    obstacle_pts_pub_   = this->create_publisher<visualization_msgs::msg::Marker>("/obstacle_points", 10);
    detour_wp_pub_      = this->create_publisher<visualization_msgs::msg::Marker>("/detour_waypoint", 10);
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
            PublishObstacleMarkers();
        });

    RCLCPP_INFO(this->get_logger(), "SlamNavNode 初始化完成");
    RCLCPP_INFO(this->get_logger(), "  模式: %s", nav_mode_.c_str());
    RCLCPP_INFO(this->get_logger(), "  定位话题: %s", localization_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  订阅: %s, goal_pose, /nav_waypoints, /nav_cancel, /initialpose, %s",
                localization_odom_topic_.c_str(), map_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  发布: /planned_path, /actual_trajectory, /nav_status, "
                "/current_pose, /tracking_debug, /cmd_vel, /goal_marker");
    RCLCPP_INFO(this->get_logger(), "  规划器: %s", use_astar_ ? "A* 避障规划" : "直线规划");
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s → %s", map_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  控制频率: %.1f Hz", control_rate_);
    RCLCPP_INFO(this->get_logger(), "  TF位姿模式: %s", use_tf_pose_ ? "开启" : "关闭");
}

// ==================== 参数声明与加载 ====================

void SlamNavNode::DeclareAndLoadParams() {
    this->declare_parameter<std::string>("nav.mode", "slam");
    nav_mode_ = this->get_parameter("nav.mode").as_string();
    gps_mode_ = (nav_mode_ == "gps");

    // -- SLAM 配置 --
    this->declare_parameter<std::string>("slam.odom_topic", "/slam/odom");
    this->declare_parameter<std::string>("slam.map_frame", "map");
    this->declare_parameter<std::string>("slam.base_frame", "base_link");
    this->declare_parameter<bool>("slam.use_tf_pose", false);
    slam_odom_topic_ = this->get_parameter("slam.odom_topic").as_string();
    map_frame_  = this->get_parameter("slam.map_frame").as_string();
    base_frame_ = this->get_parameter("slam.base_frame").as_string();
    use_tf_pose_ = this->get_parameter("slam.use_tf_pose").as_bool();

    // -- 室外定位配置 --
    this->declare_parameter<std::string>("outdoor.odom_topic", "/outdoor/odom");
    this->declare_parameter<double>("outdoor.ref_latitude", 36.66111);
    this->declare_parameter<double>("outdoor.ref_longitude", 117.01665);
    outdoor_odom_topic_ = this->get_parameter("outdoor.odom_topic").as_string();
    outdoor_ref_latitude_ = this->get_parameter("outdoor.ref_latitude").as_double();
    outdoor_ref_longitude_ = this->get_parameter("outdoor.ref_longitude").as_double();

    // -- 地图配置 --
    this->declare_parameter<std::string>("planning.map_topic", "map");
    this->declare_parameter<bool>("planning.use_astar", true);
    map_topic_  = this->get_parameter("planning.map_topic").as_string();
    use_astar_  = this->get_parameter("planning.use_astar").as_bool();

    localization_odom_topic_ = gps_mode_ ? outdoor_odom_topic_ : slam_odom_topic_;
    if (gps_mode_) {
        if (use_tf_pose_) {
            RCLCPP_WARN(this->get_logger(), "gps 模式下忽略 slam.use_tf_pose, 改为使用 %s", outdoor_odom_topic_.c_str());
            use_tf_pose_ = false;
        }
        if (use_astar_) {
            RCLCPP_WARN(this->get_logger(), "gps 模式下强制关闭 A*，改用直线规划");
            use_astar_ = false;
        }
    }

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
    this->declare_parameter<double>("tracking.heading_align_threshold", 30.0);  // 度
    this->declare_parameter<double>("tracking.max_accel", tp.max_accel);
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
    tp.heading_align_threshold = this->get_parameter("tracking.heading_align_threshold").as_double() * M_PI / 180.0;  // 度→弧度
    tp.max_accel            = this->get_parameter("tracking.max_accel").as_double();
    tp.control_dt           = (control_rate_ > 0.0) ? (1.0 / control_rate_) : 0.05;
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

    // -- 动态避障参数 --
    this->declare_parameter<std::string>("obstacle.lidar_topic", "/livox/lidar");
    this->declare_parameter<std::string>("obstacle.lidar_frame", "livox_frame");
    this->declare_parameter<double>("obstacle.z_min", 0.15);
    this->declare_parameter<double>("obstacle.z_max", 2.0);
    this->declare_parameter<double>("obstacle.fov_half_deg", 90.0);
    this->declare_parameter<double>("obstacle.dynamic_ttl", 3.0);
    this->declare_parameter<double>("obstacle.replan_lookahead", 3.0);
    this->declare_parameter<double>("obstacle.replan_cooldown", 2.0);
    this->declare_parameter<int>("obstacle.lidar_subsample", 5);
    this->declare_parameter<bool>("obstacle.enable", true);
    this->declare_parameter<double>("obstacle.corridor_width", 0.6);
    this->declare_parameter<bool>("obstacle.gps_avoidance", false);
    this->declare_parameter<double>("obstacle.detour_lateral", 1.5);
    this->declare_parameter<double>("obstacle.detour_forward", 1.5);
    lidar_topic_          = this->get_parameter("obstacle.lidar_topic").as_string();
    lidar_frame_          = this->get_parameter("obstacle.lidar_frame").as_string();
    obstacle_z_min_       = this->get_parameter("obstacle.z_min").as_double();
    obstacle_z_max_       = this->get_parameter("obstacle.z_max").as_double();
    obstacle_fov_half_deg_= this->get_parameter("obstacle.fov_half_deg").as_double();
    dynamic_ttl_          = this->get_parameter("obstacle.dynamic_ttl").as_double();
    replan_lookahead_     = this->get_parameter("obstacle.replan_lookahead").as_double();
    replan_cooldown_      = this->get_parameter("obstacle.replan_cooldown").as_double();
    lidar_subsample_      = this->get_parameter("obstacle.lidar_subsample").as_int();
    obstacle_corridor_width_ = this->get_parameter("obstacle.corridor_width").as_double();
    gps_avoidance_    = this->get_parameter("obstacle.gps_avoidance").as_bool();
    detour_lateral_   = this->get_parameter("obstacle.detour_lateral").as_double();
    detour_forward_   = this->get_parameter("obstacle.detour_forward").as_double();
    obstacle_avoidance_enabled_ = this->get_parameter("obstacle.enable").as_bool();

    RCLCPP_INFO(this->get_logger(), "参数加载完成: mode=%s odom=%s rate=%.0fHz",
                nav_mode_.c_str(), localization_odom_topic_.c_str(), control_rate_);
}

bool SlamNavNode::ConvertOutdoorGpsToLocal(double lat, double lon, double& x, double& y) const {
    if (!std::isfinite(lat) || !std::isfinite(lon)) {
        return false;
    }
    // 与 rtk_node 保持完全一致的简化平面投影公式 (111320.0 m/deg)
    // 保证 RTK 定位坐标系 与 GPS 航点转换坐标系 完全对齐
    constexpr double kMetersPerDegreeLat = 111320.0;
    const double meters_per_degree_lon =
        111320.0 * std::cos(outdoor_ref_latitude_ * kDegToRad);

    x = (lon - outdoor_ref_longitude_) * meters_per_degree_lon;
    y = (lat - outdoor_ref_latitude_) * kMetersPerDegreeLat;
    return std::isfinite(x) && std::isfinite(y);
}

// ==================== 回调函数 ====================

void SlamNavNode::SwitchLocalizationSource(const std::string& topic, bool gps_mode) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (topic == localization_odom_topic_ && gps_mode == gps_mode_) {
            RCLCPP_INFO(this->get_logger(), "定位源未变化: %s", topic.c_str());
            return;
        }
        localization_odom_topic_ = topic;
        gps_mode_  = gps_mode;
        nav_mode_  = gps_mode ? "gps" : "slam";
        if (gps_mode_) {
            use_astar_ = false;
            // GPS 模式下必须禁用 TF 位姿, 否则 ControlLoop 会用 SLAM map 坐标覆盖
            // current_pose_, 导致 PublishStatus 反算出错误的经纬度
            use_tf_pose_ = false;
        } else {
            // 切回 SLAM 模式时, 从参数恢复 use_astar (避免室内永远直线规划撞墙)
            use_astar_ = this->get_parameter("planning.use_astar").as_bool();
            use_tf_pose_ = this->get_parameter("slam.use_tf_pose").as_bool();
        }
        localization_received_ = false;
        localization_odom_sub_.reset();  // 先重置旧订阅
    }

    // create_subscription 不能持锁调用
    localization_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, 10,
        std::bind(&SlamNavNode::LocalizationOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "定位源已切换: topic=%s gps=%d 规划器=%s",
                topic.c_str(), gps_mode_, use_astar_ ? "A*" : "直线");
}

void SlamNavNode::NavModeCmdCallback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        auto j = nlohmann::json::parse(msg->data);
        std::string action = j.value("action", "");

        if (action == "switch_odom") {
            std::string topic = j.value("topic", localization_odom_topic_);
            bool gps = j.value("gps", false);
            SwitchLocalizationSource(topic, gps);
        } else {
            RCLCPP_WARN(this->get_logger(), "[NavModeCmd] 未知 action: %s", action.c_str());
        }
    } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[NavModeCmd] JSON 解析失败: %s", e.what());
    }
}

void SlamNavNode::LocalizationOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 统一定位里程计同时提供位置和朝向
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    // 从四元数提取 yaw
    current_pose_.yaw = QuaternionToYaw(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    localization_received_ = true;
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

    nav_paused_ = false;   // 取消时同时清除暂停状态
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

// ==================== 动态障碍物检测 ====================

void SlamNavNode::LivoxCallback(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    if (!obstacle_avoidance_enabled_) return;

    // 获取 livox_frame → map 的 TF 变换
    // GPS模式下无 SLAM 输出, map 帧不存在, 用里程计位姿手动构造等效变换
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            map_frame_, lidar_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
        if (gps_mode_) {
            // GPS模式: 用当前里程计位姿 (ENU) 构造 livox_frame → ENU 的变换
            Pose2D pose_snap;
            {
                std::lock_guard<std::mutex> lk(data_mutex_);
                if (!localization_received_) return;
                pose_snap = current_pose_;
            }
            const double half_yaw = pose_snap.yaw * 0.5;
            tf_stamped.transform.rotation.w = std::cos(half_yaw);
            tf_stamped.transform.rotation.x = 0.0;
            tf_stamped.transform.rotation.y = 0.0;
            tf_stamped.transform.rotation.z = std::sin(half_yaw);
            tf_stamped.transform.translation.x = pose_snap.x;
            tf_stamped.transform.translation.y = pose_snap.y;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "[动态避障] TF 查询失败 (%s→%s): %s",
                lidar_frame_.c_str(), map_frame_.c_str(), ex.what());
            return;
        }
    }

    // 提取旋转矩阵 (四元数 → 3x3)
    const double qw = tf_stamped.transform.rotation.w;
    const double qx = tf_stamped.transform.rotation.x;
    const double qy = tf_stamped.transform.rotation.y;
    const double qz = tf_stamped.transform.rotation.z;
    const double tx = tf_stamped.transform.translation.x;
    const double ty = tf_stamped.transform.translation.y;

    const double r00 = 1 - 2*(qy*qy + qz*qz), r01 = 2*(qx*qy - qw*qz), r02 = 2*(qx*qz + qw*qy);
    const double r10 = 2*(qx*qy + qw*qz),     r11 = 1 - 2*(qx*qx + qz*qz), r12 = 2*(qy*qz - qw*qx);

    const double fov_half_rad = obstacle_fov_half_deg_ * M_PI / 180.0;

    std::vector<std::pair<double, double>> map_pts;
    map_pts.reserve(msg->point_num / lidar_subsample_ + 1);

    int idx = 0;
    for (const auto& pt : msg->points) {
        // 前向 FOV 过滤: 只保留雷达坐标系前方 ±fov_half 范围内的点
        if (std::abs(std::atan2(pt.y, pt.x)) > fov_half_rad) continue;

        // 自身点云过滤: X <= 0.15m 的点属于机器人本体
        if (pt.x <= 0.15f) continue;

        // 高度过滤 (雷达坐标系): Z <= z_min 为地面, Z > z_max 为天花板/无关物体
        if (pt.z <= static_cast<float>(obstacle_z_min_) || pt.z > static_cast<float>(obstacle_z_max_)) continue;

        // 降采样
        if ((idx++ % lidar_subsample_) != 0) continue;

        // 变换到 map 坐标系
        const double wx = r00*pt.x + r01*pt.y + r02*pt.z + tx;
        const double wy = r10*pt.x + r11*pt.y + r12*pt.z + ty;

        map_pts.emplace_back(wx, wy);
    }

    // 始终将过滤后的点存入 raw 列表 (全模式共用)
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        raw_obstacle_pts_ = map_pts;
        raw_obstacle_time_ = this->now();
    }

    // 仅在室内 A* 模式且有地图时才需要注入网格
    if (use_astar_ && astar_planner_.HasMap() && !map_pts.empty()) {
        astar_planner_.UpdateDynamicObstacles(map_pts, this->now(), dynamic_ttl_);
        astar_planner_.ClearExpiredObstacles(this->now());
    }
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
        const std::string coord_mode = ResolveCoordMode(j);

        if (coord_mode != "local" && coord_mode != "gps") {
            RCLCPP_WARN(this->get_logger(),
                        "收到 /nav_waypoints 但 coord_mode=%s 不支持, 仅支持 local/gps",
                        coord_mode.c_str());
            return;
        }
        if (coord_mode == "gps" && !gps_mode_) {
            RCLCPP_WARN(this->get_logger(),
                        "收到 GPS 航点, 但当前 nav.mode=%s, 请切换到 gps 模式",
                        nav_mode_.c_str());
            return;
        }

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
            Pose2D pose;

            if (coord_mode == "gps") {
                if (!wp.contains("lat") || !wp.contains("lon")) {
                    RCLCPP_WARN(this->get_logger(),
                        "跳过无效 GPS 航点: 缺少 lat 或 lon 字段");
                    continue;
                }
                double x = 0.0, y = 0.0;
                if (!ConvertOutdoorGpsToLocal(wp["lat"].get<double>(), wp["lon"].get<double>(), x, y)) {
                    RCLCPP_WARN(this->get_logger(), "跳过无效 GPS 航点: 经纬度转换失败");
                    continue;
                }
                pose.x = x;
                pose.y = y;
                if (wp.contains("yaw")) {
                    pose.yaw = wp["yaw"].get<double>();
                    pose.yaw_specified = true;
                } else if (wp.contains("heading_deg")) {
                    pose.yaw = HeadingDegToEnuYaw(wp["heading_deg"].get<double>());
                    pose.yaw_specified = true;
                }
            } else {
                if (!wp.contains("x") || !wp.contains("y")) {
                    RCLCPP_WARN(this->get_logger(),
                        "跳过无效本地航点: 缺少 x 或 y 字段");
                    continue;
                }
                pose.x = wp["x"].get<double>();
                pose.y = wp["y"].get<double>();
                pose.yaw = wp.value("yaw", 0.0);
                pose.yaw_specified = wp.contains("yaw");
            }
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
            } else if (localization_received_) {
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
                    "收到多航点任务: %zu 个航点, coord_mode=%s",
                    waypoint_queue_.size(), coord_mode.c_str());

        state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);

    } catch (const nlohmann::json::exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "/nav_waypoints JSON 解析失败: %s", e.what());
    }
}

// ==================== 动态避障辅助 (GPS 模式) ====================

bool SlamNavNode::IsRawPathBlocked(const std::vector<Waypoint>& path,
                                   const Pose2D& robot,
                                   double lookahead_dist) const
{
    if (raw_obstacle_pts_.empty() || path.empty()) return false;

    // 找最近路径点
    size_t nearest = 0;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
        if (d < min_d) { min_d = d; nearest = i; }
    }

    const double half_w = obstacle_corridor_width_;

    // 检查前方 lookahead_dist 范围内各路径点是否被障碍点侵入廊道
    for (size_t i = nearest; i < path.size(); ++i) {
        if (std::hypot(path[i].x - robot.x, path[i].y - robot.y) > lookahead_dist) break;
        for (const auto& [ox, oy] : raw_obstacle_pts_) {
            double d = std::hypot(ox - path[i].x, oy - path[i].y);
            if (d < half_w) return true;
        }
    }
    return false;
}

// ==================== TF 位姿获取 ====================

Pose2D SlamNavNode::ComputeGpsDetourWaypoint(
    const std::vector<Waypoint>& path, const Pose2D& robot) const
{
    // 找最近路径点
    size_t nearest = 0;
    double min_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i) {
        double d = std::hypot(path[i].x - robot.x, path[i].y - robot.y);
        if (d < min_d) { min_d = d; nearest = i; }
    }

    // 沿路径向前走 detour_forward_ 找目标参考点
    size_t ref_i = nearest;
    double acc = 0.0;
    for (size_t i = nearest + 1; i < path.size(); ++i) {
        acc += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
        ref_i = i;
        if (acc >= detour_forward_) break;
    }

    // 路径方向 (单位向量)
    double dir_x = 1.0, dir_y = 0.0;
    if (ref_i > 0) {
        double dx = path[ref_i].x - path[ref_i - 1].x;
        double dy = path[ref_i].y - path[ref_i - 1].y;
        double len = std::hypot(dx, dy);
        if (len > 1e-6) { dir_x = dx / len; dir_y = dy / len; }
    }

    // 左侧垂直方向 (+90°)
    const double left_x = -dir_y, left_y = dir_x;

    // 参考点世界坐标
    const double cx = path[ref_i].x;
    const double cy = path[ref_i].y;

    // 统计左/右各 kCheckRadius 内的障碍点数
    constexpr double kCheckRadius = 4.0;
    int left_cnt = 0, right_cnt = 0;
    for (const auto& [ox, oy] : raw_obstacle_pts_) {
        if (std::hypot(ox - cx, oy - cy) > kCheckRadius) continue;
        double side = (ox - cx) * left_x + (oy - cy) * left_y;
        if (side >= 0.0) left_cnt++;
        else             right_cnt++;
    }

    // 选障碍更少的一侧
    const double lat_sign = (left_cnt <= right_cnt) ? 1.0 : -1.0;

    Pose2D detour;
    detour.x   = cx + lat_sign * left_x * detour_lateral_;
    detour.y   = cy + lat_sign * left_y * detour_lateral_;
    detour.yaw = std::atan2(dir_y, dir_x);
    detour.yaw_specified = false;
    return detour;
}

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
            localization_received_ = true;
        }
    }

    NavState state = state_machine_.GetState();

    switch (state) {
        case NavState::IDLE:
        case NavState::REACHED:
            break;

        case NavState::PLANNING: {
            if (!localization_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                    "等待定位里程计数据 (%s)...", localization_odom_topic_.c_str());
                return;
            }

            if (!goal_pose_.has_value()) {
                state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                return;
            }

            bool ok = false;
            if (use_astar_ && astar_planner_.HasMap()) {
                // 保存规划前的旧路径, 供重规划失败时的停车等待使用
                std::vector<Waypoint> prev_path = astar_planner_.HasValidPath()
                    ? astar_planner_.GetPath() : std::vector<Waypoint>{};

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
                    ok = false;
                    // 恢复旧路径供 TRACKING 的 IsPathBlocked 检测使用
                    if (!prev_path.empty()) {
                        astar_planner_.RestorePath(prev_path);
                    }
                }
            }

            if (!ok) {
                // gps 模式或显式关闭 A* 时, 使用直线规划
                if (!use_astar_) {
                    if (multi_nav_active_ && waypoint_queue_.size() > 1) {
                        ok = planner_.PlanMulti(current_pose_, waypoint_queue_);
                    } else {
                        ok = planner_.Plan(current_pose_, goal_pose_.value(), true);
                    }

                    if (ok && planner_.HasValidPath()) {
                        tracker_.SetPath(planner_.GetPath(), true);
                        state_machine_.HandleEvent(NavEvent::PLAN_SUCCESS);
                        actual_trajectory_.poses.clear();
                        last_traj_x_ = current_pose_.x;
                        last_traj_y_ = current_pose_.y;
                        RCLCPP_INFO(this->get_logger(), "直线路径规划成功, %zu 个路径点",
                                    planner_.GetPath().size());
                    } else {
                        state_machine_.HandleEvent(NavEvent::PLAN_FAILED);
                        RCLCPP_ERROR(this->get_logger(), "直线路径规划失败");
                    }
                } else {
                    // A* 重规划失败: 可能是动态障碍物临时堵塞通道
                    // 不直接取消导航, 而是回退到 TRACKING 状态停车等待;
                    // 冷却时间结束后 IsPathBlocked 会再次触发重规划
                    RCLCPP_WARN(this->get_logger(),
                        "A* 重规划失败 (通道可能被障碍物临时堵塞), 停车等待障碍移开...");
                    // 旧路径已在上方 RestorePath 恢复, TRACKING 里 IsPathBlocked
                    // 会检测到堵塞并继续停车, 冷却到期后再次触发重规划
                    state_machine_.HandleEvent(NavEvent::PLAN_SUCCESS);
                }
            }
            break;
        }

        case NavState::TRACKING: {
            if (nav_paused_) {
                PublishStopCmd();
                return;
            }

            // 动态避障: 检查前方路径是否被堵塞
            if (obstacle_avoidance_enabled_) {
                const std::vector<Waypoint>& cur_path =
                    astar_planner_.HasValidPath() ? astar_planner_.GetPath() : planner_.GetPath();

                if (use_astar_ && astar_planner_.HasMap()) {
                    // 室内 A* 模式: 基于网格检测 → 重规划绕行
                    if (astar_planner_.IsPathBlocked(cur_path, current_pose_, replan_lookahead_)) {
                        double elapsed = (this->now() - last_replan_time_).seconds();
                        if (elapsed > replan_cooldown_) {
                            last_replan_time_ = this->now();
                            RCLCPP_INFO(this->get_logger(),
                                "[动态避障] 前方路径被堵塞, 触发重规划");
                            state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);
                        } else {
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "[动态避障] 前方路径被堵塞, 冷却等待中...");
                        }
                        // 无论是否触发重规划, 路径被堵时必须停车
                        PublishStopCmd();
                        return;
                    }
                } else {
                    // 室外 GPS 模式: 基于原始点列检测廈道
                    double raw_age = (this->now() - raw_obstacle_time_).seconds();
                    if (raw_age < dynamic_ttl_ && IsRawPathBlocked(cur_path, current_pose_, replan_lookahead_)) {
                        double elapsed = (this->now() - last_replan_time_).seconds();
                        if (elapsed > replan_cooldown_) {
                            last_replan_time_ = this->now();
                            if (gps_avoidance_ && goal_pose_.has_value()) {
                                // 绕行模式: 计算绕路点 → 重规划
                                Pose2D detour = ComputeGpsDetourWaypoint(cur_path, current_pose_);
                                waypoint_queue_ = {detour, goal_pose_.value()};
                                multi_nav_active_ = true;
                                waypoint_index_   = 0;
                                RCLCPP_INFO(this->get_logger(),
                                    "[动态避障-GPS] 已计算绕路点 (%.2f, %.2f), 触发重规划",
                                    detour.x, detour.y);
                                state_machine_.HandleEvent(NavEvent::GOAL_RECEIVED);
                                PublishStopCmd();
                                return;
                            } else {
                                // 停车等待模式
                                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "[动态避障-GPS] 前方路径被堵塞, 停车等待障碍移开...");
                            }
                        } else {
                            // 冷却期内无论哪种避障模式都停车等待
                            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "[动态避障-GPS] 前方路径被堵塞, 冷却等待中, 停车...");
                        }
                        // 路径被堵时统一停车
                        PublishStopCmd();
                        return;
                    }
                }
            }
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
    std::string state_str = (nav_paused_ && state_machine_.GetState() == NavState::TRACKING)
        ? "PAUSED" : NavStateToString(state_machine_.GetState());

    if (gps_mode_) {
        // 室外模式: 逆算回经纬度，yaw 为车辆朝向（弧度）
        const double meters_per_deg_lon =
            111320.0 * std::cos(outdoor_ref_latitude_ * kDegToRad);
        const double lat = outdoor_ref_latitude_  + current_pose_.y / 111320.0;
        const double lon = outdoor_ref_longitude_ + current_pose_.x / meters_per_deg_lon;
        oss << std::fixed << std::setprecision(8)
            << "{\"type\":\"pose\""
            << ",\"lat\":" << lat
            << ",\"lon\":" << lon
            << std::setprecision(4)
            << ",\"yaw\":" << current_pose_.yaw
            << ",\"state\":\"" << state_str << "\""
            << "}";
    } else {
        // 室内模式: map 坐标系 x/y（米），yaw（弧度）
        oss << std::fixed << std::setprecision(4)
            << "{\"type\":\"pose\""
            << ",\"x\":" << current_pose_.x
            << ",\"y\":" << current_pose_.y
            << ",\"yaw\":" << current_pose_.yaw
            << ",\"state\":\"" << state_str << "\""
            << "}";
    }

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
    if (!localization_received_) return;

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
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
    if (!localization_received_) return;

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

void SlamNavNode::PublishObstacleMarkers() {
    auto now = this->now();

    // ---- 障碍点 (POINTS Marker) ----
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp    = now;
        m.header.frame_id = map_frame_;
        m.ns              = "obstacle_pts";
        m.id              = 0;
        m.type            = visualization_msgs::msg::Marker::POINTS;
        m.action          = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.15;
        m.scale.y = 0.15;
        m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 0.9f;
        // TTL: 数据超过 dynamic_ttl_ 秒则清空
        double age;
        std::vector<std::pair<double,double>> pts;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            age = (now - raw_obstacle_time_).seconds();
            pts = raw_obstacle_pts_;
        }
        if (age < dynamic_ttl_ && !pts.empty()) {
            for (auto& [ox, oy] : pts) {
                geometry_msgs::msg::Point p;
                p.x = ox; p.y = oy; p.z = 0.2;
                m.points.push_back(p);
            }
        }
        obstacle_pts_pub_->publish(m);
    }

    // ---- 重规划触发点 (SPHERE Marker) ----
    // GPS 模式: 青色球 = 绕路中间航点 (multi_nav_active_ 为真时)
    // A*  模式: 红色球 = 当前路径上离障碍最近的路径点 (有障碍且在预视范围内时)
    {
        visualization_msgs::msg::Marker m;
        m.header.stamp    = now;
        m.header.frame_id = map_frame_;
        m.ns              = "detour_wp";
        m.id              = 0;
        m.type            = visualization_msgs::msg::Marker::SPHERE;
        m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;

        bool show = false;
        double sx = 0, sy = 0;
        float cr = 1.0f, cg = 0.0f, cb = 0.0f;  // 默认红色(A*堵塞点)

        double raw_age;
        std::vector<std::pair<double,double>> pts;
        bool multi_active = false;
        double dw_x = 0, dw_y = 0;
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            raw_age = (now - raw_obstacle_time_).seconds();
            pts = raw_obstacle_pts_;
            if (multi_nav_active_ && waypoint_queue_.size() >= 2) {
                multi_active = true;
                dw_x = waypoint_queue_[0].x;
                dw_y = waypoint_queue_[0].y;
            }
        }

        if (multi_active) {
            // GPS 模式: 青色球 = 当前绕路中间点
            sx = dw_x; sy = dw_y;
            cr = 0.0f; cg = 1.0f; cb = 1.0f;
            show = true;
        } else if (use_astar_ && raw_age < dynamic_ttl_ && !pts.empty()) {
            // A* 模式: 在路径点中找离任意障碍最近的一点
            const std::vector<Waypoint>* path_ptr = nullptr;
            if (astar_planner_.HasValidPath()) {
                path_ptr = &astar_planner_.GetPath();
            } else if (planner_.HasValidPath()) {
                path_ptr = &planner_.GetPath();
            }
            if (path_ptr && !path_ptr->empty()) {
                double min_d = std::numeric_limits<double>::max();
                for (const auto& wp : *path_ptr) {
                    for (const auto& [ox, oy] : pts) {
                        double d = std::hypot(ox - wp.x, oy - wp.y);
                        if (d < min_d) {
                            min_d = d;
                            sx = ox; sy = oy;
                        }
                    }
                }
                // 只有障碍点落在预视距离内才显示
                if (min_d < replan_lookahead_) {
                    show = true;
                }
            }
        }

        if (show) {
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = sx;
            m.pose.position.y = sy;
            m.pose.position.z = 0.5;
            m.pose.orientation.w = 1.0;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 0.9f;
        } else {
            m.action = visualization_msgs::msg::Marker::DELETE;
        }
        detour_wp_pub_->publish(m);
    }
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
