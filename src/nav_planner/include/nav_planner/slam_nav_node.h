// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 基于 207_ws nav_planner_node.h 重写
// 核心变更:
//   1. 定位源从 RTK(rtk/odom + rtk/heading) 改为单一 /outdoor/odom
//   2. 去除 A* 规划、livox 动态避障、代价地图
//   3. 增加 nav_pause 支持
//   4. 状态发布格式适配 bridge_node (带 lat/lon 字段)

#ifndef NAV_PLANNER_SLAM_NAV_NODE_H_
#define NAV_PLANNER_SLAM_NAV_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "nav_planner/common/types.h"
#include "nav_planner/planning/point_to_point_planner.h"
#include "nav_planner/tracking/pure_pursuit_tracker.h"
#include "nav_planner/sdk/robot_sdk_interface.h"
#include "nav_planner/fsm/navigation_state_machine.h"

#include <mutex>
#include <optional>
#include <vector>

namespace slam_nav {

/// SlamNavNode: GPS 室外导航 ROS2 节点
///
/// 职责: 订阅 /outdoor/odom 位姿、规划路径、跟踪控制、底盘指令发布
/// 定位源: RTK+INS 输出的统一里程计 (nav_msgs/Odometry), 带 GPS lat/lon
class SlamNavNode : public rclcpp::Node {
public:
    explicit SlamNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNavNode() override = default;

private:
    // ---- 参数声明与加载 ----
    void DeclareAndLoadParams();

    // ---- 坐标转换 ----
    void GpsToLocal(double lat, double lon, double& x, double& y) const;
    void LocalToGps(double x, double y, double& lat, double& lon) const;

    // ---- 回调函数 ----
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void NavWaypointsCallback(const std_msgs::msg::String::SharedPtr msg);
    void NavCancelCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void NavPauseCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void ChassisFeedbackCallback(const std_msgs::msg::String::SharedPtr msg);  // 底盘反馈实际速度
    void LivoxCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);  // 前方过滤盒避障

    // ---- 多航点队列管理 ----
    void CancelMultiNav();

    // ---- 避障 (前方过滤盒) ----
    /// 用最近一次检测到的车体系障碍质心生成绕障路径并下发给 tracker。
    /// 返回是否成功重新规划。
    bool TriggerAvoidanceReplan();

    // ---- 控制循环 ----
    void ControlLoop();

    // ---- 发布函数 ----
    void PublishPath();
    void PublishStatus();
    void PublishCurrentPose();
    void PublishTrackingDebug();
    void PublishControlCmd(const SdkControlMsg& sdk_msg);
    void PublishStopCmd();
    void PublishTF();
    void PublishActualTrajectory();
    void PublishGoalMarker();
    void PublishPlannedRouteMarker();

    // ---- 状态机回调 ----
    void OnStateChange(NavState old_state, NavState new_state);

    // ==================== 四个核心模块 ====================
    PointToPointPlanner planner_;
    PurePursuitTracker  tracker_;
    RobotSdkInterface   sdk_interface_;
    NavigationStateMachine state_machine_;

    // ==================== ROS2 接口 ====================
    // 订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   nav_cancel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   nav_pause_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chassis_feedback_sub_;  // 底盘反馈
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_cloud_sub_;  // 前方避障点云

    // 发布
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        tracking_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        robot_action_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          robot_enable_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          actual_trajectory_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planned_route_marker_pub_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ==================== 状态数据 ====================
    std::mutex data_mutex_;

    Pose2D current_pose_;
    bool   odom_received_ = false;
    double current_z_     = 0.0;

    // 实际运动轨迹
    nav_msgs::msg::Path actual_trajectory_;
    double last_traj_x_ = 0.0;
    double last_traj_y_ = 0.0;

    // 目标位姿
    std::optional<Pose2D> goal_pose_;

    // 多航点队列
    std::vector<Pose2D> waypoint_queue_;
    size_t waypoint_index_ = 0;
    bool   multi_nav_active_ = false;

    bool nav_paused_ = false;

    double actual_chassis_speed_ = -1.0;  // 底盘反馈实际线速 (m/s); 调用 SetActualSpeed 传入 tracker

    // ==================== 避障状态 (前方过滤盒) ====================
    // 车体系障碍质心 (livox 雷达坐标系: x前 y左 z上, 与 body 几乎重合)
    double obs_local_x_  = 0.0;
    double obs_local_y_  = 0.0;
    int    obs_hit_count_   = 0;        // 连续命中帧数 (用于触发)
    int    obs_miss_count_  = 0;        // 连续未命中帧数 (用于解除)
    bool   obs_trigger_replan_ = false; // 在 control loop 中执行重规划
    bool   obs_trigger_restore_ = false; // 障碍解除后触发恢复原路线
    bool   avoiding_        = false;    // 当前是否处于绕障状态
    // 避障终点 (世界系) + 触发时的前向单位向量, 用于判断车是否已绕过障碍
    double obs_bp2_world_x_ = 0.0;
    double obs_bp2_world_y_ = 0.0;
    double obs_forward_x_   = 1.0;
    double obs_forward_y_   = 0.0;
    // 避障触发时的车体位置, 用于判断"车是否还基本未启动"
    double obs_trigger_x_   = 0.0;
    double obs_trigger_y_   = 0.0;
    bool   obs_triggered_while_static_ = false;  // 触发时车几乎静止 (允许快速 restore)
    rclcpp::Time last_replan_time_;     // 节流: 距上次重规划至少 N 秒才能再次触发

    // ==================== 参数 ====================
    double control_rate_           = 10.0;
    double status_rate_            = 2.0;
    std::string odom_topic_        = "/outdoor/odom";
    double ref_latitude_           = 36.66111;
    double ref_longitude_          = 117.01665;
    std::string map_frame_         = "map";
    std::string base_frame_        = "base_link";

    // 避障参数
    bool        obs_enabled_       = false;
    std::string obs_cloud_topic_   = "/livox/lidar";
    double      obs_x_min_         = 0.3;
    double      obs_x_max_         = 1.2;
    double      obs_y_min_         = -1.0;
    double      obs_y_max_         = 1.0;
    double      obs_z_min_         = 1.0;
    double      obs_z_max_         = 2.0;
    int         obs_min_points_    = 10;
    int         obs_hold_clear_frames_ = 5;
    int         obs_trigger_frames_    = 1;     // 连续 N 帧命中才触发, 防偶发噪声
    double      obs_bypass_offset_     = 1.2;
    double      obs_bypass_forward_clear_ = 1.5;
    std::string obs_bypass_side_       = "right";  // "right" / "left"
    double      obs_replan_cooldown_sec_  = 2.0;   // 两次重规划最小间隔
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_SLAM_NAV_NODE_H_

