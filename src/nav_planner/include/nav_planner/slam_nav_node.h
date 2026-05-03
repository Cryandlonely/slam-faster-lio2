// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 改造自 207_ws nav_planner_node.h
// 核心变更:
//   1. 定位源从 RTK(rtk/odom + rtk/heading) 改为 SLAM(Faster-LIO2 输出的单一odom)
//   2. 去除 GPS 坐标系 (ref_lat/lon, GpsToLocal, NavSatFix)
//   3. 航点使用 map 坐标系本地坐标 (x, y, yaw)
//   4. 增加 tf2 监听器获取 map→base_link 变换
//   5. 参数默认值适配室内环境 (低速、小空间)

#ifndef NAV_PLANNER_SLAM_NAV_NODE_H_
#define NAV_PLANNER_SLAM_NAV_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "nav_planner/common/types.h"
#include "nav_planner/planning/astar_planner.h"
#include "nav_planner/planning/point_to_point_planner.h"
#include "nav_planner/tracking/pure_pursuit_tracker.h"
#include "nav_planner/sdk/robot_sdk_interface.h"
#include "nav_planner/fsm/navigation_state_machine.h"

#include <mutex>
#include <optional>
#include <vector>

namespace slam_nav {

/// SlamNavNode: 室内SLAM导航 ROS2 节点
///
/// 职责: 订阅SLAM位姿、规划路径、跟踪控制、底盘指令发布
/// 定位源: Faster-LIO2 输出的里程计 (nav_msgs/Odometry)
///        或通过 tf2 监听 map → base_link 变换
class SlamNavNode : public rclcpp::Node {
public:
    explicit SlamNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SlamNavNode() override = default;

private:
    // ---- 参数声明与加载 ----
    void DeclareAndLoadParams();
    bool ConvertOutdoorGpsToLocal(double lat, double lon, double& x, double& y) const;

    // ---- 回调函数 ----
    /// 统一定位里程计回调
    void LocalizationOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// 运行时定位源切换命令回调 (来自 bridge 的 /nav_mode_cmd)
    /// JSON 格式: {"action":"switch_odom","topic":"/localization","gps":false}
    void NavModeCmdCallback(const std_msgs::msg::String::SharedPtr msg);

    /// 运行时切换定位订阅话题（重新创建 subscription）
    void SwitchLocalizationSource(const std::string& topic, bool gps_mode);

    /// RViz2 目标点回调 (map 坐标系)
    void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /// 本地坐标航点列表回调 (JSON: { "waypoints": [{"x":..,"y":..,"yaw":..},...] })
    void NavWaypointsCallback(const std_msgs::msg::String::SharedPtr msg);

    /// 取消导航回调
    void NavCancelCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /// 暂停/继续导航回调 (true=暂停, false=继续)
    void NavPauseCallback(const std_msgs::msg::Bool::SharedPtr msg);

    /// 初始位姿回调 (用于手动校正 SLAM 初始位置)
    void InitialPoseCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    /// Livox 点云回调——识别动态障碍并写入代价地图
    void LivoxCallback(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);

    /// GPS 模式下使用原始世界坐标点列检查路径廊道
    bool IsRawPathBlocked(const std::vector<Waypoint>& path,
                          const Pose2D& robot,
                          double lookahead_dist) const;

    /// GPS 模式: 计算绕行绕路点 (评估左右两侧障碍数量, 取较少一侧)
    Pose2D ComputeGpsDetourWaypoint(const std::vector<Waypoint>& path,
                                   const Pose2D& robot) const;

    // ---- 多航点队列管理 ----
    void CancelMultiNav();

    // ---- 控制循环 ----
    void ControlLoop();

    // ---- TF 位姿获取 (备用: 通过 tf2 获取 map→base_link) ----
    bool GetPoseFromTF(Pose2D& pose);

    // ---- 发布函数 ----
    void PublishPath();
    void PublishStatus();
    void PublishCurrentPose();
    void PublishTrackingDebug();
    void PublishControlCmd(const SdkControlMsg& sdk_msg);
    void PublishStopCmd();
    void PublishTF();                 // 广播 map → base_link TF (使用 SLAM 数据)
    void PublishActualTrajectory();   // 累积实际运动轨迹
    void PublishGoalMarker();         // 发布目标点 Marker
    void PublishObstacleMarkers();    // 发布障碍点(map坐标系) + 绕路点 Marker

    // ---- 状态机回调 ----
    void OnStateChange(NavState old_state, NavState new_state);

    // ==================== 四个核心模块 ====================
    AStarPlanner        astar_planner_;
    PointToPointPlanner planner_;      // 直线规划 (fallback)
    PurePursuitTracker  tracker_;
    RobotSdkInterface   sdk_interface_;
    NavigationStateMachine state_machine_;

    // ==================== ROS2 接口 ====================
    // 订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   nav_cancel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr   nav_pause_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_mode_cmd_sub_;  // 运行时定位源切换
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;

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
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_pts_pub_;   // 障碍点云 Marker
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detour_wp_pub_;      // 绕路点 Marker

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 定时器
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // ==================== 状态数据 ====================
    std::mutex data_mutex_;

    // 当前位姿 (来自统一定位里程计)
    Pose2D current_pose_;
    bool   localization_received_ = false;

    // 当前定位的 z 坐标
    double current_z_ = 0.0;

    // 实际运动轨迹 (累积)
    nav_msgs::msg::Path actual_trajectory_;
    double last_traj_x_ = 0.0;
    double last_traj_y_ = 0.0;

    // 目标位姿
    std::optional<Pose2D> goal_pose_;

    // 多航点队列
    std::vector<Pose2D> waypoint_queue_;
    size_t waypoint_index_ = 0;
    bool   multi_nav_active_ = false;

    bool   nav_paused_ = false;        // 导航暂停标志

    // 动态避障: 每帧过滤后的障碍物 map 坐标 (全模式共用)
    std::vector<std::pair<double,double>> raw_obstacle_pts_;
    rclcpp::Time raw_obstacle_time_{0, 0, RCL_ROS_TIME};

    // 动态避障重规划
    rclcpp::Time last_replan_time_{0, 0, RCL_ROS_TIME};
    bool   obstacle_avoidance_enabled_ = true;  // 动态避障开关

    // ==================== 参数 ====================
    double control_rate_ = 20.0;       // 控制循环频率 (Hz)
    double status_rate_  = 2.0;        // 状态发布频率 (Hz)
    bool   use_tf_pose_  = false;      // 是否从 TF 获取位姿 (而非 odom 话题)
    std::string nav_mode_ = "slam";   // slam / gps
    bool   gps_mode_ = false;          // 是否室外 GPS 模式
    std::string slam_odom_topic_ = "/slam/odom";   // SLAM 里程计话题名
    std::string outdoor_odom_topic_ = "/outdoor/odom";  // 室外统一位姿话题
    std::string localization_odom_topic_ = "/slam/odom";  // 当前生效的定位话题
    double outdoor_ref_latitude_ = 36.66111;
    double outdoor_ref_longitude_ = 117.01665;
    std::string map_frame_  = "map";                // 地图坐标系
    std::string base_frame_ = "base_link";           // 机器人坐标系
    std::string map_topic_  = "map";                   // OccupancyGrid 话题
    bool use_astar_ = true;                             // 是否使用 A* 避障规划

    // 动态避障参数
    std::string lidar_topic_  = "/livox/lidar";  // Livox 点云话题
    std::string lidar_frame_  = "livox_frame";   // Livox 驱动帧 ID
    double obstacle_z_min_    = 0.15;    // 点云高度下限(雷达坐标系), m
    double obstacle_z_max_    = 2.0;     // 点云高度上限, m
    double obstacle_fov_half_deg_ = 90.0; // 前向半角 FOV (度), 90=前半球
    double obstacle_corridor_width_ = 0.6; // 路径廈道半宽 (m), GPS模式障碍检测用
    bool   gps_avoidance_        = false; // GPS模式: true=计算绕路点绕行, false=停车等待
    double detour_lateral_       = 1.5;   // 绕路点横向偏移 (m)
    double detour_forward_       = 1.5;   // 绕路点向前偏移 (m)
    double dynamic_ttl_       = 3.0;     // 动态障碍 TTL (s)
    double replan_lookahead_  = 3.0;     // 检测预视距离 (m)
    double replan_cooldown_   = 2.0;     // 重规划冷却时间 (s)
    int    lidar_subsample_   = 5;       // 点云采样间隔(降低 CPU 占用)
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_SLAM_NAV_NODE_H_
