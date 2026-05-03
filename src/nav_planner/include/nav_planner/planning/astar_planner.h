// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_PLANNING_ASTAR_PLANNER_H_
#define NAV_PLANNER_PLANNING_ASTAR_PLANNER_H_

#include "nav_planner/common/types.h"

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/time.hpp>

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace slam_nav {

/// A* 全局路径规划参数
struct AStarParams {
    double path_resolution   = 0.05;   // 输出路径点间距 (m)
    double target_speed      = 0.5;    // 默认行进速度 (m/s)
    double slow_down_dist    = 0.5;    // 末段减速距离 (m)
    double min_speed         = 0.05;   // 减速最低速度 (m/s)
    int    obstacle_inflate  = 3;      // 障碍物膨胀格数 (安全距离 = inflate × map_resolution)
    int    occupied_thresh   = 65;     // 栅格值 >= 此值视为障碍 (OccupancyGrid: 0-100)
};

/// A* 全局路径规划器 (基于 OccupancyGrid)
///
/// 使用流程:
///   1. UpdateMap(msg) — 接收 OccupancyGrid
///   2. Plan(current, goal) — 基于地图做 A* 搜索
///   3. GetPath() — 获取带速度约束的路径点序列
class AStarPlanner {
public:
    AStarPlanner() = default;
    ~AStarPlanner() = default;

    void SetParams(const AStarParams& params);
    const AStarParams& GetParams() const { return params_; }

    /// 更新代价地图 (从 OccupancyGrid 话题回调)
    void UpdateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    /// 判断地图是否已加载
    bool HasMap() const;

    /// 执行 A* 规划
    bool Plan(const Pose2D& current, const Pose2D& goal, bool is_final = true);

    /// 多航点依次规划并拼接
    bool PlanMulti(const Pose2D& current, const std::vector<Pose2D>& waypoints);

    const std::vector<Waypoint>& GetPath() const;
    bool HasValidPath() const;
    void ClearPath();

    /// 将旧路径恢复到 path_ (用于重规划失败后继续停车等待)
    void RestorePath(const std::vector<Waypoint>& saved_path);

    /// 获取膨胀后的代价地图 (OccupancyGrid 格式, 用于 RViz 可视化)
    nav_msgs::msg::OccupancyGrid GetInflatedCostmapMsg() const;

    /// 注入动态障碍物 (map 坐标系 2D 点列表, ttl_sec 秒后自动过期)
    void UpdateDynamicObstacles(const std::vector<std::pair<double,double>>& map_points,
                                rclcpp::Time now, double ttl_sec);

    /// 清除已过期的动态障碍格子
    void ClearExpiredObstacles(rclcpp::Time now);

    /// 检查路径前方 lookahead_dist 范围内是否被动态/静态障碍堵塞
    bool IsPathBlocked(const std::vector<Waypoint>& path,
                       const Pose2D& robot,
                       double lookahead_dist) const;

    // 栅格坐标
    struct Cell {
        int x, y;
        bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
    };

private:

    // 世界坐标 → 栅格
    bool WorldToGrid(double wx, double wy, int& gx, int& gy) const;
    // 栅格 → 世界坐标 (栅格中心)
    void GridToWorld(int gx, int gy, double& wx, double& wy) const;

    // A* 搜索，返回栅格路径
    bool AStarSearch(const Cell& start, const Cell& goal, std::vector<Cell>& grid_path);

    // 栅格路径 → 平滑世界坐标路径 (带速度)
    void GridPathToWaypoints(const std::vector<Cell>& grid_path,
                             const Pose2D& goal, bool is_final);

    // 检查栅格是否可通行
    bool IsTraversable(int gx, int gy) const;

    // 搜索附近最近可通行栅格 (BFS)
    bool FindNearestFreeCell(const Cell& from, Cell& result, int max_radius = 30) const;

    // 膨胀障碍物
    void InflateObstacles();

    AStarParams params_;
    std::vector<Waypoint> path_;
    bool valid_ = false;

    // 地图数据
    bool has_map_ = false;
    double map_resolution_ = 0.05;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    int map_width_ = 0;
    int map_height_ = 0;
    std::vector<uint8_t> costmap_;   // 膨胀后的代价地图: 0=free, 255=obstacle
    mutable std::mutex map_mutex_;

    // 动态障碍层: grid_idx → 过期时间
    mutable std::unordered_map<int, rclcpp::Time> dynamic_cells_;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_PLANNING_ASTAR_PLANNER_H_
