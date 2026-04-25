// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
// 直线路径规划器（室外GPS导航）

#ifndef NAV_PLANNER_PLANNING_LINE_PLANNER_H_
#define NAV_PLANNER_PLANNING_LINE_PLANNER_H_

#include "nav_planner/common/types.h"
#include <vector>
#include <optional>

namespace slam_nav {

/**
 * @brief 直线路径规划器
 * 
 * 功能：
 *   - 从起点到目标点规划直线路径
 *   - 不依赖栅格地图或障碍物检测
 *   - 适用于开阔的室外环境（GPS导航）
 * 
 * 算法：
 *   - 简单直线连接
 *   - 可选的中间航点微调
 */
class LinePlanner {
public:
    struct Config {
        double waypoint_spacing = 1.0;  // 路径点间距 (m)
        int max_waypoints = 100;         // 最大路径点数
    };

    explicit LinePlanner(const Config& config = Config());
    ~LinePlanner() = default;

    /**
     * @brief 规划直线路径
     * 
     * @param start 起点
     * @param goal 目标点
     * @param target_heading 目标航向（可选）
     * @return 路径点列表，包含起点和终点
     */
    std::vector<Waypoint> Plan(const Pose2D& start, const Pose2D& goal,
                               std::optional<double> target_heading = std::nullopt);

    /**
     * @brief 规划到多个航点的路径
     * 
     * @param start 起点
     * @param goals 目标点列表
     * @return 完整路径
     */
    std::vector<Waypoint> PlanToMultipleGoals(const Pose2D& start,
                                               const std::vector<Pose2D>& goals);

    /// 设置配置
    void SetConfig(const Config& config) { config_ = config; }

private:
    Config config_;

    /// 计算两点间的距离
    static double Distance(double x1, double y1, double x2, double y2);

    /// 计算方向角 (atan2)
    static double GetHeading(double dx, double dy);
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_PLANNING_LINE_PLANNER_H_
