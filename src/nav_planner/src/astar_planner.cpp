// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/planning/astar_planner.h"
#include "nav_planner/common/math_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace slam_nav {

// ---- hash for Cell ----
struct CellHash {
    size_t operator()(const AStarPlanner::Cell& c) const {
        return std::hash<int>()(c.x) ^ (std::hash<int>()(c.y) << 16);
    }
};

void AStarPlanner::SetParams(const AStarParams& params) {
    params_ = params;
}

// ==================================================================
// 地图更新
// ==================================================================

void AStarPlanner::UpdateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(map_mutex_);

    map_resolution_ = msg->info.resolution;
    map_origin_x_   = msg->info.origin.position.x;
    map_origin_y_   = msg->info.origin.position.y;
    map_width_       = static_cast<int>(msg->info.width);
    map_height_      = static_cast<int>(msg->info.height);

    // 构建原始占用栅格
    costmap_.assign(map_width_ * map_height_, 0);
    for (int i = 0; i < map_width_ * map_height_; ++i) {
        int8_t v = msg->data[i];
        // -1 = unknown → 视为可通行(或可设为障碍)
        // 0-100: 0=free, 100=occupied
        if (v < 0 || v >= params_.occupied_thresh) {
            costmap_[i] = 255;
        }
    }

    InflateObstacles();
    has_map_ = true;
}

bool AStarPlanner::HasMap() const {
    return has_map_;
}

void AStarPlanner::InflateObstacles() {
    if (params_.obstacle_inflate <= 0) return;

    std::vector<uint8_t> inflated = costmap_;
    int r = params_.obstacle_inflate;

    for (int y = 0; y < map_height_; ++y) {
        for (int x = 0; x < map_width_; ++x) {
            if (costmap_[y * map_width_ + x] == 255) {
                // 膨胀周围
                for (int dy = -r; dy <= r; ++dy) {
                    for (int dx = -r; dx <= r; ++dx) {
                        if (dx * dx + dy * dy > r * r) continue;
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < map_width_ && ny >= 0 && ny < map_height_) {
                            inflated[ny * map_width_ + nx] = 255;
                        }
                    }
                }
            }
        }
    }
    costmap_ = inflated;
}

// ==================================================================
// 坐标转换
// ==================================================================

bool AStarPlanner::WorldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::floor((wx - map_origin_x_) / map_resolution_));
    gy = static_cast<int>(std::floor((wy - map_origin_y_) / map_resolution_));
    return (gx >= 0 && gx < map_width_ && gy >= 0 && gy < map_height_);
}

void AStarPlanner::GridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = map_origin_x_ + (gx + 0.5) * map_resolution_;
    wy = map_origin_y_ + (gy + 0.5) * map_resolution_;
}

bool AStarPlanner::IsTraversable(int gx, int gy) const {
    if (gx < 0 || gx >= map_width_ || gy < 0 || gy >= map_height_) return false;
    return costmap_[gy * map_width_ + gx] == 0;
}

bool AStarPlanner::FindNearestFreeCell(const Cell& from, Cell& result, int max_radius) const {
    // BFS 向外扩展搜索最近可通行格子
    for (int r = 1; r <= max_radius; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::abs(dx) != r && std::abs(dy) != r) continue;  // 只检查外圈
                int nx = from.x + dx;
                int ny = from.y + dy;
                if (IsTraversable(nx, ny)) {
                    result = {nx, ny};
                    return true;
                }
            }
        }
    }
    return false;
}

// ==================================================================
// A* 搜索
// ==================================================================

bool AStarPlanner::AStarSearch(const Cell& start, const Cell& goal,
                                std::vector<Cell>& grid_path) {
    grid_path.clear();

    // 如果起点/终点在障碍物内, 搜索附近最近可通行点
    Cell actual_start = start;
    Cell actual_goal = goal;

    if (!IsTraversable(start.x, start.y)) {
        if (!FindNearestFreeCell(start, actual_start)) {
            std::cerr << "[AStarPlanner] 起点在障碍物内且附近无可通行点\n";
            return false;
        }
        std::cerr << "[AStarPlanner] 起点在障碍物内, 调整到 ("
                  << actual_start.x << "," << actual_start.y << ")\n";
    }
    if (!IsTraversable(goal.x, goal.y)) {
        if (!FindNearestFreeCell(goal, actual_goal)) {
            std::cerr << "[AStarPlanner] 终点在障碍物内且附近无可通行点\n";
            return false;
        }
        std::cerr << "[AStarPlanner] 终点在障碍物内, 调整到 ("
                  << actual_goal.x << "," << actual_goal.y << ")\n";
    }

    // 8连通邻域
    static const int dx8[] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy8[] = {0, 1, 1, 1, 0, -1, -1, -1};
    static const double cost8[] = {1.0, 1.414, 1.0, 1.414, 1.0, 1.414, 1.0, 1.414};

    // 启发函数: 欧几里得距离
    auto heuristic = [&](int x, int y) -> double {
        double dx = x - actual_goal.x;
        double dy = y - actual_goal.y;
        return std::sqrt(dx * dx + dy * dy);
    };

    struct Node {
        int x, y;
        double g, f;
        bool operator>(const Node& o) const { return f > o.f; }
    };

    auto key = [&](int x, int y) -> int { return y * map_width_ + x; };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
    std::unordered_map<int, double> g_cost;
    std::unordered_map<int, int>    came_from;

    int start_key = key(actual_start.x, actual_start.y);
    int goal_key  = key(actual_goal.x, actual_goal.y);

    g_cost[start_key] = 0.0;
    open_list.push({actual_start.x, actual_start.y, 0.0, heuristic(actual_start.x, actual_start.y)});

    while (!open_list.empty()) {
        Node cur = open_list.top();
        open_list.pop();

        int cur_key = key(cur.x, cur.y);

        if (cur.x == actual_goal.x && cur.y == actual_goal.y) {
            // 回溯路径
            int k = goal_key;
            while (k != start_key) {
                int cy = k / map_width_;
                int cx = k % map_width_;
                grid_path.push_back({cx, cy});
                k = came_from[k];
            }
            grid_path.push_back(actual_start);
            std::reverse(grid_path.begin(), grid_path.end());
            return true;
        }

        // 如果当前节点 g 值已过时, 跳过
        auto it = g_cost.find(cur_key);
        if (it != g_cost.end() && cur.g > it->second) continue;

        for (int i = 0; i < 8; ++i) {
            int nx = cur.x + dx8[i];
            int ny = cur.y + dy8[i];

            if (!IsTraversable(nx, ny)) continue;

            int nk = key(nx, ny);
            double new_g = cur.g + cost8[i];

            auto git = g_cost.find(nk);
            if (git == g_cost.end() || new_g < git->second) {
                g_cost[nk] = new_g;
                came_from[nk] = cur_key;
                double f = new_g + heuristic(nx, ny);
                open_list.push({nx, ny, new_g, f});
            }
        }
    }

    std::cerr << "[AStarPlanner] A* 搜索失败: 无法到达目标\n";
    return false;
}

// ==================================================================
// 栅格路径 → 世界坐标路径
// ==================================================================

void AStarPlanner::GridPathToWaypoints(const std::vector<Cell>& grid_path,
                                        const Pose2D& goal, bool is_final) {
    path_.clear();
    if (grid_path.empty()) return;

    // 先收集世界坐标点
    std::vector<std::pair<double, double>> world_pts;
    world_pts.reserve(grid_path.size());
    for (const auto& c : grid_path) {
        double wx, wy;
        GridToWorld(c.x, c.y, wx, wy);
        world_pts.push_back({wx, wy});
    }

    // 路径简化: 去除共线冗余点 (Douglas-Peucker 简化的简易版)
    // 只保留方向变化的拐点
    std::vector<std::pair<double, double>> key_pts;
    key_pts.push_back(world_pts.front());

    for (size_t i = 1; i + 1 < world_pts.size(); ++i) {
        double ax = world_pts[i].first  - world_pts[i - 1].first;
        double ay = world_pts[i].second - world_pts[i - 1].second;
        double bx = world_pts[i + 1].first  - world_pts[i].first;
        double by = world_pts[i + 1].second - world_pts[i].second;
        // 叉积判断方向变化
        double cross = ax * by - ay * bx;
        if (std::abs(cross) > 1e-6) {
            key_pts.push_back(world_pts[i]);
        }
    }
    key_pts.push_back(world_pts.back());

    // 按 path_resolution 插值生成密集路径点
    double total_dist = 0.0;
    for (size_t i = 1; i < key_pts.size(); ++i) {
        double dx = key_pts[i].first  - key_pts[i - 1].first;
        double dy = key_pts[i].second - key_pts[i - 1].second;
        total_dist += std::sqrt(dx * dx + dy * dy);
    }

    double accumulated = 0.0;

    for (size_t i = 1; i < key_pts.size(); ++i) {
        double dx = key_pts[i].first  - key_pts[i - 1].first;
        double dy = key_pts[i].second - key_pts[i - 1].second;
        double seg_len = std::sqrt(dx * dx + dy * dy);
        double bearing = std::atan2(dy, dx);

        double d = 0.0;
        while (d <= seg_len + 1e-9) {
            double t = (seg_len > 1e-9) ? d / seg_len : 0.0;
            double wx = key_pts[i - 1].first  + dx * t;
            double wy = key_pts[i - 1].second + dy * t;

            Waypoint wp;
            wp.x = wx;
            wp.y = wy;
            wp.yaw = bearing;

            double dist_from_end = total_dist - (accumulated + d);
            if (is_final && dist_from_end < params_.slow_down_dist &&
                params_.slow_down_dist > 1e-6) {
                double ratio = dist_from_end / params_.slow_down_dist;
                wp.target_speed = params_.min_speed +
                                  ratio * (params_.target_speed - params_.min_speed);
            } else {
                wp.target_speed = params_.target_speed;
            }

            path_.push_back(wp);
            d += params_.path_resolution;
        }
        accumulated += seg_len;
    }

    // 确保最后一个点是目标点
    if (!path_.empty()) {
        path_.back().x = key_pts.back().first;
        path_.back().y = key_pts.back().second;
        path_.back().yaw = goal.yaw;
        path_.back().target_speed = 0.0;
    }

    // 去除过于密集的重复点
    if (path_.size() > 2) {
        std::vector<Waypoint> cleaned;
        cleaned.push_back(path_.front());
        for (size_t i = 1; i < path_.size(); ++i) {
            double dx = path_[i].x - cleaned.back().x;
            double dy = path_[i].y - cleaned.back().y;
            if (dx * dx + dy * dy >= (params_.path_resolution * 0.5) *
                                      (params_.path_resolution * 0.5)) {
                cleaned.push_back(path_[i]);
            }
        }
        // 确保终点在内
        double dx = path_.back().x - cleaned.back().x;
        double dy = path_.back().y - cleaned.back().y;
        if (dx * dx + dy * dy > 1e-6) {
            cleaned.push_back(path_.back());
        }
        path_ = cleaned;
    }
}

// ==================================================================
// 对外接口
// ==================================================================

bool AStarPlanner::Plan(const Pose2D& current, const Pose2D& goal, bool is_final) {
    std::lock_guard<std::mutex> lk(map_mutex_);

    path_.clear();
    valid_ = false;

    if (!has_map_) {
        std::cerr << "[AStarPlanner] 地图未加载\n";
        return false;
    }

    Cell start_cell, goal_cell;
    if (!WorldToGrid(current.x, current.y, start_cell.x, start_cell.y)) {
        std::cerr << "[AStarPlanner] 起点超出地图范围: ("
                  << current.x << ", " << current.y << ")\n";
        return false;
    }
    if (!WorldToGrid(goal.x, goal.y, goal_cell.x, goal_cell.y)) {
        std::cerr << "[AStarPlanner] 终点超出地图范围: ("
                  << goal.x << ", " << goal.y << ")\n";
        return false;
    }

    std::vector<Cell> grid_path;
    if (!AStarSearch(start_cell, goal_cell, grid_path)) {
        return false;
    }

    GridPathToWaypoints(grid_path, goal, is_final);
    valid_ = !path_.empty();
    return valid_;
}

bool AStarPlanner::PlanMulti(const Pose2D& current, const std::vector<Pose2D>& waypoints) {
    std::lock_guard<std::mutex> lk(map_mutex_);

    path_.clear();
    valid_ = false;

    if (!has_map_ || waypoints.empty()) return false;

    Pose2D from = current;
    for (size_t i = 0; i < waypoints.size(); ++i) {
        bool is_last = (i + 1 == waypoints.size());

        Cell start_cell, goal_cell;
        if (!WorldToGrid(from.x, from.y, start_cell.x, start_cell.y)) return false;
        if (!WorldToGrid(waypoints[i].x, waypoints[i].y, goal_cell.x, goal_cell.y)) return false;

        std::vector<Cell> grid_path;
        if (!AStarSearch(start_cell, goal_cell, grid_path)) return false;

        // 临时路径
        std::vector<Waypoint> old_path;
        std::swap(path_, old_path);

        GridPathToWaypoints(grid_path, waypoints[i], is_last);

        // 拼接
        if (!old_path.empty() && !path_.empty()) {
            // 跳过新路径第一个点(与前段终点重合)
            old_path.insert(old_path.end(), path_.begin() + 1, path_.end());
            path_ = old_path;
        } else if (!old_path.empty()) {
            path_ = old_path;
        }

        from = waypoints[i];
    }

    valid_ = !path_.empty();
    return valid_;
}

const std::vector<Waypoint>& AStarPlanner::GetPath() const {
    return path_;
}

bool AStarPlanner::HasValidPath() const {
    return valid_ && !path_.empty();
}

void AStarPlanner::ClearPath() {
    path_.clear();
    valid_ = false;
}

nav_msgs::msg::OccupancyGrid AStarPlanner::GetInflatedCostmapMsg() const {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = map_resolution_;
    grid.info.width = map_width_;
    grid.info.height = map_height_;
    grid.info.origin.position.x = map_origin_x_;
    grid.info.origin.position.y = map_origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(costmap_.size());
    for (size_t i = 0; i < costmap_.size(); ++i) {
        grid.data[i] = (costmap_[i] == 255) ? 100 : 0;
    }
    return grid;
}

}  // namespace slam_nav
