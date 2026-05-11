// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
// 定位层抽象（支持SLAM和GPS两种模式）

#ifndef NAV_PLANNER_COMMON_LOCALIZATION_H_
#define NAV_PLANNER_COMMON_LOCALIZATION_H_

#include "nav_planner/common/types.h"
#include <memory>
#include <optional>

namespace slam_nav {

/**
 * @brief 定位数据提供器接口 (抽象基类)
 * 
 * 职责：
 *   - 从各种定位源（SLAM/GPS）获取当前位姿和航向
 *   - 屏蔽掉底层定位方案差异
 * 
 * 实现：
 *   - SlamLocalizationProvider (从SLAM话题)
 *   - GpsLocalizationProvider (从RTK话题，含坐标转换)
 */
class LocalizationProvider {
public:
    virtual ~LocalizationProvider() = default;

    /// 获取当前位姿（本地坐标系）
    /// @return Pose2D 或 std::nullopt（如果未初始化）
    virtual std::optional<Pose2D> GetCurrentPose() const = 0;

    /// 获取当前航向（弧度，ENU约定：正东=0，逆时针为正）
    /// @return 航向角(rad) 或 std::nullopt
    virtual std::optional<double> GetCurrentHeading() const = 0;

    /// 是否已初始化（有有效的定位信息）
    virtual bool IsReady() const = 0;

    /// 获取定位信息的时间戳（秒）
    virtual double GetTimestamp() const = 0;
};

/**
 * @brief SLAM定位提供器
 * 
 * 数据源：
 *   - 订阅 /Odometry (Faster-LIO2输出)
 *   - 或通过 tf2 监听 map→base_link
 */
class SlamLocalizationProvider : public LocalizationProvider {
public:
    explicit SlamLocalizationProvider(class rclcpp::Node* node);
    ~SlamLocalizationProvider() override = default;

    std::optional<Pose2D> GetCurrentPose() const override;
    std::optional<double> GetCurrentHeading() const override;
    bool IsReady() const override;
    double GetTimestamp() const override;

    /// 设置当前位姿（通常在回调中调用）
    void SetPose(const Pose2D& pose, double timestamp);

private:
    Pose2D current_pose_;
    double timestamp_ = -1.0;
    bool is_ready_ = false;
};

/**
 * @brief GPS/RTK定位提供器
 * 
 * 数据源：
 *   - 订阅 rtk/fix (NavSatFix, WGS84)
 *   - 订阅 rtk/heading (Float64)
 *   - 订阅 rtk/odom (Odometry, 可选)
 * 
 * 坐标变换：
 *   WGS84 (经纬度) → 本地平面坐标 (米)
 *   使用参考点和墨卡托投影
 */
class GpsLocalizationProvider : public LocalizationProvider {
public:
    /// @param node ROS2节点指针
    /// @param ref_lat 参考点纬度 (度)
    /// @param ref_lon 参考点经度 (度)
    explicit GpsLocalizationProvider(class rclcpp::Node* node, 
                                     double ref_lat, double ref_lon);
    ~GpsLocalizationProvider() override = default;

    std::optional<Pose2D> GetCurrentPose() const override;
    std::optional<double> GetCurrentHeading() const override;
    bool IsReady() const override;
    double GetTimestamp() const override;

    /// 设置GPS坐标（WGS84）和航向
    void SetGpsData(double lat, double lon, double altitude, 
                    double heading, double timestamp);

private:
    /// WGS84 (lat, lon) → 本地平面坐标 (x, y)
    /// 基于参考点和简化的等距圆柱投影
    void ConvertGpsToLocal(double lat, double lon, double& x, double& y) const;

    double ref_lat_, ref_lon_;  // 参考点 (度)
    double current_lat_, current_lon_, current_alt_;
    double current_heading_;
    double timestamp_ = -1.0;
    bool is_ready_ = false;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_COMMON_LOCALIZATION_H_
