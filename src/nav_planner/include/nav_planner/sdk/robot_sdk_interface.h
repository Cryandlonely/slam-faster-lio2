// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_
#define NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_

#include "nav_planner/common/types.h"

namespace slam_nav {

/// SDK 接口参数
struct SdkInterfaceParams {
    double max_vx       = 1.5;   // 最大前进速度 (m/s)
    double max_vy       = 0.0;   // 最大横移速度 (m/s); 履带车为 0
    double max_yaw_rate = 1.0;   // 最大偏航角速度 (rad/s)
};

/// 底盘控制接口模块
///
/// 职责: 将中间控制量 (OmniControlCmd) 限幅并映射为 SdkControlMsg
class RobotSdkInterface {
public:
    RobotSdkInterface() = default;
    ~RobotSdkInterface() = default;

    void SetParams(const SdkInterfaceParams& params);
    void MapToSdkMsg(const OmniControlCmd& cmd, SdkControlMsg& sdk_msg) const;
    SdkControlMsg MakeStopMsg() const;

private:
    SdkInterfaceParams params_;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_
