// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_
#define NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_

#include "nav_planner/common/types.h"

namespace slam_nav {

/// SDK 接口参数
struct SdkInterfaceParams {
    double max_vx           = 1.0;   // 最大前进速度 (m/s), 室内限速
    double max_vy           = 0.8;   // 最大横移速度 (m/s)
    double max_yaw_rate     = 1.0;   // 最大偏航角速度 (rad/s)
    int8_t default_mode     = 3;     // 默认 SDK 模式 (3=站立)
    bool   enable_posture   = false; // 是否启用姿态控制字段
};

/// 机械狗 SDK 控制接口模块
///
/// 职责: 将中间控制量 (OmniControlCmd) 映射为 SDK 控制消息 (SdkControlMsg)
class RobotSdkInterface {
public:
    RobotSdkInterface() = default;
    ~RobotSdkInterface() = default;

    void SetParams(const SdkInterfaceParams& params);
    void MapToSdkMsg(const OmniControlCmd& cmd, SdkControlMsg& sdk_msg) const;
    SdkControlMsg MakeStopMsg() const;
    void SetMode(int8_t mode);
    int8_t GetMode() const;

private:
    SdkInterfaceParams params_;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_SDK_ROBOT_SDK_INTERFACE_H_
