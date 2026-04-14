// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.
//
// 改造自 207_ws robot_sdk_interface.cpp
// 命名空间: slam_nav (原 nav_planner)
// 功能完全一致

#include "nav_planner/sdk/robot_sdk_interface.h"
#include "nav_planner/common/math_utils.h"

namespace slam_nav {

void RobotSdkInterface::SetParams(const SdkInterfaceParams& params) {
    params_ = params;
}

void RobotSdkInterface::MapToSdkMsg(const OmniControlCmd& cmd,
                                     SdkControlMsg& sdk_msg) const {
    sdk_msg.vx       = Clamp(cmd.vx,       params_.max_vx);
    sdk_msg.vy       = Clamp(cmd.vy,       params_.max_vy);
    sdk_msg.yaw_rate = Clamp(cmd.yaw_rate, params_.max_yaw_rate);

    sdk_msg.mode = params_.default_mode;

    if (params_.enable_posture) {
        // 保留外部设置
    } else {
        sdk_msg.roll        = 0.0;
        sdk_msg.pitch       = 0.0;
        sdk_msg.yaw         = 0.0;
        sdk_msg.height_rate = 0.0;
    }
}

SdkControlMsg RobotSdkInterface::MakeStopMsg() const {
    SdkControlMsg msg;
    msg.vx          = 0.0;
    msg.vy          = 0.0;
    msg.yaw_rate    = 0.0;
    msg.mode        = params_.default_mode;
    msg.roll        = 0.0;
    msg.pitch       = 0.0;
    msg.yaw         = 0.0;
    msg.height_rate = 0.0;
    return msg;
}

void RobotSdkInterface::SetMode(int8_t mode) {
    params_.default_mode = mode;
}

int8_t RobotSdkInterface::GetMode() const {
    return params_.default_mode;
}

}  // namespace slam_nav
