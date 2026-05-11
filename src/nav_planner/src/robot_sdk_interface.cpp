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
    sdk_msg.vy       = 0.0;  // 履带车无横向移动
    sdk_msg.yaw_rate = Clamp(cmd.yaw_rate, params_.max_yaw_rate);
}

SdkControlMsg RobotSdkInterface::MakeStopMsg() const {
    return SdkControlMsg{};
}

}  // namespace slam_nav
