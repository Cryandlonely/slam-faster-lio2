// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#ifndef NAV_PLANNER_FSM_NAVIGATION_STATE_MACHINE_H_
#define NAV_PLANNER_FSM_NAVIGATION_STATE_MACHINE_H_

#include "nav_planner/common/types.h"

#include <functional>
#include <string>

namespace slam_nav {

/// 导航状态机事件
enum class NavEvent : uint8_t {
    GOAL_RECEIVED    = 0,  // 收到新目标
    PLAN_SUCCESS     = 1,  // 规划成功
    PLAN_FAILED      = 2,  // 规划失败
    GOAL_REACHED     = 3,  // 到达目标
    TRACKING_LOST    = 4,  // 跟踪丢失 / 超时
    CANCEL           = 5,  // 取消导航
    RESET            = 6,  // 重置
};

/// 状态转换回调
using StateChangeCallback = std::function<void(NavState old_state,
                                               NavState new_state)>;

/// 导航状态机
///
/// 状态转移图:
///   IDLE      --GOAL_RECEIVED-->  PLANNING
///   PLANNING  --PLAN_SUCCESS-->   TRACKING
///   PLANNING  --PLAN_FAILED-->    ERROR
///   TRACKING  --GOAL_REACHED-->   REACHED
///   TRACKING  --TRACKING_LOST-->  ERROR
///   REACHED   --GOAL_RECEIVED-->  PLANNING
///   REACHED   --RESET-->          IDLE
///   ERROR     --GOAL_RECEIVED-->  PLANNING
///   ERROR     --RESET-->          IDLE
///   ANY       --CANCEL-->         IDLE
class NavigationStateMachine {
public:
    NavigationStateMachine() = default;
    ~NavigationStateMachine() = default;

    void HandleEvent(NavEvent event);
    NavState GetState() const;
    std::string GetStateString() const;
    void SetCallback(StateChangeCallback cb);

private:
    void TransitionTo(NavState new_state);
    NavState state_ = NavState::IDLE;
    StateChangeCallback callback_;
};

}  // namespace slam_nav

#endif  // NAV_PLANNER_FSM_NAVIGATION_STATE_MACHINE_H_
