// Copyright 2024 nav_planner Authors. All rights reserved.
// Licensed under the Apache-2.0 License.

#include "nav_planner/fsm/navigation_state_machine.h"

namespace slam_nav {

void NavigationStateMachine::HandleEvent(NavEvent event) {
    if (event == NavEvent::CANCEL) {
        TransitionTo(NavState::IDLE);
        return;
    }

    if (event == NavEvent::RESET) {
        if (state_ == NavState::REACHED || state_ == NavState::ERROR ||
            state_ == NavState::IDLE) {
            TransitionTo(NavState::IDLE);
        }
        return;
    }

    switch (state_) {
        case NavState::IDLE:
            if (event == NavEvent::GOAL_RECEIVED) {
                TransitionTo(NavState::PLANNING);
            }
            break;

        case NavState::PLANNING:
            if (event == NavEvent::PLAN_SUCCESS) {
                TransitionTo(NavState::TRACKING);
            } else if (event == NavEvent::PLAN_FAILED) {
                TransitionTo(NavState::ERROR);
            }
            break;

        case NavState::TRACKING:
            if (event == NavEvent::GOAL_REACHED) {
                TransitionTo(NavState::REACHED);
            } else if (event == NavEvent::TRACKING_LOST) {
                TransitionTo(NavState::ERROR);
            } else if (event == NavEvent::GOAL_RECEIVED) {
                TransitionTo(NavState::PLANNING);
            }
            break;

        case NavState::REACHED:
            if (event == NavEvent::GOAL_RECEIVED) {
                TransitionTo(NavState::PLANNING);
            }
            break;

        case NavState::ERROR:
            if (event == NavEvent::GOAL_RECEIVED) {
                TransitionTo(NavState::PLANNING);
            }
            break;
    }
}

NavState NavigationStateMachine::GetState() const {
    return state_;
}

std::string NavigationStateMachine::GetStateString() const {
    return NavStateToString(state_);
}

void NavigationStateMachine::SetCallback(StateChangeCallback cb) {
    callback_ = std::move(cb);
}

void NavigationStateMachine::TransitionTo(NavState new_state) {
    if (new_state == state_) return;

    NavState old_state = state_;
    state_ = new_state;

    if (callback_) {
        callback_(old_state, new_state);
    }
}

}  // namespace slam_nav
