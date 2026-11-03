/**
 * @file auto_mode_controller.h
 * @brief 航点启动、停稳等待和任务进度管理
 */
#pragma once

#include "auto_move.h"

struct AutoRouteState
{
    uint32_t currentPointIndex = 0U;
    bool finished = false;
};

struct AutoModeOutput
{
    ChassisBodyVelocity velocity = {};
    bool enabled = false;
};

class AutoModeController
{
public:
    /** @brief 参数在应用层已验证，复位当前路线，参数由调用者保证生命周期 */
    void Reset(const AutoMoveTune& param);
    /** @brief 返回一拍速度，调用者保证 dt 有效，反馈失效后锁存故障 */
    AutoModeOutput Update(const ChassisPose2D& pose, const ChassisBodyVelocity& velocity, float dt, bool feedbackValid);
    void Fault() { autoMove_.SetFault(); }
    const AutoRouteState& RouteState() const { return routeState_; }
    const AutoMove& Motion() const { return autoMove_; }

private:
    const AutoMoveTune* param_ = nullptr;
    AutoMove autoMove_;
    AutoRouteState routeState_ = {};
};
