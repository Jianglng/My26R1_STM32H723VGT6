/**
 * @file auto_mode_controller.cpp
 * @brief 每个航点独立规划，实际到达并停稳后才推进路线
 */
#include "auto_mode_controller.h"

#include <math.h>

#include "auto_route_config.h"
#include "math_utils.h"

void AutoModeController::Reset(const AutoMoveTune& param)
{
    param_ = &param;
    autoMove_.Reset();
    routeState_ = {};
    routeState_.finished = AutoRouteConfig::POINT_COUNT == 0U;
}

AutoModeOutput AutoModeController::Update(const ChassisPose2D& pose, const ChassisBodyVelocity& velocity, float dt,
                                          bool feedbackValid)
{
    if (routeState_.finished)
    {
        return {};
    }
    if (autoMove_.State().phase == AutoMovePhase::Idle)
    {
        if (feedbackValid && hypotf(velocity.vx, velocity.vy) <= param_->speedTolerance &&
            fabsf(velocity.wz) <= MathUtils::DegToRad(param_->yawSpeedToleranceDegPerSec))
        {
            const AutoRoutePoint& point = AutoRouteConfig::POINTS[routeState_.currentPointIndex];
            const AutoMoveTarget target = {point.x, point.y, MathUtils::DegToRad(point.yawDeg)};
            autoMove_.Start(pose, target, *param_);
        }
        return {};
    }
    autoMove_.Update(pose, velocity, dt, feedbackValid);
    if (autoMove_.Faulted())
    {
        return {};
    }
    if (autoMove_.Finished())
    {
        ++routeState_.currentPointIndex;
        routeState_.finished = routeState_.currentPointIndex == AutoRouteConfig::POINT_COUNT;
        if (!routeState_.finished)
        {
            autoMove_.Reset();
        }
        return {};
    }
    return {autoMove_.Velocity(), true};
}
