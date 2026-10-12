#include "auto_mode_controller.h"

#include "auto_move_config.h"
#include "math_utils.h"

#include <math.h>

namespace
{
    bool ChassisStopped(const ChassisBodyVelocity &velocity, const AutoMoveParam &autoMoveParam)
    {
        return isfinite(velocity.vx) && isfinite(velocity.vy) &&
               isfinite(velocity.wz) &&
               hypotf(velocity.vx, velocity.vy) <= autoMoveParam.speedTolerance &&
               fabsf(velocity.wz) <= autoMoveParam.yawSpeedTolerance;
    }
}

void AutoModeController::Reset(PoseSource poseSource)
{
    poseSource_ = poseSource;
    autoMoveParam_ = AutoMoveConfig::ForSource(poseSource_);
    autoMove_.Reset();
    routeState_ = AutoRouteState{};
    routeState_.poseSource = poseSource_;
    routeState_.pointCount = AutoRouteConfig::POINT_COUNT;
    routeState_.finished = (AutoRouteConfig::POINT_COUNT == 0U);
}

bool AutoModeController::Update(const ChassisPose2D &currentPose,
                                const ChassisBodyVelocity &measuredVelocity,
                                float dt, bool feedbackValid,
                                ChassisBodyVelocity &command)
{
    command = ChassisBodyVelocity{};
    const AutoMoveParam &autoMoveParam = autoMoveParam_;

    /* 航点表为空或已经执行完毕时，保持底盘停止。 */
    if (routeState_.finished ||
        routeState_.currentPointIndex >= AutoRouteConfig::POINT_COUNT)
    {
        routeState_.finished = true;
        return false;
    }

    /* 当前航点尚未启动时，先等待反馈有效且底盘停稳。 */
    if (autoMove_.State().phase == AutoMovePhase::Idle)
    {
        if (feedbackValid && ChassisStopped(measuredVelocity, autoMoveParam))
        {
            const AutoRoutePoint &point =
                AutoRouteConfig::POINTS[routeState_.currentPointIndex];
            const AutoMoveTarget target =
            {
                point.x,
                point.y,
                MathUtils::DegToRad(point.yawDeg)
            };

            /* 每个航点独立建立一段梯形规划。 */
            if (!autoMove_.Start(currentPose, target, autoMoveParam))
            {
                return false;
            }
        }

        /* 启动这一拍保持停机，下一拍才把目标速度交给底盘。 */
        return false;
    }

    /* AutoMove 内部负责更新规划、反馈修正和异常状态。 */
    autoMove_.Update(currentPose, measuredVelocity, dt, feedbackValid);

    if (autoMove_.Faulted())
    {
        command = ChassisBodyVelocity{};
        return false;
    }

    /* 只有当前航点实际到达并停稳后，才切换到下一个航点。 */
    if (autoMove_.Finished())
    {
        ++routeState_.currentPointIndex;

        if (routeState_.currentPointIndex >= AutoRouteConfig::POINT_COUNT)
        {
            routeState_.finished = true;
        }
        else
        {
            /* 下一拍重新等待停稳条件并启动下一个航点。 */
            autoMove_.Reset();
        }

        return false;
    }

    command = autoMove_.Velocity();
    return true;
}
