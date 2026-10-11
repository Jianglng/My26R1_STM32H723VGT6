#include "auto_mode_controller.h"

#include "math_utils.h"
#include "robot_config.h"

#include <math.h>

namespace
{
    const AutoMoveParam AUTO_PARAM =
    {
        RobotConfig::AUTO_MAX_VELOCITY,
        RobotConfig::AUTO_ACCELERATION,
        RobotConfig::AUTO_DECELERATION,
        RobotConfig::AUTO_ALONG_KP,
        RobotConfig::AUTO_ALONG_KD,
        RobotConfig::AUTO_CROSS_KP,
        RobotConfig::AUTO_YAW_KP,
        RobotConfig::AUTO_MAX_CORRECTION_VELOCITY,
        RobotConfig::AUTO_MAX_OUTPUT_VELOCITY,
        MathUtils::DegToRad(RobotConfig::AUTO_MAX_YAW_VELOCITY_DEG_PER_SEC),
        MathUtils::DegToRad(RobotConfig::AUTO_YAW_ACCELERATION_DEG_PER_SEC2),
        RobotConfig::AUTO_POSITION_TOLERANCE,
        MathUtils::DegToRad(RobotConfig::AUTO_YAW_TOLERANCE_DEG),
        RobotConfig::AUTO_SPEED_TOLERANCE,
        MathUtils::DegToRad(RobotConfig::AUTO_YAW_SPEED_TOLERANCE_DEG_PER_SEC),
        RobotConfig::AUTO_SETTLE_DURATION
    };

    bool ChassisStopped(const ChassisBodyVelocity &velocity)
    {
        return isfinite(velocity.vx) && isfinite(velocity.vy) &&
               isfinite(velocity.wz) &&
               hypotf(velocity.vx, velocity.vy) <= AUTO_PARAM.speedTolerance &&
               fabsf(velocity.wz) <= AUTO_PARAM.yawSpeedTolerance;
    }
}

void AutoModeController::Reset()
{
    autoMove_.Reset();
    routeState_ = AutoRouteState{};
    routeState_.pointCount = AutoRouteConfig::POINT_COUNT;
    routeState_.finished = (AutoRouteConfig::POINT_COUNT == 0U);
}

bool AutoModeController::Update(const ChassisPose2D &currentPose,
                                const ChassisBodyVelocity &measuredVelocity,
                                float dt, bool feedbackValid,
                                ChassisBodyVelocity &command)
{
    command = ChassisBodyVelocity{};

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
        if (feedbackValid && ChassisStopped(measuredVelocity))
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
            if (!autoMove_.Start(currentPose, target, AUTO_PARAM))
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
