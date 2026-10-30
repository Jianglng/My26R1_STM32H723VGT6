#include "auto_mode_controller.h"

#include "math_utils.h"

#include <math.h>

/* 两个结构体字段一一对应，增删字段时须同时修改，并同步 ToAutoMoveParam()。 */
static_assert(sizeof(AutoMoveTune) == sizeof(AutoMoveParam), "AutoMoveTune 与 AutoMoveParam 字段数不一致");

namespace
{
    bool ChassisStopped(const ChassisBodyVelocity &velocity, const AutoMoveParam &autoMoveParam)
    {
        return isfinite(velocity.vx) && isfinite(velocity.vy) &&
               isfinite(velocity.wz) &&
               hypotf(velocity.vx, velocity.vy) <= autoMoveParam.speedTolerance &&
               fabsf(velocity.wz) <= autoMoveParam.yawSpeedTolerance;
    }

    /* 度制配置换算为控制器使用的 rad 参数，其余字段原样复制。 */
    AutoMoveParam ToAutoMoveParam(const AutoMoveTune &tune)
    {
        AutoMoveParam param = {};
        param.maxVelocity = tune.maxVelocity;
        param.acceleration = tune.acceleration;
        param.deceleration = tune.deceleration;
        param.trackingAlongKp = tune.trackingAlongKp;
        param.trackingAlongKd = tune.trackingAlongKd;
        param.settlingAlongKp = tune.settlingAlongKp;
        param.settlingAlongKd = tune.settlingAlongKd;
        param.crossKp = tune.crossKp;
        param.yawKp = tune.yawKp;
        param.maxCorrectionVelocity = tune.maxCorrectionVelocity;
        param.maxOutputVelocity = tune.maxOutputVelocity;
        param.maxYawVelocity = MathUtils::DegToRad(tune.maxYawVelocityDegPerSec);
        param.yawAcceleration = MathUtils::DegToRad(tune.yawAccelerationDegPerSec2);
        param.positionTolerance = tune.positionTolerance;
        param.yawTolerance = MathUtils::DegToRad(tune.yawToleranceDeg);
        param.speedTolerance = tune.speedTolerance;
        param.yawSpeedTolerance = MathUtils::DegToRad(tune.yawSpeedToleranceDegPerSec);
        param.settleDuration = tune.settleDuration;
        return param;
    }
}

bool AutoModeController::Reset(PoseSource poseSource, const AutoMoveTune &tune)
{
    poseSource_ = poseSource;

    /* 在 AutoMove 的通用校验之外再限制输出速度，防止调试器中误填过大的值。 */
    const AutoMoveParam param = ToAutoMoveParam(tune);
    const bool paramValid = AutoMoveParamValid(param) &&
                            param.maxOutputVelocity <= AutoMoveConfig::MAX_TUNE_OUTPUT_VELOCITY;
    if (paramValid)
    {
        autoMoveParam_ = param;
    }

    autoMove_.Reset();
    routeState_ = AutoRouteState{};
    routeState_.poseSource = poseSource_;
    routeState_.pointCount = AutoRouteConfig::POINT_COUNT;
    routeState_.finished = (AutoRouteConfig::POINT_COUNT == 0U);
    return paramValid;
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
            /* 航点与 currentPose 均为车体中心；这里只换算角度，不再补偿安装偏移。 */
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
