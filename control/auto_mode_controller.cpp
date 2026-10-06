#include "auto_mode_controller.h"

#include "robot_config.h"

#include <math.h>

namespace
{
    const AutoMoveTarget AUTO_TARGET =
    {
        RobotConfig::AUTO_TARGET_X,
        RobotConfig::AUTO_TARGET_Y,
        RobotConfig::AUTO_TARGET_YAW
    };

    const AutoMoveParam AUTO_PARAM =
    {
        RobotConfig::AUTO_MAX_VELOCITY,
        RobotConfig::AUTO_ACCELERATION,
        RobotConfig::AUTO_DECELERATION,
        RobotConfig::AUTO_ALONG_KP,
        RobotConfig::AUTO_CROSS_KP,
        RobotConfig::AUTO_YAW_KP,
        RobotConfig::AUTO_MAX_CORRECTION_VELOCITY,
        RobotConfig::AUTO_MAX_OUTPUT_VELOCITY,
        RobotConfig::AUTO_MAX_YAW_VELOCITY,
        RobotConfig::AUTO_YAW_ACCELERATION,
        RobotConfig::AUTO_POSITION_TOLERANCE,
        RobotConfig::AUTO_YAW_TOLERANCE,
        RobotConfig::AUTO_SPEED_TOLERANCE,
        RobotConfig::AUTO_YAW_SPEED_TOLERANCE,
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
}

bool AutoModeController::Update(const ChassisPose2D &currentPose,
                                const ChassisBodyVelocity &measuredVelocity,
                                float dt, bool feedbackValid,
                                ChassisBodyVelocity &command)
{
    command = ChassisBodyVelocity{};

    /* 还没有启动时，先等待反馈有效且底盘停稳。 */
    if (autoMove_.State().phase == AutoMovePhase::Idle)
    {
        if (feedbackValid && ChassisStopped(measuredVelocity))
        {
            autoMove_.Start(currentPose, AUTO_TARGET, AUTO_PARAM);
        }

        /* 启动这一拍保持停机，下一拍才把目标速度交给底盘。 */
        return false;
    }

    /* AutoMove 内部负责更新规划、反馈修正和异常状态。 */
    autoMove_.Update(currentPose, measuredVelocity, dt, feedbackValid);
    command = autoMove_.Velocity();
    return !autoMove_.Faulted();
}
