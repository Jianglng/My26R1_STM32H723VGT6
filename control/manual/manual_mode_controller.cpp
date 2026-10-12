#include "manual_mode_controller.h"

#include "math_utils.h"
#include "robot_config.h"

ChassisBodyVelocity ManualModeController::Update(const RemoteState &remoteState) const
{
    ChassisBodyVelocity manualVelocity = {};
    if (!remoteState.online)
    {
        return manualVelocity;
    }

    /* 遥控死区、方向和量程沿用整改前的映射，角速度配置在入口转换为 rad/s。 */
    const float scaleX = RobotConfig::MAX_VX / RobotConfig::REMOTE_CH_MAX;
    const float scaleY = RobotConfig::MAX_VY / RobotConfig::REMOTE_CH_MAX;
    const float maxYawVelocityRadPerSec = MathUtils::DegToRad(RobotConfig::MAX_WZ_DEG_PER_SEC);
    const float scaleYaw = maxYawVelocityRadPerSec / RobotConfig::REMOTE_CH_MAX;
    const float stickForward = MathUtils::ApplyDeadband(static_cast<float>(remoteState.leftY), RobotConfig::REMOTE_DEADBAND);
    const float stickRight = MathUtils::ApplyDeadband(static_cast<float>(remoteState.leftX), RobotConfig::REMOTE_DEADBAND);
    const float stickYaw = MathUtils::ApplyDeadband(static_cast<float>(remoteState.rightX), RobotConfig::REMOTE_DEADBAND);

    manualVelocity.vx = MathUtils::ClampAbs(stickForward * scaleX * RobotConfig::REMOTE_VX_SIGN, RobotConfig::MAX_VX);
    manualVelocity.vy = MathUtils::ClampAbs(stickRight * scaleY * RobotConfig::REMOTE_VY_SIGN, RobotConfig::MAX_VY);
    manualVelocity.wz = MathUtils::ClampAbs(stickYaw * scaleYaw * RobotConfig::REMOTE_WZ_SIGN, maxYawVelocityRadPerSec);
    return manualVelocity;
}
