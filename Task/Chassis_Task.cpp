#include "Chassis_Task.h"

#include "Chassis.h"
#include "cmsis_os.h"
#include "math_utils.h"
#include "remote_input.h"
#include "robot_config.h"
#include "usart.h"

namespace
{
    ChassisBodyVelocity MapRemote(const RemoteState &remote)
    {
        ChassisBodyVelocity vel = {};
        if (!remote.online)
        {
            return vel;
        }

        const float scaleX = RobotConfig::MAX_VX / RobotConfig::REMOTE_CH_MAX;
        const float scaleY = RobotConfig::MAX_VY / RobotConfig::REMOTE_CH_MAX;
        const float scaleW = RobotConfig::MAX_WZ / RobotConfig::REMOTE_CH_MAX;

        const float stickForward = MathUtils::ApplyDeadband(static_cast<float>(remote.leftY), RobotConfig::REMOTE_DEADBAND);
        const float stickRight = MathUtils::ApplyDeadband(static_cast<float>(remote.leftX), RobotConfig::REMOTE_DEADBAND);
        const float stickYaw = MathUtils::ApplyDeadband(static_cast<float>(remote.rightX), RobotConfig::REMOTE_DEADBAND);

        vel.vx = MathUtils::ClampAbs(stickForward * scaleX * RobotConfig::REMOTE_VX_SIGN, RobotConfig::MAX_VX);
        vel.vy = MathUtils::ClampAbs(stickRight * scaleY * RobotConfig::REMOTE_VY_SIGN, RobotConfig::MAX_VY);
        vel.wz = MathUtils::ClampAbs(stickYaw * scaleW * RobotConfig::REMOTE_WZ_SIGN, RobotConfig::MAX_WZ);
        return vel;
    }
}

extern "C" void StartChassisTask(void *argument)
{
    (void)argument;

    static Chassis chassis;
    static RemoteInput remote;

    chassis.Init();
    remote.Init(&huart5);

    for (;;)
    {
        const RemoteState state = remote.State();
        if (!state.online)
        {
            chassis.Stop();
        }
        else
        {
            chassis.SetVelocity(MapRemote(state));
        }
        chassis.Update();
        osDelay(1);
    }
}
