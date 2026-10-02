#include "Chassis_Task.h"

#include "Chassis.h"
#include "Debug_Snapshot.h"
#include "chaohe_imu.h"
#include "cmsis_os.h"
#include "math_utils.h"
#include "remote_input.h"
#include "robot_config.h"
#include "usart.h"

namespace
{
    enum class ChassisMode : uint8_t
    {
        Stop,
        Manual,
        Auto
    };

    ChassisMode SelectMode(const RemoteState &remote)
    {
        if (!remote.online)
        {
            return ChassisMode::Stop;
        }

        switch (remote.leftSwitch)
        {
            case 1U: return ChassisMode::Auto;   // 上档
            case 3U: return ChassisMode::Manual; // 中档
            case 2U: return ChassisMode::Stop;   // 下档
            default: return ChassisMode::Stop;
        }
    }

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
    static ChaoheImu imu;
    uint32_t lastDebugSnapshotTick = 0U;

    chassis.Init();
    remote.Init(&huart5);
    imu.Init(&huart10);
    
    for (;;)
    {
        const RemoteState state = remote.State();
        switch (SelectMode(state))
        {
            case ChassisMode::Manual:
                chassis.SetVelocity(MapRemote(state));
                break;

            case ChassisMode::Auto:
                /* 自动目标速度尚未接入，保持停机。 */
                chassis.Stop();
                break;

            case ChassisMode::Stop:
            default:
                chassis.Stop();
                break;
        }
        chassis.Update();
        imu.Update();

        /* 调试镜像不参与控制，低频更新以免给 1 ms 控制循环增加额外开销。 */
        const uint32_t now = HAL_GetTick();
        if ((now - lastDebugSnapshotTick) >= DebugSnapshot::UPDATE_PERIOD_MS)
        {
            DebugSnapshot::Update(state, chassis, imu.State());
            lastDebugSnapshotTick = now;
        }
        osDelay(1);
    }
}
