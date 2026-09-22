#include "Chassis_Task.h"

#include <math.h>

#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "vesc_motor.h"
#include "dji_6020.h"
#include "remote_input.h"
//#include "chaohe_imu.h"
#include "chassis_kinematics.h"
#include "pid.h"
#include "robot_config.h"
#include "usart.h"

namespace
{
    float applyDeadband(float value, float deadband)
    {
        if (fabsf(value) < deadband)
        {
            return 0.0f;
        }
        return value;
    }

    float clampAbs(float value, float limit)
    {
        if (value > limit)
        {
            return limit;
        }
        if (value < -limit)
        {
            return -limit;
        }
        return value;
    }

    ChassisBodyVelocity mapRemote(const RemoteState &remote)
    {
        ChassisBodyVelocity vel = {};
        if (!remote.online)
        {
            return vel;
        }

        const float scaleX = RobotConfig::MAX_VX / RobotConfig::REMOTE_CH_MAX;
        const float scaleY = RobotConfig::MAX_VY / RobotConfig::REMOTE_CH_MAX;
        const float scaleW = RobotConfig::MAX_WZ / RobotConfig::REMOTE_CH_MAX;

        const float stickForward = applyDeadband(static_cast<float>(remote.leftY),
                                                 RobotConfig::REMOTE_DEADBAND);
        const float stickRight = applyDeadband(static_cast<float>(remote.leftX),
                                               RobotConfig::REMOTE_DEADBAND);
        const float stickYaw = applyDeadband(static_cast<float>(remote.rightX),
                                             RobotConfig::REMOTE_DEADBAND);

        vel.vx = clampAbs(stickForward * scaleX * RobotConfig::REMOTE_VX_SIGN,
                          RobotConfig::MAX_VX);
        vel.vy = clampAbs(stickRight * scaleY * RobotConfig::REMOTE_VY_SIGN,
                          RobotConfig::MAX_VY);
        vel.wz = clampAbs(stickYaw * scaleW * RobotConfig::REMOTE_WZ_SIGN,
                          RobotConfig::MAX_WZ);
        return vel;
    }

    void stopMotors()
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            g_steerAnglePid[i].Reset();
            g_steerSpeedPid[i].Reset();
            Dji6020Motors[i].setCurrent(0.0f);
            VescMotors[i].setRpm(0);
        }
    }

    /** @brief 6020 串级：角度环出速度目标，速度环出电流。掉线则清积分并电流置 0。 */
    void runSteerPid(const ChassisWheelCommand &command,
                     const Dji6020RxData steerFeedback[ChassisKinematicsCfg::WHEEL_COUNT])
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            const Dji6020RxData &rx = steerFeedback[i];
            if (!rx.online)
            {
                g_steerAnglePid[i].Reset();
                g_steerSpeedPid[i].Reset();
                Dji6020Motors[i].setCurrent(0.0f);
                continue;
            }

            const float speedRef = g_steerAnglePid[i].calculateEncoder(
                static_cast<float>(rx.encoder),
                command.steerEncoder[i],
                PidCfg::ENCODER_RANGE);

            const float currentRaw = g_steerSpeedPid[i].calculate(
                static_cast<float>(rx.speedRpm),
                speedRef);

            Dji6020Motors[i].setCurrentRaw(static_cast<int16_t>(currentRaw));
        }
    }

    /** @brief 把逆解轮速发给 VESC。对应舵向 6020 掉线时该路转速置 0。 */
    void runWheelRpm(const ChassisWheelCommand &command,
                     const Dji6020RxData steerFeedback[ChassisKinematicsCfg::WHEEL_COUNT])
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            if (!steerFeedback[i].online)
            {
                VescMotors[i].setRpm(0);
                continue;
            }

            VescMotors[i].setRpm(static_cast<int32_t>(command.wheelRpm[i]));
        }
    }
}

extern "C" void StartChassisTask(void *argument)
{
    (void)argument;

    /* CPU 480 MHz，CYCCNT 按核时钟计数，必须在时钟配置之后打开。 */
    DWT_.Init(SystemCoreClock / 1000000U);
    InitChassisPid();

    /* VESC 在 FDCAN1，6020 在 FDCAN3。ID 见 robot_config.h。 */
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        VescMotors[i].Init(&hfdcan1, RobotConfig::VESC_NODE_ID[i]);
        Dji6020Motors[i].Init(RobotConfig::DJI_MOTOR_ID[i]);
    }

    BSP_CAN::Init();
    g_remoteInput.Init(&huart5);
    //g_chaoheImu.Init(&huart1);
    stopMotors();

    for (;;)
    {
        g_remoteInput.update();
       // g_chaoheImu.update();
        /* IMU 已在 USART1 接收，当前不参与底盘控制。需要时读 g_chaoheImu.state()。 */
        const RemoteState &remoteState = g_remoteInput.state();

        Dji6020RxData steerFeedback[ChassisKinematicsCfg::WHEEL_COUNT];
        int16_t steerEncoder[ChassisKinematicsCfg::WHEEL_COUNT];
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            steerFeedback[i] = Dji6020Motors[i].getRxData();
            steerEncoder[i] = steerFeedback[i].encoder;
        }

        if (!remoteState.online)
        {
            stopMotors();
            Dji6020Bus::Control();
            osDelay(1);
            continue;
        }

        const ChassisBodyVelocity bodyVel = mapRemote(remoteState);
        const ChassisWheelCommand command =
            g_chassisKinematics.inverse(bodyVel, steerEncoder);

        runSteerPid(command, steerFeedback);
        Dji6020Bus::Control();
        runWheelRpm(command, steerFeedback);

        osDelay(1);
    }
}
