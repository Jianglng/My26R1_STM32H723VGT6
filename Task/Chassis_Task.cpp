#include "Chassis_Task.h"

#include "cmsis_os.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "vesc_motor.h"
#include "dji_6020.h"
#include "remote_input.h"
#include "chassis_kinematics.h"
#include "pid.h"
#include "usart.h"

namespace
{
    void stopMotors()
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            g_steerAnglePid[i].reset();
            g_steerSpeedPid[i].reset();
            Dji6020Motors[i].setCurrent(0.0f);
            VescMotors[i].setRpm(0);
        }
    }

    void runSteerPid(const ChassisWheelCommand &command)
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            const Dji6020RxData &rx = Dji6020Motors[i].getRxData();
            if (!rx.online)
            {
                g_steerAnglePid[i].reset();
                g_steerSpeedPid[i].reset();
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

    void runWheelRpm(const ChassisWheelCommand &command)
    {
        for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
        {
            if (!Dji6020Motors[i].getRxData().online)
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
    DWT_.init(SystemCoreClock / 1000000U);
    initChassisPid();

    /* VESC (FDCAN1)：扩展帧，每个电机一帧独立下发。 */
    VescMotors[0].init(&hfdcan1, 90);
    VescMotors[1].init(&hfdcan1, 69);
    VescMotors[2].init(&hfdcan1, 97);

    /* 6020 (FDCAN3)：标准帧，三个电机共用 0x1FF。
     * 下标按机械位置：[0] 前轮 ID1，[1] 左轮 ID4，[2] 右轮 ID3。
     */
    Dji6020Motors[0].init(1);
    Dji6020Motors[1].init(4);
    Dji6020Motors[2].init(3);

    BSP_CAN::Init();
    g_remoteInput.init(&huart5);
    stopMotors();

    for (;;)
    {
        g_remoteInput.update();
        const RemoteState &remoteState = g_remoteInput.state();

        int16_t steerEncoder[ChassisKinematicsCfg::WHEEL_COUNT] =
        {
            Dji6020Motors[0].getRxData().encoder,
            Dji6020Motors[1].getRxData().encoder,
            Dji6020Motors[2].getRxData().encoder
        };

        if (!remoteState.online)
        {
            stopMotors();
            Dji6020Bus::Control();
            osDelay(1);
            continue;
        }

        const ChassisBodyVelocity bodyVel = g_chassisKinematics.mapRemote(remoteState);
        const ChassisWheelCommand command =
            g_chassisKinematics.inverse(bodyVel, steerEncoder);

        runSteerPid(command);
        Dji6020Bus::Control();
        runWheelRpm(command);

        osDelay(1);
    }
}
