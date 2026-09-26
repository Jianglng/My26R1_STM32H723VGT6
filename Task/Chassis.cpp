#include "Chassis.h"

#include "bsp_can.h"
#include "fdcan.h"
#include "main.h"

void Chassis::Init()
{
    /* 先开 DWT。PID 用它测 dt，必须在时钟配置之后。 */
    dwt_.Init(SystemCoreClock / 1000000U);

    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerAnglePid_[i].Init(RobotConfig::STEER_ANGLE, dwt_);
        steerSpeedPid_[i].Init(RobotConfig::STEER_SPEED, dwt_);
        /* VESC 在 FDCAN1，6020 在 FDCAN3。ID 只来自 RobotConfig。 */
        wheelMotors_[i].Init(&hfdcan1, RobotConfig::VESC_NODE_ID[i]);
        steerMotors_[i].Init(RobotConfig::DJI_MOTOR_ID[i]);
    }

    BspCan::Init();
    Stop();
    StopMotors();
}

void Chassis::SetVelocity(const ChassisBodyVelocity &velocity)
{
    bodyVelocity_ = velocity;
    enabled_ = true;
}

void Chassis::Stop()
{
    enabled_ = false;
    bodyVelocity_ = {};
}

void Chassis::Update()
{
    Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT];
    int16_t steerEncoder[RobotConfig::WHEEL_COUNT];
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerFeedback[i] = steerMotors_[i].GetRxData();
        steerEncoder[i] = steerFeedback[i].encoder;
    }

    if (!enabled_)
    {
        StopMotors();
        Dji6020Bus::Control();
        return;
    }

    const ChassisWheelCommand command = kinematics_.Inverse(bodyVelocity_, steerEncoder);

    RunSteerPid(command, steerFeedback);
    Dji6020Bus::Control();
    RunWheelRpm(command, steerFeedback);
}

void Chassis::StopMotors()
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerAnglePid_[i].Reset();
        steerSpeedPid_[i].Reset();
        steerMotors_[i].SetCurrent(0.0f);
        wheelMotors_[i].SetRpm(0);
    }
}

void Chassis::RunSteerPid(const ChassisWheelCommand &command,
                          const Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT])
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        const Dji6020RxData &rx = steerFeedback[i];
        if (!rx.online)
        {
            steerAnglePid_[i].Reset();
            steerSpeedPid_[i].Reset();
            steerMotors_[i].SetCurrent(0.0f);
            continue;
        }

        const float speedRef = steerAnglePid_[i].CalculateEncoder(
            static_cast<float>(rx.encoder),
            command.steerEncoder[i],
            RobotConfig::DJI_ENCODER_MAX);

        const float currentRaw = steerSpeedPid_[i].Calculate(
            static_cast<float>(rx.speedRpm),
            speedRef);

        steerMotors_[i].SetCurrentRaw(static_cast<int16_t>(currentRaw));
    }
}

void Chassis::RunWheelRpm(const ChassisWheelCommand &command,
                          const Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT])
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        if (!steerFeedback[i].online)
        {
            wheelMotors_[i].SetRpm(0);
            continue;
        }

        wheelMotors_[i].SetRpm(static_cast<int32_t>(command.wheelRpm[i]));
    }
}
