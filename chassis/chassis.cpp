/**
 * @file chassis.cpp
 * @brief 三舵轮反馈与电机控制
 */
#include "chassis.h"

#include <math.h>

#include "main.h"

#include "bsp_can.h"

bool Chassis::Init()
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerAnglePid_[i].Init(RobotConfig::STEER_ANGLE);
        steerSpeedPid_[i].Init(RobotConfig::STEER_SPEED);
        /* VESC 在 FDCAN1，6020 在 FDCAN3*/
        wheelMotors_[i].Init(CanBus::Wheel, RobotConfig::VESC_NODE_ID[i], RobotConfig::VESC_POLE_PAIRS);
        steerMotors_[i].Init(RobotConfig::DJI_MOTOR_ID[i]);
    }

    BspCan::RegisterReceiver(CanBus::Wheel, &VescMotor::ParseCanFeedback);
    BspCan::RegisterReceiver(CanBus::Steer, &Dji6020Bus::ParseFeedback);
    const bool initialized = BspCan::Init();
    Stop();
    StopMotors();
    transmitOk_ = Dji6020Bus::Control() && transmitOk_;
    return initialized;
}

bool Chassis::SetVelocity(const ChassisBodyVelocity& velocity)
{
    if (!isfinite(velocity.vx) || !isfinite(velocity.vy) || !isfinite(velocity.wz))
    {
        Stop();
        return false;
    }
    bodyVelocity_ = velocity;
    enabled_ = true;
    return true;
}

void Chassis::Stop()
{
    enabled_ = false;
    bodyVelocity_ = {};
}

void Chassis::UpdateFeedback()
{
    BspCan::ProcessReceive();
    const uint32_t nowTick = HAL_GetTick();
    float wheelRpm[RobotConfig::WHEEL_COUNT];
    measuredVelocityValid_ = true;
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerFeedback_[i] = steerMotors_[i].GetRxData(nowTick);
        steerEncoder_[i] = steerFeedback_[i].encoder;
        const VescRxData wheelFeedback = wheelMotors_[i].GetRxData(nowTick);
        wheelRpm[i] = wheelFeedback.rpm;
        if (!steerFeedback_[i].online || !wheelFeedback.speedOnline)
        {
            measuredVelocityValid_ = false;
        }
    }

    measuredVelocity_ = measuredVelocityValid_ ? kinematics_.Forward(steerEncoder_, wheelRpm) : ChassisBodyVelocity{};
}

void Chassis::UpdateControl(float pidDtMs)
{
    transmitOk_ = true;
    outputValid_ = true;
    if (!enabled_ || !measuredVelocityValid_)
    {
        StopMotors();
        transmitOk_ = Dji6020Bus::Control() && transmitOk_;
        return;
    }

    const ChassisWheelCommand& command = kinematics_.Inverse(bodyVelocity_, steerEncoder_);

    if (!RunSteerPid(command, pidDtMs))
    {
        Stop();
        StopMotors();
        outputValid_ = false;
        transmitOk_ = Dji6020Bus::Control() && transmitOk_;
        return;
    }
    transmitOk_ = Dji6020Bus::Control() && transmitOk_;
    RunWheelRpm(command);
}

bool Chassis::ApplySteerPidParam(const PidParam& angle, const PidParam& speed)
{
    /* 只在停机时换参数，避免行进中增益突变 */
    if (enabled_ || !PidParamValid(angle, RobotConfig::STEER_ANGLE.maxOut) ||
        !PidParamValid(speed, RobotConfig::STEER_SPEED.maxOut))
    {
        return false;
    }

    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerAnglePid_[i].Init(angle);
        steerSpeedPid_[i].Init(speed);
    }
    return true;
}

void Chassis::StopMotors()
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        steerAnglePid_[i].Reset();
        steerSpeedPid_[i].Reset();
        steerMotors_[i].Stop();
        transmitOk_ = wheelMotors_[i].SetRpm(0) && transmitOk_;
    }
}

bool Chassis::RunSteerPid(const ChassisWheelCommand& command, float pidDtMs)
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        const Dji6020RxData& rx = steerFeedback_[i];
        const float speedRef = steerAnglePid_[i].CalculateEncoder(
            static_cast<float>(rx.encoder), command.steerEncoder[i], RobotConfig::GEOMETRY.encoderRange, pidDtMs);

        const float currentRaw = steerSpeedPid_[i].Calculate(static_cast<float>(rx.speedRpm), speedRef, pidDtMs);

        // 电流转为整数前保留输出端检查，避免异常调参产生 NaN
        if (!isfinite(currentRaw))
        {
            return false;
        }
        steerMotors_[i].SetCurrentRaw(static_cast<int16_t>(currentRaw));
    }
    return true;
}

void Chassis::RunWheelRpm(const ChassisWheelCommand& command)
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        transmitOk_ = wheelMotors_[i].SetRpm(static_cast<int32_t>(command.wheelRpm[i])) && transmitOk_;
    }
}
