/**
 * @file dji_6020.cpp
 * @brief DJI 6020 电流组帧与反馈解析
 */
#include "dji_6020.h"

#include "bsp_can.h"

Dji6020Bus::ControlSlot Dji6020Bus::ctrlSlot_[Dji6020Cfg::MOTOR_ID_MAX];
bool Dji6020Bus::hasMotor_ = false;

/* 已注册的电机下标是机械位置，不是 CAN ID */
Dji6020Motor* Dji6020Bus::motors_[Dji6020Cfg::MOTOR_ID_MAX] = {};
uint32_t Dji6020Bus::motorCount_ = 0;

bool Dji6020Bus::Control()
{
    if (!hasMotor_)
    {
        return true;
    }
    const bool group1Sent = SendGroup(0U, Dji6020Cfg::CTRL_ID_GROUP1);
    const bool group2Sent = SendGroup(Dji6020Cfg::GROUP_SIZE, Dji6020Cfg::CTRL_ID_GROUP2);
    return group1Sent && group2Sent;
}

bool Dji6020Bus::SendGroup(uint32_t firstSlot, uint32_t id)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {};
    bool enabled = false;
    for (uint32_t i = 0U; i < Dji6020Cfg::GROUP_SIZE; ++i)
    {
        const ControlSlot& slot = ctrlSlot_[firstSlot + i];
        enabled = enabled || slot.enabled;
        const int16_t current = slot.currentRaw;
        data[i * 2U] = static_cast<uint8_t>((current >> 8) & 0xFF);
        data[i * 2U + 1U] = static_cast<uint8_t>(current & 0xFF);
    }
    return !enabled || BspCan::Send(CanBus::Steer, id, CanIdType::Standard, data);
}

void Dji6020Bus::ParseFeedback(uint32_t identifier, const uint8_t* data, uint32_t receiveTick)
{
    /* 反馈 ID 从 0x205 开始；超出本工程电机数量的 ID 会在下面匹配失败 */
    if (data == nullptr || identifier < Dji6020Cfg::FEEDBACK_ID_BASE)
    {
        return;
    }

    /* 反馈 ID 反推电机标号：0x205 → 1 号电机 */
    const uint32_t motorId = identifier - Dji6020Cfg::FEEDBACK_ID_BASE + 1;

    if (motorId > Dji6020Cfg::MOTOR_ID_MAX)
    {
        return;
    }

    // 注册顺序是机械轮序，须按电调 ID 匹配
    for (uint32_t i = 0; i < motorCount_; ++i)
    {
        Dji6020Motor& motor = *motors_[i];

        if (motor.motorId_ != motorId)
        {
            continue;
        }

        motor.ApplyFeedback(data, receiveTick);
        return;
    }
}

void Dji6020Motor::ApplyFeedback(const uint8_t* data, uint32_t receiveTick)
{
    Dji6020RxData& rx = rxData_;

    const int16_t encoder =
        static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]));

    if (encoder < 0 || encoder >= static_cast<int16_t>(Dji6020Cfg::ENCODER_MAX))
    {
        return;
    }

    rx.encoder = encoder;
    rx.temperature = data[6];

    /* 转速必须用有符号解析：反转时是负数，用 uint16_t 接会变成 60000+ */
    rx.speedRpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));

    rx.currentRaw = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]));

    rx.currentAmp = static_cast<float>(rx.currentRaw) / Dji6020Cfg::RAW_PER_AMP;
    rx.angleDeg = static_cast<float>(rx.encoder) * Dji6020Cfg::DEG_PER_TICK;

    UpdateAngle();

    rx.lastRxTick = receiveTick;
    rx.online = true;
    rx.rxCount++;
}

Dji6020RxData Dji6020Motor::GetRxData(uint32_t nowTick) const
{
    Dji6020RxData copy = rxData_;
    copy.online = copy.online && nowTick - copy.lastRxTick <= Dji6020Cfg::ONLINE_TIMEOUT_MS;
    return copy;
}

void Dji6020Bus::Register(Dji6020Motor* motor)
{
    for (uint32_t i = 0; i < motorCount_; ++i)
    {
        if (motors_[i] == motor)
        {
            return;
        }
    }

    if (motorCount_ >= Dji6020Cfg::MOTOR_ID_MAX)
    {
        return;
    }

    motors_[motorCount_++] = motor;
}

Dji6020Motor::Dji6020Motor() : motorId_(0), slotIndex_(-1), rxData_()
{
}

void Dji6020Motor::Init(uint32_t motorId)
{
    if (motorId < Dji6020Cfg::MOTOR_ID_MIN || motorId > Dji6020Cfg::MOTOR_ID_MAX)
    {
        motorId_ = 0;
        slotIndex_ = -1;
        return;
    }

    motorId_ = motorId;
    slotIndex_ = static_cast<int32_t>(motorId - 1);  // ID 1 → 槽 0

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = 0;
    Dji6020Bus::ctrlSlot_[slotIndex_].enabled = true;

    Dji6020Bus::hasMotor_ = true;
    Dji6020Bus::Register(this);
}

void Dji6020Motor::SetCurrent(float currentA)
{
    if (slotIndex_ < 0)
    {
        return;  // 未初始化
    }

    /* 先限幅再换算：避免 currentA 极大时乘法溢出 */
    if (currentA > Dji6020Cfg::CURRENT_MAX_A)
    {
        currentA = Dji6020Cfg::CURRENT_MAX_A;
    }
    else if (currentA < -Dji6020Cfg::CURRENT_MAX_A)
    {
        currentA = -Dji6020Cfg::CURRENT_MAX_A;
    }

    const int32_t raw = static_cast<int32_t>(currentA * Dji6020Cfg::RAW_PER_AMP);

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = static_cast<int16_t>(raw);
}

void Dji6020Motor::SetCurrentRaw(int16_t raw)
{
    if (slotIndex_ < 0)
    {
        return;
    }

    const int16_t limit = static_cast<int16_t>(Dji6020Cfg::RAW_MAX);
    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = raw > limit ? limit : (raw < -limit ? -limit : raw);
}

void Dji6020Motor::Stop()
{
    if (slotIndex_ < 0)
    {
        return;
    }

    Dji6020Bus::ctrlSlot_[slotIndex_].currentRaw = 0;
}

void Dji6020Motor::UpdateAngle()
{
    Dji6020RxData& rx = rxData_;

    if (!rx.hasEncoder)
    {
        /* 第一帧：以当前位置为累计角度原点，避免上电瞬间跳一个随机角度 */
        rx.lastEncoder = rx.encoder;
        rx.totalAngleDeg = 0.0f;
        rx.hasEncoder = true;
        return;
    }

    int32_t delta = static_cast<int32_t>(rx.encoder) - static_cast<int32_t>(rx.lastEncoder);

    /* 跨零点修正：满量程 8192，半圈 4096 */
    const int32_t half = static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX / 2.0f);

    if (delta > half)
    {
        delta -= static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX);
    }
    else if (delta < -half)
    {
        delta += static_cast<int32_t>(Dji6020Cfg::ENCODER_MAX);
    }

    rx.totalAngleDeg += static_cast<float>(delta) * Dji6020Cfg::DEG_PER_TICK;

    rx.lastEncoder = rx.encoder;
}
