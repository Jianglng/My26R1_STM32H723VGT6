/**
 * @file vesc_motor.cpp
 * @brief VESC 控制组帧与状态反馈
 */
#include "vesc_motor.h"

#include "bsp_can.h"

VescMotor* VescMotor::registry_[MAX_REGISTERED_MOTORS] = {};
uint32_t VescMotor::registryCount_ = 0;

VescMotor::VescMotor() : nodeId_(0), rxData_()
{
}

void VescMotor::Register()
{
    for (uint32_t i = 0; i < registryCount_; ++i)
    {
        if (registry_[i] == this)
        {
            return;
        }
    }

    if (registryCount_ >= MAX_REGISTERED_MOTORS)
    {
        return;
    }

    registry_[registryCount_++] = this;
}

void VescMotor::Init(CanBus bus, uint16_t nodeId, uint32_t polePairs)
{
    bus_ = bus;
    polePairs_ = polePairs;
    nodeId_ = nodeId;
    Register();
}

void VescMotor::PackInt32BigEndian(int32_t val, uint8_t* data)
{
    data[0] = static_cast<uint8_t>((val >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val & 0xFF);
}

bool VescMotor::SendFrame(VescCommand cmd, const uint8_t (&data)[CAN_CLASSIC_PAYLOAD_SIZE])
{
    const uint32_t id = (static_cast<uint32_t>(nodeId_) & 0xFFU) | (static_cast<uint32_t>(cmd) << 8U);
    return BspCan::Send(bus_, id, CanIdType::Extended, data);
}

bool VescMotor::SetCurrent(int32_t currentMa)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(currentMa, data);
    return SendFrame(VescCommand::SetCurrent, data);
}

bool VescMotor::SetRpm(int32_t rpm)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(rpm * static_cast<int32_t>(polePairs_), data);
    return SendFrame(VescCommand::SetRpm, data);
}

bool VescMotor::SetPwm(float pwm)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(static_cast<int32_t>(pwm * 100000.0f), data);
    return SendFrame(VescCommand::SetDuty, data);
}

bool VescMotor::SetPos(int32_t pos)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(pos, data);
    return SendFrame(VescCommand::SetPos, data);
}

bool VescMotor::SetBrakeCurrent(int32_t currentMa)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(currentMa, data);
    return SendFrame(VescCommand::SetCurrentBrake, data);
}

bool VescMotor::SetHandbrakeCurrent(int32_t currentMa)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(currentMa, data);
    return SendFrame(VescCommand::SetCurrentHandbrake, data);
}

bool VescMotor::SetHandbrakeCurrentRel(float relative)
{
    uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE] = {0};
    PackInt32BigEndian(static_cast<int32_t>(relative * 100000.0f), data);
    return SendFrame(VescCommand::SetCurrentHandbrakeRel, data);
}

void VescMotor::ParseCanFeedback(uint32_t identifier, const uint8_t* data, uint32_t receiveTick)
{
    if (data == nullptr)
    {
        return;
    }

    const uint8_t rxNodeId = static_cast<uint8_t>(identifier & 0xFF);
    const VescCommand cmd = static_cast<VescCommand>((identifier >> 8) & 0xFF);

    for (uint32_t i = 0; i < registryCount_; ++i)
    {
        VescMotor* motor = registry_[i];
        if (static_cast<uint8_t>(motor->nodeId_) != rxNodeId)
        {
            continue;
        }

        motor->ParseStatusPayload(cmd, data, receiveTick);
        return;
    }
}

void VescMotor::ParseStatusPayload(VescCommand cmd, const uint8_t* data, uint32_t receiveTick)
{
    switch (cmd)
    {
        case VescCommand::Status:
            ParseSpeed(data, receiveTick);
            break;
        case VescCommand::Status4:
            ParsePosition(data);
            break;
        default:
            break;
    }
}

void VescMotor::ParseSpeed(const uint8_t* data, uint32_t receiveTick)
{
    const uint32_t erpmRaw = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
                             (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
    const int32_t erpm = (erpmRaw < 0x80000000U) ? static_cast<int32_t>(erpmRaw)
                                                 : static_cast<int32_t>(static_cast<int64_t>(erpmRaw) - 0x100000000LL);
    rxData_.eRpm = static_cast<float>(erpm);
    rxData_.rpm = rxData_.eRpm / static_cast<float>(polePairs_);
    rxData_.speedOnline = true;
    rxData_.lastSpeedTick = receiveTick;

    rxData_.totalCurrent = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                                                                   static_cast<uint16_t>(data[5]))) /
                           10.0f;

    rxData_.duty = static_cast<float>(
                       static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | static_cast<uint16_t>(data[7]))) /
                   1000.0f;
}

void VescMotor::ParsePosition(const uint8_t* data)
{
    const bool firstPosition = !rxData_.hasPosition;
    rxData_.pidPositionLast = rxData_.pidPositionNow;
    rxData_.pidPositionNow = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                                                     static_cast<uint16_t>(data[7]))) /
                             50.0f;

    const float delta = rxData_.pidPositionNow - rxData_.pidPositionLast;
    if (!firstPosition && delta < -180.0f)
    {
        rxData_.turnCount++;
    }
    else if (!firstPosition && delta > 180.0f)
    {
        rxData_.turnCount--;
    }

    rxData_.totalPosition = static_cast<float>(rxData_.turnCount) * 360.0f + rxData_.pidPositionNow;
    rxData_.hasPosition = true;
}

VescRxData VescMotor::GetRxData(uint32_t nowTick) const
{
    VescRxData copy = rxData_;
    copy.speedOnline = copy.speedOnline && nowTick - copy.lastSpeedTick <= SPEED_ONLINE_TIMEOUT_MS;
    return copy;
}
