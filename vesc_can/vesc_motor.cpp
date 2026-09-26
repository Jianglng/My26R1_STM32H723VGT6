#include "vesc_motor.h"
#include "bsp_can.h"
#include "fdcan.h"

VescMotor* VescMotor::registry_[RobotConfig::WHEEL_COUNT] = {};
uint32_t VescMotor::registryCount_ = 0;

VescMotor::VescMotor()
    : hfdcan_(nullptr)
    , nodeId_(0)
    , rxData_()
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

    if (registryCount_ >= RobotConfig::WHEEL_COUNT)
    {
        return;
    }

    registry_[registryCount_++] = this;
}

void VescMotor::Init(FDCAN_HandleTypeDef *hfdcan, uint16_t nodeId)
{
    hfdcan_ = hfdcan;
    nodeId_ = nodeId;
    Register();
}

void VescMotor::PackInt32BigEndian(int32_t val, uint8_t *data)
{
    data[0] = static_cast<uint8_t>((val >> 24) & 0xFF);
    data[1] = static_cast<uint8_t>((val >> 16) & 0xFF);
    data[2] = static_cast<uint8_t>((val >> 8) & 0xFF);
    data[3] = static_cast<uint8_t>(val & 0xFF);
}

void VescMotor::SendFrame(CanPacketID cmd, const uint8_t data[8])
{
    FdcanTxFrame *txFrame = nullptr;
    if (hfdcan_ == &hfdcan1)
    {
        txFrame = &BspCan::fdcan1TxFrame_;
    }
    else
    {
        return;
    }

    txFrame->header.IdType = FDCAN_EXTENDED_ID;
    txFrame->header.TxFrameType = FDCAN_DATA_FRAME;
    txFrame->header.DataLength = FDCAN_DLC_BYTES_8;
    txFrame->header.Identifier =
        (static_cast<uint32_t>(nodeId_) & 0xFF) |
        (static_cast<uint32_t>(cmd) << 8);

    txFrame->header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txFrame->header.BitRateSwitch = FDCAN_BRS_OFF;
    txFrame->header.FDFormat = FDCAN_CLASSIC_CAN;
    txFrame->header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txFrame->header.MessageMarker = 0;

    for (uint32_t i = 0; i < 8; ++i)
    {
        txFrame->data[i] = data[i];
    }

    BspCan::AddMessageToTxFifoQ(txFrame);
}

void VescMotor::SetCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(current_mA, data);
    SendFrame(CanPacketID::SET_CURRENT, data);
}

void VescMotor::SetRpm(int32_t rpm)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(rpm * RobotConfig::VESC_POLE_PAIRS, data);
    SendFrame(CanPacketID::SET_RPM, data);
}

void VescMotor::SetPwm(double pwm)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(static_cast<int32_t>(pwm * 100000.0), data);
    SendFrame(CanPacketID::SET_DUTY, data);
}

void VescMotor::SetPos(int32_t pos)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(pos, data);
    SendFrame(CanPacketID::SET_POS, data);
}

void VescMotor::SetBrakeCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(current_mA, data);
    SendFrame(CanPacketID::SET_CURRENT_BRAKE, data);
}

void VescMotor::SetHandbrakeCurrent(int32_t current_mA)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(current_mA, data);
    SendFrame(CanPacketID::SET_CURRENT_HANDBRAKE, data);
}

void VescMotor::SetHandbrakeCurrentRel(float relative)
{
    uint8_t data[8] = {0};
    PackInt32BigEndian(static_cast<int32_t>(relative * 100000.0f), data);
    SendFrame(CanPacketID::SET_CURRENT_HANDBRAKE_REL, data);
}

void VescMotor::ParseCanFeedback(uint32_t identifier, const uint8_t data[8])
{
    if (data == nullptr)
    {
        return;
    }

    const uint8_t rxNodeId = static_cast<uint8_t>(identifier & 0xFF);
    const CanPacketID cmd = static_cast<CanPacketID>((identifier >> 8) & 0xFF);

    for (uint32_t i = 0; i < registryCount_; ++i)
    {
        VescMotor* motor = registry_[i];
        if (motor == nullptr || motor->hfdcan_ == nullptr)
        {
            continue;
        }
        if (static_cast<uint8_t>(motor->nodeId_) != rxNodeId)
        {
            continue;
        }

        motor->ParseStatusPayload(cmd, data);
        return;
    }
}

void VescMotor::ParseStatusPayload(CanPacketID cmd, const uint8_t data[8])
{
    switch (cmd)
    {
        /* STATUS：int32 eRPM、int16 电流×10、int16 占空比×1000。 */
        case CanPacketID::STATUS:
        {
            const int32_t erpm =
                (static_cast<int32_t>(data[0]) << 24) |
                (static_cast<int32_t>(data[1]) << 16) |
                (static_cast<int32_t>(data[2]) << 8) |
                static_cast<int32_t>(data[3]);
            rxData_.eRpm = static_cast<float>(erpm);
            rxData_.rpm = rxData_.eRpm / static_cast<float>(RobotConfig::VESC_POLE_PAIRS);

            rxData_.totalCurrent = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) |
                                     static_cast<uint16_t>(data[5]))) / 10.0f;

            rxData_.duty = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                     static_cast<uint16_t>(data[7]))) / 1000.0f;
            break;
        }

        case CanPacketID::STATUS_2:
        case CanPacketID::STATUS_3:
        case CanPacketID::STATUS_5:
            break;

        /*
         * STATUS_4：Byte6/7 为 PID 角度×50，0~360°。
         * 跨零：delta < -180 正转加圈，delta > 180 反转减圈。
         */
        case CanPacketID::STATUS_4:
        {
            const bool first_position = !rxData_.hasPosition;
            rxData_.pidPositionLast = rxData_.pidPositionNow;
            rxData_.pidPositionNow = static_cast<float>(
                static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) |
                                     static_cast<uint16_t>(data[7]))) / 50.0f;

            const float delta = rxData_.pidPositionNow - rxData_.pidPositionLast;
            if (!first_position && delta < -180.0f)
            {
                rxData_.turnCount++;
            }
            else if (!first_position && delta > 180.0f)
            {
                rxData_.turnCount--;
            }

            rxData_.totalPosition =
                static_cast<float>(rxData_.turnCount) * 360.0f +
                rxData_.pidPositionNow;
            rxData_.hasPosition = true;
            break;
        }

        default:
            break;
    }
}

VescRxData VescMotor::GetRxData() const
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    VescRxData copy = rxData_;
    __set_PRIMASK(primask);
    return copy;
}
