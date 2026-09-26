#include "chaohe_imu.h"

#include <string.h>

#include "bsp_uart.h"

HAL_StatusTypeDef ChaoheImu::Init(UART_HandleTypeDef *uart)
{
    if (uart == nullptr || uart->hdmarx == nullptr)
    {
        return HAL_ERROR;
    }

    uart_ = uart;
    rxIndex_ = 0;
    memset(&decoder_, 0, sizeof(decoder_));
    state_ = ChaoheImuState{};
    const HAL_StatusTypeDef status = BspUart::StartReceive(
        uart_, rxBuf_, RX_BUF_NUM, this, nullptr, &ChaoheImu::HandleError);
    if (status == HAL_OK)
    {
        rxIndex_ = 0;
    }
    return status;
}

void ChaoheImu::Update()
{
    DrainRx();

    if (state_.validFrames != 0U &&
        (HAL_GetTick() - state_.lastUpdateTick) > ONLINE_TIMEOUT_MS)
    {
        state_.online = false;
    }
}

void ChaoheImu::HandleError(void *context, UART_HandleTypeDef *uart)
{
    (void)uart;
    ChaoheImu *self = static_cast<ChaoheImu *>(context);
    self->rxIndex_ = 0;
    memset(&self->decoder_, 0, sizeof(self->decoder_));
}

void ChaoheImu::DrainRx()
{
    if (uart_ == nullptr)
    {
        return;
    }

    const uint16_t pos = BspUart::WriteIndex(uart_, RX_BUF_NUM);

    while (rxIndex_ != pos)
    {
        const uint8_t byte = rxBuf_[rxIndex_];
        rxIndex_ = static_cast<uint16_t>((rxIndex_ + 1U) % RX_BUF_NUM);

        if (hipnuc_input(&decoder_, byte) > 0 &&
            decoder_.hi91.tag == HIPNUC_ID_HI91)
        {
            ApplyHi91(decoder_.hi91);
        }
    }
}

void ChaoheImu::ApplyHi91(const hi91_t &frame)
{
    state_.roll = frame.roll;
    state_.pitch = frame.pitch;
    state_.yaw = frame.yaw;
    state_.acc[0] = frame.acc[0];
    state_.acc[1] = frame.acc[1];
    state_.acc[2] = frame.acc[2];
    state_.gyr[0] = frame.gyr[0];
    state_.gyr[1] = frame.gyr[1];
    state_.gyr[2] = frame.gyr[2];
    state_.quat[0] = frame.quat[0];
    state_.quat[1] = frame.quat[1];
    state_.quat[2] = frame.quat[2];
    state_.quat[3] = frame.quat[3];
    state_.temperature = static_cast<float>(frame.temp);
    state_.status = frame.main_status;
    state_.attitudeOk = (frame.main_status & HIPNUC_STATUS_ATT_CONV) == 0U;
    state_.gyroBiasOk = (frame.main_status & HIPNUC_STATUS_WB_CONV) == 0U;
    ++state_.validFrames;
    state_.lastUpdateTick = HAL_GetTick();
    state_.online = true;
}

/** @brief HAL 空闲接收完成回调：按串口分发给遥控器和 IMU。 */
