/**
 * @file remote_input.cpp
 * @brief 遥控帧接收与通道解码
 */
#include "remote_input.h"

#include <string.h>

#include "bsp_critical.h"
#include "bsp_uart.h"

HAL_StatusTypeDef RemoteInput::Init(UART_HandleTypeDef* uart)
{
    uart_ = uart;
    lastIndex_ = 0U;
    const BspUart::ReceiveConfig config = {rxBuf_, RX_BUF_NUM, this, &RemoteInput::HandleRx, &RemoteInput::HandleError};
    const HAL_StatusTypeDef status = BspUart::StartReceive(uart_, config);
    if (status != HAL_OK)
    {
        uart_ = nullptr;
    }
    return status;
}

RemoteState RemoteInput::State()
{
    uint8_t frame[FRAME_LENGTH];
    uint32_t receiveTick = 0U;
    bool available = false;
    bool error = false;
    {
        CriticalSection lock;
        available = pending_;
        error = receiveError_;
        if (available)
        {
            memcpy(frame, pendingFrame_, FRAME_LENGTH);
            receiveTick = pendingTick_;
            pending_ = false;
        }
        receiveError_ = false;
    }
    if (error)
    {
        state_.online = false;
    }
    if (available)
    {
        Decode(frame, receiveTick);
    }
    state_.online = state_.online && HAL_GetTick() - state_.lastUpdateTick <= ONLINE_TIMEOUT_MS;
    return state_;
}

void RemoteInput::HandleRx(void* context, UART_HandleTypeDef* uart, uint16_t size)
{
    static_cast<RemoteInput*>(context)->OnRxEvent(uart, size);
}

void RemoteInput::HandleError(void* context, UART_HandleTypeDef* uart)
{
    static_cast<RemoteInput*>(context)->OnError(uart);
}

void RemoteInput::OnError(UART_HandleTypeDef* uart)
{
    if (uart != uart_)
    {
        return;
    }

    lastIndex_ = 0U;
    pending_ = false;
    receiveError_ = true;
}

void RemoteInput::OnRxEvent(UART_HandleTypeDef* uart, uint16_t size)
{
    /* size 是循环 DMA 这一圈的写位置，不是本帧长度
     * 绕回时 HAL 传入缓冲区长度，写位置记为 0
     * 距上次处理正好 18 字节才解码；同一位置的重复回调距离为 0，直接丢掉
     * 长度不是 18 也推进 lastIndex_，下一帧才能重新对齐 */
    if (uart != uart_ || size == 0U || size > RX_BUF_NUM)
    {
        return;
    }

    const uint16_t writeIndex = (size == RX_BUF_NUM) ? 0U : size;
    const uint16_t received = static_cast<uint16_t>((writeIndex + RX_BUF_NUM - lastIndex_) % RX_BUF_NUM);

    if (received == FRAME_LENGTH)
    {
        const uint16_t tail = static_cast<uint16_t>(RX_BUF_NUM - lastIndex_);
        if (tail >= FRAME_LENGTH)
        {
            memcpy(pendingFrame_, &rxBuf_[lastIndex_], FRAME_LENGTH);
        }
        else
        {
            memcpy(pendingFrame_, &rxBuf_[lastIndex_], tail);
            memcpy(&pendingFrame_[tail], rxBuf_, static_cast<size_t>(FRAME_LENGTH - tail));
        }
        pendingTick_ = HAL_GetTick();
        pending_ = true;
    }

    if (received != 0U)
    {
        lastIndex_ = writeIndex;
    }
}

void RemoteInput::Decode(const uint8_t* sbus, uint32_t receiveTick)
{
    const int16_t ch0 = static_cast<int16_t>(((sbus[0] | (sbus[1] << 8)) & 0x07FF) - CH_OFFSET);
    const int16_t ch1 = static_cast<int16_t>((((sbus[1] >> 3) | (sbus[2] << 5)) & 0x07FF) - CH_OFFSET);
    const int16_t ch2 =
        static_cast<int16_t>((((sbus[2] >> 6) | (sbus[3] << 2) | (sbus[4] << 10)) & 0x07FF) - CH_OFFSET);
    const int16_t ch3 = static_cast<int16_t>((((sbus[4] >> 1) | (sbus[5] << 7)) & 0x07FF) - CH_OFFSET);
    const int16_t ch4 = static_cast<int16_t>((sbus[16] | (sbus[17] << 8)) - CH_OFFSET);

    constexpr int16_t CHANNEL_LIMIT = 700;
    const uint8_t rightSwitch = static_cast<uint8_t>((sbus[5] >> 4) & 0x03U);
    const uint8_t leftSwitch = static_cast<uint8_t>((sbus[5] >> 6) & 0x03U);
    if (ch0 < -CHANNEL_LIMIT || ch0 > CHANNEL_LIMIT || ch1 < -CHANNEL_LIMIT || ch1 > CHANNEL_LIMIT ||
        ch2 < -CHANNEL_LIMIT || ch2 > CHANNEL_LIMIT || ch3 < -CHANNEL_LIMIT || ch3 > CHANNEL_LIMIT ||
        leftSwitch == 0U || rightSwitch == 0U)
    {
        return;
    }

    state_.rightSwitch = static_cast<RemoteSwitch>((sbus[5] >> 4) & 0x03U);
    state_.leftSwitch = static_cast<RemoteSwitch>(((sbus[5] >> 4) & 0x0CU) >> 2);
    state_.mouseX = static_cast<int16_t>(sbus[6] | (sbus[7] << 8));
    state_.mouseY = static_cast<int16_t>(sbus[8] | (sbus[9] << 8));
    state_.mouseZ = static_cast<int16_t>(sbus[10] | (sbus[11] << 8));
    state_.mouseLeft = sbus[12];
    state_.mouseRight = sbus[13];
    state_.key = static_cast<uint16_t>(sbus[14] | (sbus[15] << 8));

    state_.rightX = ch0;
    state_.rightY = ch1;
    /* 本机实测：ch2 是左摇杆左右，ch3 是左摇杆上下 */
    state_.leftX = ch2;
    state_.leftY = ch3;
    state_.wheel = ch4;

    ++state_.validFrames;
    state_.lastUpdateTick = receiveTick;
    state_.online = true;
}
