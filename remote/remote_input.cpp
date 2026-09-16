#include "remote_input.h"

// 全局遥控器对象，由底盘任务初始化和读取。
RemoteInput g_remoteInput;

/** @brief 绑定串口并启动 DMA 空闲接收。 */
HAL_StatusTypeDef RemoteInput::init(UART_HandleTypeDef *uart)
{
    if (uart == nullptr || uart->hdmarx == nullptr)
    {
        return HAL_ERROR;
    }

    uart_ = uart;
    return startReceive();
}

/** @brief 超过 300 ms 没有合法帧时，认为遥控器失联。 */
void RemoteInput::update()
{
    if (state_.validFrames != 0U &&
        (HAL_GetTick() - state_.lastUpdateTick) > 300U)
    {
        state_.online = false;
    }
}

/** @brief 空闲中断收到一包数据后，只解码完整的 18 字节 SBUS 帧。 */
void RemoteInput::onRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != uart_)
    {
        return;
    }

    if (size == FRAME_LENGTH)
    {
        decode(rxBuf_);
    }
}

/** @brief 丢弃当前接收，清除溢出标志并重新挂起 DMA。 */
void RemoteInput::onError(UART_HandleTypeDef *uart)
{
    if (uart != uart_)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(uart);
    startReceive();
}

/** @brief 启动 UART DMA 空闲接收，并关闭半传输中断。 */
HAL_StatusTypeDef RemoteInput::startReceive()
{
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, rxBuf_, RX_BUF_NUM);

    if (status == HAL_OK && uart_->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
    }

    return status;
}

/** @brief 按大疆官方 SBUS 位域拆出通道、拨杆、鼠标和键盘。 */
void RemoteInput::decode(const uint8_t *sbus)
{
    if (sbus == nullptr)
    {
        return;
    }

    state_.ch[0] = static_cast<int16_t>((sbus[0] | (sbus[1] << 8)) & 0x07FF);
    state_.ch[1] = static_cast<int16_t>(((sbus[1] >> 3) | (sbus[2] << 5)) & 0x07FF);
    state_.ch[2] = static_cast<int16_t>(((sbus[2] >> 6) | (sbus[3] << 2) | (sbus[4] << 10)) & 0x07FF);
    state_.ch[3] = static_cast<int16_t>(((sbus[4] >> 1) | (sbus[5] << 7)) & 0x07FF);
    state_.rightSwitch = static_cast<uint8_t>((sbus[5] >> 4) & 0x03U);
    state_.leftSwitch = static_cast<uint8_t>(((sbus[5] >> 4) & 0x0CU) >> 2);
    state_.mouseX = static_cast<int16_t>(sbus[6] | (sbus[7] << 8));
    state_.mouseY = static_cast<int16_t>(sbus[8] | (sbus[9] << 8));
    state_.mouseZ = static_cast<int16_t>(sbus[10] | (sbus[11] << 8));
    state_.mouseLeft = sbus[12];
    state_.mouseRight = sbus[13];
    state_.key = static_cast<uint16_t>(sbus[14] | (sbus[15] << 8));
    state_.ch[4] = static_cast<int16_t>(sbus[16] | (sbus[17] << 8));

    state_.ch[0] = static_cast<int16_t>(state_.ch[0] - CH_OFFSET);
    state_.ch[1] = static_cast<int16_t>(state_.ch[1] - CH_OFFSET);
    state_.ch[2] = static_cast<int16_t>(state_.ch[2] - CH_OFFSET);
    state_.ch[3] = static_cast<int16_t>(state_.ch[3] - CH_OFFSET);
    state_.ch[4] = static_cast<int16_t>(state_.ch[4] - CH_OFFSET);

    state_.rightX = state_.ch[0];
    state_.rightY = state_.ch[1];
    /* 本机实测：ch2 是左摇杆左右，ch3 是左摇杆上下。 */
    state_.leftX = state_.ch[2];
    state_.leftY = state_.ch[3];

    ++state_.validFrames;
    state_.lastUpdateTick = HAL_GetTick();
    state_.online = true;
}

/** @brief HAL 空闲接收完成回调，转交给遥控器对象。 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    g_remoteInput.onRxEvent(uart, size);
}

/** @brief HAL 串口错误回调，转交给遥控器对象。 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    g_remoteInput.onError(uart);
}
