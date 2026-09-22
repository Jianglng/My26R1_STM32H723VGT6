#include "chaohe_imu.h"

#include <string.h>

#include "dma.h"
#include "remote_input.h"

ChaoheImu g_chaoheImu;

/** @brief 绑定串口并启动 DMA 循环接收。 */
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
    return startReceive();
}

/** @brief 取出 DMA 新字节解码 HI91；超过 100 ms 无合法帧则离线。 */
void ChaoheImu::update()
{
    drainRx();

    if (state_.validFrames != 0U &&
        (HAL_GetTick() - state_.lastUpdateTick) > ONLINE_TIMEOUT_MS)
    {
        state_.online = false;
    }
}

/** @brief 空闲中断只表明有新数据，解码统一放到 update()，避免和任务抢 decoder_。 */
void ChaoheImu::onRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    (void)uart;
    (void)size;
}

/** @brief 丢弃当前接收，清除溢出标志并重新挂起 DMA。 */
void ChaoheImu::onError(UART_HandleTypeDef *uart)
{
    if (uart != uart_)
    {
        return;
    }

    rxIndex_ = 0;
    memset(&decoder_, 0, sizeof(decoder_));
    __HAL_UART_CLEAR_OREFLAG(uart);
    startReceive();
}

/** @brief 启动 UART DMA 空闲接收，并关闭半传输中断。 */
HAL_StatusTypeDef ChaoheImu::startReceive()
{
    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(uart_, rxBuf_, RX_BUF_NUM);

    if (status == HAL_OK && uart_->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE_IT(uart_->hdmarx, DMA_IT_HT);
        rxIndex_ = 0;
    }

    return status;
}

/** @brief 按 DMA 写入位置取出新字节，交给官方解析器找 HI91 帧。 */
void ChaoheImu::drainRx()
{
    if (uart_ == nullptr || uart_->hdmarx == nullptr)
    {
        return;
    }

    const uint16_t pos = static_cast<uint16_t>(
        RX_BUF_NUM - __HAL_DMA_GET_COUNTER(uart_->hdmarx));

    while (rxIndex_ != pos)
    {
        const uint8_t byte = rxBuf_[rxIndex_];
        rxIndex_ = static_cast<uint16_t>((rxIndex_ + 1U) % RX_BUF_NUM);

        if (hipnuc_input(&decoder_, byte) > 0 &&
            decoder_.hi91.tag == HIPNUC_ID_HI91)
        {
            applyHi91(decoder_.hi91);
        }
    }
}

/** @brief 把 HI91 线单位填进对外状态，不换成 SI，方便对照说明书。 */
void ChaoheImu::applyHi91(const hi91_t &frame)
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
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    g_remoteInput.onRxEvent(uart, size);
    g_chaoheImu.onRxEvent(uart, size);
}

/** @brief HAL 串口错误回调：按串口分发给遥控器和 IMU。 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    g_remoteInput.onError(uart);
    g_chaoheImu.onError(uart);
}
