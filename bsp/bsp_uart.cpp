/**
 * @file bsp_uart.cpp
 * @brief UART DMA 接收与回调分发
 */
#include "bsp_uart.h"

#include "dma.h"
#include "usart.h"

UART_HandleTypeDef* BspUart::RemotePort()
{
    return &huart5;
}
UART_HandleTypeDef* BspUart::ImuPort()
{
    return &huart10;
}

BspUart::Slot BspUart::slots_[BspUart::SLOT_COUNT] = {};

BspUart::Slot* BspUart::Find(UART_HandleTypeDef* uart)
{
    if (uart == nullptr)
    {
        return nullptr;
    }

    for (uint32_t i = 0; i < SLOT_COUNT; ++i)
    {
        if (slots_[i].uart == uart)
        {
            return &slots_[i];
        }
    }
    return nullptr;
}

BspUart::Slot* BspUart::FindEmpty()
{
    for (uint32_t i = 0; i < SLOT_COUNT; ++i)
    {
        if (slots_[i].uart == nullptr)
        {
            return &slots_[i];
        }
    }
    return nullptr;
}

HAL_StatusTypeDef BspUart::StartDma(Slot& slot)
{
    const HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(slot.uart, slot.config.buffer, slot.config.length);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(slot.uart->hdmarx, DMA_IT_HT);
    }
    slot.receiveStatus = status;
    return status;
}

HAL_StatusTypeDef BspUart::StartReceive(UART_HandleTypeDef* uart, const ReceiveConfig& config)
{
    if (uart == nullptr || uart->hdmarx == nullptr || config.buffer == nullptr || config.length == 0U)
    {
        return HAL_ERROR;
    }
    Slot* slot = Find(uart);
    if (slot == nullptr)
    {
        slot = FindEmpty();
        if (slot == nullptr)
        {
            return HAL_ERROR;
        }
    }
    slot->uart = uart;
    slot->config = config;
    return StartDma(*slot);
}

uint16_t BspUart::WriteIndex(UART_HandleTypeDef* uart, uint16_t bufferLength)
{
    return static_cast<uint16_t>(bufferLength - __HAL_DMA_GET_COUNTER(uart->hdmarx));
}

void BspUart::HandleRx(UART_HandleTypeDef* uart, uint16_t size)
{
    Slot* slot = Find(uart);
    if (slot == nullptr || slot->config.onRx == nullptr)
    {
        return;
    }
    slot->config.onRx(slot->config.context, uart, size);
}

void BspUart::HandleError(UART_HandleTypeDef* uart)
{
    Slot* slot = Find(uart);
    if (slot == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(uart);
    if (slot->config.onError != nullptr)
    {
        slot->config.onError(slot->config.context, uart);
    }
    // 失败状态保存在槽中，设备超时后自然进入离线
    (void)StartDma(*slot);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size)
{
    BspUart::HandleRx(uart, size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
    BspUart::HandleError(uart);
}
