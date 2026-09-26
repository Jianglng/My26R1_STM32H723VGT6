#include "bsp_uart.h"

#include "dma.h"

BspUart::Slot BspUart::slots_[BspUart::SLOT_COUNT] = {};

BspUart::Slot *BspUart::Find(UART_HandleTypeDef *uart)
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

BspUart::Slot *BspUart::FindEmpty()
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

HAL_StatusTypeDef BspUart::StartDma(Slot *slot)
{
    if (slot == nullptr || slot->uart == nullptr || slot->buffer == nullptr)
    {
        return HAL_ERROR;
    }

    const HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(slot->uart, slot->buffer, slot->length);

    if (status == HAL_OK && slot->uart->hdmarx != nullptr)
    {
        __HAL_DMA_DISABLE_IT(slot->uart->hdmarx, DMA_IT_HT);
    }
    return status;
}

HAL_StatusTypeDef BspUart::StartReceive(UART_HandleTypeDef *uart,
                                        uint8_t *buffer,
                                        uint16_t length,
                                        void *context,
                                        RxHandler onRx,
                                        ErrorHandler onError)
{
    if (uart == nullptr || uart->hdmarx == nullptr || buffer == nullptr || length == 0U)
    {
        return HAL_ERROR;
    }

    Slot *slot = Find(uart);
    if (slot == nullptr)
    {
        slot = FindEmpty();
        if (slot == nullptr)
        {
            return HAL_ERROR;
        }
    }

    slot->uart = uart;
    slot->buffer = buffer;
    slot->length = length;
    slot->context = context;
    slot->onRx = onRx;
    slot->onError = onError;
    return StartDma(slot);
}

uint16_t BspUart::WriteIndex(UART_HandleTypeDef *uart, uint16_t bufferLength)
{
    if (uart == nullptr || uart->hdmarx == nullptr || bufferLength == 0U)
    {
        return 0U;
    }

    return static_cast<uint16_t>(bufferLength - __HAL_DMA_GET_COUNTER(uart->hdmarx));
}

void BspUart::HandleRx(UART_HandleTypeDef *uart, uint16_t size)
{
    Slot *slot = Find(uart);
    if (slot == nullptr || slot->onRx == nullptr)
    {
        return;
    }
    slot->onRx(slot->context, uart, size);
}

void BspUart::HandleError(UART_HandleTypeDef *uart)
{
    Slot *slot = Find(uart);
    if (slot == nullptr)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(uart);
    if (slot->onError != nullptr)
    {
        slot->onError(slot->context, uart);
    }
    StartDma(slot);
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    BspUart::HandleRx(uart, size);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    BspUart::HandleError(uart);
}
