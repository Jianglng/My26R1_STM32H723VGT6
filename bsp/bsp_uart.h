/**
 * @file bsp_uart.h
 * @brief UART DMA 接收与回调分发
 */
#pragma once

#include "stm32h7xx_hal.h"

/**
 * @brief 串口空闲接收
 *
 * 全工程只有这里实现 HAL 串口回调；设备 Init 时登记自己，
 * 回调按串口句柄把字节交回去，不解析 SBUS 或 HI91
 */
class BspUart
{
public:
    static UART_HandleTypeDef* RemotePort();
    static UART_HandleTypeDef* ImuPort();

    using RxHandler = void (*)(void* context, UART_HandleTypeDef* uart, uint16_t size);
    using ErrorHandler = void (*)(void* context, UART_HandleTypeDef* uart);

    struct ReceiveConfig
    {
        uint8_t* buffer;
        uint16_t length;
        void* context;
        RxHandler onRx;
        ErrorHandler onError;
    };

    /**
     * @brief 登记设备并启动 DMA 空闲接收
     * 同一串口再次调用会更新登记，不会占第二个槽
     */
    static HAL_StatusTypeDef StartReceive(UART_HandleTypeDef* uart, const ReceiveConfig& config);

    /** @brief 循环 DMA 写位置，仅对已成功初始化的串口和非零缓冲区长度调用 */
    static uint16_t WriteIndex(UART_HandleTypeDef* uart, uint16_t bufferLength);

    static void HandleRx(UART_HandleTypeDef* uart, uint16_t size);
    static void HandleError(UART_HandleTypeDef* uart);

private:
    static constexpr uint32_t SLOT_COUNT = 4U;

    struct Slot
    {
        UART_HandleTypeDef* uart;
        ReceiveConfig config;
        HAL_StatusTypeDef receiveStatus;
    };

    static Slot slots_[SLOT_COUNT];

    static Slot* Find(UART_HandleTypeDef* uart);
    static Slot* FindEmpty();
    static HAL_StatusTypeDef StartDma(Slot& slot);
};
