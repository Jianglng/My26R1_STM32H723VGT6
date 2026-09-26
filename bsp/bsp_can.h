#pragma once

#include "stm32h7xx.h"
#include "fdcan.h"

/** @brief FDCAN 发送帧。FDCAN1 给 VESC，FDCAN3 给 6020。 */
struct FdcanTxFrame
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef header; ///< CAN 报文头。
    uint8_t data[8];              ///< 最多 8 字节。
};

/** @brief FDCAN 接收帧。FIFO1 是 VESC，FIFO0 是 6020。 */
struct FdcanRxFrame
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef header; ///< CAN 报文头。
    uint8_t data[8];              ///< 最多 8 字节。
};

/**
 * @brief FDCAN 底层。
 *
 * 初始化 FDCAN1（VESC，扩展帧）和 FDCAN3（6020，标准帧），配置滤波器并统一发送。
 * 接收中断只把帧交给对应协议层。总线关闭时尝试恢复。
 */
class BspCan
{
public:
    static FdcanTxFrame fdcan1TxFrame_;      ///< VESC 发送帧，扩展帧。
    static FdcanTxFrame fdcan3TxFrame_;      ///< 6020 发送帧，标准帧。
    static FdcanRxFrame fdcanRxFifo1Frame_;  ///< VESC 接收帧，FIFO1。
    static FdcanRxFrame fdcanRxFifo0Frame_;  ///< 6020 接收帧，FIFO0。

public:
    /** @brief 配置滤波器并启动两路 FDCAN。 */
    static void Init();

    /** @brief 把已填好的帧放进发送队列。 */
    static void AddMessageToTxFifoQ(FdcanTxFrame *txFrame);

    /** @brief FDCAN1 FIFO1 接收。只把 VESC 的扩展数据帧交给协议层。 */
    static void RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);

    /** @brief FDCAN3 FIFO0 接收。只把 6020 的标准数据帧交给协议层。 */
    static void RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

    /** @brief 读 BUS_OFF 标志，必要时 Stop 后再 Start。不要用 HAL_FDCAN_GetError() 判断。 */
    static void CheckBusOff(FDCAN_HandleTypeDef *hfdcan);

};
