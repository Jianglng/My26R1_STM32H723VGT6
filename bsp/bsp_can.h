#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32h7xx.h"
#include "fdcan.h"

/**
 * @brief The structure that contains the Information of FDCAN Transmit.
 */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_TxHeaderTypeDef Header; //CAN 报文头
    uint8_t Data[8];              //最多 8 字节的数据
} FDCAN_TxFrame_TypeDef;

/**
 * @brief The structure that contains the Information of FDCAN Receive.
 */
typedef struct
{
    FDCAN_HandleTypeDef *hcan;
    FDCAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} FDCAN_RxFrame_TypeDef;

/**
 * @brief
 *        FDCAN BSP class
 *        负责：
 *        1、初始化FDCAN（FDCAN1 = VESC，FDCAN3 = DJI 6020）
 *        2、配置滤波器
 *        3、统一发送接口
 *        4、接收中断回调分发
 *        5、总线 BUS_OFF 恢复
 */
class BSP_CAN
{
public:
    static FDCAN_TxFrame_TypeDef FDCAN1_TxFrame;      // VESC 发送帧（扩展帧）
    static FDCAN_TxFrame_TypeDef FDCAN3_TxFrame;      // 6020 发送帧（标准帧）
    static FDCAN_RxFrame_TypeDef FDCAN_RxFIFO1Frame;  // VESC 接收帧（FIFO1）
    static FDCAN_RxFrame_TypeDef FDCAN_RxFIFO0Frame;  // 6020 接收帧（FIFO0）

public:
    static void Init(void);//FDCAN初始化
    static void AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame);//统一发送接口

    static void RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs);//FIFO1接收回调(VESC)
    static void RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);//FIFO0接收回调(6020)

    static void CheckBusOff(FDCAN_HandleTypeDef *hfdcan);//检测并恢复总线BUS_OFF

};

#endif
