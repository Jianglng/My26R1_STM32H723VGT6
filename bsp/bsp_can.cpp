/* USER CODE BEGIN Header */
// 负责：
// 1、把FDCAN 初始化好
// 2、设置滤波器
// 3、统一发送 CAN 数据
// 4、接收中断回调分发
/* USER CODE END Header */

#include "bsp_can.h"

#include "vesc_motor.h" // VescMotor::ParseCanFeedback
#include "dji_6020.h"   // 引入 Dji6020Bus 协议解析接口

/* 接收帧对象 ---------------------------------------------------------------*/
FDCAN_RxFrame_TypeDef BSP_CAN::FDCAN_RxFIFO1Frame;  // VESC  (FDCAN1, FIFO1)
FDCAN_RxFrame_TypeDef BSP_CAN::FDCAN_RxFIFO0Frame;  // 6020  (FDCAN3, FIFO0)

/* 发送帧对象 ---------------------------------------------------------------*/
/* ARMCC used by this project does not accept C++ designated initializers. */
FDCAN_TxFrame_TypeDef BSP_CAN::FDCAN1_TxFrame = {};
FDCAN_TxFrame_TypeDef BSP_CAN::FDCAN3_TxFrame = {};



void BSP_CAN::Init(void)
{
    FDCAN_FilterTypeDef FilterConfig; 

    /* Initialize the reusable transmit frame once after CubeMX initialized FDCAN1. */
    FDCAN1_TxFrame.hcan = &hfdcan1;
    FDCAN1_TxFrame.Header.Identifier = 0;
    FDCAN1_TxFrame.Header.IdType = FDCAN_EXTENDED_ID;
    FDCAN1_TxFrame.Header.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN1_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_8;
    FDCAN1_TxFrame.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN1_TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN1_TxFrame.Header.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN1_TxFrame.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN1_TxFrame.Header.MessageMarker = 0;
    for (uint32_t i = 0; i < 8; ++i) FDCAN1_TxFrame.Data[i] = 0;

    /* FDCAN1 --------------------------------------------------------------*/
    FilterConfig.IdType       = FDCAN_EXTENDED_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  //通过该过滤器的报文放入 FDCAN1 的硬件 FIFO1
    FilterConfig.FilterID1    = 0x00000000;
    FilterConfig.FilterID2    = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, FDCAN_INTERRUPT_LINE1) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) Error_Handler();

    /* FDCAN3 (DJI 6020) -----------------------------------------------------*/
    /* 6020 用标准帧，与 VESC 的扩展帧不同，发送帧头必须按标准帧配置。 */
    FDCAN3_TxFrame.hcan = &hfdcan3;
    FDCAN3_TxFrame.Header.Identifier = 0x1FF;         // 1~4 号电机控制帧，发送前由协议层重设
    FDCAN3_TxFrame.Header.IdType = FDCAN_STANDARD_ID;
    FDCAN3_TxFrame.Header.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN3_TxFrame.Header.DataLength = FDCAN_DLC_BYTES_8;
    FDCAN3_TxFrame.Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN3_TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN3_TxFrame.Header.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN3_TxFrame.Header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    FDCAN3_TxFrame.Header.MessageMarker = 0;
    for (uint32_t i = 0; i < 8; ++i) FDCAN3_TxFrame.Data[i] = 0;

    /* 6020 反馈帧是标准帧（0x205~0x208），滤波器必须用 FDCAN_STANDARD_ID。
     * 用 FDCAN_EXTENDED_ID 会导致滤波器安装失败，帧被静默丢弃。 */
    FilterConfig.IdType       = FDCAN_STANDARD_ID;
    FilterConfig.FilterIndex  = 0;
    FilterConfig.FilterType   = FDCAN_FILTER_MASK;
    FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // 6020 走 FIFO0（VESC 走 FIFO1）
    FilterConfig.FilterID1    = 0x00000000;
    FilterConfig.FilterID2    = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FilterConfig) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ConfigInterruptLines(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) Error_Handler();
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) Error_Handler();

}
/* ============================================================
 *  添加消息到发送FIFO队列
 * ============================================================ */
void BSP_CAN::AddMessageToTxFifoQ(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame)
{
    HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame->hcan,
                                  &FDCAN_TxFrame->Header,
                                  FDCAN_TxFrame->Data);
}
/* ============================================================
 *  Vesc 总线接收回调，FDCAN1 FIFO1
 * ============================================================ */
void BSP_CAN::RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;

    FDCAN_RxFIFO1Frame.hcan = hfdcan;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO1,
                           &FDCAN_RxFIFO1Frame.Header,
                           FDCAN_RxFIFO1Frame.Data) != HAL_OK) return;

    /* VESC 总线：只把 ID 和数据交给协议层，硬件细节不往上透传 */
    if (hfdcan == &hfdcan1) {
        VescMotor::ParseCanFeedback(FDCAN_RxFIFO1Frame.Header.Identifier,
                                    FDCAN_RxFIFO1Frame.Data);
    }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    BSP_CAN::RxFifo1Callback(hfdcan, RxFifo1ITs);
}
/* ============================================================
 *  dji6020 总线接收回调，FDCAN3 FIFO0
 * ============================================================ */
void BSP_CAN::RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    FDCAN_RxFIFO0Frame.hcan = hfdcan;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO0,
                           &FDCAN_RxFIFO0Frame.Header,
                           FDCAN_RxFIFO0Frame.Data) != HAL_OK) return;

    /* 6020 总线：只把 ID 和数据交给协议层，硬件细节不往上透传 */
    if (hfdcan == &hfdcan3) {
        Dji6020Bus::ParseFeedback(FDCAN_RxFIFO0Frame.Header.Identifier,
                              FDCAN_RxFIFO0Frame.Data);
    }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BSP_CAN::RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void BSP_CAN::CheckBusOff(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == nullptr) return;

    /* AutoRetransmission=DISABLE 时，发送连续失败会进入 BUS_OFF，
     * 此时收发全部停止，必须手动 Stop + Start 才能恢复。
     *
     * 注意：不能用 HAL_FDCAN_GetError() 判断 BUS_OFF —— 它返回的是 HAL
     * 内部分类错误码（例如 512 = HAL_FDCAN_ERROR_FIFO_FULL，FIFO 满），
     * 与总线状态无关。正确做法是读协议状态标志 FDCAN_FLAG_BUS_OFF。 */
    if ((hfdcan->Instance->IR & FDCAN_FLAG_BUS_OFF) == 0) return;

    HAL_FDCAN_Stop(hfdcan);

    /* 清 BUS_OFF 标志，否则 Start 之后立刻又被判定为 BUS_OFF */
    hfdcan->Instance->IR = FDCAN_FLAG_BUS_OFF;

    HAL_FDCAN_Start(hfdcan);
}
