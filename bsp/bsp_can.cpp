/* USER CODE BEGIN Header */
/** @brief FDCAN 初始化、发送，以及把接收帧交给 VESC / 6020。 */
/* USER CODE END Header */

#include "bsp_can.h"

#include "vesc_motor.h"
#include "dji_6020.h"

/* 接收帧。VESC 用 FIFO1，6020 用 FIFO0。 */
FdcanRxFrame BspCan::fdcanRxFifo1Frame_;
FdcanRxFrame BspCan::fdcanRxFifo0Frame_;

/* 当前 C++ 方言不用指派初始化，发送帧在 Init() 里逐项赋值。 */
FdcanTxFrame BspCan::fdcan1TxFrame_ = {};
FdcanTxFrame BspCan::fdcan3TxFrame_ = {};



void BspCan::Init()
{
    FDCAN_FilterTypeDef FilterConfig; 

    /* Cube 已经初始化 FDCAN1，这里只填可复用的发送帧。 */
    fdcan1TxFrame_.hcan = &hfdcan1;
    fdcan1TxFrame_.header.Identifier = 0;
    fdcan1TxFrame_.header.IdType = FDCAN_EXTENDED_ID;
    fdcan1TxFrame_.header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan1TxFrame_.header.DataLength = FDCAN_DLC_BYTES_8;
    fdcan1TxFrame_.header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan1TxFrame_.header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan1TxFrame_.header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan1TxFrame_.header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan1TxFrame_.header.MessageMarker = 0;
    for (uint32_t i = 0; i < 8; ++i) fdcan1TxFrame_.data[i] = 0;

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
    fdcan3TxFrame_.hcan = &hfdcan3;
    fdcan3TxFrame_.header.Identifier = 0x1FF;         // 1~4 号电机控制帧，发送前由协议层重设
    fdcan3TxFrame_.header.IdType = FDCAN_STANDARD_ID;
    fdcan3TxFrame_.header.TxFrameType = FDCAN_DATA_FRAME;
    fdcan3TxFrame_.header.DataLength = FDCAN_DLC_BYTES_8;
    fdcan3TxFrame_.header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan3TxFrame_.header.BitRateSwitch = FDCAN_BRS_OFF;
    fdcan3TxFrame_.header.FDFormat = FDCAN_CLASSIC_CAN;
    fdcan3TxFrame_.header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan3TxFrame_.header.MessageMarker = 0;
    for (uint32_t i = 0; i < 8; ++i) fdcan3TxFrame_.data[i] = 0;

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
/** @brief 把已填好的帧放进硬件发送队列。 */
void BspCan::AddMessageToTxFifoQ(FdcanTxFrame *txFrame)
{
    HAL_FDCAN_AddMessageToTxFifoQ(txFrame->hcan,
                                  &txFrame->header,
                                  txFrame->data);
}
/** @brief FDCAN1 FIFO1：取出 VESC 扩展帧并交给协议层。 */
void BspCan::RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;

    fdcanRxFifo1Frame_.hcan = hfdcan;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO1,
                           &fdcanRxFifo1Frame_.header,
                           fdcanRxFifo1Frame_.data) != HAL_OK) return;

    /* VESC 状态帧必须是 29 位扩展数据帧，负载固定 8 字节。 */
    if (fdcanRxFifo1Frame_.header.IdType != FDCAN_EXTENDED_ID ||
        fdcanRxFifo1Frame_.header.RxFrameType != FDCAN_DATA_FRAME ||
        fdcanRxFifo1Frame_.header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    /* VESC 总线：只把 ID 和数据交给协议层，硬件细节不往上透传 */
    if (hfdcan == &hfdcan1) {
        VescMotor::ParseCanFeedback(fdcanRxFifo1Frame_.header.Identifier,
                                    fdcanRxFifo1Frame_.data);
    }
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    BspCan::RxFifo1Callback(hfdcan, RxFifo1ITs);
}
/** @brief FDCAN3 FIFO0：取出 6020 标准帧并交给协议层。 */
void BspCan::RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;

    fdcanRxFifo0Frame_.hcan = hfdcan;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                           FDCAN_RX_FIFO0,
                           &fdcanRxFifo0Frame_.header,
                           fdcanRxFifo0Frame_.data) != HAL_OK) return;

    /* DJI 6020 反馈必须是 11 位标准的数据帧，负载固定 8 字节。 */
    if (fdcanRxFifo0Frame_.header.IdType != FDCAN_STANDARD_ID ||
        fdcanRxFifo0Frame_.header.RxFrameType != FDCAN_DATA_FRAME ||
        fdcanRxFifo0Frame_.header.DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    /* 6020 总线：只把 ID 和数据交给协议层，硬件细节不往上透传 */
    if (hfdcan == &hfdcan3) {
        Dji6020Bus::ParseFeedback(fdcanRxFifo0Frame_.header.Identifier,
                              fdcanRxFifo0Frame_.data);
    }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BspCan::RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void BspCan::CheckBusOff(FDCAN_HandleTypeDef *hfdcan)
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
