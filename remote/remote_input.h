#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

/**
 * @brief 大疆 DR16 / SBUS 遥控器反馈。
 * @note 通道值已减去中位 1024，静止时约为 0，范围大约 -660~660。
 *       拨杆取值：1=上，3=中，2=下。
 */
struct RemoteState
{
    int16_t ch[5];               ///< ch0 右水平，ch1 右垂直，ch2 左水平，ch3 左垂直，ch4 波轮。
    int16_t leftX;               ///< 左摇杆水平，对应 ch[2]。往右为正。
    int16_t leftY;               ///< 左摇杆垂直，对应 ch[3]。往上为正。
    int16_t rightX;              ///< 右摇杆水平，对应 ch[0]。
    int16_t rightY;              ///< 右摇杆垂直，对应 ch[1]。
    uint8_t leftSwitch;          ///< 左拨杆 s[1]：1 上 / 3 中 / 2 下。
    uint8_t rightSwitch;         ///< 右拨杆 s[0]：1 上 / 3 中 / 2 下。
    int16_t mouseX;              ///< 鼠标 X。
    int16_t mouseY;              ///< 鼠标 Y。
    int16_t mouseZ;              ///< 鼠标滚轮。
    uint8_t mouseLeft;           ///< 鼠标左键，按下为 1。
    uint8_t mouseRight;          ///< 鼠标右键，按下为 1。
    uint16_t key;                ///< 键盘位域，与大疆官方 KEY_PRESSED_OFFSET_* 一致。
    bool online;                 ///< 收到 18 字节有效帧后置 true，由 update() 检查超时。
    uint32_t validFrames;        ///< 成功解码的帧数。
    uint32_t lastUpdateTick;     ///< 最近有效帧的 HAL_GetTick()，单位 ms。
};

class RemoteInput
{
public:
    /** @brief 绑定已初始化的 UART，并启动 DMA 空闲中断接收。
     *  @param uart 串口句柄，当前工程传入 &huart5。
     *  @return HAL 启动结果；空指针或未配置 DMA 时返回 HAL_ERROR。
     */
    HAL_StatusTypeDef Init(UART_HandleTypeDef *uart);

    /** @brief 在底盘任务中周期调用，超过 300 ms 无有效帧则标记离线。 */
    void update();

    /** @brief 返回当前反馈的只读引用。 */
    const RemoteState &state() const
    {
        return state_;
    }

    /** @brief UART 空闲接收完成回调：长度正确时解码一帧 SBUS。
     *  @param uart 触发回调的串口。
     *  @param size 本次 DMA 收到的字节数，完整帧为 18。
     */
    void onRxEvent(UART_HandleTypeDef *uart, uint16_t size);

    /** @brief 串口错误回调：清除溢出并重新启动 DMA 接收。 */
    void onError(UART_HandleTypeDef *uart);

private:
    /** @brief 按大疆 SBUS 布局解析 18 字节原始数据。 */
    void decode(const uint8_t *sbus);

    /** @brief 重新启动 UART DMA 空闲接收。 */
    HAL_StatusTypeDef startReceive();

    static const uint16_t RX_BUF_NUM = 36U;
    static const uint16_t FRAME_LENGTH = 18U;
    static const int16_t CH_OFFSET = 1024;

    UART_HandleTypeDef *uart_ = nullptr;
    uint8_t rxBuf_[RX_BUF_NUM] = {0};
    RemoteState state_ = {};
};

extern RemoteInput g_remoteInput;
