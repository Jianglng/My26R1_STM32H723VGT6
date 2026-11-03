/**
 * @file remote_input.h
 * @brief 遥控帧接收与通道解码
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

/**
 * @brief 大疆 DR16 / SBUS 遥控器反馈
 * @note 通道值已减去中位 1024，静止时约为 0，范围大约 -660~660
 *       拨杆取值：1=上，3=中，2=下
 */
enum class RemoteSwitch : uint8_t
{
    Invalid = 0U,
    Up = 1U,
    Down = 2U,
    Middle = 3U
};

struct RemoteState
{
    int16_t leftX;             ///< 左摇杆水平；对应 ch[2] 往右为正
    int16_t leftY;             ///< 左摇杆垂直；对应 ch[3] 往上为正
    int16_t rightX;            ///< 右摇杆水平，对应 ch[0]
    int16_t rightY;            ///< 右摇杆垂直，对应 ch[1]
    int16_t wheel;             ///< 波轮，减去中位后静止约为 0
    RemoteSwitch leftSwitch;   ///< 左拨杆 s[1]：1 上 / 3 中 / 2 下
    RemoteSwitch rightSwitch;  ///< 右拨杆 s[0]：1 上 / 3 中 / 2 下
    int16_t mouseX;            ///< 鼠标 X
    int16_t mouseY;            ///< 鼠标 Y
    int16_t mouseZ;            ///< 鼠标滚轮
    uint8_t mouseLeft;         ///< 鼠标左键，按下为 1
    uint8_t mouseRight;        ///< 鼠标右键，按下为 1
    uint16_t key;              ///< 键盘位域，与大疆官方 KEY_PRESSED_OFFSET_* 一致
    bool online;               ///< 收到有效帧后置 true；超过 300 ms 无新帧由 State() 标为离线
    uint32_t validFrames;      ///< 成功解码的帧数
    uint32_t lastUpdateTick;   ///< 最近有效帧的 HAL_GetTick()，单位 ms
};

class RemoteInput
{
public:
    RemoteInput() = default;
    RemoteInput(const RemoteInput&) = delete;
    RemoteInput& operator=(const RemoteInput&) = delete;

    /**
     * @brief 绑定遥控串口并启动循环 DMA 接收
     * @param uart 已初始化的遥控串口句柄
     * @return HAL 启动结果
     */
    HAL_StatusTypeDef Init(UART_HandleTypeDef* uart);

    /** @brief 返回一份与中断隔离的反馈拷贝；超过 300 ms 无有效帧则标记离线 */
    RemoteState State();

private:
    static void HandleRx(void* context, UART_HandleTypeDef* uart, uint16_t size);
    static void HandleError(void* context, UART_HandleTypeDef* uart);
    void OnRxEvent(UART_HandleTypeDef* uart, uint16_t size);
    void OnError(UART_HandleTypeDef* uart);
    void Decode(const uint8_t* sbus, uint32_t receiveTick);

    static constexpr uint16_t RX_BUF_NUM = 36U;
    static constexpr uint16_t FRAME_LENGTH = 18U;
    static constexpr int16_t CH_OFFSET = 1024;
    static constexpr uint32_t ONLINE_TIMEOUT_MS = 300U;

    UART_HandleTypeDef* uart_ = nullptr;
    uint8_t rxBuf_[RX_BUF_NUM] = {0};
    uint16_t lastIndex_ = 0;                   ///< 上一帧结束时的写位置；DMA 重开后从 0 重新计
    uint8_t pendingFrame_[FRAME_LENGTH] = {};  // UART 中断写，State() 内短临界区复制
    volatile uint32_t pendingTick_ = 0U;
    volatile bool pending_ = false;
    volatile bool receiveError_ = false;
    RemoteState state_ = {};  // 仅底盘任务更新
};
