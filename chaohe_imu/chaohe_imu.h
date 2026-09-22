#pragma once

#include "main.h"
#include "hipnuc_dec.h"

/**
 * @brief 超核 HI12 串口 IMU 反馈。
 * @note  单位与说明书 HI91 一致：角度为度，加速度为 G，角速度为 deg/s。
 *        本型号 HI12M0 是 6 轴，yaw 会随时间漂移，不要当绝对航向用。
 */
struct ChaoheImuState
{
    float roll;                 ///< 横滚角 (deg)，绕 Y。
    float pitch;                ///< 俯仰角 (deg)，绕 X。
    float yaw;                  ///< 航向角 (deg)，绕 Z。6 轴会漂。
    float acc[3];               ///< 加速度 XYZ (G)。
    float gyr[3];               ///< 角速度 XYZ (deg/s)。
    float quat[4];              ///< 四元数 WXYZ。
    float temperature;          ///< 模组温度 (°C)。
    uint16_t status;            ///< MAIN_STATUS 原始状态字。
    bool attitudeOk;            ///< 姿态已收敛（ATT_CONV 为 0）。
    bool gyroBiasOk;            ///< 陀螺零偏已收敛（WB_CONV 为 0）。
    bool online;                ///< 超时时间内收到过合法 HI91 帧。
    uint32_t validFrames;       ///< 成功解码的 HI91 帧数。
    uint32_t lastUpdateTick;    ///< 最近合法帧的 HAL_GetTick()，单位 ms。
};

class ChaoheImu
{
public:
    /**
     * @brief 绑定已初始化的 UART，并启动 DMA 循环接收。
     * @param uart 串口句柄，当前工程传入 &huart1。
     * @return HAL 启动结果；空指针或未配置 DMA 时返回 HAL_ERROR。
     */
    HAL_StatusTypeDef Init(UART_HandleTypeDef *uart);

    /**
     * @brief 在底盘任务中周期调用：取出 DMA 新字节并解码，超时则标记离线。
     */
    void update();

    /** @brief 返回当前反馈的只读引用。 */
    const ChaoheImuState &state() const
    {
        return state_;
    }

    /**
     * @brief UART 空闲接收回调。解码在 update() 中完成，这里无需处理数据。
     */
    void onRxEvent(UART_HandleTypeDef *uart, uint16_t size);

    /**
     * @brief 串口错误回调：清除溢出、丢掉半帧并重新启动 DMA。
     */
    void onError(UART_HandleTypeDef *uart);

private:
    HAL_StatusTypeDef startReceive();
    void drainRx();
    void applyHi91(const hi91_t &frame);

    static const uint16_t RX_BUF_NUM = 256U;
    static const uint32_t ONLINE_TIMEOUT_MS = 100U;

    UART_HandleTypeDef *uart_ = nullptr;
    uint8_t rxBuf_[RX_BUF_NUM] = {0};
    uint16_t rxIndex_ = 0;
    hipnuc_raw_t decoder_ = {};
    ChaoheImuState state_ = {};
};

extern ChaoheImu g_chaoheImu;
