/**
 * @file vesc_motor.h
 * @brief VESC 控制组帧与状态反馈
 */
#pragma once

/**
 * @brief VESC CAN 协议层
 *
 * 扩展帧 29 位 ID：bits[7:0]=节点号，bits[15:8]=命令，高位为 0
 * 8 字节负载，有效数据按大端 int32 放在前 4 字节
 * eRPM = 机械 RPM × 极对数；极对数在初始化时传入
 * 滤波器和 FDCAN 启动在 bsp_can，本文件只组帧和解析
 */

#include <stdint.h>

#include "bsp_can.h"

enum class VescCommand : uint8_t
{
    SetDuty = 0,          ///< 占空比，-1.0~1.0，协议 ×100000
    SetCurrent = 1,       ///< 电流，接口单位 mA
    SetCurrentBrake = 2,  ///< 制动电流，mA
    SetRpm = 3,           ///< 转速，发送前乘极对数得到 eRPM
    SetPos = 4,           ///< 位置，单位由 VESC 配置决定
    FillRxBuffer = 5,
    FillRxBufferLong = 6,
    ProcessRxBuffer = 7,
    ProcessShortBuffer = 8,
    Status = 9,  ///< 状态1：eRPM、电流、占空比
    SetCurrentRel = 10,
    SetCurrentBrakeRel = 11,
    SetCurrentHandbrake = 12,     ///< 手刹电流，mA
    SetCurrentHandbrakeRel = 13,  ///< 手刹比例，0~1，协议 ×100000
    Status2 = 14,                 ///< 状态2：安时，当前未解析
    Status3 = 15,                 ///< 状态3：瓦时，当前未解析
    Status4 = 16,                 ///< 状态4：温度、PID 角度
    Ping = 17,
    Pong = 18,
    DetectApplyAllFoc = 19,
    DetectApplyAllFocRes = 20,
    ConfCurrentLimits = 21,
    ConfStoreCurrentLimits = 22,
    ConfCurrentLimitsIn = 23,
    ConfStoreCurrentLimitsIn = 24,
    ConfFocErpms = 25,
    ConfStoreFocErpms = 26,
    Status5 = 27  ///< 状态5：电压、里程，当前未解析
};

/** @brief 电调反馈；由状态帧解析填入 */
struct VescRxData
{
    float eRpm;              ///< 电气转速
    float rpm;               ///< 机械转速 = eRpm / 极对数
    float duty;              ///< 占空比，-1.0~1.0
    float totalCurrent;      ///< 相电流 (A)
    float pidPositionNow;    ///< PID 角度 0~360°，协议 ÷50
    float pidPositionLast;   ///< 上一帧角度，用于过零
    int32_t turnCount;       ///< 过零圈数
    float totalPosition;     ///< turnCount×360 + pidPositionNow
    bool hasPosition;        ///< 是否已收到过 Status4
    bool speedOnline;        ///< Status 转速反馈在超时时间内有效
    uint32_t lastSpeedTick;  ///< 最近一帧 Status 的接收时刻 (ms)

    VescRxData()
        : eRpm(0.f),
          rpm(0.f),
          duty(0.f),
          totalCurrent(0.f),
          pidPositionNow(0.f),
          pidPositionLast(0.f),
          turnCount(0),
          totalPosition(0.f),
          hasPosition(false),
          speedOnline(false),
          lastSpeedTick(0)
    {
    }
};

class VescMotor
{
public:
    VescMotor();
    VescMotor(const VescMotor&) = delete;
    VescMotor& operator=(const VescMotor&) = delete;

    /** @brief 绑定逻辑总线、节点号和非零极对数，在任务初始化阶段调用 */
    void Init(CanBus bus, uint16_t nodeId, uint32_t polePairs);

    /** @brief 力矩电流，单位 mA，正转正值，协议按 A×1000 发送 */
    bool SetCurrent(int32_t currentMa);

    bool SetRpm(int32_t rpm);

    /** @brief 占空比 -1.0~1.0，底盘未使用 */
    bool SetPwm(float pwm);

    /** @brief 位置模式原始值，底盘未使用 */
    bool SetPos(int32_t pos);

    /** @brief 专用制动电流，mA，与 SetCurrent 负值不同，只制动不驱动 */
    bool SetBrakeCurrent(int32_t currentMa);

    /** @brief 手刹电流，mA，用于位置保持，底盘未使用 */
    bool SetHandbrakeCurrent(int32_t currentMa);

    /** @brief 手刹比例 0~1，底盘未使用 */
    bool SetHandbrakeCurrentRel(float relative);

    /**
     * @brief 解析一帧已取出的反馈
     * @note 由 bsp_can 在底盘任务中分发，不碰 HAL
     */
    static void ParseCanFeedback(uint32_t identifier, const uint8_t* data, uint32_t receiveTick);

    VescRxData GetRxData(uint32_t nowTick) const;
    uint16_t GetNodeId() const { return nodeId_; }

private:
    static constexpr uint32_t SPEED_ONLINE_TIMEOUT_MS = 100U;

    static constexpr uint32_t MAX_REGISTERED_MOTORS = 16U;
    CanBus bus_ = CanBus::Wheel;
    uint32_t polePairs_ = 1U;
    uint16_t nodeId_;
    VescRxData rxData_;

    bool SendFrame(VescCommand cmd, const uint8_t (&data)[CAN_CLASSIC_PAYLOAD_SIZE]);

    /** @brief 大端拆 int32 到 data[0..3]；Cortex-M 上有符号右移是算术右移 */
    static void PackInt32BigEndian(int32_t val, uint8_t* data);

    void ParseStatusPayload(VescCommand cmd, const uint8_t* data, uint32_t receiveTick);
    void ParseSpeed(const uint8_t* data, uint32_t receiveTick);
    void ParsePosition(const uint8_t* data);

    static VescMotor* registry_[MAX_REGISTERED_MOTORS];
    static uint32_t registryCount_;

    void Register();
};
