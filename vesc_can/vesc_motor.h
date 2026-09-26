#pragma once

/**
 * @brief VESC CAN 协议层。
 *
 * 扩展帧 29 位 ID：bits[7:0]=节点号，bits[15:8]=命令，高位为 0。
 * 8 字节负载，有效数据按大端 int32 放在前 4 字节。
 * eRPM = 机械 RPM × 极对数；本项目 24N28P，极对数 14。
 * 滤波器和 FDCAN 启动在 bsp_can，本文件只组帧和解析。
 */

#include "main.h"
#include "robot_config.h"
#include "stm32h7xx_hal_fdcan.h"

enum class CanPacketID : uint8_t
{
    SET_DUTY                     = 0,   ///< 占空比，-1.0~1.0，协议 ×100000。
    SET_CURRENT                  = 1,   ///< 电流，接口单位 mA。
    SET_CURRENT_BRAKE            = 2,   ///< 制动电流，mA。
    SET_RPM                      = 3,   ///< 转速，发送前乘极对数得到 eRPM。
    SET_POS                      = 4,   ///< 位置，单位由 VESC 配置决定。
    FILL_RX_BUFFER               = 5,
    FILL_RX_BUFFER_LONG          = 6,
    PROCESS_RX_BUFFER            = 7,
    PROCESS_SHORT_BUFFER         = 8,
    STATUS                       = 9,   ///< 状态1：eRPM、电流、占空比。
    SET_CURRENT_REL              = 10,
    SET_CURRENT_BRAKE_REL        = 11,
    SET_CURRENT_HANDBRAKE        = 12,  ///< 手刹电流，mA。
    SET_CURRENT_HANDBRAKE_REL    = 13,  ///< 手刹比例，0~1，协议 ×100000。
    STATUS_2                     = 14,  ///< 状态2：安时，当前未解析。
    STATUS_3                     = 15,  ///< 状态3：瓦时，当前未解析。
    STATUS_4                     = 16,  ///< 状态4：温度、PID 角度。
    PING                         = 17,
    PONG                         = 18,
    DETECT_APPLY_ALL_FOC         = 19,
    DETECT_APPLY_ALL_FOC_RES     = 20,
    CONF_CURRENT_LIMITS          = 21,
    CONF_STORE_CURRENT_LIMITS    = 22,
    CONF_CURRENT_LIMITS_IN       = 23,
    CONF_STORE_CURRENT_LIMITS_IN = 24,
    CONF_FOC_ERPMS               = 25,
    CONF_STORE_FOC_ERPMS         = 26,
    STATUS_5                     = 27   ///< 状态5：电压、里程，当前未解析。
};

/** @brief 电调反馈。由状态帧解析填入。 */
struct VescRxData
{
    float eRpm;             ///< 电气转速。
    float rpm;              ///< 机械转速 = eRpm / 极对数。
    float duty;             ///< 占空比，-1.0~1.0。
    float totalCurrent;     ///< 相电流 (A)。
    float pidPositionNow;   ///< PID 角度 0~360°，协议 ÷50。
    float pidPositionLast;  ///< 上一帧角度，用于过零。
    int   turnCount;        ///< 过零圈数。
    float totalPosition;    ///< turnCount×360 + pidPositionNow。
    bool  hasPosition;      ///< 是否已收到过 STATUS_4。

    VescRxData()
        : eRpm(0.f), rpm(0.f), duty(0.f), totalCurrent(0.f)
        , pidPositionNow(0.f), pidPositionLast(0.f)
        , turnCount(0), totalPosition(0.f), hasPosition(false)
    {}
};

class VescMotor
{
public:
    VescMotor();

    /** @brief 绑定 FDCAN 句柄和节点号。滤波器与启动由 BspCan::Init() 完成。 */
    void Init(FDCAN_HandleTypeDef *hfdcan, uint16_t nodeId);

    /** @brief 力矩电流，单位 mA，正转正值。协议按 A×1000 发送。 */
    void SetCurrent(int32_t current_mA);

    void SetRpm(int32_t rpm);

    /** @brief 占空比 -1.0~1.0。底盘未使用。 */
    void SetPwm(double pwm);

    /** @brief 位置模式原始值。底盘未使用。 */
    void SetPos(int32_t pos);

    /** @brief 专用制动电流，mA。与 SetCurrent 负值不同，只制动不驱动。 */
    void SetBrakeCurrent(int32_t current_mA);

    /** @brief 手刹电流，mA，用于位置保持。底盘未使用。 */
    void SetHandbrakeCurrent(int32_t current_mA);

    /** @brief 手刹比例 0~1。底盘未使用。 */
    void SetHandbrakeCurrentRel(float relative);

    /**
     * @brief 解析一帧已取出的反馈。
     * @note 由 bsp_can 在 FDCAN1 FIFO1 取帧后调用，不碰 HAL。
     */
    static void ParseCanFeedback(uint32_t identifier, const uint8_t data[8]);

    VescRxData GetRxData() const;
    uint16_t GetNodeId() const { return nodeId_; }
    FDCAN_HandleTypeDef *GetFdcan() const { return hfdcan_; }

private:
    FDCAN_HandleTypeDef *hfdcan_;
    uint16_t nodeId_;
    VescRxData rxData_;


    /** @brief 组一帧扩展数据帧；hfdcan_ 不是 FDCAN1 则不发送。 */
    void SendFrame(CanPacketID cmd, const uint8_t data[8]);

    /** @brief 大端拆 int32 到 data[0..3]。Cortex-M 上有符号右移是算术右移。 */
    static void PackInt32BigEndian(int32_t val, uint8_t *data);

    void ParseStatusPayload(CanPacketID cmd, const uint8_t data[8]);

    static VescMotor* registry_[RobotConfig::WHEEL_COUNT];
    static uint32_t registryCount_;

    void Register();
};
