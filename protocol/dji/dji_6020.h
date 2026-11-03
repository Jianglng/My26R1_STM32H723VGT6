/**
 * @file dji_6020.h
 * @brief DJI 6020 电流组帧与反馈解析
 */

#pragma once

#include <stdint.h>

/* 协议常量 */
namespace Dji6020Cfg
{
    constexpr uint32_t GROUP_SIZE = 4U;

    /* CAN 标识符 */
    constexpr uint32_t CTRL_ID_GROUP1 = 0x1FF;    ///< 控制 1~4 号电机
    constexpr uint32_t CTRL_ID_GROUP2 = 0x2FF;    ///< 控制 5~8 号电机
    constexpr uint32_t FEEDBACK_ID_BASE = 0x205;  ///< 反馈 ID 基数，电机 n 为 BASE + n - 1

    constexpr uint32_t MOTOR_ID_MIN = 1;  ///< 可配置的电机 ID 范围
    constexpr uint32_t MOTOR_ID_MAX = 8;

    /* 电流标定 */
    constexpr int32_t RAW_MAX = 25000;      ///< 协议原始值上限，对应 +20 A
    constexpr float CURRENT_MAX_A = 20.0f;  ///< 满量程电流 (A)
    constexpr float RAW_PER_AMP = 1250.0f;  ///< 每安培对应原始值 = 25000 / 20

    /* 反馈解析 */
    constexpr float ENCODER_MAX = 8192.0f;  ///< 机械角度分辨率
    constexpr float DEG_PER_TICK = 360.0f / ENCODER_MAX;

    /* 在线判定 */
    constexpr uint32_t ONLINE_TIMEOUT_MS = 100;  ///< 超时未收到反馈判定掉线
}  // namespace Dji6020Cfg

/**
 * @brief 单个 6020 的反馈，仅由底盘任务解析和读取
 */
struct Dji6020RxData
{
    int16_t encoder;      ///< 机械角度原始值 0~8191
    float angleDeg;       ///< 单圈机械角度 0~360°
    float totalAngleDeg;  ///< 累计多圈角度（跨零点自动处理，可正可负）
    int16_t speedRpm;     ///< 转速 (rpm)，正负表示方向
    int16_t currentRaw;   ///< 转矩电流原始值（±25000）
    float currentAmp;     ///< 转矩电流 (A) = currentRaw / 1250
    uint8_t temperature;  ///< 电机温度 (°C)

    bool online;          ///< 是否在线（收到过反馈且在超时时间内）
    uint32_t lastRxTick;  ///< 最近一次收到反馈的系统时刻 (ms)
    uint32_t rxCount;     ///< 累计收到帧数（调试用，可判断丢帧）

    /* 内部状态：累计角度换算用，外部只读 */
    int16_t lastEncoder;  ///< 上一帧角度
    bool hasEncoder;      ///< 是否已收到过第一帧

    Dji6020RxData()
        : encoder(0),
          angleDeg(0.f),
          totalAngleDeg(0.f),
          speedRpm(0),
          currentRaw(0),
          currentAmp(0.f),
          temperature(0),
          online(false),
          lastRxTick(0),
          rxCount(0),
          lastEncoder(0),
          hasEncoder(false)
    {
    }
};

/**
 * @brief 6020 电流槽由底盘任务写入，Control() 按组打包发送
 *
 * BspCan 按逻辑总线接收原始报文，
 * 周期任务调用 Control()，bsp_can 把收到的帧交给 ParseFeedback()
 */
class Dji6020Motor;

class Dji6020Bus
{
public:
    /** @brief 打包已注册电机的电流并发送，返回发送是否成功 */
    static bool Control();

    /**
     * @brief 在底盘任务中解析一帧反馈，不做 HAL 操作
     * @param identifier 11 位标准帧 ID
     * @param data 8 字节负载
     */
    static void ParseFeedback(uint32_t identifier, const uint8_t* data, uint32_t receiveTick);

private:
    /* 各电机在本帧中的电流槽 */
    struct ControlSlot
    {
        int16_t currentRaw;  ///< 原始电流值 ±25000
        bool enabled;        ///< 是否参与控制（未 Init 的电机不写入）
    };

    static ControlSlot ctrlSlot_[Dji6020Cfg::MOTOR_ID_MAX];
    static bool hasMotor_;  ///< 是否已有电机注册（决定 Control() 是否发包）
    static Dji6020Motor* motors_[Dji6020Cfg::MOTOR_ID_MAX];
    static uint32_t motorCount_;

    static void Register(Dji6020Motor* motor);
    static bool SendGroup(uint32_t firstSlot, uint32_t id);

    /* 电机通过友元调用 Register()，写入自己的电流槽 */
    friend class Dji6020Motor;
};

/**
 * @brief 单个 6020，电机 ID 按电调配置，数组下标按机械位置，两者不必相同
 */
class Dji6020Motor
{
public:
    Dji6020Motor();
    Dji6020Motor(const Dji6020Motor&) = delete;
    Dji6020Motor& operator=(const Dji6020Motor&) = delete;

    /**
     * @brief 绑定电机 ID，范围 1~8，1~4 号走 0x1FF，5~8 号走 0x2FF
     * @note 超出范围会拒绝初始化；ID 配错时电机不响应，也不会另外报错
     */
    void Init(uint32_t motorId);

    /**
     * @brief 设置目标转矩电流，单位 A，超出 ±20 A 会限幅
     * @note 只写入电流槽，发送要等 Dji6020Bus::Control()
     */
    void SetCurrent(float currentA);

    /** @brief 按协议原始值设置电流，量程 ±25000 */
    void SetCurrentRaw(int16_t raw);

    /** @brief 目标电流置 0，表示零力矩，不是刹车锁死 */
    void Stop();

    /** @brief 返回本拍反馈及按接收时间计算的在线状态，仅底盘任务调用 */
    Dji6020RxData GetRxData(uint32_t nowTick) const;

    /** @brief 电机 ID 为 0 表示还没有 Init() */
    uint32_t GetMotorId() const { return motorId_; }

private:
    uint32_t motorId_;      ///< 电机标号 1~8，0 表示未初始化
    int32_t slotIndex_;     ///< 在 ctrlSlot_ 中的下标，-1 表示未注册
    Dji6020RxData rxData_;  ///< 反馈数据

    /* 编码器增量法累计多圈角度 */
    void UpdateAngle();
    void ApplyFeedback(const uint8_t* data, uint32_t receiveTick);

    friend class Dji6020Bus;
};
