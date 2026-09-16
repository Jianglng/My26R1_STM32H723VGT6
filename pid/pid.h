#pragma once

#include <stdint.h>

/**
 * @brief 底盘 PID 参数，数值来自 Chassis2026_R1_H723。
 *
 * 参考工程底盘环把 dt 当成 1（约 1 ms 一个控制周期）。
 * 本工程用 DWT 测真实间隔，再换成毫秒，因此 Kp/Ki/Kd 可以原样使用。
 *
 * 6020 是串级：
 *   角度环输出 -> 速度环目标
 *   速度环输出 -> 电流原始值 ±25000，对应 ±20 A
 *
 * 轮向 VESC 在参考工程里走电调自身转速环，主控不另做 PID。
 */
namespace PidCfg
{
    const uint32_t STEER_COUNT = 3U;           ///< 三个舵向 6020。
    const float ENCODER_RANGE = 8192.0f;       ///< 6020 单圈编码器。

    const float STEER_ANGLE_KP = 4.0f;         ///< 舵向角度环 Kp。
    const float STEER_ANGLE_KI = 0.0f;         ///< 舵向角度环 Ki，参考工程为 0。
    const float STEER_ANGLE_KD = 0.8f;         ///< 舵向角度环 Kd。
    const float STEER_ANGLE_I_LIMIT = 3000.0f; ///< 舵向角度环积分限幅。
    const float STEER_ANGLE_MAX = 8192.0f;     ///< 舵向角度环输出限幅，作为速度环目标。
    const float STEER_ANGLE_DEADBAND = 0.0f;   ///< 舵向角度环死区。

    const float STEER_SPEED_KP = 4.0f;         ///< 舵向速度环 Kp。
    const float STEER_SPEED_KI = 0.005f;       ///< 舵向速度环 Ki。
    const float STEER_SPEED_KD = 0.2f;         ///< 舵向速度环 Kd。
    const float STEER_SPEED_I_LIMIT = 4000.0f; ///< 舵向速度环积分限幅。
    const float STEER_SPEED_MAX = 25000.0f;    ///< 舵向速度环输出限幅，6020 电流原始值。
    const float STEER_SPEED_DEADBAND = 0.0f;   ///< 舵向速度环死区。
}

/**
 * @brief PID 可选功能，可按位或。
 * @note 输出限幅始终生效。
 */
enum PidImprove
{
    PID_IMPROVE_NONE           = 0x00,  ///< 只用 P/I/D 和输出限幅。
    PID_IMPROVE_INTEGRAL_LIMIT = 0x01   ///< 打开积分限幅，底盘环需要打开。
};

/**
 * @brief 一组可直接交给 Pid::init() 的参数。
 */
struct PidParam
{
    float maxOut;          ///< 输出限幅，正负对称。
    float integralLimit;   ///< 积分限幅，正负对称。
    float deadband;        ///< 误差死区。
    float kp;              ///< 比例系数。
    float ki;              ///< 积分系数，与 dt(毫秒) 相乘。
    float kd;              ///< 微分系数，与 dt(毫秒) 相除。
    uint8_t improve;       ///< PidImprove 位掩码。
};

namespace PidCfg
{
    const PidParam STEER_ANGLE =
    {
        STEER_ANGLE_MAX,
        STEER_ANGLE_I_LIMIT,
        STEER_ANGLE_DEADBAND,
        STEER_ANGLE_KP,
        STEER_ANGLE_KI,
        STEER_ANGLE_KD,
        PID_IMPROVE_INTEGRAL_LIMIT
    };

    const PidParam STEER_SPEED =
    {
        STEER_SPEED_MAX,
        STEER_SPEED_I_LIMIT,
        STEER_SPEED_DEADBAND,
        STEER_SPEED_KP,
        STEER_SPEED_KI,
        STEER_SPEED_KD,
        PID_IMPROVE_INTEGRAL_LIMIT
    };
}

/**
 * @brief 单路 PID 的参数和运行状态。
 * @note 调试时 Watch 对象的 state。dt 单位是毫秒，1 kHz 时应接近 1。
 */
struct PidState
{
    float ref;             ///< 目标值。
    float kp;              ///< 比例系数。
    float ki;              ///< 积分系数。
    float kd;              ///< 微分系数。

    float measure;         ///< 当前测量值。
    float err;             ///< 当前误差 = 目标 - 测量。
    float lastErr;         ///< 上次误差，给微分用。

    float pOut;            ///< 比例项。
    float iOut;            ///< 积分项累计。
    float dOut;            ///< 微分项。
    float iTerm;           ///< 本周期积分增量。

    float output;          ///< 本周期总输出。

    float deadband;        ///< 误差死区。
    float maxOut;          ///< 输出限幅。
    float integralLimit;   ///< 积分限幅。

    uint32_t dwtCnt;       ///< 上次计算时的 DWT 周期计数。
    float dt;              ///< 距上次计算的时间间隔，单位毫秒。

    uint8_t improve;       ///< PidImprove 位掩码。
};

/**
 * @brief 位置式 PID，时间间隔由 DWT 实测后换成毫秒。
 *
 * 输出饱和且误差仍朝同一方向时停止积分。
 * 6020 角度环请用 calculateEncoder()，避免编码器过零打满。
 */
class Pid
{
public:
    PidState state;   ///< 公开状态，方便调试器展开观察。

    Pid();

    /**
     * @brief 按底盘参数表初始化。
     * @param param 例如 PidCfg::STEER_ANGLE、PidCfg::STEER_SPEED。
     */
    void init(const PidParam &param);

    /**
     * @brief 写入参数，并清零积分和误差历史。
     */
    void init(float maxOut,
              float integralLimit,
              float deadband,
              float kp,
              float ki,
              float kd,
              uint8_t improve);

    /** @brief 清零积分和输出。遥控器离线时应调用。 */
    void reset();

    /**
     * @brief 普通 PID，给速度环用。
     * @param measure 当前测量值。
     * @param ref 目标值。
     * @return 限幅后的输出。
     */
    float calculate(float measure, float ref);

    /**
     * @brief 带编码器过零的 PID，给 6020 角度环用。
     * @param measure 当前编码器。
     * @param ref 目标编码器。
     * @param encoderRange 一圈刻度，6020 为 8192。
     * @return 限幅后的输出。
     */
    float calculateEncoder(float measure, float ref, float encoderRange);

private:
    float wrapEncoderError(float err, float range) const;
    void applyIntegralLimit();
    void applyOutputLimit();
    float run(float err);
};

/** @brief 三个舵向角度环，下标 0/1/2 对应前/左/右。 */
extern Pid g_steerAnglePid[PidCfg::STEER_COUNT];

/** @brief 三个舵向速度环，下标与角度环一致。 */
extern Pid g_steerSpeedPid[PidCfg::STEER_COUNT];

/** @brief 按参考底盘参数初始化全部舵向 PID。须在 DWT_.init() 之后调用。 */
void initChassisPid();
