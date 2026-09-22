#pragma once

#include <stdint.h>

#include "robot_config.h"

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
 * @brief 一组可直接交给 Pid::Init() 的参数。
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
    const uint32_t STEER_COUNT = RobotConfig::WHEEL_COUNT;
    const float ENCODER_RANGE = RobotConfig::DJI_ENCODER_MAX;

    const PidParam STEER_ANGLE =
    {
        RobotConfig::STEER_ANGLE_MAX,
        RobotConfig::STEER_ANGLE_I_LIMIT,
        RobotConfig::STEER_ANGLE_DEADBAND,
        RobotConfig::STEER_ANGLE_KP,
        RobotConfig::STEER_ANGLE_KI,
        RobotConfig::STEER_ANGLE_KD,
        PID_IMPROVE_INTEGRAL_LIMIT
    };

    const PidParam STEER_SPEED =
    {
        RobotConfig::STEER_SPEED_MAX,
        RobotConfig::STEER_SPEED_I_LIMIT,
        RobotConfig::STEER_SPEED_DEADBAND,
        RobotConfig::STEER_SPEED_KP,
        RobotConfig::STEER_SPEED_KI,
        RobotConfig::STEER_SPEED_KD,
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
     * @brief 写入参数并清零积分、误差历史。
     * @param param 例如 PidCfg::STEER_ANGLE、PidCfg::STEER_SPEED。
     */
    void Init(const PidParam &param);

    /** @brief 清零积分和输出。遥控器离线时应调用。 */
    void Reset();

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

/** @brief 按参考底盘参数初始化全部舵向 PID。须在 DWT_.Init() 之后调用。 */
void InitChassisPid();
