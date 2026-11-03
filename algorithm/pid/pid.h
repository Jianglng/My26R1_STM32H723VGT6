/**
 * @file pid.h
 * @brief 位置式 PID，调用者传入已验证且统一的毫秒周期
 */
#pragma once

#include <stdint.h>

#include "math_utils.h"

enum PidImprove
{
    PID_IMPROVE_NONE = 0x00,
    PID_IMPROVE_INTEGRAL_LIMIT = 0x01
};

struct PidParam
{
    float maxOut;
    float integralLimit;
    float deadband;
    float kp;
    float ki;  // 积分使用 ms
    float kd;  // 微分使用 ms
    uint8_t improve;
};

/** @brief 编译期检查默认参数，运行时仅在 Watch 参数应用处检查 */
constexpr bool PidParamValid(const PidParam& p, float maxOutLimit)
{
    return p.maxOut > 0.0f && p.maxOut <= maxOutLimit && p.integralLimit >= 0.0f &&
           MathUtils::Finite(p.integralLimit) && p.deadband >= 0.0f && MathUtils::Finite(p.deadband) && p.kp >= 0.0f &&
           MathUtils::Finite(p.kp) && p.ki >= 0.0f && MathUtils::Finite(p.ki) && p.kd >= 0.0f &&
           MathUtils::Finite(p.kd) && (p.improve & ~PID_IMPROVE_INTEGRAL_LIMIT) == 0;
}

struct PidState
{
    float ref = 0.0f;
    float measure = 0.0f;
    float err = 0.0f;
    float lastErr = 0.0f;
    float pOut = 0.0f;
    float iOut = 0.0f;
    float dOut = 0.0f;
    float iTerm = 0.0f;
    float output = 0.0f;
    float dtMs = 0.0f;
};

class Pid
{
public:
    void Init(const PidParam& param);
    void Reset();
    /** @brief 已验证的测量和目标，dtMs 由应用层统一限制在 [0.2, 5] ms */
    float Calculate(float measure, float ref, float dtMs);
    /** @brief 编码器周期误差，量程由机械配置保证为正 */
    float CalculateEncoder(float measure, float ref, float encoderRange, float dtMs);
    const PidState& State() const { return state_; }
    const PidParam& Param() const { return param_; }

private:
    float Run(float err, float dtMs);
    void ApplyIntegralLimit();

    PidParam param_ = {};
    PidState state_ = {};
};
