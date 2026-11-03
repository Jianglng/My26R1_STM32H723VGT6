/** @file pid.cpp
 * @brief PID 计算和积分限幅，无硬件访问
 */
#include "pid.h"

#include <math.h>

void Pid::Init(const PidParam& param)
{
    param_ = param;
    Reset();
}

void Pid::Reset()
{
    state_ = PidState{};
}

float Pid::Calculate(float measure, float ref, float dtMs)
{
    state_.measure = measure;
    state_.ref = ref;

    float err = state_.ref - state_.measure;
    if (fabsf(err) < param_.deadband)
    {
        err = 0.0f;
    }

    return Run(err, dtMs);
}

float Pid::CalculateEncoder(float measure, float ref, float encoderRange, float dtMs)
{
    state_.measure = measure;
    state_.ref = ref;

    float err = MathUtils::WrapPeriodicError(state_.ref - state_.measure, encoderRange);
    if (fabsf(err) < param_.deadband)
    {
        err = 0.0f;
    }

    return Run(err, dtMs);
}

float Pid::Run(float err, float dtMs)
{
    state_.err = err;

    state_.dtMs = dtMs;

    state_.pOut = param_.kp * state_.err;
    state_.iTerm = param_.ki * state_.err * state_.dtMs;

    state_.dOut = param_.kd * (state_.err - state_.lastErr) / dtMs;

    ApplyIntegralLimit();
    state_.iOut += state_.iTerm;

    state_.output = state_.pOut + state_.iOut + state_.dOut;
    state_.output = MathUtils::ClampAbs(state_.output, param_.maxOut);

    state_.lastErr = state_.err;
    return state_.output;
}

void Pid::ApplyIntegralLimit()
{
    if ((param_.improve & PID_IMPROVE_INTEGRAL_LIMIT) == 0)
    {
        return;
    }

    const float nextIout = state_.iOut + state_.iTerm;
    const float unsaturated = state_.pOut + state_.iOut + state_.dOut;

    /* 输出已经顶满，且积分还在往同一侧堆，就丢掉本周期积分 */
    if (fabsf(unsaturated) > param_.maxOut && (state_.err * state_.iOut > 0.0f))
    {
        state_.iTerm = 0.0f;
    }

    if (nextIout > param_.integralLimit)
    {
        state_.iTerm = 0.0f;
        state_.iOut = param_.integralLimit;
    }
    else if (nextIout < -param_.integralLimit)
    {
        state_.iTerm = 0.0f;
        state_.iOut = -param_.integralLimit;
    }
}
