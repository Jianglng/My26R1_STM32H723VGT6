#include "pid.h"

#include "math_utils.h"

#include <math.h>

#include "bsp_dwt.h"

namespace
{
    constexpr float DT_MIN_MS = 0.2f;     // 小于 0.2 ms 时微分会过大。
    constexpr float DT_MAX_MS = 5.0f;     // 大于 5 ms 视为任务被卡住。
    constexpr float DT_DEFAULT_MS = 1.0f; // 与参考工程底盘环使用的 dt=1 一致。
}


Pid::Pid()
    : timer_(nullptr)
{
    state_ = {};
    state_.dt = DT_DEFAULT_MS;
}

void Pid::Init(const PidParam &param, DwtTimer &timer)
{
    timer_ = &timer;
    state_.maxOut = param.maxOut;
    state_.integralLimit = param.integralLimit;
    state_.deadband = param.deadband;
    state_.kp = param.kp;
    state_.ki = param.ki;
    state_.kd = param.kd;
    state_.improve = param.improve;
    Reset();
}

void Pid::Reset()
{
    state_.measure = 0.0f;
    state_.err = 0.0f;
    state_.lastErr = 0.0f;
    state_.pOut = 0.0f;
    state_.iOut = 0.0f;
    state_.dOut = 0.0f;
    state_.iTerm = 0.0f;
    state_.output = 0.0f;
    state_.dt = DT_DEFAULT_MS;
    state_.dwtCnt = DWT->CYCCNT;
}

float Pid::Calculate(float measure, float ref)
{
    state_.measure = measure;
    state_.ref = ref;

    float err = state_.ref - state_.measure;
    if (fabsf(err) < state_.deadband)
    {
        err = 0.0f;
    }

    return Run(err);
}

float Pid::CalculateEncoder(float measure, float ref, float encoderRange)
{
    state_.measure = measure;
    state_.ref = ref;

    float err = MathUtils::WrapPeriodicError(state_.ref - state_.measure, encoderRange);
    if (fabsf(err) < state_.deadband)
    {
        err = 0.0f;
    }

    return Run(err);
}

float Pid::Run(float err)
{
    state_.err = err;

    /* DWT 返回秒，乘 1000 得到毫秒，与参考工程底盘 Ki/Kd 对齐。 */
    if (timer_ == nullptr)
    {
        state_.dt = DT_DEFAULT_MS;
    }
    else
    {
        state_.dt = timer_->GetDeltaT(&state_.dwtCnt) * 1000.0f;
    }
    if (state_.dt < DT_MIN_MS)
    {
        state_.dt = DT_MIN_MS;
    }
    else if (state_.dt > DT_MAX_MS)
    {
        state_.dt = DT_MAX_MS;
    }

    state_.pOut = state_.kp * state_.err;
    state_.iTerm = state_.ki * state_.err * state_.dt;

    if (state_.dt > 1.0e-6f)
    {
        state_.dOut = state_.kd * (state_.err - state_.lastErr) / state_.dt;
    }
    else
    {
        state_.dOut = 0.0f;
    }

    ApplyIntegralLimit();
    state_.iOut += state_.iTerm;

    state_.output = state_.pOut + state_.iOut + state_.dOut;
    ApplyOutputLimit();

    state_.lastErr = state_.err;
    return state_.output;
}

void Pid::ApplyIntegralLimit()
{
    if ((state_.improve & PID_IMPROVE_INTEGRAL_LIMIT) == 0)
    {
        return;
    }

    const float nextIout = state_.iOut + state_.iTerm;
    const float unsaturated = state_.pOut + state_.iOut + state_.dOut;

    /* 输出已经顶满，且积分还在往同一侧堆，就丢掉本周期积分。 */
    if (fabsf(unsaturated) > state_.maxOut &&
        (state_.err * state_.iOut > 0.0f))
    {
        state_.iTerm = 0.0f;
    }

    if (nextIout > state_.integralLimit)
    {
        state_.iTerm = 0.0f;
        state_.iOut = state_.integralLimit;
    }
    else if (nextIout < -state_.integralLimit)
    {
        state_.iTerm = 0.0f;
        state_.iOut = -state_.integralLimit;
    }
}

void Pid::ApplyOutputLimit()
{
    state_.output = MathUtils::ClampAbs(state_.output, state_.maxOut);
}
