#include "pid.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"

namespace
{
    const float kDtMinMs = 0.2f;     // 小于 0.2 ms 时微分会过大。
    const float kDtMaxMs = 5.0f;     // 大于 5 ms 视为任务被卡住。
    const float kDtDefaultMs = 1.0f; // 与参考工程底盘环使用的 dt=1 一致。
}

Pid g_steerAnglePid[PidCfg::STEER_COUNT];
Pid g_steerSpeedPid[PidCfg::STEER_COUNT];

Pid::Pid()
{
    memset(&state, 0, sizeof(state));
    state.dt = kDtDefaultMs;
}

void Pid::Init(const PidParam &param)
{
    state.maxOut = param.maxOut;
    state.integralLimit = param.integralLimit;
    state.deadband = param.deadband;
    state.kp = param.kp;
    state.ki = param.ki;
    state.kd = param.kd;
    state.improve = param.improve;
    Reset();
}

void Pid::Reset()
{
    state.measure = 0.0f;
    state.err = 0.0f;
    state.lastErr = 0.0f;
    state.pOut = 0.0f;
    state.iOut = 0.0f;
    state.dOut = 0.0f;
    state.iTerm = 0.0f;
    state.output = 0.0f;
    state.dt = kDtDefaultMs;
    state.dwtCnt = DWT->CYCCNT;
}

float Pid::wrapEncoderError(float err, float range) const
{
    if (range <= 0.0f)
    {
        return err;
    }

    err = fmodf(err, range);
    const float half = 0.5f * range;
    if (err > half)
    {
        err -= range;
    }
    else if (err < -half)
    {
        err += range;
    }
    return err;
}

float Pid::calculate(float measure, float ref)
{
    state.measure = measure;
    state.ref = ref;

    float err = state.ref - state.measure;
    if (fabsf(err) < state.deadband)
    {
        err = 0.0f;
    }

    return run(err);
}

float Pid::calculateEncoder(float measure, float ref, float encoderRange)
{
    state.measure = measure;
    state.ref = ref;

    float err = wrapEncoderError(state.ref - state.measure, encoderRange);
    if (fabsf(err) < state.deadband)
    {
        err = 0.0f;
    }

    return run(err);
}

float Pid::run(float err)
{
    state.err = err;

    /* DWT 返回秒，乘 1000 得到毫秒，与参考工程底盘 Ki/Kd 对齐。 */
    state.dt = DWT_.getDeltaT(&state.dwtCnt) * 1000.0f;
    if (state.dt < kDtMinMs)
    {
        state.dt = kDtMinMs;
    }
    else if (state.dt > kDtMaxMs)
    {
        state.dt = kDtMaxMs;
    }

    state.pOut = state.kp * state.err;
    state.iTerm = state.ki * state.err * state.dt;

    if (state.dt > 1.0e-6f)
    {
        state.dOut = state.kd * (state.err - state.lastErr) / state.dt;
    }
    else
    {
        state.dOut = 0.0f;
    }

    applyIntegralLimit();
    state.iOut += state.iTerm;

    state.output = state.pOut + state.iOut + state.dOut;
    applyOutputLimit();

    state.lastErr = state.err;
    return state.output;
}

void Pid::applyIntegralLimit()
{
    if ((state.improve & PID_IMPROVE_INTEGRAL_LIMIT) == 0)
    {
        return;
    }

    const float nextIout = state.iOut + state.iTerm;
    const float unsaturated = state.pOut + state.iOut + state.dOut;

    /* 输出已经顶满，且积分还在往同一侧堆，就丢掉本周期积分。 */
    if (fabsf(unsaturated) > state.maxOut &&
        (state.err * state.iOut > 0.0f))
    {
        state.iTerm = 0.0f;
    }

    if (nextIout > state.integralLimit)
    {
        state.iTerm = 0.0f;
        state.iOut = state.integralLimit;
    }
    else if (nextIout < -state.integralLimit)
    {
        state.iTerm = 0.0f;
        state.iOut = -state.integralLimit;
    }
}

void Pid::applyOutputLimit()
{
    if (state.output > state.maxOut)
    {
        state.output = state.maxOut;
    }
    else if (state.output < -state.maxOut)
    {
        state.output = -state.maxOut;
    }
}

void InitChassisPid()
{
    for (uint32_t i = 0; i < PidCfg::STEER_COUNT; ++i)
    {
        g_steerAnglePid[i].Init(PidCfg::STEER_ANGLE);
        g_steerSpeedPid[i].Init(PidCfg::STEER_SPEED);
    }
}
