/**
 * @file trapezoidal_profile.cpp
 * @brief 一维梯形参考轨迹
 */
#include "trapezoidal_profile.h"

#include <math.h>

void TrapezoidalProfile::Reset()
{
    state_ = TrapezoidalProfileState{};
    distance_ = 0.0f;
    acceleration_ = 0.0f;
    deceleration_ = 0.0f;
    peakVelocity_ = 0.0f;
    accelTime_ = 0.0f;
    cruiseTime_ = 0.0f;
    totalTime_ = 0.0f;
    accelDistance_ = 0.0f;
}

void TrapezoidalProfile::Start(float distance, float maxVelocity, float acceleration, float deceleration)
{
    Reset();

    if (distance == 0.0f)
    {
        state_.phase = TrapezoidalPhase::Finished;
        return;
    }

    const float maxVelocitySquared = maxVelocity * maxVelocity;
    // 达到指定最大速度所需的加减速距离
    const float fullAccelDistance = maxVelocitySquared / (2.0f * acceleration);
    const float fullDecelDistance = maxVelocitySquared / (2.0f * deceleration);

    float peakVelocity = maxVelocity;
    float cruiseDistance = 0.0f;
    if (distance >= fullAccelDistance + fullDecelDistance)
    {
        cruiseDistance = distance - fullAccelDistance - fullDecelDistance;
    }
    else
    {
        /* 短距离没有匀速段；加速段和减速段的距离之和等于总距离 */
        peakVelocity = sqrtf(2.0f * distance * acceleration * deceleration / (acceleration + deceleration));
    }

    const float accelTime = peakVelocity / acceleration;
    const float cruiseTime = cruiseDistance / peakVelocity;
    const float decelTime = peakVelocity / deceleration;
    const float totalTime = accelTime + cruiseTime + decelTime;
    const float accelDistance = 0.5f * peakVelocity * accelTime;

    distance_ = distance;
    acceleration_ = acceleration;
    deceleration_ = deceleration;
    peakVelocity_ = peakVelocity;
    accelTime_ = accelTime;
    cruiseTime_ = cruiseTime;
    totalTime_ = totalTime;
    accelDistance_ = accelDistance;
    state_.phase = TrapezoidalPhase::Accelerating;
    return;
}

void TrapezoidalProfile::Update(float dt)
{
    if (state_.phase == TrapezoidalPhase::Idle)
    {
        return;
    }

    if (Finished())
    {
        return;
    }

    /* 先处理跨过终点的情况，避免大 dt 累加后溢出 */
    if (dt >= totalTime_ - state_.elapsedTime)
    {
        state_.elapsedTime = totalTime_;
        state_.position = distance_;
        state_.velocity = 0.0f;
        state_.phase = TrapezoidalPhase::Finished;
        return;
    }

    state_.elapsedTime += dt;
    if (state_.elapsedTime < accelTime_)
    {
        const float t = state_.elapsedTime;
        state_.position = 0.5f * acceleration_ * t * t;
        state_.velocity = acceleration_ * t;
        state_.phase = TrapezoidalPhase::Accelerating;
    }
    else if (state_.elapsedTime < accelTime_ + cruiseTime_)
    {
        const float t = state_.elapsedTime - accelTime_;
        state_.position = accelDistance_ + peakVelocity_ * t;
        state_.velocity = peakVelocity_;
        state_.phase = TrapezoidalPhase::Cruising;
    }
    else
    {
        /* 从终点反算剩余距离，避免减速末端大数相减损失精度 */
        const float remainingTime = totalTime_ - state_.elapsedTime;
        state_.position = distance_ - 0.5f * deceleration_ * remainingTime * remainingTime;
        state_.velocity = deceleration_ * remainingTime;
        state_.phase = TrapezoidalPhase::Decelerating;
    }

    /* 将阶段边界处的浮点误差限制在合法输出范围内 */
    state_.position = fminf(distance_, fmaxf(0.0f, state_.position));
    state_.velocity = fminf(peakVelocity_, fmaxf(0.0f, state_.velocity));
    return;
}
