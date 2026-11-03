/**
 * @file chassis_odometry.cpp
 * @brief IMU 断连使用轮速角速度，重连首帧只更新航向基准
 */
#include "chassis_odometry.h"

#include <math.h>

#include "math_utils.h"

void ChassisOdometry::Reset()
{
    state_ = State::Uninitialized;
    feedback_ = {};
    feedback_.source = PoseSource::Odometry;
    lastImuYawDeg_ = 0.0f;
    imuBaselineReady_ = false;
}

void ChassisOdometry::Invalidate()
{
    if (state_ == State::Valid)
    {
        state_ = State::Lost;
    }
    feedback_.valid = false;
}

void ChassisOdometry::Update(const OdometryInput& input)
{
    if (state_ == State::Lost)
    {
        return;
    }
    if (!input.wheelFeedbackValid)
    {
        Invalidate();
        return;
    }
    float deltaYaw = input.velocity.wz * input.dt;
    if (state_ == State::Uninitialized)
    {
        deltaYaw = 0.0f;
    }
    else if (input.imuOnline && imuBaselineReady_)
    {
        deltaYaw = MathUtils::DegToRad(MathUtils::WrapPeriodicError(input.imuYawDeg - lastImuYawDeg_, 360.0f));
    }
    lastImuYawDeg_ = input.imuYawDeg;
    imuBaselineReady_ = input.imuOnline;
    if (state_ == State::Valid)
    {
        const float middleYaw = feedback_.pose.yaw + 0.5f * deltaYaw;
        const float cosYaw = cosf(middleYaw);
        const float sinYaw = sinf(middleYaw);
        feedback_.pose.x += (input.velocity.vx * cosYaw - input.velocity.vy * sinYaw) * input.dt;
        feedback_.pose.y += (input.velocity.vx * sinYaw + input.velocity.vy * cosYaw) * input.dt;
        feedback_.pose.yaw += deltaYaw;
    }
    state_ = State::Valid;
    feedback_.valid = true;
    feedback_.updateTick = input.nowTick;
}
