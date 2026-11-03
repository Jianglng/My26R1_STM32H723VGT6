/**
 * @file chassis_odometry.h
 * @brief 轮速积分和 IMU 航向增量，反馈丢失后需要 Reset
 */
#pragma once

#include "motion_types.h"

struct OdometryInput
{
    ChassisBodyVelocity velocity;  // m/s、rad/s
    float imuYawDeg;
    float dt;          // s，应用层已验证
    uint32_t nowTick;  // ms
    bool imuOnline;
    bool wheelFeedbackValid;
};

class ChassisOdometry
{
public:
    void Reset();
    /** @brief 输入数值由解码边界保证有效，只管理在线状态和积分 */
    void Update(const OdometryInput& input);
    void Invalidate();
    const PoseFeedback& Feedback() const { return feedback_; }

private:
    enum class State : uint8_t
    {
        Uninitialized,
        Valid,
        Lost
    };
    State state_ = State::Uninitialized;
    PoseFeedback feedback_ = {};
    float lastImuYawDeg_ = 0.0f;
    bool imuBaselineReady_ = false;  // IMU 重连时须重建基准，与轮速定位状态独立
};
