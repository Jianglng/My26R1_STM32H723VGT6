#include "chassis_odometry.h"

#include "math_utils.h"

#include <math.h>

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float DEG_TO_RAD = PI / 180.0f;
    constexpr float MAX_UPDATE_INTERVAL_S = 0.05f;
}

void ChassisOdometry::Reset()
{
    pose_ = ChassisPose2D{};
    lastImuYawDeg_ = 0.0f;
    imuYawInitialized_ = false;
    initialized_ = false;
    valid_ = false;
    feedbackLost_ = false;
}

void ChassisOdometry::Update(const ChassisBodyVelocity &measuredBodyVelocity,
                             float imuYawDeg, bool imuOnline, float dt, bool wheelFeedbackValid)
{
    if (feedbackLost_)
    {
        return;
    }

    const float bodyVx = measuredBodyVelocity.vx;
    const float bodyVy = measuredBodyVelocity.vy;
    const float bodyWz = measuredBodyVelocity.wz;
    if (!wheelFeedbackValid || !isfinite(bodyVx) || !isfinite(bodyVy) || !isfinite(bodyWz))
    {
        if (initialized_)
        {
            feedbackLost_ = true;
        }
        valid_ = false;
        return;
    }

    const bool imuYawValid = imuOnline && isfinite(imuYawDeg);
    if (!initialized_)
    {
        /* 首帧轮速反馈有效即建立原点；IMU 可随后上线，不积分未知的先前运动。 */
        if (imuYawValid)
        {
            lastImuYawDeg_ = imuYawDeg;
        }
        imuYawInitialized_ = imuYawValid;
        initialized_ = true;
        valid_ = true;
        return;
    }

    if (!isfinite(dt) || dt <= 0.0f || dt > MAX_UPDATE_INTERVAL_S)
    {
        feedbackLost_ = true;
        valid_ = false;
        return;
    }

    /* IMU 离线时由轮速角速度积分；重新上线的首帧只重建 IMU 角度基准。 */
    float deltaYaw = bodyWz * dt;
    if (imuYawValid)
    {
        if (imuYawInitialized_)
        {
            deltaYaw = MathUtils::WrapPeriodicError(imuYawDeg - lastImuYawDeg_, 360.0f) * DEG_TO_RAD;
        }
        lastImuYawDeg_ = imuYawDeg;
    }
    imuYawInitialized_ = imuYawValid;

    const float middleYaw = pose_.yaw + 0.5f * deltaYaw;
    const float cosYaw = cosf(middleYaw);
    const float sinYaw = sinf(middleYaw);

    pose_.x += (bodyVx * cosYaw - bodyVy * sinYaw) * dt;
    pose_.y += (bodyVx * sinYaw + bodyVy * cosYaw) * dt;
    pose_.yaw += deltaYaw;
    valid_ = true;
}
