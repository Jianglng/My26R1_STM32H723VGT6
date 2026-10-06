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
    initialized_ = false;
    valid_ = false;
    feedbackLost_ = false;
}

void ChassisOdometry::Update(float bodyVx, float bodyVy, float imuYawDeg,
                             float dt, bool feedbackValid)
{
    if (feedbackLost_)
    {
        return;
    }

    if (!feedbackValid || !isfinite(bodyVx) || !isfinite(bodyVy) || !isfinite(imuYawDeg))
    {
        if (initialized_)
        {
            feedbackLost_ = true;
        }
        valid_ = false;
        return;
    }

    if (!initialized_)//若尚未初始化，则初始化
    {
        /* 初次收到有效反馈时，只记录 IMU 角度，不积分未知的先前运动。 */
        lastImuYawDeg_ = imuYawDeg;
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

    const float deltaYaw = MathUtils::WrapPeriodicError(imuYawDeg - lastImuYawDeg_, 360.0f) * DEG_TO_RAD;
    const float middleYaw = pose_.yaw + 0.5f * deltaYaw;
    const float cosYaw = cosf(middleYaw);
    const float sinYaw = sinf(middleYaw);

    pose_.x += (bodyVx * cosYaw - bodyVy * sinYaw) * dt;
    pose_.y += (bodyVx * sinYaw + bodyVy * cosYaw) * dt;
    pose_.yaw += deltaYaw;
    lastImuYawDeg_ = imuYawDeg;
    valid_ = true;
}
