/**
 * @file chassis_kinematics.cpp
 * @brief 模块接口与状态管理
 */
#include "chassis_kinematics.h"

#include <math.h>

#include "math_utils.h"

namespace
{
    constexpr float STOP_SPEED_MPS = 1.0e-4f;

}

ChassisKinematics::ChassisKinematics(const ChassisGeometry& geometry) : geometry_(geometry)
{
    for (uint32_t i = 0U; i < ChassisGeometry::WHEEL_COUNT; ++i)
    {
        command_.steerEncoder[i] = geometry_.zeroEncoder[i];
    }
}

const ChassisWheelCommand& ChassisKinematics::Inverse(const ChassisBodyVelocity& vel,
                                                      const int16_t encoder[ChassisGeometry::WHEEL_COUNT])
{
    const float encoderRange = geometry_.encoderRange;
    const float radToEncoder = encoderRange / (2.0f * MathUtils::PI);
    const float rpmPerMps = 60.0f * geometry_.gearRatio / (MathUtils::PI * geometry_.wheelDiameter);

    for (uint32_t i = 0; i < ChassisGeometry::WHEEL_COUNT; ++i)
    {
        const float vxWheel = vel.vx - vel.wz * geometry_.wheelY[i];
        const float vyWheel = vel.vy + vel.wz * geometry_.wheelX[i];
        const float speedMps = hypotf(vxWheel, vyWheel);
        float rpm = speedMps * rpmPerMps;

        if (speedMps > STOP_SPEED_MPS)
        {
            const float angleRad = atan2f(vyWheel, vxWheel);
            /* 6020 编码器增加方向为顺时针，故目标角取负 */
            float target = MathUtils::WrapPeriodic(geometry_.zeroEncoder[i] - angleRad * radToEncoder, encoderRange);
            const float err = MathUtils::WrapPeriodicError(target - static_cast<float>(encoder[i]), encoderRange);

            /* 目标与当前超过 90 度时，舵向改走对侧，轮速取反 */
            if (fabsf(err) > (0.25f * encoderRange))
            {
                target = MathUtils::WrapPeriodic(target + (0.5f * encoderRange), encoderRange);
                rpm = -rpm;
            }

            command_.steerEncoder[i] = target;
            command_.wheelRpm[i] = rpm;
        }
        else
        {
            command_.wheelRpm[i] = 0.0f;
        }
    }

    return command_;
}

ChassisBodyVelocity ChassisKinematics::Forward(const int16_t encoder[ChassisGeometry::WHEEL_COUNT],
                                               const float wheelRpm[ChassisGeometry::WHEEL_COUNT]) const
{
    const float encoderRange = geometry_.encoderRange;
    const float radToEncoder = encoderRange / (2.0f * MathUtils::PI);
    const float mpsPerRpm = MathUtils::PI * geometry_.wheelDiameter / (60.0f * geometry_.gearRatio);
    ChassisBodyVelocity velocity = {};
    float rotationSum = 0.0f;
    float radiusSquaredSum = 0.0f;

    for (uint32_t i = 0; i < ChassisGeometry::WHEEL_COUNT; ++i)
    {
        /* 与逆解使用相同零点和编码器方向；轮速正负表示沿舵向前进或后退 */
        const float angleRad =
            MathUtils::WrapPeriodicError(geometry_.zeroEncoder[i] - static_cast<float>(encoder[i]), encoderRange) /
            radToEncoder;
        const float speedMps = wheelRpm[i] * mpsPerRpm;
        const float vxWheel = speedMps * cosf(angleRad);
        const float vyWheel = speedMps * sinf(angleRad);

        velocity.vx += vxWheel;
        velocity.vy += vyWheel;
        rotationSum += -geometry_.wheelY[i] * vxWheel + geometry_.wheelX[i] * vyWheel;
        radiusSquaredSum += geometry_.wheelX[i] * geometry_.wheelX[i] + geometry_.wheelY[i] * geometry_.wheelY[i];
    }

    /* 当前轮心以车体中心为原点，三轮坐标和为零 */
    velocity.vx /= static_cast<float>(ChassisGeometry::WHEEL_COUNT);
    velocity.vy /= static_cast<float>(ChassisGeometry::WHEEL_COUNT);
    velocity.wz = rotationSum / radiusSquaredSum;
    return velocity;
}
