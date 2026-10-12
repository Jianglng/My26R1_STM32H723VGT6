#include "chassis_kinematics.h"

#include "math_utils.h"

#include <math.h>

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float ENCODER_MAX = RobotConfig::DJI_ENCODER_MAX;
    constexpr float HALF_ENCODER = 0.5f * ENCODER_MAX;
    constexpr float QUARTER_ENCODER = 0.25f * ENCODER_MAX;
    constexpr float RAD_TO_ENCODER = ENCODER_MAX / (2.0f * PI);
    constexpr float WHEEL_RADIUS_M = 0.5f * RobotConfig::WHEEL_DIAMETER_M;
    constexpr float STOP_SPEED_MPS = 1.0e-4f;

    constexpr float ZERO_ENCODER[RobotConfig::WHEEL_COUNT] =
    {
        RobotConfig::FRONT_ZERO_ENCODER,
        RobotConfig::LEFT_ZERO_ENCODER,
        RobotConfig::RIGHT_ZERO_ENCODER
    };

    float MpsToRpm(float speedMps)
    {
        const float wheelCircumference = 2.0f * PI * WHEEL_RADIUS_M;
        return speedMps * 60.0f * RobotConfig::GEAR_RATIO / wheelCircumference;
    }

    float RpmToMps(float rpm)
    {
        const float wheelCircumference = 2.0f * PI * WHEEL_RADIUS_M;
        return rpm * wheelCircumference / (60.0f * RobotConfig::GEAR_RATIO);
    }
}

ChassisKinematics::ChassisKinematics()
    : bodyVelocity_()
    , command_()
{
    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        command_.steerEncoder[i] = ZERO_ENCODER[i];
        command_.wheelRpm[i] = 0.0f;
    }
}

ChassisWheelCommand ChassisKinematics::Inverse(const ChassisBodyVelocity &vel,
                                               const int16_t encoder[RobotConfig::WHEEL_COUNT])
{
    bodyVelocity_ = vel;

    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        const float vxWheel = vel.vx - vel.wz * RobotConfig::WHEEL_Y[i];
        const float vyWheel = vel.vy + vel.wz * RobotConfig::WHEEL_X[i];
        const float speedMps = hypotf(vxWheel, vyWheel);
        float rpm = MpsToRpm(speedMps);

        if (speedMps > STOP_SPEED_MPS)
        {
            const float angleRad = atan2f(vyWheel, vxWheel);
            /* 6020 编码器增加方向为顺时针，故目标角取负。 */
            float target = MathUtils::WrapPeriodic(ZERO_ENCODER[i] - angleRad * RAD_TO_ENCODER, ENCODER_MAX);
            const float err = MathUtils::WrapPeriodicError(target - static_cast<float>(encoder[i]), ENCODER_MAX);

            /* 目标与当前超过 90 度时，舵向改走对侧，轮速取反。 */
            if (fabsf(err) > QUARTER_ENCODER)
            {
                target = MathUtils::WrapPeriodic(target + HALF_ENCODER, ENCODER_MAX);
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

ChassisBodyVelocity ChassisKinematics::Forward(const int16_t encoder[RobotConfig::WHEEL_COUNT],
                                               const float wheelRpm[RobotConfig::WHEEL_COUNT]) const
{
    ChassisBodyVelocity velocity = {};
    float rotationSum = 0.0f;
    float radiusSquaredSum = 0.0f;

    for (uint32_t i = 0; i < RobotConfig::WHEEL_COUNT; ++i)
    {
        /* 与逆解使用相同零点和编码器方向。轮速正负表示沿舵向前进或后退。 */
        const float angleRad = MathUtils::WrapPeriodicError(ZERO_ENCODER[i] - static_cast<float>(encoder[i]), ENCODER_MAX) / RAD_TO_ENCODER;
        const float speedMps = RpmToMps(wheelRpm[i]);
        const float vxWheel = speedMps * cosf(angleRad);
        const float vyWheel = speedMps * sinf(angleRad);

        velocity.vx += vxWheel;
        velocity.vy += vyWheel;
        rotationSum += -RobotConfig::WHEEL_Y[i] * vxWheel + RobotConfig::WHEEL_X[i] * vyWheel;
        radiusSquaredSum += RobotConfig::WHEEL_X[i] * RobotConfig::WHEEL_X[i]
                          + RobotConfig::WHEEL_Y[i] * RobotConfig::WHEEL_Y[i];
    }

    /* 当前轮心以车体中心为原点，三轮坐标和为零。 */
    velocity.vx /= static_cast<float>(RobotConfig::WHEEL_COUNT);
    velocity.vy /= static_cast<float>(RobotConfig::WHEEL_COUNT);
    velocity.wz = rotationSum / radiusSquaredSum;
    return velocity;
}
