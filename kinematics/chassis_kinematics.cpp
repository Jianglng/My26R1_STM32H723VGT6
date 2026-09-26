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

float ChassisKinematics::WrapEncoder(float value)
{
    value = fmodf(value, ENCODER_MAX);
    if (value < 0.0f)
    {
        value += ENCODER_MAX;
    }
    return value;
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
            float target = WrapEncoder(ZERO_ENCODER[i] - angleRad * RAD_TO_ENCODER);
            const float err = MathUtils::WrapEncoderError(target - static_cast<float>(encoder[i]), ENCODER_MAX);

            /* 目标与当前超过 90 度时，舵向改走对侧，轮速取反。 */
            if (fabsf(err) > QUARTER_ENCODER)
            {
                target = WrapEncoder(target + HALF_ENCODER);
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
