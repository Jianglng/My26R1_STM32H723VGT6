#include "chassis_kinematics.h"

#include <math.h>

ChassisKinematics g_chassisKinematics;

namespace
{
    const float kZeroEncoder[ChassisKinematicsCfg::WHEEL_COUNT] =
    {
        ChassisKinematicsCfg::FRONT_ZERO_ENCODER,
        ChassisKinematicsCfg::LEFT_ZERO_ENCODER,
        ChassisKinematicsCfg::RIGHT_ZERO_ENCODER
    };

    /*
     * 等边三角形轮心坐标，车体坐标：+x 向前，+y 向左。
     * 前轮在 +x，左轮在左后方，右轮在右后方。
     */
    const float kWheelX[ChassisKinematicsCfg::WHEEL_COUNT] =
    {
        ChassisKinematicsCfg::CHASSIS_RADIUS_M,
        -0.5f * ChassisKinematicsCfg::CHASSIS_RADIUS_M,
        -0.5f * ChassisKinematicsCfg::CHASSIS_RADIUS_M
    };

    /* 左轮 y 为负、右轮 y 为正：与 +y 向左的几何位置相反，
     * 按实车自转标定。右摇杆向右（顺时针）时前轮正确，左右轮对调后切线才对。 */
    const float kWheelY[ChassisKinematicsCfg::WHEEL_COUNT] =
    {
        0.0f,
        ChassisKinematicsCfg::LEFT_WHEEL_Y_SIGN * 0.5f * sqrtf(3.0f)
            * ChassisKinematicsCfg::CHASSIS_RADIUS_M,
        ChassisKinematicsCfg::RIGHT_WHEEL_Y_SIGN * 0.5f * sqrtf(3.0f)
            * ChassisKinematicsCfg::CHASSIS_RADIUS_M
    };

    float mpsToRpm(float speedMps)
    {
        const float wheelCircumference =
            2.0f * ChassisKinematicsCfg::PI * ChassisKinematicsCfg::WHEEL_RADIUS_M;
        return speedMps * 60.0f * ChassisKinematicsCfg::GEAR_RATIO / wheelCircumference;
    }
}

ChassisKinematics::ChassisKinematics(): bodyVelocity_() , command_()
{
    for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
    {
        command_.steerEncoder[i] = kZeroEncoder[i];
        command_.wheelRpm[i] = 0.0f;
    }
}

float ChassisKinematics::wrapEncoder(float value)
{
    value = fmodf(value, ChassisKinematicsCfg::ENCODER_MAX);
    if (value < 0.0f)
    {
        value += ChassisKinematicsCfg::ENCODER_MAX;
    }
    return value;
}

float ChassisKinematics::wrapEncoderError(float value)
{
    value = fmodf(value, ChassisKinematicsCfg::ENCODER_MAX);
    if (value > ChassisKinematicsCfg::HALF_ENCODER)
    {
        value -= ChassisKinematicsCfg::ENCODER_MAX;
    }
    else if (value < -ChassisKinematicsCfg::HALF_ENCODER)
    {
        value += ChassisKinematicsCfg::ENCODER_MAX;
    }
    return value;
}

ChassisWheelCommand ChassisKinematics::inverse(const ChassisBodyVelocity &vel, const int16_t encoder[ChassisKinematicsCfg::WHEEL_COUNT])
{
    bodyVelocity_ = vel;

    for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
    {
        const float vxWheel = vel.vx - vel.wz * kWheelY[i];
        const float vyWheel = vel.vy + vel.wz * kWheelX[i];
        const float speedMps = hypotf(vxWheel, vyWheel);//合成速度
        float rpm = mpsToRpm(speedMps);

        if (speedMps > ChassisKinematicsCfg::STOP_SPEED_MPS)
        {
            const float angleRad = atan2f(vyWheel, vxWheel);
            //6020编码器增加方向为顺时针旋转，故为‘-’号
            float target = wrapEncoder(kZeroEncoder[i] - angleRad * ChassisKinematicsCfg::RAD_TO_ENCODER);
            const float err = wrapEncoderError(target - static_cast<float>(encoder[i]));

            /* 目标与当前超过 90° 时，舵向改走对侧，轮速取反。 */
            if (fabsf(err) > ChassisKinematicsCfg::QUARTER_ENCODER)
            {
                target = wrapEncoder(target + ChassisKinematicsCfg::HALF_ENCODER);
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
