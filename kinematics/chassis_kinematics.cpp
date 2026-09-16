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
        -0.5f * sqrtf(3.0f) * ChassisKinematicsCfg::CHASSIS_RADIUS_M,
         0.5f * sqrtf(3.0f) * ChassisKinematicsCfg::CHASSIS_RADIUS_M
    };

    float mpsToRpm(float speedMps)
    {
        const float wheelCircumference =
            2.0f * ChassisKinematicsCfg::PI * ChassisKinematicsCfg::WHEEL_RADIUS_M;
        return speedMps * 60.0f * ChassisKinematicsCfg::GEAR_RATIO / wheelCircumference;
    }
}

ChassisKinematics::ChassisKinematics()
    : bodyVelocity_()
    , command_()
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

float ChassisKinematics::applyDeadband(float value, float deadband)
{
    if (fabsf(value) < deadband)
    {
        return 0.0f;
    }
    return value;
}

float ChassisKinematics::clampAbs(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

ChassisBodyVelocity ChassisKinematics::mapRemote(const RemoteState &remote) const
{
    ChassisBodyVelocity vel = {};
    if (!remote.online)
    {
        return vel;
    }

    const float scaleX = ChassisKinematicsCfg::DEFAULT_MAX_VX / ChassisKinematicsCfg::REMOTE_CH_MAX;
    const float scaleY = ChassisKinematicsCfg::DEFAULT_MAX_VY / ChassisKinematicsCfg::REMOTE_CH_MAX;
    const float scaleW = ChassisKinematicsCfg::DEFAULT_MAX_WZ / ChassisKinematicsCfg::REMOTE_CH_MAX;

    const float stickForward = applyDeadband(static_cast<float>(remote.leftY),
                                             ChassisKinematicsCfg::REMOTE_DEADBAND);
    const float stickRight = applyDeadband(static_cast<float>(remote.leftX),
                                           ChassisKinematicsCfg::REMOTE_DEADBAND);
    const float stickYaw = applyDeadband(static_cast<float>(remote.rightX),
                                         ChassisKinematicsCfg::REMOTE_DEADBAND);

    /* 左摇杆上推前进；左推车体向左，右推车体向右。 */
    vel.vx = clampAbs(stickForward * scaleX, ChassisKinematicsCfg::DEFAULT_MAX_VX);
    vel.vy = clampAbs(stickRight * scaleY, ChassisKinematicsCfg::DEFAULT_MAX_VY);
    vel.wz = clampAbs(-stickYaw * scaleW, ChassisKinematicsCfg::DEFAULT_MAX_WZ);
    return vel;
}

ChassisWheelCommand ChassisKinematics::inverse(const ChassisBodyVelocity &vel, const int16_t encoder[ChassisKinematicsCfg::WHEEL_COUNT])
{
    bodyVelocity_ = vel;

    for (uint32_t i = 0; i < ChassisKinematicsCfg::WHEEL_COUNT; ++i)
    {
        const float vxWheel = vel.vx - vel.wz * kWheelY[i];
        const float vyWheel = vel.vy + vel.wz * kWheelX[i];
        const float speedMps = hypotf(vxWheel, vyWheel);
        float rpm = mpsToRpm(speedMps);

        if (speedMps > ChassisKinematicsCfg::STOP_SPEED_MPS)
        {
            const float angleRad = atan2f(vyWheel, vxWheel);
            float target = wrapEncoder(kZeroEncoder[i] + angleRad * ChassisKinematicsCfg::RAD_TO_ENCODER);
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
