#pragma once

#include <math.h>

/**
 * @brief 底盘和 PID 共用的小运算。只放没有硬件含义的函数。
 */
namespace MathUtils
{
    inline float ApplyDeadband(float value, float deadband)
    {
        if (fabsf(value) < deadband)
        {
            return 0.0f;
        }
        return value;
    }

    inline float ClampAbs(float value, float limit)
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

    /** @brief 把误差收到 (-range/2, range/2]，取较短的一侧。 */
    inline float WrapEncoderError(float err, float range)
    {
        if (range <= 0.0f)
        {
            return err;
        }

        err = fmodf(err, range);
        const float half = 0.5f * range;
        if (err > half)
        {
            err -= range;
        }
        else if (err < -half)
        {
            err += range;
        }
        return err;
    }
}
