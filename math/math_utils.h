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

    /** @brief 把周期值收到 [0, period)。 */
    inline float WrapPeriodic(float value, float period)
    {
        if (period <= 0.0f)
        {
            return value;
        }

        value = fmodf(value, period);
        if (value < 0.0f)
        {
            value += period;
        }
        return value;
    }

    /** @brief 把周期差值收到 (-period/2, period/2]，取较短的一侧。 */
    inline float WrapPeriodicError(float err, float period)
    {
        if (period <= 0.0f)
        {
            return err;
        }

        err = fmodf(err, period);
        const float half = 0.5f * period;
        if (err > half)
        {
            err -= period;
        }
        else if (err < -half)
        {
            err += period;
        }
        return err;
    }
}
