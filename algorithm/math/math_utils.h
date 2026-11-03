/**
 * @file math_utils.h
 * @brief 通用数学和单位换算
 */
#pragma once

#include <float.h>
#include <math.h>

/**
 * @brief 底盘和 PID 共用的小运算；只放没有硬件含义的函数
 */
namespace MathUtils
{
    /** @brief 同时供 constexpr 配置和运行时调参使用，比较可排除 NaN 和 Inf */
    constexpr bool Finite(float value)
    {
        return value >= -FLT_MAX && value <= FLT_MAX;
    }

    constexpr float PI = 3.14159265358979323846f;

    /** @brief 将角度从度转换为弧度 */
    constexpr float DegToRad(float degrees)
    {
        return degrees * PI / 180.0f;
    }

    /** @brief 将角度从弧度转换为度 */
    constexpr float RadToDeg(float radians)
    {
        return radians * 180.0f / PI;
    }

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

    /** @brief 把周期值收到 [0, period) */
    inline float WrapPeriodic(float value, float period)
    {
        value = fmodf(value, period);
        if (value < 0.0f)
        {
            value += period;
        }
        return value;
    }

    /** @brief 把周期差值收到 (-period/2, period/2]，取较短的一侧 */
    inline float WrapPeriodicError(float err, float period)
    {
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
}  // namespace MathUtils
