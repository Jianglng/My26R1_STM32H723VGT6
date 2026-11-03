/**
 * @file auto_move_params.h
 * @brief 自动运动参数的唯一定义和配置边界校验，角度使用度
 */
#pragma once

#include "math_utils.h"

struct AutoMoveTune
{
    float maxVelocity;                 ///< 梯形规划最大速度 (m/s)
    float acceleration;                ///< 加速度限制 (m/s^2)
    float deceleration;                ///< 减速度限制 (m/s^2)
    float trackingAlongKp;             ///< Tracking 阶段沿路径位置修正增益 (1/s)
    float trackingAlongKd;             ///< Tracking 阶段沿路径速度误差反馈增益
    float settlingAlongKp;             ///< Settling 阶段沿路径位置修正增益 (1/s)
    float settlingAlongKd;             ///< Settling 阶段沿路径速度误差反馈增益
    float crossKp;                     ///< 横向修正增益 (1/s)
    float yawKp;                       ///< 航向修正增益
    float maxCorrectionVelocity;       ///< 反馈合成速度上限 (m/s)
    float maxOutputVelocity;           ///< 前馈和反馈合成速度上限 (m/s)
    float maxYawVelocityDegPerSec;     ///< 最大角速度 (°/s)
    float yawAccelerationDegPerSec2;   ///< 角速度变化率限制 (°/s^2)
    float positionTolerance;           ///< 到达位置阈值 (m)
    float yawToleranceDeg;             ///< 到达航向阈值 (°)
    float speedTolerance;              ///< 到达及启动时平移速度阈值 (m/s)
    float yawSpeedToleranceDegPerSec;  ///< 到达及启动时角速度阈值 (°/s)
    float settleDuration;              ///< 到达条件持续时间 (s)
};

/** @brief 默认值在编译期校验，Watch 参数只在应用边界调用 */

constexpr bool AutoMoveTuneValid(const AutoMoveTune& p)
{
    return p.maxVelocity > 0.0f && MathUtils::Finite(p.maxVelocity) && p.acceleration > 0.0f &&
           MathUtils::Finite(p.acceleration) && p.deceleration > 0.0f && MathUtils::Finite(p.deceleration) &&
           p.trackingAlongKp >= 0.0f && MathUtils::Finite(p.trackingAlongKp) && p.trackingAlongKd >= 0.0f &&
           MathUtils::Finite(p.trackingAlongKd) && p.settlingAlongKp >= 0.0f && MathUtils::Finite(p.settlingAlongKp) &&
           p.settlingAlongKd >= 0.0f && MathUtils::Finite(p.settlingAlongKd) && p.crossKp >= 0.0f &&
           MathUtils::Finite(p.crossKp) && p.yawKp >= 0.0f && MathUtils::Finite(p.yawKp) &&
           p.maxCorrectionVelocity > 0.0f && MathUtils::Finite(p.maxCorrectionVelocity) &&
           p.maxOutputVelocity >= p.maxVelocity && MathUtils::Finite(p.maxOutputVelocity) &&
           p.maxYawVelocityDegPerSec > 0.0f && MathUtils::Finite(p.maxYawVelocityDegPerSec) &&
           p.yawAccelerationDegPerSec2 > 0.0f && MathUtils::Finite(p.yawAccelerationDegPerSec2) &&
           p.positionTolerance > 0.0f && MathUtils::Finite(p.positionTolerance) && p.yawToleranceDeg > 0.0f &&
           p.yawToleranceDeg <= 180.0f && p.speedTolerance > 0.0f && MathUtils::Finite(p.speedTolerance) &&
           p.yawSpeedToleranceDegPerSec > 0.0f && MathUtils::Finite(p.yawSpeedToleranceDegPerSec) &&
           p.settleDuration > 0.0f && MathUtils::Finite(p.settleDuration);
}
