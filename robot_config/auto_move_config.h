/**
 * @file auto_move_config.h
 * @brief 自动运动默认值和调参输出上限
 */
#pragma once

#include "auto_move_params.h"

namespace AutoMoveConfig
{
    constexpr float MAX_TUNE_OUTPUT_VELOCITY = 5.0f;  // m/s

    constexpr AutoMoveTune MakeOdometryDefault()
    {
        AutoMoveTune p = {};
        p.maxVelocity = 1.0f;
        p.acceleration = 0.6f;
        p.deceleration = 0.4f;
        p.trackingAlongKp = 0.1f;
        p.trackingAlongKd = 1.2f;
        p.settlingAlongKp = 0.1f;
        p.settlingAlongKd = 1.2f;
        p.crossKp = 1.0f;
        p.yawKp = 2.2f;
        p.maxCorrectionVelocity = 1.0f;
        p.maxOutputVelocity = 2.0f;
        p.maxYawVelocityDegPerSec = 60.0f;
        p.yawAccelerationDegPerSec2 = 60.0f;
        p.positionTolerance = 0.01f;
        p.yawToleranceDeg = 1.0f;
        p.speedTolerance = 0.02f;
        p.yawSpeedToleranceDegPerSec = 1.1459f;
        p.settleDuration = 0.2f;
        return p;
    }

    constexpr AutoMoveTune MakeLidarDefault()
    {
        AutoMoveTune p = {};
        p.maxVelocity = 4.0f;
        p.acceleration = 2.5f;
        p.deceleration = 2.0f;
        p.trackingAlongKp = 0.1f;
        p.trackingAlongKd = 0.0f;
        p.settlingAlongKp = 1.2f;
        p.settlingAlongKd = 0.01f;
        p.crossKp = 1.2f;
        p.yawKp = 1.8f;
        p.maxCorrectionVelocity = 2.0f;
        p.maxOutputVelocity = 5.0f;
        p.maxYawVelocityDegPerSec = 150.0f;
        p.yawAccelerationDegPerSec2 = 130.0f;
        p.positionTolerance = 0.02f;
        p.yawToleranceDeg = 2.0f;
        p.speedTolerance = 0.02f;
        p.yawSpeedToleranceDegPerSec = 1.1459f;
        p.settleDuration = 0.2f;
        return p;
    }

    constexpr AutoMoveTune ODOMETRY_DEFAULT = MakeOdometryDefault();
    constexpr AutoMoveTune LIDAR_DEFAULT = MakeLidarDefault();
    static_assert(AutoMoveTuneValid(ODOMETRY_DEFAULT), "Invalid odometry motion parameters");
    static_assert(AutoMoveTuneValid(LIDAR_DEFAULT), "Invalid lidar motion parameters");
    static_assert(ODOMETRY_DEFAULT.maxOutputVelocity <= MAX_TUNE_OUTPUT_VELOCITY &&
                      LIDAR_DEFAULT.maxOutputVelocity <= MAX_TUNE_OUTPUT_VELOCITY,
                  "Output velocity exceeds tuning limit");
}  // namespace AutoMoveConfig
