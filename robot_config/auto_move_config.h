#pragma once

#include "auto_move.h"
#include "math_utils.h"

/** @brief 两套独立的跑点参数；角度输入使用度，交给控制器时转换为 rad。 */
namespace AutoMoveConfig
{
    /* 已调好的里程计参数，数值从整改前 robot_config.h 原样保留。 */
    namespace Odometry
    {
        constexpr float MAX_VELOCITY = 1.0f;                           ///< 梯形规划最大速度 (m/s)
        constexpr float ACCELERATION = 0.6f;                           ///< 加速度限制 (m/s^2)
        constexpr float DECELERATION = 0.4f;                           ///< 减速度限制 (m/s^2)
        constexpr float ALONG_KP = 0.8f;                               ///< 沿路径修正增益 (1/s)
        constexpr float ALONG_KD = 1.2f;                               ///< 沿路径速度误差反馈增益 (无量纲)，零时关闭
        constexpr float CROSS_KP = 1.0f;                               ///< 横向修正增益 (1/s)
        constexpr float YAW_KP = 2.2f;                                 ///< 航向修正增益
        constexpr float MAX_CORRECTION_VELOCITY = 1.0f;                ///< 反馈合成速度上限 (m/s)。
        constexpr float MAX_OUTPUT_VELOCITY = 2.0f;                    ///< 前馈和反馈合成速度上限 (m/s)。
        constexpr float MAX_YAW_VELOCITY_DEG_PER_SEC = 60.0f;          ///< 最大角速度 (°/s)，内部转换为 rad/s
        constexpr float YAW_ACCELERATION_DEG_PER_SEC2 = 60.0f;         ///< 角速度变化率限制 (°/s^2)，内部转换为 rad/s^2
        constexpr float POSITION_TOLERANCE = 0.01f;                    ///< 到达位置阈值 (m)
        constexpr float YAW_TOLERANCE_DEG = 1.0f;                      ///< 到达航向阈值 (°)，内部转换为 rad
        constexpr float SPEED_TOLERANCE = 0.02f;                       ///< 到达及启动时平移速度阈值 (m/s)
        constexpr float YAW_SPEED_TOLERANCE_DEG_PER_SEC = 1.1459f;     ///< 到达及启动时角速度阈值 (°/s)
        constexpr float SETTLE_DURATION = 0.2f;                        ///< 到达条件持续时间 (s)
    }

    /* 雷达参数独立存放，以下为待实车调试的初值，不引用里程计参数。 */
    namespace Lidar
    {
        constexpr float MAX_VELOCITY = 0.3f;                           ///< 梯形规划最大速度 (m/s)
        constexpr float ACCELERATION = 0.3f;                           ///< 加速度限制 (m/s^2)
        constexpr float DECELERATION = 0.3f;                           ///< 减速度限制 (m/s^2)
        constexpr float ALONG_KP = 0.8f;                               ///< 沿路径修正增益 (1/s)
        constexpr float ALONG_KD = 0.1f;                               ///< 沿路径速度误差反馈增益 (无量纲)，零时关闭
        constexpr float CROSS_KP = 1.0f;                               ///< 横向修正增益 (1/s)
        constexpr float YAW_KP = 2.2f;                                 ///< 航向修正增益
        constexpr float MAX_CORRECTION_VELOCITY = 0.3f;                ///< 反馈合成速度上限 (m/s)。
        constexpr float MAX_OUTPUT_VELOCITY = 0.6f;                    ///< 前馈和反馈合成速度上限 (m/s)。
        constexpr float MAX_YAW_VELOCITY_DEG_PER_SEC = 30.0f;          ///< 最大角速度 (°/s)，内部转换为 rad/s
        constexpr float YAW_ACCELERATION_DEG_PER_SEC2 = 60.0f;         ///< 角速度变化率限制 (°/s^2)，内部转换为 rad/s^2
        constexpr float POSITION_TOLERANCE = 0.02f;                    ///< 到达位置阈值 (m)
        constexpr float YAW_TOLERANCE_DEG = 1.0f;                      ///< 到达航向阈值 (°)，内部转换为 rad
        constexpr float SPEED_TOLERANCE = 0.02f;                       ///< 到达及启动时平移速度阈值 (m/s)
        constexpr float YAW_SPEED_TOLERANCE_DEG_PER_SEC = 1.1459f;     ///< 到达及启动时角速度阈值 (°/s)
        constexpr float SETTLE_DURATION = 0.2f;                        ///< 到达条件持续时间 (s)
    }

    constexpr AutoMoveParam odometryAutoMoveParam =
    {
        Odometry::MAX_VELOCITY,
        Odometry::ACCELERATION,
        Odometry::DECELERATION,
        Odometry::ALONG_KP,
        Odometry::ALONG_KD,
        Odometry::CROSS_KP,
        Odometry::YAW_KP,
        Odometry::MAX_CORRECTION_VELOCITY,
        Odometry::MAX_OUTPUT_VELOCITY,
        MathUtils::DegToRad(Odometry::MAX_YAW_VELOCITY_DEG_PER_SEC),
        MathUtils::DegToRad(Odometry::YAW_ACCELERATION_DEG_PER_SEC2),
        Odometry::POSITION_TOLERANCE,
        MathUtils::DegToRad(Odometry::YAW_TOLERANCE_DEG),
        Odometry::SPEED_TOLERANCE,
        MathUtils::DegToRad(Odometry::YAW_SPEED_TOLERANCE_DEG_PER_SEC),
        Odometry::SETTLE_DURATION
    };

    constexpr AutoMoveParam lidarAutoMoveParam =
    {
        Lidar::MAX_VELOCITY,
        Lidar::ACCELERATION,
        Lidar::DECELERATION,
        Lidar::ALONG_KP,
        Lidar::ALONG_KD,
        Lidar::CROSS_KP,
        Lidar::YAW_KP,
        Lidar::MAX_CORRECTION_VELOCITY,
        Lidar::MAX_OUTPUT_VELOCITY,
        MathUtils::DegToRad(Lidar::MAX_YAW_VELOCITY_DEG_PER_SEC),
        MathUtils::DegToRad(Lidar::YAW_ACCELERATION_DEG_PER_SEC2),
        Lidar::POSITION_TOLERANCE,
        MathUtils::DegToRad(Lidar::YAW_TOLERANCE_DEG),
        Lidar::SPEED_TOLERANCE,
        MathUtils::DegToRad(Lidar::YAW_SPEED_TOLERANCE_DEG_PER_SEC),
        Lidar::SETTLE_DURATION
    };

    /** @brief 来源与参数成套选择，避免雷达位姿误用里程计参数。 */
    inline AutoMoveParam ForSource(PoseSource source)
    {
        return source == PoseSource::Lidar ? lidarAutoMoveParam : odometryAutoMoveParam;
    }
}
