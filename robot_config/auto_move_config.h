#pragma once

/**
 * @brief 一组跑点参数，字段顺序和单位与下方常量一致，角度使用度。
 * @note 自动模式复位时由控制器转换为 AutoMoveParam（rad）。
 *       运行时副本见 tune_params.h 中的 g_autoMoveTune。
 */
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

/** @brief 两套独立的跑点参数；角度输入使用度，交给控制器时转换为 rad。 */
namespace AutoMoveConfig
{
    /* 调参时允许的输出速度上限，防止调试器中误填过大的值。 */
    constexpr float MAX_TUNE_OUTPUT_VELOCITY = 5.0f;           ///< (m/s)

    /* 已调好的里程计参数，数值从整改前 robot_config.h 原样保留。 */
    namespace Odometry
    {
        constexpr float MAX_VELOCITY = 1.0f;                           ///< 梯形规划最大速度 (m/s)
        constexpr float ACCELERATION = 0.6f;                           ///< 加速度限制 (m/s^2)
        constexpr float DECELERATION = 0.4f;                           ///< 减速度限制 (m/s^2)
        constexpr float TRACKING_ALONG_KP = 0.1f;                      ///< Tracking 阶段沿路径位置修正增益 (1/s)
        constexpr float TRACKING_ALONG_KD = 1.2f;                      ///< Tracking 阶段沿路径速度误差反馈增益
        constexpr float SETTLING_ALONG_KP = 0.1f;                      ///< Settling 阶段沿路径位置修正增益 (1/s)
        constexpr float SETTLING_ALONG_KD = 1.2f;                      ///< Settling 阶段沿路径速度误差反馈增益
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
        constexpr float MAX_VELOCITY = 3.0f;                           ///< 梯形规划最大速度 (m/s)
        constexpr float ACCELERATION = 1.0f;                           ///< 加速度限制 (m/s^2)
        constexpr float DECELERATION = 0.8f;                           ///< 减速度限制 (m/s^2)
        constexpr float TRACKING_ALONG_KP = 0.1f;                      ///< Tracking 阶段沿路径位置修正增益 (1/s)
        constexpr float TRACKING_ALONG_KD = 0.0f;                      ///< Tracking 阶段沿路径速度误差反馈增益
        constexpr float SETTLING_ALONG_KP = 0.5f;                      ///< Settling 阶段沿路径位置修正增益 (1/s)
        constexpr float SETTLING_ALONG_KD = 0.2f;                      ///< Settling 阶段沿路径速度误差反馈增益
        constexpr float CROSS_KP = 1.0f;                               ///< 横向修正增益 (1/s)
        constexpr float YAW_KP = 2.2f;                                 ///< 航向修正增益
        constexpr float MAX_CORRECTION_VELOCITY = 2.0f;                ///< 反馈合成速度上限 (m/s)。
        constexpr float MAX_OUTPUT_VELOCITY = 5.0f;                    ///< 前馈和反馈合成速度上限 (m/s)。
        constexpr float MAX_YAW_VELOCITY_DEG_PER_SEC = 60.0f;          ///< 最大角速度 (°/s)，内部转换为 rad/s
        constexpr float YAW_ACCELERATION_DEG_PER_SEC2 = 60.0f;         ///< 角速度变化率限制 (°/s^2)，内部转换为 rad/s^2
        constexpr float POSITION_TOLERANCE = 0.01f;                    ///< 到达位置阈值 (m)
        constexpr float YAW_TOLERANCE_DEG = 1.0f;                      ///< 到达航向阈值 (°)，内部转换为 rad
        constexpr float SPEED_TOLERANCE = 0.02f;                       ///< 到达及启动时平移速度阈值 (m/s)
        constexpr float YAW_SPEED_TOLERANCE_DEG_PER_SEC = 1.1459f;     ///< 到达及启动时角速度阈值 (°/s)
        constexpr float SETTLE_DURATION = 0.2f;                        ///< 到达条件持续时间 (s)
    }

    /* 默认值按 AutoMoveTune 字段顺序排列，上电时复制到 g_autoMoveTune。 */
    constexpr AutoMoveTune ODOMETRY_DEFAULT =
    {
        Odometry::MAX_VELOCITY,
        Odometry::ACCELERATION,
        Odometry::DECELERATION,
        Odometry::TRACKING_ALONG_KP,
        Odometry::TRACKING_ALONG_KD,
        Odometry::SETTLING_ALONG_KP,
        Odometry::SETTLING_ALONG_KD,
        Odometry::CROSS_KP,
        Odometry::YAW_KP,
        Odometry::MAX_CORRECTION_VELOCITY,
        Odometry::MAX_OUTPUT_VELOCITY,
        Odometry::MAX_YAW_VELOCITY_DEG_PER_SEC,
        Odometry::YAW_ACCELERATION_DEG_PER_SEC2,
        Odometry::POSITION_TOLERANCE,
        Odometry::YAW_TOLERANCE_DEG,
        Odometry::SPEED_TOLERANCE,
        Odometry::YAW_SPEED_TOLERANCE_DEG_PER_SEC,
        Odometry::SETTLE_DURATION
    };

    constexpr AutoMoveTune LIDAR_DEFAULT =
    {
        Lidar::MAX_VELOCITY,
        Lidar::ACCELERATION,
        Lidar::DECELERATION,
        Lidar::TRACKING_ALONG_KP,
        Lidar::TRACKING_ALONG_KD,
        Lidar::SETTLING_ALONG_KP,
        Lidar::SETTLING_ALONG_KD,
        Lidar::CROSS_KP,
        Lidar::YAW_KP,
        Lidar::MAX_CORRECTION_VELOCITY,
        Lidar::MAX_OUTPUT_VELOCITY,
        Lidar::MAX_YAW_VELOCITY_DEG_PER_SEC,
        Lidar::YAW_ACCELERATION_DEG_PER_SEC2,
        Lidar::POSITION_TOLERANCE,
        Lidar::YAW_TOLERANCE_DEG,
        Lidar::SPEED_TOLERANCE,
        Lidar::YAW_SPEED_TOLERANCE_DEG_PER_SEC,
        Lidar::SETTLE_DURATION
    };

    static_assert(ODOMETRY_DEFAULT.maxOutputVelocity <= MAX_TUNE_OUTPUT_VELOCITY &&
                  LIDAR_DEFAULT.maxOutputVelocity <= MAX_TUNE_OUTPUT_VELOCITY,
                  "默认输出速度超过调参上限，上电即会被拒绝");
}
