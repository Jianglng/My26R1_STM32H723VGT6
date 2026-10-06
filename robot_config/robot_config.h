#pragma once

#include <stdint.h>
#include <math.h>

#include "pid.h"

/**
 * @brief 本车参数。换车、换电机、改零点、改手感只改这个文件。轮数、零点和电机 ID 不要在其它模块再声明一份。
 *
 * 轮序 [0] 前轮、[1] 左轮、[2] 右轮。
 * 车体坐标：+x 向前，+y 向左，+wz 俯视逆时针。
 */
namespace RobotConfig
{
    constexpr uint32_t WHEEL_COUNT = 3U;

    /* ---- CAN ---- */
    constexpr uint16_t VESC_NODE_ID[WHEEL_COUNT] = {90U, 69U, 97U};
    constexpr uint32_t DJI_MOTOR_ID[WHEEL_COUNT] = {1U, 4U, 3U};
    constexpr int VESC_POLE_PAIRS = 14;              ///< 24N28P，eRPM = RPM x 极对数。

    /* ---- 机械 ---- */
    constexpr float WHEEL_DIAMETER_M = 0.120f;       ///< 轮胎直径 (m)。
    constexpr float GEAR_RATIO = 1.0f;               ///< 轮向减速比，直驱为 1。
    constexpr float CHASSIS_RADIUS_M = 0.36667f;     ///< 等边三角形中心到轮心 (m)。
    constexpr float DJI_ENCODER_MAX = 8192.0f;       ///< 6020 单圈编码器。

    constexpr float FRONT_ZERO_ENCODER = 6922.0f;    ///< 前轮朝前时的编码器。
    constexpr float LEFT_ZERO_ENCODER = 7180.0f;     ///< 左轮朝前时的编码器。
    constexpr float RIGHT_ZERO_ENCODER = 5077.0f;    ///< 右轮朝前时的编码器。

    /* 轮心坐标 (m)。车体坐标 +x 向前，+y 向左。
     * [0] 前轮，[1] 左轮，[2] 右轮。
     * 左轮 y 为正，右轮 y 为负，与车体坐标一致 */
    constexpr float WHEEL_X[WHEEL_COUNT] =
    {
        CHASSIS_RADIUS_M,
        -0.5f * CHASSIS_RADIUS_M,
        -0.5f * CHASSIS_RADIUS_M
    };
    /* sqrtf 不是编译期计算，这个数组保持 const。 */
    const float WHEEL_Y[WHEEL_COUNT] =
    {
        0.0f,
        0.5f * sqrtf(3.0f) * CHASSIS_RADIUS_M,
        -0.5f * sqrtf(3.0f) * CHASSIS_RADIUS_M
    };

    /* ---- 遥控手感 ---- */
    constexpr float REMOTE_CH_MAX = 660.0f;          ///< 摇杆满量程，中位已减到 0。
    constexpr float REMOTE_DEADBAND = 20.0f;         ///< 摇杆死区。
    constexpr float MAX_VX = 1.5f;                   ///< 最大前进速度 (m/s)。
    constexpr float MAX_VY = 1.5f;                   ///< 最大平移速度 (m/s)。
    constexpr float MAX_WZ = 1.5f;                   ///< 最大旋转速度 (rad/s)。
    constexpr float REMOTE_VX_SIGN = 1.0f;           ///< 左纵轴上推 x 此符号 = vx。
    constexpr float REMOTE_VY_SIGN = 1.0f;           ///< 左横轴右推 x 此符号 = vy。
    constexpr float REMOTE_WZ_SIGN = -1.0f;          ///< 右横轴右推 x 此符号 = wz。

    /* ---- 单点自动运动，目标使用里程计坐标系 ---- */
    constexpr float AUTO_TARGET_X = 0.0f;                  ///< 目标 x (m)，相对里程计原点。
    constexpr float AUTO_TARGET_Y = 0.0f;                  ///< 目标 y (m)，相对里程计原点。
    constexpr float AUTO_TARGET_YAW = 0.5236f;                ///< 指定目标航向 (rad)，逆时针为正。

    constexpr float AUTO_MAX_VELOCITY = 0.2f;              ///< 梯形规划最大速度 (m/s)。
    constexpr float AUTO_ACCELERATION = 0.2f;              ///< 加速度限制 (m/s^2)。
    constexpr float AUTO_DECELERATION = 0.2f;              ///< 减速度限制 (m/s^2)。
    constexpr float AUTO_ALONG_KP = 0.5f;                  ///< 沿路径修正增益 (1/s)，实车调试初值。
    constexpr float AUTO_CROSS_KP = 0.5f;                  ///< 横向修正增益 (1/s)，实车调试初值。
    constexpr float AUTO_YAW_KP = 0.5f;                    ///< 航向修正增益 (1/s)，实车调试初值。

    constexpr float AUTO_MAX_CORRECTION_VELOCITY = 0.1f;   ///< 反馈合成速度上限 (m/s)。
    constexpr float AUTO_MAX_OUTPUT_VELOCITY = 0.3f;       ///< 前馈和反馈合成速度上限 (m/s)。
    constexpr float AUTO_MAX_YAW_VELOCITY = 0.2f;          ///< 最大角速度 (rad/s)。
    constexpr float AUTO_YAW_ACCELERATION = 0.1f;          ///< 角速度变化率限制 (rad/s^2)。

    constexpr float AUTO_POSITION_TOLERANCE = 0.01f;       ///< 到达位置阈值 (m)。
    constexpr float AUTO_YAW_TOLERANCE = 0.01745f;         ///< 到达航向阈值 (rad)，约 1 度。
    constexpr float AUTO_SPEED_TOLERANCE = 0.02f;          ///< 到达及启动时平移速度阈值 (m/s)。
    constexpr float AUTO_YAW_SPEED_TOLERANCE = 0.02f;      ///< 到达及启动时角速度阈值 (rad/s)。
    constexpr float AUTO_SETTLE_DURATION = 0.2f;           ///< 到达条件持续时间 (s)。

    /* ---- 舵向角度环 ---- */
    constexpr float STEER_ANGLE_KP = 4.0f;
    constexpr float STEER_ANGLE_KI = 0.0f;
    constexpr float STEER_ANGLE_KD = 0.8f;
    constexpr float STEER_ANGLE_I_LIMIT = 3000.0f;
    constexpr float STEER_ANGLE_MAX = 8192.0f;
    constexpr float STEER_ANGLE_DEADBAND = 0.0f;

    /* ---- 舵向速度环，输出为 6020 电流原始值 ---- */
    constexpr float STEER_SPEED_KP = 4.0f;
    constexpr float STEER_SPEED_KI = 0.005f;
    constexpr float STEER_SPEED_KD = 0.2f;
    constexpr float STEER_SPEED_I_LIMIT = 4000.0f;
    constexpr float STEER_SPEED_MAX = 25000.0f;
    constexpr float STEER_SPEED_DEADBAND = 0.0f;

    /** @brief 舵向串级参数，交给 Pid::Init()。数值仍然只在上面这些常量里改。 */
    constexpr PidParam STEER_ANGLE =
    {
        STEER_ANGLE_MAX,
        STEER_ANGLE_I_LIMIT,
        STEER_ANGLE_DEADBAND,
        STEER_ANGLE_KP,
        STEER_ANGLE_KI,
        STEER_ANGLE_KD,
        PID_IMPROVE_INTEGRAL_LIMIT
    };

    constexpr PidParam STEER_SPEED =
    {
        STEER_SPEED_MAX,
        STEER_SPEED_I_LIMIT,
        STEER_SPEED_DEADBAND,
        STEER_SPEED_KP,
        STEER_SPEED_KI,
        STEER_SPEED_KD,
        PID_IMPROVE_INTEGRAL_LIMIT
    };

}
