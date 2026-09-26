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
