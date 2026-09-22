#pragma once

#include <stdint.h>

/**
 * @brief 本车参数。换车、换电机、改零点、改手感只改这个文件。
 *
 * 轮序 [0] 前轮、[1] 左轮、[2] 右轮。
 * 车体坐标：+x 向前，+y 向左，+wz 俯视逆时针。
 */
namespace RobotConfig
{
    const uint32_t WHEEL_COUNT = 3U;

    /* ---- CAN ---- */
    const uint16_t VESC_NODE_ID[WHEEL_COUNT] = {90U, 69U, 97U};
    const uint32_t DJI_MOTOR_ID[WHEEL_COUNT] = {1U, 4U, 3U};
    const int VESC_POLE_PAIRS = 14;              ///< 24N28P，eRPM = RPM x 极对数。

    /* ---- 机械 ---- */
    const float WHEEL_DIAMETER_M = 0.120f;       ///< 轮胎直径 (m)。
    const float GEAR_RATIO = 1.0f;               ///< 轮向减速比，直驱为 1。
    const float CHASSIS_RADIUS_M = 0.36667f;     ///< 中心到轮心 (m)。
    const float DJI_ENCODER_MAX = 8192.0f;       ///< 6020 单圈编码器。

    const float FRONT_ZERO_ENCODER = 6922.0f;    ///< 前轮朝前时的编码器。
    const float LEFT_ZERO_ENCODER = 7180.0f;     ///< 左轮朝前时的编码器。
    const float RIGHT_ZERO_ENCODER = 5077.0f;    ///< 右轮朝前时的编码器。

    /* 左右轮 y 的符号按实车自转标定，+1 为 +y（左）。 */
    const float LEFT_WHEEL_Y_SIGN = 1.0f;
    const float RIGHT_WHEEL_Y_SIGN = -1.0f;

    /* ---- 遥控手感 ---- */
    const float REMOTE_CH_MAX = 660.0f;          ///< 摇杆满量程，中位已减到 0。
    const float REMOTE_DEADBAND = 20.0f;         ///< 摇杆死区。
    const float MAX_VX = 1.5f;                   ///< 最大前进速度 (m/s)。
    const float MAX_VY = 1.5f;                   ///< 最大平移速度 (m/s)。
    const float MAX_WZ = 1.5f;                   ///< 最大旋转速度 (rad/s)。
    const float REMOTE_VX_SIGN = 1.0f;           ///< 左纵轴上推 x 此符号 = vx。
    const float REMOTE_VY_SIGN = 1.0f;           ///< 左横轴右推 x 此符号 = vy。
    const float REMOTE_WZ_SIGN = -1.0f;          ///< 右横轴右推 x 此符号 = wz。

    /* ---- 舵向角度环 ---- */
    const float STEER_ANGLE_KP = 4.0f;
    const float STEER_ANGLE_KI = 0.0f;
    const float STEER_ANGLE_KD = 0.8f;
    const float STEER_ANGLE_I_LIMIT = 3000.0f;
    const float STEER_ANGLE_MAX = 8192.0f;
    const float STEER_ANGLE_DEADBAND = 0.0f;

    /* ---- 舵向速度环，输出为 6020 电流原始值 ---- */
    const float STEER_SPEED_KP = 4.0f;
    const float STEER_SPEED_KI = 0.005f;
    const float STEER_SPEED_KD = 0.2f;
    const float STEER_SPEED_I_LIMIT = 4000.0f;
    const float STEER_SPEED_MAX = 25000.0f;
    const float STEER_SPEED_DEADBAND = 0.0f;
}
