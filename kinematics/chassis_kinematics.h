#pragma once

#include <stdint.h>

#include "remote_input.h"

/**
 * @brief 三舵轮底盘运动学。
 *
 * 车体坐标与 Chassis2026_R1_H723 一致：
 *   +x 车头向前
 *   +y 车体向左
 *   +wz 俯视逆时针
 *
 * 轮序与当前电机对象一致：
 *   [0] 前轮  6020 ID1 / VESC 90
 *   [1] 左轮  6020 ID4 / VESC 69
 *   [2] 右轮  6020 ID3 / VESC 97
 *
 * 三轮按等边三角形布置，前轮在 +x 轴上。本层只做速度换算，
 * 不调用 PID，也不发送 CAN。
 */
namespace ChassisKinematicsCfg
{
    const uint32_t WHEEL_COUNT = 3U;                 ///< 舵轮数量：前、左、右。

    const float PI = 3.14159265358979323846f;        ///< 圆周率，角度与弧度换算用。
    const float WHEEL_DIAMETER_M = 0.120f;           ///< 轮胎直径 (m)，来自参考底盘。
    const float WHEEL_RADIUS_M = 0.5f * WHEEL_DIAMETER_M; ///< 轮胎半径 (m) = 直径 / 2。
    const float GEAR_RATIO = 1.0f;                   ///< 轮向电机减速比，当前直驱为 1。
    const float CHASSIS_RADIUS_M = 0.36667f;         ///< 底盘中心到轮心的距离 (m)。

    const float ENCODER_MAX = 8192.0f;               ///< 6020 单圈编码器满量程。
    const float HALF_ENCODER = 0.5f * ENCODER_MAX;   ///< 半圈，180°，用于舵向对侧取反。
    const float QUARTER_ENCODER = 0.25f * ENCODER_MAX; ///< 1/4 圈，90°，超过则改走对侧。
    const float RAD_TO_ENCODER = ENCODER_MAX / (2.0f * PI); ///< 弧度换算成编码器刻度。

    const float FRONT_ZERO_ENCODER = 6922.0f;        ///< 前轮舵向机械零点。
    const float LEFT_ZERO_ENCODER  = 7180.0f;        ///< 左轮舵向机械零点。
    const float RIGHT_ZERO_ENCODER = 5077.0f;        ///< 右轮舵向机械零点。

    const float REMOTE_CH_MAX = 660.0f;              ///< 大疆摇杆满量程，中位已减到 0。
    const float REMOTE_DEADBAND = 20.0f;             ///< 摇杆死区，小于该值视为回中。
    const float DEFAULT_MAX_VX = 1.5f;               ///< 默认最大前进速度 (m/s)。
    const float DEFAULT_MAX_VY = 1.5f;               ///< 默认最大平移速度 (m/s)。
    const float DEFAULT_MAX_WZ = 1.5f;               ///< 默认最大旋转速度 (rad/s)。
    const float STOP_SPEED_MPS = 1.0e-4f;            ///< 小于该轮速时保持当前舵角、轮速置 0。
}

/** @brief 车体目标速度。 */
struct ChassisBodyVelocity
{
    float vx;   ///< 前进速度 (m/s)，向前为正
    float vy;   ///< 平移速度 (m/s)，向左为正
    float wz;   ///< 旋转速度 (rad/s)，逆时针为正
};

/** @brief 三路舵轮逆解结果。 */
struct ChassisWheelCommand
{
    float steerEncoder[ChassisKinematicsCfg::WHEEL_COUNT]; ///< 舵向目标编码器，0~8192。
    float wheelRpm[ChassisKinematicsCfg::WHEEL_COUNT];     ///< 轮向目标转速 (RPM)。
};

class ChassisKinematics
{
public:
    ChassisKinematics();

    /**
     * @brief 把遥控器摇杆映射为车体目标速度。
     * @note 左纵轴前进，左横轴左推为车体向左，右横轴控制旋转。
     *       遥控器离线时输出全 0。
     */
    ChassisBodyVelocity mapRemote(const RemoteState &remote) const;

    /**
     * @brief 由车体速度和当前舵向编码器解出三路目标。
     * @param vel 车体目标速度。
     * @param encoder 当前三路 6020 编码器，下标 0/1/2 对应前/左/右。
     */
    ChassisWheelCommand inverse(const ChassisBodyVelocity &vel,
                                const int16_t encoder[ChassisKinematicsCfg::WHEEL_COUNT]);

    const ChassisBodyVelocity &bodyVelocity() const
    {
        return bodyVelocity_;
    }

    const ChassisWheelCommand &command() const
    {
        return command_;
    }

private:
    static float wrapEncoder(float value);
    static float wrapEncoderError(float value);
    static float applyDeadband(float value, float deadband);
    static float clampAbs(float value, float limit);

    ChassisBodyVelocity bodyVelocity_;
    ChassisWheelCommand command_;
};

extern ChassisKinematics g_chassisKinematics;
