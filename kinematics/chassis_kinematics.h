#pragma once

#include <stdint.h>

#include "robot_config.h"

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
 * 三轮按等边三角形布置，前轮在 +x 轴上。本层只做正逆解，
 * 不读遥控器，不调用 PID，也不发送 CAN。
 */
namespace ChassisKinematicsCfg
{
    const uint32_t WHEEL_COUNT = RobotConfig::WHEEL_COUNT;

    const float PI = 3.14159265358979323846f;
    const float WHEEL_DIAMETER_M = RobotConfig::WHEEL_DIAMETER_M;
    const float WHEEL_RADIUS_M = 0.5f * WHEEL_DIAMETER_M;
    const float GEAR_RATIO = RobotConfig::GEAR_RATIO;
    const float CHASSIS_RADIUS_M = RobotConfig::CHASSIS_RADIUS_M;

    const float ENCODER_MAX = RobotConfig::DJI_ENCODER_MAX;
    const float HALF_ENCODER = 0.5f * ENCODER_MAX;
    const float QUARTER_ENCODER = 0.25f * ENCODER_MAX;
    const float RAD_TO_ENCODER = ENCODER_MAX / (2.0f * PI);

    const float FRONT_ZERO_ENCODER = RobotConfig::FRONT_ZERO_ENCODER;
    const float LEFT_ZERO_ENCODER = RobotConfig::LEFT_ZERO_ENCODER;
    const float RIGHT_ZERO_ENCODER = RobotConfig::RIGHT_ZERO_ENCODER;
    const float LEFT_WHEEL_Y_SIGN = RobotConfig::LEFT_WHEEL_Y_SIGN;
    const float RIGHT_WHEEL_Y_SIGN = RobotConfig::RIGHT_WHEEL_Y_SIGN;

    const float STOP_SPEED_MPS = 1.0e-4f;
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
    /** @brief 把编码器收到 [0, ENCODER_MAX)，过零后仍是合法单圈值。 */
    static float wrapEncoder(float value);

    /** @brief 把误差收到 (-HALF_ENCODER, HALF_ENCODER]，取较短的那一侧。 */
    static float wrapEncoderError(float value);

    ChassisBodyVelocity bodyVelocity_;
    ChassisWheelCommand command_;
};

extern ChassisKinematics g_chassisKinematics;
