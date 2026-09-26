#pragma once

#include <stdint.h>

#include "robot_config.h"

/**
 * @brief 三舵轮底盘运动学。
 *
 * 车体坐标：+x 向前，+y 向左，+wz 俯视逆时针。
 * 轮序与 RobotConfig 的电机 ID 数组一致：[0] 前轮，[1] 左轮，[2] 右轮。
 * 轮数、零点和轮心坐标只在 RobotConfig 里，本层直接引用。
 *
 * 三轮按等边三角形布置，前轮在 +x 轴上。本层只做逆解，
 * 不读遥控器，不调用 PID，也不发送 CAN。
 * 车体速度低于停止阈值时，轮速为 0，舵向保持上一拍。
 */

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
    float steerEncoder[RobotConfig::WHEEL_COUNT]; ///< 舵向目标编码器，0~8192。
    float wheelRpm[RobotConfig::WHEEL_COUNT];     ///< 轮向目标转速 (RPM)。
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
    ChassisWheelCommand Inverse(const ChassisBodyVelocity &vel,
                                const int16_t encoder[RobotConfig::WHEEL_COUNT]);

    const ChassisBodyVelocity &BodyVelocity() const
    {
        return bodyVelocity_;
    }

    const ChassisWheelCommand &Command() const
    {
        return command_;
    }

private:
    /** @brief 把编码器收到 [0, ENCODER_MAX)。 */
    static float WrapEncoder(float value);

    ChassisBodyVelocity bodyVelocity_;
    ChassisWheelCommand command_;
};
