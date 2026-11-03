/**
 * @file chassis_kinematics.h
 * @brief 三舵轮正逆解，车体 +x 向前、+y 向左、逆时针为正
 */
#pragma once

#include <stdint.h>

#include "motion_types.h"

/** @brief 三轮对称布置，轮心坐标和为零，几何数据由配置保证有效 */
struct ChassisGeometry
{
    static constexpr uint32_t WHEEL_COUNT = 3U;
    float wheelX[WHEEL_COUNT];  // m
    float wheelY[WHEEL_COUNT];  // m
    float zeroEncoder[WHEEL_COUNT];
    float wheelDiameter;  // m
    float gearRatio;
    float encoderRange;
};

struct ChassisWheelCommand
{
    float steerEncoder[ChassisGeometry::WHEEL_COUNT];
    float wheelRpm[ChassisGeometry::WHEEL_COUNT];
};

class ChassisKinematics
{
public:
    explicit ChassisKinematics(const ChassisGeometry& geometry);
    /** @brief 已验证的车体速度和实际编码器，静止时保留上一拍舵向 */
    const ChassisWheelCommand& Inverse(const ChassisBodyVelocity& velocity,
                                       const int16_t encoder[ChassisGeometry::WHEEL_COUNT]);
    /** @brief 在线轮速和舵角正解，假定无明显打滑 */
    ChassisBodyVelocity Forward(const int16_t encoder[ChassisGeometry::WHEEL_COUNT],
                                const float wheelRpm[ChassisGeometry::WHEEL_COUNT]) const;

private:
    const ChassisGeometry& geometry_;
    ChassisWheelCommand command_ = {};
};
