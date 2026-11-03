/**
 * @file robot_config.h
 * @brief 底盘机械与控制配置
 */
#pragma once

#include <math.h>
#include <stdint.h>

#include "chassis_kinematics.h"
#include "dji_6020.h"
#include "pid.h"

/**
 * @brief 本车参数，换车、换电机、改零点、改手感只改这个文件
 *        轮数、零点和电机 ID 不要在其它模块再声明一份
 *
 * 轮序 [0] 前轮、[1] 左轮、[2] 右轮
 * 车体坐标：+x 向前，+y 向左，+wz 俯视逆时针
 */
namespace RobotConfig
{
    constexpr uint32_t WHEEL_COUNT = ChassisGeometry::WHEEL_COUNT;
    constexpr uint32_t CONTROL_PERIOD_MS = 1U;
    constexpr float MAX_CONTROL_DT_S = 0.05f;
    constexpr float PID_DT_MIN_MS = 0.2f;
    constexpr float PID_DT_MAX_MS = 5.0f;

    /* ---- CAN ---- */
    constexpr uint16_t VESC_NODE_ID[WHEEL_COUNT] = {90U, 69U, 97U};
    constexpr uint32_t DJI_MOTOR_ID[WHEEL_COUNT] = {1U, 4U, 3U};
    constexpr uint32_t VESC_POLE_PAIRS = 14U;  ///< 24N28P；eRPM = RPM x 极对数

    static_assert(VESC_POLE_PAIRS > 0U, "Invalid VESC pole pairs");
    static_assert(DJI_MOTOR_ID[0] >= Dji6020Cfg::MOTOR_ID_MIN &&
                      DJI_MOTOR_ID[0] <= Dji6020Cfg::MOTOR_ID_MAX &&
                      DJI_MOTOR_ID[1] >= Dji6020Cfg::MOTOR_ID_MIN &&
                      DJI_MOTOR_ID[1] <= Dji6020Cfg::MOTOR_ID_MAX &&
                      DJI_MOTOR_ID[2] >= Dji6020Cfg::MOTOR_ID_MIN &&
                      DJI_MOTOR_ID[2] <= Dji6020Cfg::MOTOR_ID_MAX,
                  "Invalid DJI motor ID");

    /* ---- 机械 ---- */
    constexpr float WHEEL_DIAMETER_M = 0.120f;    ///< 轮胎直径 (m)
    constexpr float GEAR_RATIO = 1.0f;            ///< 轮向减速比，直驱为 1
    constexpr float CHASSIS_RADIUS_M = 0.36667f;  ///< 等边三角形中心到轮心 (m)
    constexpr ChassisGeometry GEOMETRY = {
        {CHASSIS_RADIUS_M, -0.5f * CHASSIS_RADIUS_M, -0.5f * CHASSIS_RADIUS_M},                   // wheelX
        {0.0f, 0.8660254037844386f * CHASSIS_RADIUS_M, -0.8660254037844386f * CHASSIS_RADIUS_M},  // wheelY
        {6922.0f, 7180.0f, 5077.0f},  // zeroEncoder，前、左、右
        WHEEL_DIAMETER_M,
        GEAR_RATIO,
        Dji6020Cfg::ENCODER_MAX};
    static_assert(GEOMETRY.wheelDiameter > 0.0f && GEOMETRY.gearRatio > 0.0f && GEOMETRY.encoderRange > 0.0f,
                  "Invalid chassis geometry");

    /* ---- 遥控手感 ---- */
    constexpr float REMOTE_CH_MAX = 660.0f;         ///< 摇杆满量程，中位已减到 0
    constexpr float REMOTE_DEADBAND = 20.0f;        ///< 摇杆死区
    constexpr float MAX_VX = 1.5f;                  ///< 最大前进速度 (m/s)
    constexpr float MAX_VY = 1.5f;                  ///< 最大平移速度 (m/s)
    constexpr float MAX_WZ_DEG_PER_SEC = 85.9437f;  ///< 最大旋转速度 (°/s)，内部转换为 rad/s
    constexpr float REMOTE_VX_SIGN = 1.0f;          ///< 左纵轴上推 x 此符号 = vx
    constexpr float REMOTE_VY_SIGN = 1.0f;          ///< 左横轴右推 x 此符号 = vy
    constexpr float REMOTE_WZ_SIGN = -1.0f;         ///< 右横轴右推 x 此符号 = wz

    constexpr PidParam MakeSteerAngle()
    {
        PidParam p = {};
        p.maxOut = 8192.0f;
        p.integralLimit = 3000.0f;
        p.deadband = 0.0f;
        p.kp = 4.0f;
        p.ki = 0.0f;
        p.kd = 0.8f;
        p.improve = PID_IMPROVE_INTEGRAL_LIMIT;
        return p;
    }

    constexpr PidParam MakeSteerSpeed()
    {
        PidParam p = {};
        p.maxOut = 25000.0f;
        p.integralLimit = 4000.0f;
        p.deadband = 0.0f;
        p.kp = 4.0f;
        p.ki = 0.005f;
        p.kd = 0.2f;
        p.improve = PID_IMPROVE_INTEGRAL_LIMIT;
        return p;
    }

    constexpr PidParam STEER_ANGLE = MakeSteerAngle();
    constexpr PidParam STEER_SPEED = MakeSteerSpeed();
    static_assert(PidParamValid(STEER_ANGLE, STEER_ANGLE.maxOut), "Invalid steer angle PID");
    static_assert(PidParamValid(STEER_SPEED, Dji6020Cfg::RAW_MAX), "Invalid steer speed PID");
}  // namespace RobotConfig
