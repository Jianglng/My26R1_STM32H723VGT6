/**
 * @file chassis.h
 * @brief 三舵轮反馈与电机控制
 */
#pragma once

#include "chassis_kinematics.h"
#include "dji_6020.h"
#include "pid.h"
#include "robot_config.h"
#include "vesc_motor.h"

/**
 * @brief 三舵轮底盘
 *
 * 持有舵向电机、轮向电机、串级 PID 和运动学
 * 不解析遥控器；任务读完摇杆后调用 SetVelocity()，离线时调用 Stop()
 */
class Chassis
{
public:
    Chassis() = default;
    Chassis(const Chassis&) = delete;
    Chassis& operator=(const Chassis&) = delete;

    /** @brief 初始化 PID、电机和 CAN，并下发零输出，失败时返回 false */
    bool Init();

    /** @brief 在底盘输出边界检查速度，有限时允许下一拍控制，否则停机并返回 false */
    bool SetVelocity(const ChassisBodyVelocity& velocity);

    /** @brief 禁止驱动，下一拍 UpdateControl() 清积分并下发零输出 */
    void Stop();

    /** @brief 实际舵角和轮速正解出的车体速度；反馈无效时为零 */
    const ChassisBodyVelocity& MeasuredVelocity() const { return measuredVelocity_; }
    bool HasMeasuredVelocity() const { return measuredVelocityValid_; }
    bool TransmitOk() const { return transmitOk_; }
    bool OutputValid() const { return outputValid_; }

    /**
     * @brief 把一组参数写入三个轮子的舵向串级 PID，并清零积分
     * @return 停机且参数有效时返回 true；运行中或参数无效时不修改并返回 false
     * @note 输出限幅不得超过 RobotConfig 中的默认上限，只允许调小
     */
    bool ApplySteerPidParam(const PidParam& angle, const PidParam& speed);

    /** @brief 读取一份电机反馈并正解实测速度，在里程计和自动控制之前调用 */
    void UpdateFeedback();

    /** @brief 用本拍反馈执行电机控制，必须在 UpdateFeedback() 之后调用 */
    void UpdateControl(float pidDtMs);

private:
    void StopMotors();
    bool RunSteerPid(const ChassisWheelCommand& command, float pidDtMs);
    void RunWheelRpm(const ChassisWheelCommand& command);

    ChassisKinematics kinematics_{RobotConfig::GEOMETRY};
    int16_t steerEncoder_[RobotConfig::WHEEL_COUNT] = {};
    Pid steerAnglePid_[RobotConfig::WHEEL_COUNT];
    Pid steerSpeedPid_[RobotConfig::WHEEL_COUNT];
    Dji6020Motor steerMotors_[RobotConfig::WHEEL_COUNT];
    VescMotor wheelMotors_[RobotConfig::WHEEL_COUNT];
    Dji6020RxData steerFeedback_[RobotConfig::WHEEL_COUNT] = {};
    ChassisBodyVelocity bodyVelocity_ = {};
    ChassisBodyVelocity measuredVelocity_ = {};
    bool measuredVelocityValid_ = false;
    bool enabled_ = false;
    bool transmitOk_ = true;
    bool outputValid_ = true;
};
