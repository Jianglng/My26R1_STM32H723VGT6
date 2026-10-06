#pragma once

#include "bsp_dwt.h"
#include "chassis_kinematics.h"
#include "dji_6020.h"
#include "pid.h"
#include "vesc_motor.h"

/**
 * @brief 三舵轮底盘。
 *
 * 持有舵向电机、轮向电机、串级 PID 和运动学。
 * 不解析遥控器。任务读完摇杆后调用 SetVelocity()，离线时调用 Stop()。
 */
class Chassis
{
public:
    Chassis() = default;
    Chassis(const Chassis &) = delete;
    Chassis &operator=(const Chassis &) = delete;

    /** @brief 打开计时器，初始化 PID、电机和 CAN，然后输出停机。 */
    void Init();

    /** @brief 写入车体目标速度，并允许下一拍 Update() 跑控制。 */
    void SetVelocity(const ChassisBodyVelocity &velocity);

    /** @brief 遥控器离线或急停。下一拍 Update() 清积分并输出 0。 */
    void Stop();

    /** @brief 实际舵角和轮速正解出的车体速度；反馈无效时为零。 */
    const ChassisBodyVelocity &MeasuredVelocity() const { return measuredVelocity_; }
    bool HasMeasuredVelocity() const { return measuredVelocityValid_; }

    /** @brief 与 PID 共用已初始化的 DWT 计时器。 */
    DwtTimer &Timer() { return dwt_; }

    /**
     * @brief 复制一份仅供调试观察的电机反馈。
     * @note 不参与控制；底层 getter 自己负责与 CAN 接收中断隔离。
     */
    void CopyFeedbackForDebug(
        Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT],
        VescRxData wheelFeedback[RobotConfig::WHEEL_COUNT]) const;

    /**
     * @brief 跑一拍：读取反馈并正解速度，再逆解目标、控制舵向和轮速。
     * Stop() 之后只发零电流和零转速，不含延时。
     */
    void Update();

private:
    void StopMotors();
    void RunSteerPid(const ChassisWheelCommand &command,
                     const Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT]);
    void RunWheelRpm(const ChassisWheelCommand &command,
                     const Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT]);

    DwtTimer dwt_;
    ChassisKinematics kinematics_;
    Pid steerAnglePid_[RobotConfig::WHEEL_COUNT];
    Pid steerSpeedPid_[RobotConfig::WHEEL_COUNT];
    Dji6020Motor steerMotors_[RobotConfig::WHEEL_COUNT];
    VescMotor wheelMotors_[RobotConfig::WHEEL_COUNT];
    ChassisBodyVelocity bodyVelocity_ = {};
    ChassisBodyVelocity measuredVelocity_ = {};
    bool measuredVelocityValid_ = false;
    bool enabled_ = false;
};
