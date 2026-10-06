#pragma once

#include <stdint.h>

#include "chassis_kinematics.h"
#include "chassis_odometry.h"
#include "trapezoidal_profile.h"

/**
 * @brief 单点运动目标，使用里程计坐标系。
 * @note x/y 单位 m，yaw 单位 rad；yaw 是指定目标航向。
 */
struct AutoMoveTarget
{
    float x;
    float y;
    float yaw;
};

/** @brief 单点运动参数，增益和到达阈值由实车调试确定。 */
struct AutoMoveParam
{
    float maxVelocity;                 ///< 梯形规划最大速度 (m/s)。
    float acceleration;                ///< 规划加速度及输出加速度限制 (m/s^2)。
    float deceleration;                ///< 规划减速度及输出减速度限制 (m/s^2)。

    float alongKp;                     ///< 沿路径位置 P 增益 (1/s)，允许为零。
    float crossKp;                     ///< 横向位置 P 增益 (1/s)，允许为零。
    float yawKp;                       ///< 航向 P 增益 (1/s)，允许为零。

    float maxCorrectionVelocity;       ///< 沿路径和横向反馈合成速度上限 (m/s)。
    float maxOutputVelocity;           ///< 前馈和反馈合成速度上限 (m/s)。
    float maxYawVelocity;              ///< 输出角速度绝对值上限 (rad/s)。
    float yawAcceleration;             ///< 输出角速度变化率上限 (rad/s^2)。

    float positionTolerance;           ///< 终点位置误差阈值 (m)。
    float yawTolerance;                ///< 终点航向误差阈值 (rad)。
    float speedTolerance;              ///< 到达时实测平移速度阈值 (m/s)。
    float yawSpeedTolerance;           ///< 到达时实测角速度阈值 (rad/s)。
    float settleDuration;              ///< 连续满足到达条件的时间 (s)。
};

/** @brief 单点运动的当前阶段。 */
enum class AutoMovePhase : uint8_t
{
    Idle,       ///< 待机：尚未启动或已复位，保持零输出，等待 Start()。
    Tracking,   ///< 轨迹跟踪：推进梯形参考，同时修正沿路径、横向和航向误差。
    Settling,   ///< 终点修正：参考位置固定为目标点，继续修正误差并等待停稳。
    Finished,   ///< 已到达：位置、航向、实测速度和零输出连续满足条件，保持零输出。
    Fault       ///< 故障：反馈、参数或计算异常，清零输出，需 Reset() 或重新 Start() 恢复。
};

/** @brief 控制状态和参考输出，供调试镜像观察。 */
struct AutoMoveState
{
    AutoMovePhase phase = AutoMovePhase::Idle;
    ChassisPose2D startPose = {};
    AutoMoveTarget target = {};
    ChassisPose2D referencePose = {};

    float referenceDistance = 0.0f;    ///< 规划要求沿路径走过的距离 (m)，不是实际位移。
    float referenceSpeed = 0.0f;       ///< 规划要求沿路径运动的速度 (m/s)。
    float planningTime = 0.0f;         ///< 梯形规划已运行时间 (s)。
    TrapezoidalPhase planningPhase = TrapezoidalPhase::Idle; ///< 梯形规划阶段，与运动阶段区分。

    float alongError = 0.0f;           ///< 当前参考位置的沿路径误差 (m)。
    float crossError = 0.0f;           ///< 当前参考位置的横向误差 (m)。
    float yawError = 0.0f;             ///< 目标减实际的最短航向误差 (rad)。
    float positionError = 0.0f;        ///< 距离最终目标点的距离 (m)。
    float alongCorrection = 0.0f;      ///< 限幅后的沿路径反馈速度 (m/s)。
    float crossCorrection = 0.0f;      ///< 限幅后的横向反馈速度 (m/s)。
    float worldVx = 0.0f;              ///< 限幅、限加速度后的世界 x 速度 (m/s)。
    float worldVy = 0.0f;              ///< 限幅、限加速度后的世界 y 速度 (m/s)。
    float settledTime = 0.0f;          ///< 连续满足到达条件的时间 (s)。
    ChassisBodyVelocity velocity = {}; ///< 可直接交给底盘的车体目标速度。
};

/**
 * @brief 梯形前馈、沿路径 P、横向 P 和航向 P 组成的单点运动控制器。
 *
 * 目标位置和反馈位姿必须使用同一个里程计坐标系。
 * 只计算车体目标速度，不读取传感器，不调用底盘，也不发送 CAN。
 */
class AutoMove
{
public:
    /** @brief 取消运动，清零状态和输出，回到 Idle。 */
    void Reset();

    /**
     * @brief 从当前位姿启动一段直线运动，每段运动只调用一次。
     * @return 位姿、目标和参数有效时返回 true；否则进入 Fault 并输出零。
     * @note 调用者须确认反馈有效且底盘已停稳；零距离直接进入 Settling。
     */
    bool Start(const ChassisPose2D &currentPose,
               const AutoMoveTarget &target, const AutoMoveParam &param);

    /**
     * @brief 按实际时间间隔更新，行进和终点修正阶段均修正航向。
     * @param currentPose 当前里程计位姿，yaw 单位 rad。
     * @param measuredVelocity 实测车体速度，单位 m/s、rad/s。
     * @param dt 本次与上次更新的实际间隔 (s)，有效范围 (0, 0.05]。
     * @param feedbackValid 里程计、轮速、舵角和 IMU 反馈均有效。
     * @note Fault 保持零输出，需 Reset() 或重新 Start() 才能恢复。
     */
    void Update(const ChassisPose2D &currentPose,
                const ChassisBodyVelocity &measuredVelocity,
                float dt, bool feedbackValid);

    const ChassisBodyVelocity &Velocity() const { return motionState_.velocity; }
    const AutoMoveState &State() const { return motionState_; }
    bool Finished() const { return motionState_.phase == AutoMovePhase::Finished; }
    bool Faulted() const { return motionState_.phase == AutoMovePhase::Fault; }

private:
    void SetFault();
    void UpdateReference();
    void UpdateOutput(float worldVx, float worldVy, float yawVelocity,
                      float currentYaw, float dt);
    void UpdateArrival(const ChassisBodyVelocity &measuredVelocity, float dt);

    TrapezoidalProfile distancePlanner_;//生成一维梯形参考的规划器
    AutoMoveParam param_ = {};
    AutoMoveState motionState_ = {};//自动运动的状态与输出
    float pathLength_ = 0.0f;
    float directionX_ = 1.0f;
    float directionY_ = 0.0f;
};
