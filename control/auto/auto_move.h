/**
 * @file auto_move.h
 * @brief 梯形前馈与沿路径 PD、横向 P、航向 P 组合的单点运动
 */
#pragma once

#include <stdint.h>

#include "auto_move_params.h"
#include "motion_types.h"
#include "trapezoidal_profile.h"

struct AutoMoveTarget
{
    float x;    // m
    float y;    // m
    float yaw;  // rad
};

enum class AutoMovePhase : uint8_t
{
    Idle,
    Tracking,
    Settling,
    Finished,
    Fault
};

/** @brief 控制器真正持久保存的运动状态，角度使用 rad */
struct AutoMoveState
{
    AutoMovePhase phase = AutoMovePhase::Idle;
    ChassisPose2D startPose = {};
    AutoMoveTarget target = {};
    float worldVx = 0.0f;      // m/s
    float worldVy = 0.0f;      // m/s
    float settledTime = 0.0f;  // s
    ChassisBodyVelocity velocity = {};
};

/** @brief 每拍计算一次的跟踪量，控制直接使用此结果，调试整块复制 */
struct AutoMoveTelemetry
{
    ChassisPose2D referencePose = {};  // m、rad
    float alongError = 0.0f;           // m
    float crossError = 0.0f;           // m
    float yawError = 0.0f;             // rad
    float positionError = 0.0f;        // m
    float alongMeasuredSpeed = 0.0f;   // m/s
    float alongSpeedError = 0.0f;      // m/s
    float activeAlongKp = 0.0f;
    float activeAlongKd = 0.0f;
    float alongPositionCorrection = 0.0f;  // m/s
    float alongSpeedCorrection = 0.0f;     // m/s
    float alongCorrection = 0.0f;          // m/s
    float crossCorrection = 0.0f;          // m/s
};

class AutoMove
{
public:
    void Reset();
    /** @brief 起点、目标和参数由边界保证有效，零距离直接进入 Settling */
    void Start(const ChassisPose2D& currentPose, const AutoMoveTarget& target, const AutoMoveTune& param);
    /** @brief dt 使用 s，由应用层验证，反馈失效进入锁存故障 */
    void Update(const ChassisPose2D& currentPose, const ChassisBodyVelocity& measuredVelocity, float dt,
                bool feedbackValid);
    void SetFault();
    const ChassisBodyVelocity& Velocity() const { return state_.velocity; }
    const AutoMoveState& State() const { return state_; }
    const AutoMoveTelemetry& Telemetry() const { return telemetry_; }
    const TrapezoidalProfileState& Reference() const { return planner_.State(); }
    bool Finished() const { return state_.phase == AutoMovePhase::Finished; }
    bool Faulted() const { return state_.phase == AutoMovePhase::Fault; }

private:
    void ComputeErrors(const ChassisPose2D& pose, const ChassisBodyVelocity& velocity);
    ChassisBodyVelocity ComputeCorrection();
    void ApplySettlingDeadband(ChassisBodyVelocity& worldVelocity) const;
    void UpdateOutput(const ChassisBodyVelocity& worldVelocity, float yaw, float dt);
    void UpdateArrival(const ChassisBodyVelocity& measuredVelocity, float dt);

    TrapezoidalProfile planner_;
    AutoMoveTune param_ = {};
    AutoMoveState state_ = {};
    AutoMoveTelemetry telemetry_ = {};
    float directionX_ = 1.0f;
    float directionY_ = 0.0f;
    float maxYawVelocity_ = 0.0f;     // rad/s
    float yawAcceleration_ = 0.0f;    // rad/s^2
    float yawTolerance_ = 0.0f;       // rad
    float yawSpeedTolerance_ = 0.0f;  // rad/s
};
