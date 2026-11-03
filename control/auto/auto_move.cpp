/**
 * @file auto_move.cpp
 * @brief 世界坐标系跟踪、终点收敛及输出变化率限制
 */
#include "auto_move.h"

#include <math.h>

#include "math_utils.h"

namespace
{
    void LimitVector(float& x, float& y, float limit)
    {
        const float length = hypotf(x, y);
        if (length > limit)
        {
            const float scale = limit / length;
            x *= scale;
            y *= scale;
        }
    }
}  // namespace

void AutoMove::Reset()
{
    planner_.Reset();
    state_ = AutoMoveState{};
    telemetry_ = AutoMoveTelemetry{};
    directionX_ = 1.0f;
    directionY_ = 0.0f;
}

void AutoMove::Start(const ChassisPose2D& pose, const AutoMoveTarget& target, const AutoMoveTune& param)
{
    Reset();
    param_ = param;
    maxYawVelocity_ = MathUtils::DegToRad(param.maxYawVelocityDegPerSec);
    yawAcceleration_ = MathUtils::DegToRad(param.yawAccelerationDegPerSec2);
    yawTolerance_ = MathUtils::DegToRad(param.yawToleranceDeg);
    yawSpeedTolerance_ = MathUtils::DegToRad(param.yawSpeedToleranceDegPerSec);
    state_.startPose = pose;
    state_.target = target;
    const float dx = target.x - pose.x;
    const float dy = target.y - pose.y;
    const float length = hypotf(dx, dy);
    planner_.Start(length, param.maxVelocity, param.acceleration, param.deceleration);
    state_.phase = AutoMovePhase::Settling;
    if (length > 0.0f)
    {
        directionX_ = dx / length;
        directionY_ = dy / length;
        state_.phase = AutoMovePhase::Tracking;
    }
    telemetry_.referencePose = pose;
    telemetry_.referencePose.yaw = target.yaw;
    telemetry_.positionError = length;
}

void AutoMove::Update(const ChassisPose2D& pose, const ChassisBodyVelocity& velocity, float dt, bool feedbackValid)
{
    if (state_.phase == AutoMovePhase::Idle || Faulted())
    {
        return;
    }
    if (!feedbackValid)
    {
        SetFault();
        return;
    }
    if (Finished())
    {
        return;
    }
    if (state_.phase == AutoMovePhase::Tracking)
    {
        planner_.Update(dt);
        if (planner_.Finished())
        {
            state_.phase = AutoMovePhase::Settling;
        }
    }
    ComputeErrors(pose, velocity);
    ChassisBodyVelocity worldVelocity = ComputeCorrection();
    ApplySettlingDeadband(worldVelocity);
    UpdateOutput(worldVelocity, pose.yaw, dt);
    UpdateArrival(velocity, dt);
}

void AutoMove::SetFault()
{
    state_.phase = AutoMovePhase::Fault;
    state_.worldVx = 0.0f;
    state_.worldVy = 0.0f;
    state_.velocity = {};
    state_.settledTime = 0.0f;
}

void AutoMove::ComputeErrors(const ChassisPose2D& pose, const ChassisBodyVelocity& velocity)
{
    const float distance = planner_.State().position;
    telemetry_.referencePose.x = state_.startPose.x + directionX_ * distance;
    telemetry_.referencePose.y = state_.startPose.y + directionY_ * distance;
    if (state_.phase == AutoMovePhase::Settling)
    {
        telemetry_.referencePose.x = state_.target.x;
        telemetry_.referencePose.y = state_.target.y;
    }
    const float errorX = telemetry_.referencePose.x - pose.x;
    const float errorY = telemetry_.referencePose.y - pose.y;
    telemetry_.alongError = directionX_ * errorX + directionY_ * errorY;
    telemetry_.crossError = -directionY_ * errorX + directionX_ * errorY;
    telemetry_.yawError = MathUtils::WrapPeriodicError(state_.target.yaw - pose.yaw, 2.0f * MathUtils::PI);
    telemetry_.positionError = hypotf(state_.target.x - pose.x, state_.target.y - pose.y);
    const float cosYaw = cosf(pose.yaw);
    const float sinYaw = sinf(pose.yaw);
    const float worldVx = cosYaw * velocity.vx - sinYaw * velocity.vy;
    const float worldVy = sinYaw * velocity.vx + cosYaw * velocity.vy;
    telemetry_.alongMeasuredSpeed = directionX_ * worldVx + directionY_ * worldVy;
    telemetry_.alongSpeedError = planner_.State().velocity - telemetry_.alongMeasuredSpeed;
}

ChassisBodyVelocity AutoMove::ComputeCorrection()
{
    const bool settling = state_.phase == AutoMovePhase::Settling;
    telemetry_.activeAlongKp = settling ? param_.settlingAlongKp : param_.trackingAlongKp;
    telemetry_.activeAlongKd = settling ? param_.settlingAlongKd : param_.trackingAlongKd;
    telemetry_.alongPositionCorrection = telemetry_.activeAlongKp * telemetry_.alongError;
    telemetry_.alongSpeedCorrection = telemetry_.activeAlongKd * telemetry_.alongSpeedError;
    telemetry_.alongCorrection = telemetry_.alongPositionCorrection + telemetry_.alongSpeedCorrection;
    telemetry_.crossCorrection = param_.crossKp * telemetry_.crossError;
    LimitVector(telemetry_.alongCorrection, telemetry_.crossCorrection, param_.maxCorrectionVelocity);
    const float alongVelocity = planner_.State().velocity + telemetry_.alongCorrection;
    return {directionX_ * alongVelocity - directionY_ * telemetry_.crossCorrection,
            directionY_ * alongVelocity + directionX_ * telemetry_.crossCorrection, param_.yawKp * telemetry_.yawError};
}

void AutoMove::ApplySettlingDeadband(ChassisBodyVelocity& velocity) const
{
    if (state_.phase != AutoMovePhase::Settling)
    {
        return;
    }
    if (telemetry_.positionError <= param_.positionTolerance)
    {
        velocity.vx = 0.0f;
        velocity.vy = 0.0f;
    }
    if (fabsf(telemetry_.yawError) <= yawTolerance_)
    {
        velocity.wz = 0.0f;
    }
}

void AutoMove::UpdateOutput(const ChassisBodyVelocity& worldVelocity, float yaw, float dt)
{
    float worldVx = worldVelocity.vx;
    float worldVy = worldVelocity.vy;
    LimitVector(worldVx, worldVy, param_.maxOutputVelocity);
    const float yawVelocity = MathUtils::ClampAbs(worldVelocity.wz, maxYawVelocity_);
    const float currentSpeed = hypotf(state_.worldVx, state_.worldVy);
    const float targetSpeed = hypotf(worldVx, worldVy);
    const float rate = targetSpeed < currentSpeed ? param_.deceleration : param_.acceleration;
    float deltaVx = worldVx - state_.worldVx;
    float deltaVy = worldVy - state_.worldVy;
    if (hypotf(deltaVx, deltaVy) > rate * dt)
    {
        LimitVector(deltaVx, deltaVy, rate * dt);
        worldVx = state_.worldVx + deltaVx;
        worldVy = state_.worldVy + deltaVy;
    }
    state_.velocity.wz += MathUtils::ClampAbs(yawVelocity - state_.velocity.wz, yawAcceleration_ * dt);
    state_.worldVx = worldVx;
    state_.worldVy = worldVy;
    const float cosYaw = cosf(yaw);
    const float sinYaw = sinf(yaw);
    state_.velocity.vx = cosYaw * worldVx + sinYaw * worldVy;
    state_.velocity.vy = -sinYaw * worldVx + cosYaw * worldVy;
}

void AutoMove::UpdateArrival(const ChassisBodyVelocity& velocity, float dt)
{
    const bool arrived =
        state_.phase == AutoMovePhase::Settling && telemetry_.positionError <= param_.positionTolerance &&
        fabsf(telemetry_.yawError) <= yawTolerance_ && hypotf(velocity.vx, velocity.vy) <= param_.speedTolerance &&
        fabsf(velocity.wz) <= yawSpeedTolerance_ && state_.worldVx == 0.0f && state_.worldVy == 0.0f &&
        state_.velocity.wz == 0.0f;
    if (!arrived)
    {
        state_.settledTime = 0.0f;
        return;
    }
    state_.settledTime = fminf(param_.settleDuration, state_.settledTime + dt);
    if (state_.settledTime >= param_.settleDuration)
    {
        state_.phase = AutoMovePhase::Finished;
    }
}
