/**
 * @file debug_snapshot.cpp
 * @brief 调试快照进行角度换算，遥测整块复制
 */
#include "debug_snapshot.h"

#include "chassis_app.h"
#include "math_utils.h"

DebugRoot g_debug = {};                  // 底盘任务写，调试器读
volatile uint32_t g_debugSequence = 0U;  // 底盘任务写，调试器检查快照版本

namespace
{
    DebugPose2D ToDebugPose(const ChassisPose2D& pose)
    {
        return {pose.x, pose.y, MathUtils::RadToDeg(pose.yaw)};
    }

    DebugPoseFeedback ToDebugFeedback(const PoseFeedback& feedback)
    {
        return {ToDebugPose(feedback.pose), feedback.updateTick, feedback.valid};
    }

    DebugBodyVelocity ToDebugVelocity(const ChassisBodyVelocity& velocity)
    {
        return {velocity.vx, velocity.vy, MathUtils::RadToDeg(velocity.wz)};
    }

    void UpdateLocalization(const PoseSourceManager& source, DebugLocalizationStatus& status)
    {
        status.currentPose = ToDebugFeedback(source.CurrentPose());
        status.odometryPose = ToDebugFeedback(source.OdometryPose());
        status.lidarPose = ToDebugFeedback(source.LidarPose());
        status.radarPose = ToDebugPose(source.LidarProtocolState().radarPose);
    }

    void UpdateAutoMove(const AutoModeController& mode, DebugAutoMoveStatus& status)
    {
        const AutoMove& motion = mode.Motion();
        const AutoMoveState& state = motion.State();
        status.phase = state.phase;
        status.currentPointIndex = mode.RouteState().currentPointIndex;
        status.target = {state.target.x, state.target.y, MathUtils::RadToDeg(state.target.yaw)};
        status.telemetry = motion.Telemetry();
        status.reference = motion.Reference();
        status.yawErrorDeg = MathUtils::RadToDeg(status.telemetry.yawError);
        status.command = ToDebugVelocity(state.velocity);
    }
}  // namespace

void DebugSnapshot::Update(const ChassisApp& app)
{
    ++g_debugSequence;
    DebugStatus& status = g_debug.status;
    status.measuredVelocity = ToDebugVelocity(app.ChassisDevice().MeasuredVelocity());
    status.measuredVelocityValid = app.ChassisDevice().HasMeasuredVelocity();
    status.usbReceive = BspUsb::RxState();
    UpdateLocalization(app.Localization(), status.localization);
    UpdateAutoMove(app.AutoMode(), status.autoMove);
    status.updateTick = HAL_GetTick();
    ++g_debugSequence;
}
