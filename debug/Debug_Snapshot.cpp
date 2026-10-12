#include "Debug_Snapshot.h"

#include "math_utils.h"

#include <string.h>

DebugSnapshotData g_debugSnapshot = {};
volatile uint32_t g_debugSnapshotSequence = 0U;

namespace DebugSnapshot
{
    namespace
    {
        DebugPose2D ToDebugPose(const ChassisPose2D &pose)
        {
            DebugPose2D debugPose = {};
            debugPose.x = pose.x;
            debugPose.y = pose.y;
            debugPose.yawDeg = MathUtils::RadToDeg(pose.yaw);
            return debugPose;
        }

        DebugBodyVelocity ToDebugVelocity(const ChassisBodyVelocity &velocity)
        {
            DebugBodyVelocity debugVelocity = {};
            debugVelocity.vx = velocity.vx;
            debugVelocity.vy = velocity.vy;
            debugVelocity.wzDegPerSec = MathUtils::RadToDeg(velocity.wz);
            return debugVelocity;
        }

        DebugPoseFeedback ToDebugPoseFeedback(const PoseFeedback &feedback)
        {
            DebugPoseFeedback debugFeedback = {};
            debugFeedback.pose = ToDebugPose(feedback.pose);
            debugFeedback.source = feedback.source;
            debugFeedback.updateTick = feedback.updateTick;
            debugFeedback.valid = feedback.valid;
            return debugFeedback;
        }

        DebugLocalizationState ToDebugLocalization(const PoseSourceManager &poseSourceManager)
        {
            DebugLocalizationState debugLocalization = {};
            PoseFeedback feedback = {};
            debugLocalization.selectedSource = poseSourceManager.Source();
            poseSourceManager.GetCurrentPose(feedback);
            debugLocalization.currentPose = ToDebugPoseFeedback(feedback);
            poseSourceManager.GetOdometryPose(feedback);
            debugLocalization.odometryPose = ToDebugPoseFeedback(feedback);
            poseSourceManager.GetLidarPose(feedback);
            debugLocalization.lidarPose = ToDebugPoseFeedback(feedback);

            const LidarPoseProtocolState &lidarState = poseSourceManager.LidarProtocolState();
            DebugLidarProtocol &debugLidar = debugLocalization.lidarProtocol;
            debugLidar.radarPose = ToDebugPose(lidarState.radarPose);
            debugLidar.receiveTick = lidarState.receiveTick;
            debugLidar.frameCount = lidarState.frameCount;
            debugLidar.checksumErrorCount = lidarState.checksumErrorCount;
            debugLidar.invalidValueCount = lidarState.invalidValueCount;
            debugLidar.hasFrame = lidarState.hasFrame;
            memcpy(debugLidar.lastFrame, lidarState.lastFrame, sizeof(debugLidar.lastFrame));
            return debugLocalization;
        }

        DebugAutoMoveState ToDebugAutoMoveState(const AutoMoveState &state)
        {
            DebugAutoMoveState debugState = {};
            debugState.phase = state.phase;
            debugState.startPose = ToDebugPose(state.startPose);
            DebugAutoMoveTarget target = {};
            target.x = state.target.x;
            target.y = state.target.y;
            target.yawDeg = MathUtils::RadToDeg(state.target.yaw);
            debugState.target = target;
            debugState.referencePose = ToDebugPose(state.referencePose);

            debugState.referenceDistance = state.referenceDistance;
            debugState.referenceSpeed = state.referenceSpeed;
            debugState.planningTime = state.planningTime;
            debugState.planningPhase = state.planningPhase;
            debugState.alongError = state.alongError;
            debugState.crossError = state.crossError;
            debugState.yawErrorDeg = MathUtils::RadToDeg(state.yawError);
            debugState.positionError = state.positionError;
            debugState.alongMeasuredSpeed = state.alongMeasuredSpeed;
            debugState.alongSpeedError = state.alongSpeedError;
            debugState.alongPositionCorrection = state.alongPositionCorrection;
            debugState.alongSpeedCorrection = state.alongSpeedCorrection;
            debugState.alongCorrection = state.alongCorrection;
            debugState.crossCorrection = state.crossCorrection;
            debugState.worldVx = state.worldVx;
            debugState.worldVy = state.worldVy;
            debugState.settledTime = state.settledTime;
            debugState.velocity = ToDebugVelocity(state.velocity);
            return debugState;
        }
    }

    void Update(const RemoteState &remote, const Chassis &chassis,
                const ChaoheImuState &imu, const PoseSourceManager &poseSourceManager,
                const AutoModeController &autoMode)
    {
        /* 序号用于判断 Watch 是否正好读到更新过程中的中间状态。 */
        ++g_debugSnapshotSequence;

        g_debugSnapshot.remote = remote;
        chassis.CopyFeedbackForDebug(g_debugSnapshot.steerFeedback, g_debugSnapshot.wheelFeedback);
        g_debugSnapshot.imu = imu;
        g_debugSnapshot.measuredVelocity = ToDebugVelocity(chassis.MeasuredVelocity());
        g_debugSnapshot.measuredVelocityValid = chassis.HasMeasuredVelocity();
        g_debugSnapshot.usbReceive = BspUsb::RxState();
        g_debugSnapshot.localization = ToDebugLocalization(poseSourceManager);
        g_debugSnapshot.autoRoute = autoMode.RouteState();
        g_debugSnapshot.autoMove = ToDebugAutoMoveState(autoMode.Motion().State());
        g_debugSnapshot.updateTick = HAL_GetTick();

        ++g_debugSnapshotSequence;
    }
}
