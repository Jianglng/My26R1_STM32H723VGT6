#include "Debug_Snapshot.h"

#include "math_utils.h"

DebugRoot g_debug = {};
volatile uint32_t g_debugSequence = 0U;

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
            debugFeedback.updateTick = feedback.updateTick;
            debugFeedback.valid = feedback.valid;
            return debugFeedback;
        }

        void UpdateLocalization(const PoseSourceManager &poseSourceManager,
                                DebugLocalizationStatus &localization)
        {
            PoseFeedback feedback = {};
            localization.selectedSource = poseSourceManager.Source();
            poseSourceManager.GetCurrentPose(feedback);
            localization.currentPose = ToDebugPoseFeedback(feedback);
            poseSourceManager.GetOdometryPose(feedback);
            localization.odometryPose = ToDebugPoseFeedback(feedback);
            poseSourceManager.GetLidarPose(feedback);
            localization.lidarPose = ToDebugPoseFeedback(feedback);
            localization.radarPose = ToDebugPose(poseSourceManager.LidarProtocolState().radarPose);
        }

        void UpdateAutoMove(const AutoModeController &autoMode, DebugAutoMoveStatus &autoMove)
        {
            const AutoMoveState &state = autoMode.Motion().State();
            autoMove.phase = state.phase;
            autoMove.currentPointIndex = autoMode.RouteState().currentPointIndex;
            autoMove.target.x = state.target.x;
            autoMove.target.y = state.target.y;
            autoMove.target.yawDeg = MathUtils::RadToDeg(state.target.yaw);
            autoMove.referencePose = ToDebugPose(state.referencePose);
            autoMove.yawErrorDeg = MathUtils::RadToDeg(state.yawError);
            autoMove.alongError = state.alongError;
            autoMove.crossError = state.crossError;
            autoMove.positionError = state.positionError;
            autoMove.referenceSpeed = state.referenceSpeed;
            autoMove.alongMeasuredSpeed = state.alongMeasuredSpeed;
            autoMove.command = ToDebugVelocity(state.velocity);
        }
    }

    void Update(const Chassis &chassis, const PoseSourceManager &poseSourceManager,
                const AutoModeController &autoMode)
    {
        /* 序号用于判断 Watch 是否正好读到更新过程中的中间状态。 */
        ++g_debugSequence;

        DebugStatus &status = g_debug.status;
        status.measuredVelocity = ToDebugVelocity(chassis.MeasuredVelocity());
        status.measuredVelocityValid = chassis.HasMeasuredVelocity();
        status.usbReceive = BspUsb::RxState();
        UpdateLocalization(poseSourceManager, status.localization);
        UpdateAutoMove(autoMode, status.autoMove);
        status.updateTick = HAL_GetTick();

        ++g_debugSequence;
    }
}
