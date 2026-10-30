#include "pose_source.h"

void PoseSourceManager::Reset(PoseSource source)
{
    selectedSource_ = source;
    chassisOdometry_.Reset();
    lidarLocalization_.Reset();
    odometryFeedback_ = PoseFeedback{};
    odometryFeedback_.source = PoseSource::Odometry;
}

void PoseSourceManager::UpdateLidar()
{
    lidarLocalization_.Update();
}

void PoseSourceManager::UpdateOdometry(const ChassisBodyVelocity &measuredBodyVelocity,
                                      float imuYawDeg, bool imuOnline, float dt,
                                      bool wheelFeedbackValid, uint32_t nowTick)
{
    if (selectedSource_ != PoseSource::Odometry)
    {
        return;
    }

    chassisOdometry_.Update(measuredBodyVelocity, imuYawDeg, imuOnline, dt, wheelFeedbackValid);
    odometryFeedback_.pose = chassisOdometry_.Pose();
    odometryFeedback_.valid = chassisOdometry_.Valid();
    if (odometryFeedback_.valid)
    {
        odometryFeedback_.updateTick = nowTick;
    }
}

bool PoseSourceManager::GetOdometryPose(PoseFeedback &feedback) const
{
    feedback = odometryFeedback_;
    return feedback.valid;
}

bool PoseSourceManager::GetLidarPose(PoseFeedback &feedback) const
{
    feedback = lidarLocalization_.Feedback();
    return feedback.valid;
}

bool PoseSourceManager::GetCurrentPose(PoseFeedback &feedback) const
{
    switch (selectedSource_)
    {
        case PoseSource::Odometry: return GetOdometryPose(feedback);
        case PoseSource::Lidar:    return GetLidarPose(feedback);
        default:
            feedback = PoseFeedback{};
            feedback.source = selectedSource_;
            return false;
    }
}
