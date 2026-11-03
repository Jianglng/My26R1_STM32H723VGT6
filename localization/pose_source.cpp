/**
 * @file pose_source.cpp
 * @brief 定位更新转发，不复制或存储第二份位姿
 */
#include "pose_source.h"

void PoseSourceManager::Reset()
{
    odometry_.Reset();
    lidar_.Reset();
}

void PoseSourceManager::UpdateLidar()
{
    lidar_.Update();
}

void PoseSourceManager::UpdateOdometry(const OdometryInput& input)
{
    odometry_.Update(input);
}

void PoseSourceManager::InvalidateOdometry()
{
    odometry_.Invalidate();
}

const PoseFeedback& PoseSourceManager::CurrentPose() const
{
    return Source() == PoseSource::Lidar ? LidarPose() : OdometryPose();
}
