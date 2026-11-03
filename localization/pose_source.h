/**
 * @file pose_source.h
 * @brief 编译期选择定位来源，只读访问由生产者持有的反馈
 */
#pragma once

#include "chassis_odometry.h"
#include "lidar_localization.h"
#include "localization_config.h"

class PoseSourceManager
{
public:
    void Reset();
    void UpdateLidar();
    /** @brief 仅里程计模式调用，调用者保证来源选择和 dt 有效 */
    void UpdateOdometry(const OdometryInput& input);
    void InvalidateOdometry();
    const PoseFeedback& OdometryPose() const { return odometry_.Feedback(); }
    const PoseFeedback& LidarPose() const { return lidar_.Feedback(); }
    const PoseFeedback& CurrentPose() const;
    constexpr PoseSource Source() const { return LocalizationConfig::POSE_SOURCE; }
    const LidarPoseProtocolState& LidarProtocolState() const { return lidar_.ProtocolState(); }

private:
    ChassisOdometry odometry_;
    LidarLocalization lidar_;
};
