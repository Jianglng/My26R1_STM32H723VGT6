#pragma once

#include "chassis_odometry.h"
#include "lidar_localization.h"

/**
 * @brief 位姿来源管理。接收和计算在 Update 中完成，Get 函数只复制快照。
 * @note 来源只在初始化 Reset 时选择；运动中不自动切换或融合两种定位。
 */
class PoseSourceManager
{
public:
    void Reset(PoseSource source);
    /** @brief 所有模式下均可接收雷达，便于停机时观察，不改变选定来源。 */
    void UpdateLidar();
    /** @brief 仅选择里程计时积分；IMU 航向不进入雷达定位。 */
    void UpdateOdometry(const ChassisBodyVelocity &measuredBodyVelocity,
                        float imuYawDeg, float dt, bool odometryFeedbackValid,
                        uint32_t nowTick);

    bool GetOdometryPose(PoseFeedback &feedback) const;
    bool GetLidarPose(PoseFeedback &feedback) const;
    bool GetCurrentPose(PoseFeedback &feedback) const;
    PoseSource Source() const { return selectedSource_; }
    const LidarPoseProtocolState &LidarProtocolState() const { return lidarLocalization_.ProtocolState(); }

private:
    PoseSource selectedSource_ = PoseSource::Odometry;
    ChassisOdometry chassisOdometry_;
    LidarLocalization lidarLocalization_;
    PoseFeedback odometryFeedback_ = {};
};
