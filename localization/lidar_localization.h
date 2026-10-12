#pragma once

#include "lidar_pose_protocol.h"
#include "localization_config.h"

/**
 * @brief 雷达定位：在底盘周期中限量读取 USB，解析并转换为车体中心位姿。
 * @note 不使用 IMU 或里程计；没有数据立即返回，不等待完整帧。
 */
class LidarLocalization
{
public:
    void Reset();
    void Update();
    const PoseFeedback &Feedback() const { return lidarFeedback_; }
    const LidarPoseProtocolState &ProtocolState() const { return lidarProtocol_.State(); }

private:
    void UpdateChassisPose();

    LidarPoseProtocol lidarProtocol_;
    PoseFeedback lidarFeedback_ = {};
    uint8_t usbRxBytes_[LocalizationConfig::USB_READ_BUDGET_BYTES] = {};
    uint32_t usbRxTicks_[LocalizationConfig::USB_READ_BUDGET_BYTES] = {};
};
