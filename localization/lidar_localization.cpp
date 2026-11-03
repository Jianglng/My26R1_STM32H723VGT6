/**
 * @file lidar_localization.cpp
 * @brief 雷达参考点换算为旋转中心，Reset 后首次有效帧建立位置原点
 */
#include "lidar_localization.h"

#include <math.h>

#include "main.h"

#include "bsp_usb.h"
#include "math_utils.h"

void LidarLocalization::Reset()
{
    protocol_.Reset();
    feedback_ = {};
    feedback_.source = PoseSource::Lidar;
    origin_ = {};
    state_ = State::Uninitialized;
}

void LidarLocalization::Update()
{
    bool streamReset = false;
    const uint16_t count =
        BspUsb::Read(usbRxBytes_, usbRxTicks_, LocalizationConfig::USB_READ_BUDGET_BYTES, streamReset);
    if (streamReset)
    {
        protocol_.ResetStream();
        state_ = State::Lost;
    }
    for (uint16_t i = 0U; i < count; ++i)
    {
        if (protocol_.FeedByte(usbRxBytes_[i], usbRxTicks_[i]))
        {
            const bool fresh = BspUsb::Configured() &&
                               HAL_GetTick() - protocol_.State().receiveTick <= LocalizationConfig::LIDAR_TIMEOUT_MS;
            state_ = fresh && UpdateChassisPose() ? State::Valid : State::Lost;
        }
    }
    const uint32_t nowTick = HAL_GetTick();
    if (!BspUsb::Configured() || nowTick - feedback_.updateTick > LocalizationConfig::LIDAR_TIMEOUT_MS)
    {
        state_ = State::Lost;
    }
    feedback_.valid = state_ == State::Valid;
}

bool LidarLocalization::UpdateChassisPose()
{
    const LidarPoseProtocolState& frame = protocol_.State();
    const float yaw = MathUtils::WrapPeriodicError(
        frame.radarPose.yaw - MathUtils::DegToRad(LocalizationConfig::LIDAR_YAW_OFFSET_DEG), 2.0f * MathUtils::PI);
    const float cosYaw = cosf(yaw);
    const float sinYaw = sinf(yaw);
    const float worldX = frame.radarPose.x - cosYaw * LocalizationConfig::LIDAR_OFFSET_X_M +
                         sinYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    const float worldY = frame.radarPose.y - sinYaw * LocalizationConfig::LIDAR_OFFSET_X_M -
                         cosYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    const float originX = origin_.initialized ? origin_.initialCenterWorldX : worldX;
    const float originY = origin_.initialized ? origin_.initialCenterWorldY : worldY;
    const ChassisPose2D pose = {worldX - originX, worldY - originY, yaw};
    // 仅在发布定位的出口检查，协议层已排除输入 NaN/Inf
    if (!isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.yaw))
    {
        return false;
    }
    if (!origin_.initialized)
    {
        origin_.initialCenterWorldX = worldX;
        origin_.initialCenterWorldY = worldY;
        origin_.initialized = true;
    }
    feedback_.pose = pose;
    feedback_.updateTick = frame.receiveTick;
    return true;
}
