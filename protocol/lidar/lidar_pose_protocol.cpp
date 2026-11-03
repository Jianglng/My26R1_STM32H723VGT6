/**
 * @file lidar_pose_protocol.cpp
 * @brief 雷达位姿字节流解码
 */
#include "lidar_pose_protocol.h"

#include <string.h>

namespace
{
    bool DecodeFloat(const uint8_t* bytes, float& value)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "Protocol requires float32");
        const uint32_t bits = static_cast<uint32_t>(bytes[0]) | (static_cast<uint32_t>(bytes[1]) << 8U) |
                              (static_cast<uint32_t>(bytes[2]) << 16U) | (static_cast<uint32_t>(bytes[3]) << 24U);
        /* 在任何浮点运算前排除 NaN/Inf，避免异常数据进入角度归一化 */
        if ((bits & 0x7F800000U) == 0x7F800000U)
        {
            return false;
        }
        memcpy(&value, &bits, sizeof(value));
        return true;
    }
}  // namespace

void LidarPoseProtocol::Reset()
{
    state_ = LidarPoseProtocolState{};
    frameLength_ = 0U;
}

void LidarPoseProtocol::ResetStream()
{
    frameLength_ = 0U;
}

void LidarPoseProtocol::DiscardFirstByte()
{
    --frameLength_;
    memmove(frame_, frame_ + 1U, frameLength_);
}

bool LidarPoseProtocol::FeedByte(uint8_t byte, uint32_t receiveTick)
{
    frame_[frameLength_++] = byte;
    /* 滑动找帧头；AA AA 55 和坏帧内出现的下一帧头均可重新同步 */
    while (frameLength_ >= 2U && (frame_[0] != 0xAAU || frame_[1] != 0x55U))
    {
        DiscardFirstByte();
    }
    if (frameLength_ != FRAME_SIZE)
    {
        return false;
    }

    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < FRAME_SIZE - 1U; ++i)
    {
        checksum = static_cast<uint8_t>(checksum + frame_[i]);
    }
    if (checksum != frame_[FRAME_SIZE - 1U])
    {
        ++state_.checksumErrorCount;
        DiscardFirstByte();
        return false;
    }

    ChassisPose2D radarPose = {};
    if (!DecodeFloat(frame_ + 2U, radarPose.x) || !DecodeFloat(frame_ + 6U, radarPose.y) ||
        !DecodeFloat(frame_ + 10U, radarPose.yaw))
    {
        ++state_.invalidValueCount;
        DiscardFirstByte();
        return false;
    }

    state_.radarPose = radarPose;
    state_.receiveTick = receiveTick;
    ++state_.frameCount;
    memcpy(state_.lastFrame, frame_, FRAME_SIZE);
    frameLength_ = 0U;
    return true;
}
