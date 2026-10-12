#include "lidar_pose_protocol.h"

#include "math_utils.h"

#include <string.h>

namespace
{
    bool DecodeFloat(const uint8_t *bytes, float &value)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "Protocol requires float32");
        const uint32_t bits = static_cast<uint32_t>(bytes[0]) |
                              (static_cast<uint32_t>(bytes[1]) << 8U) |
                              (static_cast<uint32_t>(bytes[2]) << 16U) |
                              (static_cast<uint32_t>(bytes[3]) << 24U);
        /* 在任何浮点运算前排除 NaN/Inf，避免异常数据进入角度归一化。 */
        if ((bits & 0x7F800000U) == 0x7F800000U)
        {
            return false;
        }
        memcpy(&value, &bits, sizeof(value));
        return true;
    }
}

void LidarPoseProtocol::Reset()
{
    lidarProtocolState_ = LidarPoseProtocolState{};
    lidarFrameLength_ = 0U;
}

void LidarPoseProtocol::ResetStream()
{
    lidarFrameLength_ = 0U;
    lidarProtocolState_.hasFrame = false;
}

void LidarPoseProtocol::DiscardFirstByte()
{
    --lidarFrameLength_;
    memmove(lidarFrame_, lidarFrame_ + 1U, lidarFrameLength_);
}

bool LidarPoseProtocol::FeedByte(uint8_t byte, uint32_t receiveTick)
{
    lidarFrame_[lidarFrameLength_++] = byte;
    /* 滑动找帧头；AA AA 55 和坏帧内出现的下一帧头均可重新同步。 */
    while (lidarFrameLength_ >= 2U &&
           (lidarFrame_[0] != 0xAAU || lidarFrame_[1] != 0x55U))
    {
        DiscardFirstByte();
    }
    if (lidarFrameLength_ != FRAME_SIZE)
    {
        return false;
    }

    uint8_t checksum = 0U;
    for (uint8_t i = 0U; i < FRAME_SIZE - 1U; ++i)
    {
        checksum = static_cast<uint8_t>(checksum + lidarFrame_[i]);
    }
    if (checksum != lidarFrame_[FRAME_SIZE - 1U])
    {
        ++lidarProtocolState_.checksumErrorCount;
        DiscardFirstByte();
        return false;
    }

    ChassisPose2D radarPose = {};
    if (!DecodeFloat(lidarFrame_ + 2U, radarPose.x) ||
        !DecodeFloat(lidarFrame_ + 6U, radarPose.y) ||
        !DecodeFloat(lidarFrame_ + 10U, radarPose.yaw))
    {
        ++lidarProtocolState_.invalidValueCount;
        DiscardFirstByte();
        return false;
    }

    radarPose.yaw = MathUtils::WrapPeriodicError(radarPose.yaw, 2.0f * MathUtils::PI);
    lidarProtocolState_.radarPose = radarPose;
    lidarProtocolState_.receiveTick = receiveTick;
    ++lidarProtocolState_.frameCount;
    lidarProtocolState_.hasFrame = true;
    memcpy(lidarProtocolState_.lastFrame, lidarFrame_, FRAME_SIZE);
    lidarFrameLength_ = 0U;
    return true;
}
