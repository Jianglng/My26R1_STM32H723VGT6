/**
 * @file lidar_pose_protocol.h
 * @brief 雷达位姿字节流解码
 */
#pragma once

#include <stdint.h>

#include "motion_types.h"

/** @brief 雷达原始位姿及协议统计；原始 yaw 的单位为 rad */
struct LidarPoseProtocolState
{
    ChassisPose2D radarPose = {};
    uint32_t receiveTick = 0U;
    uint32_t frameCount = 0U;
    uint32_t checksumErrorCount = 0U;
    uint32_t invalidValueCount = 0U;
    uint8_t lastFrame[15] = {};
};

/**
 * @brief 雷达位姿字节流解析，不读取 USB，不执行坐标变换
 * @note 帧格式：AA 55 + x/y/yaw 三个小端 float32 + 前 14 字节累加和低 8 位
 *       x/y 单位 m，yaw 单位 rad；支持分包、连包、噪声和坏帧后重新同步
 */
class LidarPoseProtocol
{
public:
    void Reset();
    /** @brief 字节流中断时丢弃半帧，保留协议统计，定位层负责失效状态 */
    void ResetStream();
    /** @return 本字节完成一帧有效位姿时返回 true */
    bool FeedByte(uint8_t byte, uint32_t receiveTick);
    const LidarPoseProtocolState& State() const { return state_; }

private:
    void DiscardFirstByte();

    static constexpr uint8_t FRAME_SIZE = 15U;
    uint8_t frame_[FRAME_SIZE] = {};
    uint8_t frameLength_ = 0U;
    LidarPoseProtocolState state_ = {};
};
