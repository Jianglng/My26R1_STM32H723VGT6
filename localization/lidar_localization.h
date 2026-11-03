/**
 * @file lidar_localization.h
 * @brief 雷达中心换算与固定原点管理
 */
#pragma once

#include "lidar_pose_protocol.h"
#include "localization_config.h"

/** @brief 跑点原点在雷达世界坐标系中的位置；只平移原点，不旋转坐标轴 */
struct LidarOriginState
{
    float initialCenterWorldX = 0.0f;  ///< 初始旋转中心世界 x，单位 m
    float initialCenterWorldY = 0.0f;  ///< 初始旋转中心世界 y，单位 m
    bool initialized = false;          ///< 首帧有效且未超时的定位是否已建立原点
};

/**
 * @brief 雷达定位：换算实测旋转中心，并以首次有效定位的位置为跑点原点
 * @note 不使用 IMU 或里程计；没有数据立即返回，不等待完整帧
 *       建立原点时须停车且上位机定位已稳定；yaw 保留世界航向，不减初始值
 */
class LidarLocalization
{
public:
    /** @brief 清除定位和原点；普通模式切换、通信超时不调用此复位 */
    void Reset();
    void Update();
    const PoseFeedback& Feedback() const { return feedback_; }
    const LidarPoseProtocolState& ProtocolState() const { return protocol_.State(); }
    const LidarOriginState& OriginState() const { return origin_; }

private:
    bool UpdateChassisPose();

    LidarPoseProtocol protocol_;
    PoseFeedback feedback_ = {};
    LidarOriginState origin_ = {};
    enum class State : uint8_t
    {
        Uninitialized,
        Valid,
        Lost
    };
    State state_ = State::Uninitialized;
    uint8_t usbRxBytes_[LocalizationConfig::USB_READ_BUDGET_BYTES] = {};
    uint32_t usbRxTicks_[LocalizationConfig::USB_READ_BUDGET_BYTES] = {};
};
