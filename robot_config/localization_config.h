/**
 * @file localization_config.h
 * @brief 定位来源和安装参数
 */
#pragma once

#include "motion_types.h"

/** @brief 定位来源和雷达安装配置；修改来源后重新编译，上电后保持不变 */
namespace LocalizationConfig
{
    constexpr PoseSource POSE_SOURCE = PoseSource::Lidar;

    constexpr uint32_t LIDAR_TIMEOUT_MS = 200U;       ///< 完整有效帧超过此时间未更新则无效
    constexpr uint16_t USB_READ_BUDGET_BYTES = 128U;  ///< 每个底盘周期最多处理的 USB 字节数

    /* 车体坐标系中，从实测旋转中心指向雷达中心的安装偏移 (m)
     * 雷达在旋转中心前方 0.4636 m、左方 0.0160 m，两个分量均为正
     * yaw 偏移为雷达坐标轴相对车体坐标轴的逆时针角度，配置使用度 */
    constexpr float LIDAR_OFFSET_X_M = 0.4636f;
    constexpr float LIDAR_OFFSET_Y_M = 0.0160f;
    constexpr float LIDAR_YAW_OFFSET_DEG = 0.0f;
}  // namespace LocalizationConfig
