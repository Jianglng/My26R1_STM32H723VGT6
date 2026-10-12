#pragma once

#include <stdint.h>

/**
 * @brief 车体中心在当前定位坐标系中的位姿。
 * @note x/y 单位 m，yaw 单位 rad；车体 +x 向前，+y 向左，逆时针为正。
 *       里程计使用上电建立的局部坐标系，雷达使用其地图世界坐标系。
 */
struct ChassisPose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
};

/** @brief 车体坐标系实测或目标速度，平移单位 m/s，角速度单位 rad/s。 */
struct ChassisBodyVelocity
{
    float vx;   ///< 向前为正。
    float vy;   ///< 向左为正。
    float wz;   ///< 逆时针为正。
};

/** @brief 自动跑点使用的位姿来源，一次运行只选用一种，不自动切换。 */
enum class PoseSource : uint8_t
{
    Odometry,
    Lidar
};

/** @brief 带来源、更新时间和有效状态的完整位姿快照。 */
struct PoseFeedback
{
    ChassisPose2D pose = {};
    PoseSource source = PoseSource::Odometry;
    uint32_t updateTick = 0U;           ///< 里程计更新或雷达完整帧接收时间 (ms)。
    bool valid = false;
};
