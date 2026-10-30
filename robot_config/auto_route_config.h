#pragma once

#include <stdint.h>

/**
 * @brief 车体中心的绝对目标位姿，使用当前选定来源的定位坐标系。
 * @note x/y 单位 m，yawDeg 单位 °；航点按数组顺序执行。
 *       x/y 不是雷达中心坐标，也不是相对上一航点的位移。
 */
struct AutoRoutePoint
{
    float x;       ///< 车体中心目标 x，单位 m。
    float y;       ///< 车体中心目标 y，单位 m。
    float yawDeg;  ///< 车体目标航向，单位 °。
};

/** @brief 自动运动航点表。修改航点任务时只编辑这个文件。 */
namespace AutoRouteConfig
{
    /*
     * 里程计使用上电局部坐标；雷达以首次有效定位时的旋转中心位置为原点。
     * 雷达保留世界坐标轴方向和航向，x/y 是相对该固定原点的旋转中心目标。
     * 航点不再补偿雷达安装偏移，也不逐段归零；yawDeg 是世界航向。
     * 连续两点 x/y 相同、仅 yawDeg 不同时，车体中心目标保持不动。
     */
    constexpr AutoRoutePoint POINTS[] =
    {
        {3.0f, 0.0f, 0.0f},
        {3.0f, 0.0f, 90.0f},
				{3.0f, 0.0f, 0.0f},
				{0.0f, 0.0f, 0.0f}
        // {1.0f, 0.0f, 0.0f},
        // {5.0f, 0.0f, 0.0f},
        // {5.0f, 0.0f, 90.0f},
        // {5.0f, 0.0f, 0.0f},
        // {0.0f, 0.0f, 90.0f},
        // {0.0f, 0.0f, 180.0f},
        // {2.0f, 0.5f, 90.0f}
        /* 后续航点按 {x, y, yawDeg} 继续添加。 */
    };

    constexpr uint32_t POINT_COUNT =
        sizeof(POINTS) / sizeof(POINTS[0]);
}
