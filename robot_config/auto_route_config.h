#pragma once

#include <stdint.h>

/**
 * @brief 自动运动航点，坐标必须使用当前选定来源的定位坐标系。
 * @note x/y 单位 m，yawDeg 单位 °；航点按数组顺序执行。
 */
struct AutoRoutePoint
{
    float x;       ///< 目标 x，单位 m。
    float y;       ///< 目标 y，单位 m。
    float yawDeg;  ///< 目标航向，单位 °。
};

/** @brief 自动运动航点表。修改航点任务时只编辑这个文件。 */
namespace AutoRouteConfig
{
    /* 里程计为上电局部坐标；雷达为地图坐标，切换来源前需核对航点坐标。 */
    constexpr AutoRoutePoint POINTS[] =
    {
        //{0.0f, 0.0f, 30.0f},
        {1.0f, 0.0f, 0.0f},
        {1.5f, 0.0f, 0.0f}
        /* 后续航点按 {x, y, yawDeg} 继续添加。 */
    };

    constexpr uint32_t POINT_COUNT =
        sizeof(POINTS) / sizeof(POINTS[0]);
}
