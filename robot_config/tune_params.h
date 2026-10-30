#pragma once

#include <stdint.h>

#include "auto_move_config.h"
#include "robot_config.h"

/*
 * 运行时调参区
 *
 * 上电时由启动代码把 constexpr 默认值复制到 RAM，调试器修改的是这份副本。
 * 复位或断电后恢复默认值；调好的数值须手动抄回 auto_move_config.h 和 robot_config.h。
 * 只有底盘任务读写这两个变量，控制类通过参数接收，不直接依赖全局变量。
 */

/** @brief 自动运动调参区：odometry / lidar 可写，其余字段为只读回显。 */
struct AutoMoveTuneSet
{
    AutoMoveTune odometry;    ///< 里程计来源使用的参数，可在 Watch 中修改。
    AutoMoveTune lidar;       ///< 雷达来源使用的参数，可在 Watch 中修改。

    AutoMoveTune applied;     ///< 最近一次成功应用的参数。
    uint32_t applyCount;      ///< 成功应用次数，每次切档加 1。
    uint32_t rejectCount;     ///< 参数无效被拒绝的次数。
    bool lastApplyValid;      ///< 最近一次切档时参数是否有效。
};

/** @brief 舵向 PID 调参区：三个轮子共用，仅在 Stop 档写入 PID。 */
struct SteerPidTuneSet
{
    PidParam angle;           ///< 舵向角度环参数，可在 Watch 中修改。
    PidParam speed;           ///< 舵向速度环参数，可在 Watch 中修改。
    bool lastApplyValid;      ///< 最近一次 Stop 档写入是否成功；实际生效值见 g_debug.obj.chassis。
};

/* 切档时应用：在 Watch 中修改后拨动左拨杆，再进入自动档即生效。 */
extern AutoMoveTuneSet g_autoMoveTune;

/* Stop 档每拍应用：在下档修改立即生效，拨到中档或上档后保持不变。 */
extern SteerPidTuneSet g_steerPidTune;
