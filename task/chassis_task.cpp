/**
 * @file chassis_task.cpp
 * @brief 固定周期调度底盘应用，任务栈不持有大对象
 */
#include "chassis_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "chassis_app.h"
#include "debug_snapshot.h"
#include "robot_config.h"

extern "C" void StartChassisTask(void* argument)
{
    (void)argument;
    static ChassisApp app;
#if DEBUG_SNAPSHOT_ENABLE
    g_debug.app = &app;
#endif
    app.Init();
    TickType_t lastWakeTick = xTaskGetTickCount();
    constexpr TickType_t period = pdMS_TO_TICKS(RobotConfig::CONTROL_PERIOD_MS);
    static_assert(period > 0U, "Control period must be at least one OS tick");
    for (;;)
    {
        app.RunOnce();
        vTaskDelayUntil(&lastWakeTick, period);
    }
}
