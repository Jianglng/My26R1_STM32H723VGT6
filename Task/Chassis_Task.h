#pragma once

/**
 * @brief 底盘任务入口。读遥控器，调用 Chassis::SetVelocity() 或 Stop()，再按周期调度。
 *
 * freertos.c 创建该任务。
 */

#ifdef __cplusplus
extern "C"
{
#endif

void StartChassisTask(void *argument);

#ifdef __cplusplus
}
#endif
