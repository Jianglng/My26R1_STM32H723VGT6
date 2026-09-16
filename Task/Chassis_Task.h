#pragma once

/**
 * @brief 底盘任务：遥控器 -> 逆解 -> 6020 串级 PID -> VESC 转速。
 *
 * 在 freertos.c 中创建，入口为 StartChassisTask()。
 */
