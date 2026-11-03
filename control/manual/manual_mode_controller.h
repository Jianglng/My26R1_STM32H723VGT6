/**
 * @file manual_mode_controller.h
 * @brief 手动模式速度映射
 */
#pragma once

#include "motion_types.h"
#include "remote_input.h"

/** @brief 手动模式只负责把遥控输入映射为车体目标速度，不直接操作电机 */
class ManualModeController
{
public:
    /** @brief 调用者保证遥控已在线且处于手动模式，通道已在解码边界校验 */
    ChassisBodyVelocity Update(const RemoteState& remoteState) const;
};
