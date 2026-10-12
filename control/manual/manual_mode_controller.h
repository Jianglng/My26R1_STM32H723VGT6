#pragma once

#include "motion_types.h"
#include "remote_input.h"

/** @brief 手动模式只负责把遥控输入映射为车体目标速度，不直接操作电机。 */
class ManualModeController
{
public:
    ChassisBodyVelocity Update(const RemoteState &remoteState) const;
};
