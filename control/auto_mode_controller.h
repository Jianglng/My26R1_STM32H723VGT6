#pragma once

#include "auto_move.h"

/**
 * @brief 自动模式流程控制器。
 *
 * 负责管理自动模式的启动条件、运动更新和退出复位；具体的轨迹跟踪
 * 计算由内部 AutoMove 完成。该类只返回车体目标速度，不直接操作底盘。
 */
class AutoModeController
{
public:
    /** @brief 取消当前自动运动，回到等待启动状态。 */
    void Reset();

    /**
     * @brief 更新一次自动模式并返回车体目标速度。
     *
     * 进入自动模式后，只有反馈有效且底盘停稳，才会启动一段运动。
     * 返回 true 表示 command 可以交给底盘；返回 false 表示本拍应停机。
     */
    bool Update(const ChassisPose2D &currentPose,
                const ChassisBodyVelocity &measuredVelocity,
                float dt, bool feedbackValid,
                ChassisBodyVelocity &command);

    /** @brief 返回内部运动控制器，供调试镜像读取状态。 */
    const AutoMove &Motion() const { return autoMove_; }

private:
    AutoMove autoMove_;
};
