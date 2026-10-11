#pragma once

#include "auto_move.h"
#include "auto_route_config.h"

/** @brief 自动航点任务的运行状态，供任务和调试窗口观察。 */
struct AutoRouteState
{
    uint32_t currentPointIndex = 0U; ///< 当前航点下标，从 0 开始。
    uint32_t pointCount = 0U;        ///< 航点总数。
    bool finished = false;            ///< 所有航点是否都已完成。
};

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
     * 进入自动模式后，只有反馈有效且底盘停稳，才会启动当前航点。
     * 返回 true 表示 command 可以交给底盘；返回 false 表示本拍应停机。
     */
    bool Update(const ChassisPose2D &currentPose,
                const ChassisBodyVelocity &measuredVelocity,
                float dt, bool feedbackValid,
                ChassisBodyVelocity &command);

    /** @brief 返回当前航点任务状态，供调试镜像读取。 */
    const AutoRouteState &RouteState() const { return routeState_; }

    /** @brief 返回内部运动控制器，供调试镜像读取状态。 */
    const AutoMove &Motion() const { return autoMove_; }

private:
    AutoMove autoMove_;
    AutoRouteState routeState_ = {};
};
