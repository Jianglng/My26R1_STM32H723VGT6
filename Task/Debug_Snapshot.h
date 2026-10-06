#pragma once

/*
 * 调试专用数据镜像
 *
 * 该模块只保存一份便于 Keil Watch 查看 的副本，不参与任何控制计算。
 * 遥控器、VESC 和 DJI 反馈均通过已有的安全 getter 读取；IMU 由底盘任务
 * 更新后再复制。控制任务继续使用原来的局部变量和对象状态。
 */

#include "Chassis.h"
#include "chaohe_imu.h"
#include "chassis_odometry.h"
#include "remote_input.h"

struct DebugSnapshotData
{
    RemoteState remote = {};
    Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT] = {};
    VescRxData wheelFeedback[RobotConfig::WHEEL_COUNT] = {};
    ChaoheImuState imu = {};
    ChassisBodyVelocity measuredVelocity = {};
    bool measuredVelocityValid = false;
    ChassisPose2D odometryPose = {};
    bool odometryValid = false;
    uint32_t updateTick = 0U;
};

/* Keil Watch 中展开这个变量即可查看三类数据。 */
extern DebugSnapshotData g_debugSnapshot;

/* 更新序号：奇数表示正在更新，偶数表示本次更新已完成。 */
extern volatile uint32_t g_debugSnapshotSequence;

namespace DebugSnapshot
{
    constexpr uint32_t UPDATE_PERIOD_MS = 20U;

    void Update(const RemoteState &remote, const Chassis &chassis,
                const ChaoheImuState &imu, const ChassisOdometry &odometry);
}
