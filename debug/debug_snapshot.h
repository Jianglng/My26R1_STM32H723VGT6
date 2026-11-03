/**
 * @file debug_snapshot.h
 * @brief 低频调试快照，业务控制不读取此数据
 */
#pragma once

#include <stdint.h>

#include "auto_move.h"
#include "bsp_usb.h"

#ifndef DEBUG_SNAPSHOT_ENABLE
#define DEBUG_SNAPSHOT_ENABLE 1
#endif

class ChassisApp;

struct DebugPose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float yawDeg = 0.0f;
};

struct DebugPoseFeedback
{
    DebugPose2D pose = {};
    uint32_t updateTick = 0U;
    bool valid = false;
};

struct DebugBodyVelocity
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wzDegPerSec = 0.0f;
};

struct DebugLocalizationStatus
{
    DebugPoseFeedback currentPose = {};
    DebugPoseFeedback odometryPose = {};
    DebugPoseFeedback lidarPose = {};
    DebugPose2D radarPose = {};
};

struct DebugAutoMoveStatus
{
    AutoMovePhase phase = AutoMovePhase::Idle;
    uint32_t currentPointIndex = 0U;
    DebugPose2D target = {};
    AutoMoveTelemetry telemetry = {};        // 原始 SI 遥测，yawError 和 referencePose.yaw 为 rad
    TrapezoidalProfileState reference = {};  // 直接复制规划器状态，无逐字段镜像
    float yawErrorDeg = 0.0f;
    DebugBodyVelocity command = {};
};

struct DebugStatus
{
    DebugBodyVelocity measuredVelocity = {};
    bool measuredVelocityValid = false;
    BspUsbRxState usbReceive = {};
    DebugLocalizationStatus localization = {};
    DebugAutoMoveStatus autoMove = {};
    uint32_t updateTick = 0U;
};

struct DebugRoot
{
    ChassisApp* app = nullptr;  // 应用任务启动后登记，只用于 Watch 展开
    DebugStatus status = {};
};

extern DebugRoot g_debug;
extern volatile uint32_t g_debugSequence;  // 奇数表示正在更新，偶数表示快照已完成

namespace DebugSnapshot
{
    constexpr uint32_t UPDATE_PERIOD_MS = 10U;
    void Update(const ChassisApp& app);
}  // namespace DebugSnapshot
