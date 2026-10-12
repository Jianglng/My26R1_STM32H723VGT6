#pragma once

/*
 * 调试专用数据镜像
 *
 * 该模块只保存一份便于 Keil Watch 查看 的副本，不参与任何控制计算。
 * 遥控器、VESC 和 DJI 反馈均通过已有的安全 getter 读取；IMU 由底盘任务
 * 更新后再复制。控制任务继续使用原来的局部变量和对象状态。
 */

#include "Chassis.h"
#include "auto_mode_controller.h"
#include "chaohe_imu.h"
#include "bsp_usb.h"
#include "pose_source.h"
#include "remote_input.h"

/** @brief Watch 中显示的二维位姿，航向单位为度。 */
struct DebugPose2D
{
    float x = 0.0f;
    float y = 0.0f;
    float yawDeg = 0.0f;
};

/** @brief Watch 中显示的底盘速度，角速度单位为度每秒。 */
struct DebugBodyVelocity
{
    float vx = 0.0f;
    float vy = 0.0f;
    float wzDegPerSec = 0.0f;
};

/** @brief Watch 中显示的自动运动目标，航向单位为度。 */
struct DebugAutoMoveTarget
{
    float x = 0.0f;
    float y = 0.0f;
    float yawDeg = 0.0f;
};

/** @brief 自动运动调试镜像，角度和角速度均转换为度制。 */
struct DebugAutoMoveState
{
    AutoMovePhase phase = AutoMovePhase::Idle;
    DebugPose2D startPose = {};
    DebugAutoMoveTarget target = {};
    DebugPose2D referencePose = {};

    float referenceDistance = 0.0f;
    float referenceSpeed = 0.0f;
    float planningTime = 0.0f;
    TrapezoidalPhase planningPhase = TrapezoidalPhase::Idle;

    float alongError = 0.0f;
    float crossError = 0.0f;
    float yawErrorDeg = 0.0f;
    float positionError = 0.0f;
    float alongMeasuredSpeed = 0.0f;   ///< 实测沿路径速度 (m/s)。
    float alongSpeedError = 0.0f;      ///< 规划速度减实测沿路径速度 (m/s)。
    float alongPositionCorrection = 0.0f; ///< 限幅前的沿路径 P 修正速度 (m/s)。
    float alongSpeedCorrection = 0.0f; ///< 限幅前的沿路径 D 修正速度 (m/s)。
    float alongCorrection = 0.0f;
    float crossCorrection = 0.0f;
    float worldVx = 0.0f;
    float worldVy = 0.0f;
    float settledTime = 0.0f;
    DebugBodyVelocity velocity = {};
};

/** @brief 定位反馈的度制镜像，updateTick 为对应数据的更新时间。 */
struct DebugPoseFeedback
{
    DebugPose2D pose = {};
    PoseSource source = PoseSource::Odometry;
    uint32_t updateTick = 0U;
    bool valid = false;
};

/** @brief 雷达协议调试镜像，原始雷达中心与换算后的车体中心分开观察。 */
struct DebugLidarProtocol
{
    DebugPose2D radarPose = {};
    uint32_t receiveTick = 0U;
    uint32_t frameCount = 0U;
    uint32_t checksumErrorCount = 0U;
    uint32_t invalidValueCount = 0U;
    bool hasFrame = false;
    uint8_t lastFrame[15] = {};
};

struct DebugLocalizationState
{
    PoseSource selectedSource = PoseSource::Odometry;
    DebugPoseFeedback currentPose = {};
    DebugPoseFeedback odometryPose = {};
    DebugPoseFeedback lidarPose = {};
    DebugLidarProtocol lidarProtocol = {};
};

struct DebugSnapshotData
{
    RemoteState remote = {};
    Dji6020RxData steerFeedback[RobotConfig::WHEEL_COUNT] = {};
    VescRxData wheelFeedback[RobotConfig::WHEEL_COUNT] = {};
    ChaoheImuState imu = {}; ///< imu.yaw 已是度，来自 IMU 原始输出。
    DebugBodyVelocity measuredVelocity = {};
    bool measuredVelocityValid = false;
    BspUsbRxState usbReceive = {};
    DebugLocalizationState localization = {};
    AutoRouteState autoRoute = {};
    DebugAutoMoveState autoMove = {};
    uint32_t updateTick = 0U;
};

/* Keil Watch 中展开这个变量即可查看反馈、里程计和自动运动状态。 */
extern DebugSnapshotData g_debugSnapshot;

/* 更新序号：奇数表示正在更新，偶数表示本次更新已完成。 */
extern volatile uint32_t g_debugSnapshotSequence;

namespace DebugSnapshot
{
    constexpr uint32_t UPDATE_PERIOD_MS = 10U;

    void Update(const RemoteState &remote, const Chassis &chassis,
                const ChaoheImuState &imu, const PoseSourceManager &poseSourceManager,
                const AutoModeController &autoMode);
}
