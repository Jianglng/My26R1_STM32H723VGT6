#pragma once

/*
 * 调试入口，不参与任何控制计算。
 *
 * Keil Watch 中只需添加 g_debug：
 *   obj    指向底盘任务中的真实对象，展开即可实时查看全部成员；
 *          舵向 PID 请改 g_steerPidTune；Stop 档下直接改 state_ 会被它每拍覆盖。
 *   status 每 10 ms 更新一次的副本：航向换算为度，另含便于 CubeMonitor 画曲线的量。
 *          修改 status 不会影响控制；调参使用 tune_params.h 中的 g_autoMoveTune、g_steerPidTune。
 */

#include "Chassis.h"
#include "auto_mode_controller.h"
#include "bsp_usb.h"
#include "chaohe_imu.h"
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

/** @brief 定位反馈的度制副本，updateTick 为对应数据的更新时间。 */
struct DebugPoseFeedback
{
    DebugPose2D pose = {};
    uint32_t updateTick = 0U;
    bool valid = false;
};

/** @brief 定位状态；原点、协议计数等不需换算的量通过 obj.poseSourceManager 查看。 */
struct DebugLocalizationStatus
{
    PoseSource selectedSource = PoseSource::Odometry;
    DebugPoseFeedback currentPose = {};    ///< 自动控制实际采用的车体中心位姿。
    DebugPoseFeedback odometryPose = {};   ///< 雷达模式下不积分且保持无效。
    DebugPoseFeedback lidarPose = {};      ///< 相对固定起点的旋转中心位姿。
    DebugPose2D radarPose = {};            ///< 换算前的雷达中心原始位姿。
};

/** @brief 自动运动中需要换算或画曲线的量；其余字段见 obj.autoMode。 */
struct DebugAutoMoveStatus
{
    AutoMovePhase phase = AutoMovePhase::Idle;
    uint32_t currentPointIndex = 0U;
    DebugPose2D target = {};
    DebugPose2D referencePose = {};
    float yawErrorDeg = 0.0f;
    float alongError = 0.0f;               ///< (m)
    float crossError = 0.0f;               ///< (m)
    float positionError = 0.0f;            ///< 距最终目标点的距离 (m)。
    float referenceSpeed = 0.0f;           ///< 规划速度 (m/s)。
    float alongMeasuredSpeed = 0.0f;       ///< 实测沿路径速度 (m/s)。
    DebugBodyVelocity command = {};        ///< 交给底盘的车体目标速度。
};

struct DebugStatus
{
    DebugBodyVelocity measuredVelocity = {};
    bool measuredVelocityValid = false;
    BspUsbRxState usbReceive = {};         ///< USB 接收统计，原变量在匿名命名空间中不便查找。
    DebugLocalizationStatus localization = {};
    DebugAutoMoveStatus autoMove = {};
    uint32_t updateTick = 0U;
};

/** @brief 底盘任务中各对象的地址；任务启动前为空指针。 */
struct DebugObjects
{
    Chassis *chassis = nullptr;
    RemoteInput *remote = nullptr;
    ChaoheImu *imu = nullptr;              ///< imu->state_.yaw 为度；雷达模式下不更新。
    PoseSourceManager *poseSourceManager = nullptr;
    AutoModeController *autoMode = nullptr;
};

struct DebugRoot
{
    DebugObjects obj = {};
    DebugStatus status = {};
};

extern DebugRoot g_debug;

/* status 更新序号：奇数表示正在更新，偶数表示本次更新已完成。 */
extern volatile uint32_t g_debugSequence;

namespace DebugSnapshot
{
    constexpr uint32_t UPDATE_PERIOD_MS = 10U;

    void Update(const Chassis &chassis, const PoseSourceManager &poseSourceManager,
                const AutoModeController &autoMode);
}
