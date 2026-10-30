#include "Chassis_Task.h"

#include "Chassis.h"
#include "Debug_Snapshot.h"
#include "auto_mode_controller.h"
#include "bsp_usb.h"
#include "chaohe_imu.h"
#include "cmsis_os.h"
#include "localization_config.h"
#include "manual_mode_controller.h"
#include "pose_source.h"
#include "remote_input.h"
#include "tune_params.h"
#include "usart.h"

namespace
{
    /* 复位自动任务并应用当前来源的调参值，结果回显到 g_autoMoveTune。 */
    void ResetAutoMode(AutoModeController &autoMode, PoseSource source)
    {
        const AutoMoveTune &tune = (source == PoseSource::Lidar) ? g_autoMoveTune.lidar
                                                                 : g_autoMoveTune.odometry;
        const bool applied = autoMode.Reset(source, tune);
        g_autoMoveTune.lastApplyValid = applied;
        if (applied)
        {
            g_autoMoveTune.applied = tune;
            ++g_autoMoveTune.applyCount;
        }
        else
        {
            ++g_autoMoveTune.rejectCount;
        }
    }

    enum class ChassisMode : uint8_t
    {
        Stop,
        Manual,
        Auto
    };

    ChassisMode SelectChassisMode(const RemoteState &remote)
    {
        if (!remote.online)
        {
            return ChassisMode::Stop;
        }

        switch (remote.leftSwitch)
        {
            case 1U: return ChassisMode::Auto;   // 上档
            case 3U: return ChassisMode::Manual; // 中档
            case 2U: return ChassisMode::Stop;   // 下档
            default: return ChassisMode::Stop;
        }
    }
}

extern "C" void StartChassisTask(void *argument)
{
    (void)argument;

    static Chassis chassis;
    static RemoteInput remote;
    static ChaoheImu imu;
    static PoseSourceManager poseSourceManager;
    static ManualModeController manualMode;
    static AutoModeController autoMode;
    ChassisMode lastMode = ChassisMode::Stop;
    uint32_t lastDebugSnapshotTick = 0U;

    /* 只登记地址供调试器展开，对象仍归本任务所有。 */
    g_debug.obj.chassis = &chassis;
    g_debug.obj.remote = &remote;
    g_debug.obj.imu = &imu;
    g_debug.obj.poseSourceManager = &poseSourceManager;
    g_debug.obj.autoMode = &autoMode;

    chassis.Init();
    remote.Init(&huart5);
    poseSourceManager.Reset(LocalizationConfig::POSE_SOURCE);
    ResetAutoMode(autoMode, poseSourceManager.Source());
    if (poseSourceManager.Source() == PoseSource::Odometry)
    {
        /* IMU 只属于里程计定位，雷达模式不启动 IMU 接收 */
        imu.Init(&huart10);
    }
    BspUsb::Init();
    uint32_t lastControlCycles = DWT->CYCCNT;

    for (;;)
    {
        /* 先更新反馈和选定位姿，再计算目标速度，最后执行底盘控制。 */
        chassis.UpdateFeedback();
        const ChassisBodyVelocity &measuredBodyVelocity = chassis.MeasuredVelocity();
        const float dt = chassis.Timer().GetDeltaT(&lastControlCycles);
        if (poseSourceManager.Source() == PoseSource::Odometry)
        {
            imu.Update();
            const ChaoheImuState &odometryImuState = imu.State();
            /* IMU 离线不阻断里程计，由定位模块切换为轮速航向积分。 */
            poseSourceManager.UpdateOdometry(measuredBodyVelocity, odometryImuState.yaw,
                                            odometryImuState.online, dt, chassis.HasMeasuredVelocity(), HAL_GetTick());
        }

        /* USB 非阻塞、限量处理；所有模式下都可观察雷达，不另建接收任务。 */
        poseSourceManager.UpdateLidar();
        PoseFeedback currentPoseFeedback = {};
        const bool poseValid = poseSourceManager.GetCurrentPose(currentPoseFeedback);
        const bool feedbackValid = chassis.HasMeasuredVelocity() && poseValid;

        const RemoteState state = remote.State();
        const ChassisMode mode = SelectChassisMode(state);
        if (mode != lastMode)
        {
            /* 新切入自动档重新开始航点任务；退出时取消当前任务。
             * 切档同时应用 g_autoMoveTune，行进中修改不生效。 */
            ResetAutoMode(autoMode, poseSourceManager.Source());
            lastMode = mode;
        }

        switch (mode)
        {
            case ChassisMode::Manual:
                chassis.SetVelocity(manualMode.Update(state));
                break;

            case ChassisMode::Auto:
            {
                ChassisBodyVelocity autoVelocity = {};
                if (autoMode.Update(currentPoseFeedback.pose, measuredBodyVelocity, dt, feedbackValid, autoVelocity))
                {
                    chassis.SetVelocity(autoVelocity);
                }
                else
                {
                    chassis.Stop();
                }
                break;
            }

            case ChassisMode::Stop:
            default:
                chassis.Stop();
                /* 停机时 PID 每拍都会清零积分，此时换参数与原有行为一致。 */
                g_steerPidTune.lastApplyValid =
                    chassis.ApplySteerPidParam(g_steerPidTune.angle, g_steerPidTune.speed);
                break;
        }
        chassis.UpdateControl();

        /* 调试镜像不参与控制，低频更新以免给 1 ms 控制循环增加额外开销。 */
        const uint32_t now = HAL_GetTick();
        if ((now - lastDebugSnapshotTick) >= DebugSnapshot::UPDATE_PERIOD_MS)
        {
            DebugSnapshot::Update(chassis, poseSourceManager, autoMode);
            lastDebugSnapshotTick = now;
        }
        osDelay(1);
    }
}
