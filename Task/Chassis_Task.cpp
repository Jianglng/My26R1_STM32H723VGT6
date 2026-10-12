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
#include "usart.h"

namespace
{
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

    chassis.Init();
    remote.Init(&huart5);
    poseSourceManager.Reset(LocalizationConfig::POSE_SOURCE);
    autoMode.Reset(poseSourceManager.Source());
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
            const bool odometryFeedbackValid = chassis.HasMeasuredVelocity() && odometryImuState.online;
            poseSourceManager.UpdateOdometry(measuredBodyVelocity, odometryImuState.yaw, dt, odometryFeedbackValid, HAL_GetTick());
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
            /* 新切入自动档重新开始航点任务；退出时取消当前任务。 */
            autoMode.Reset(poseSourceManager.Source());
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
                if (autoMode.Update(currentPoseFeedback.pose, measuredBodyVelocity, dt,
                                    feedbackValid, autoVelocity))
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
                break;
        }
        chassis.UpdateControl();

        /* 调试镜像不参与控制，低频更新以免给 1 ms 控制循环增加额外开销。 */
        const uint32_t now = HAL_GetTick();
        if ((now - lastDebugSnapshotTick) >= DebugSnapshot::UPDATE_PERIOD_MS)
        {
            DebugSnapshot::Update(state, chassis, imu.State(), poseSourceManager, autoMode);
            lastDebugSnapshotTick = now;
        }
        osDelay(1);
    }
}
