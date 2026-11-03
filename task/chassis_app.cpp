/**
 * @file chassis_app.cpp
 * @brief 反馈、定位、模式控制和输出按固定顺序执行
 */
#include "chassis_app.h"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_uart.h"
#include "bsp_usb.h"
#include "debug_snapshot.h"

void ChassisApp::Init()
{
    timer_.Init(SystemCoreClock / 1000000U);
    status_.canInitialized = chassis_.Init();
    status_.remoteInit = remote_.Init(BspUart::RemotePort());
    localization_.Reset();
    activeAutoTune_ = LocalizationConfig::POSE_SOURCE == PoseSource::Lidar ? AutoMoveConfig::LIDAR_DEFAULT
                                                                           : AutoMoveConfig::ODOMETRY_DEFAULT;
    ApplyAutoTune();
    if (LocalizationConfig::POSE_SOURCE == PoseSource::Odometry)
    {
        status_.imuInit = imu_.Init(BspUart::ImuPort());
    }
    status_.initialized = status_.canInitialized && status_.remoteInit == HAL_OK && status_.imuInit == HAL_OK;
    BspUsb::Init();
    lastControlCycles_ = DWT->CYCCNT;
}

ChassisMode ChassisApp::SelectMode(const RemoteState& remote) const
{
    if (!status_.initialized || !remote.online)
    {
        return ChassisMode::Stop;
    }
    switch (remote.leftSwitch)
    {
        case RemoteSwitch::Up:
            return ChassisMode::Auto;
        case RemoteSwitch::Middle:
            return ChassisMode::Manual;
        case RemoteSwitch::Down:
        default:
            return ChassisMode::Stop;
    }
}

void ChassisApp::ApplyAutoTune()
{
    const AutoMoveTune candidate =
        LocalizationConfig::POSE_SOURCE == PoseSource::Lidar ? g_autoMoveTune.lidar : g_autoMoveTune.odometry;
    const bool valid =
        AutoMoveTuneValid(candidate) && candidate.maxOutputVelocity <= AutoMoveConfig::MAX_TUNE_OUTPUT_VELOCITY;
    g_autoMoveTune.lastApplyValid = valid;
    if (valid)
    {
        activeAutoTune_ = candidate;
        g_autoMoveTune.applied = candidate;
        ++g_autoMoveTune.applyCount;
    }
    else
    {
        ++g_autoMoveTune.rejectCount;
    }
    autoMode_.Reset(activeAutoTune_);
}

void ChassisApp::UpdateLocalization(float dt)
{
    //雷达位姿更新已在前面被调用，此处无需再说明雷达模式下的调用
    if (LocalizationConfig::POSE_SOURCE == PoseSource::Odometry)
    {
        imu_.Update();
        const ChaoheImuState& imu = imu_.State();
        const OdometryInput input = {chassis_.MeasuredVelocity(),   imu.yaw, dt, HAL_GetTick(), imu.online,
                                     chassis_.HasMeasuredVelocity()};
        localization_.UpdateOdometry(input);
    }
}

void ChassisApp::RunMode(const RemoteState& remote, float dt)
{
    const ChassisMode mode = SelectMode(remote);
    if (mode != status_.mode)
    {
        ApplyAutoTune();
        status_.mode = mode;
    }
    if (mode == ChassisMode::Stop)
    {
        chassis_.Stop();
        g_steerPidTune.lastApplyValid = chassis_.ApplySteerPidParam(g_steerPidTune.angle, g_steerPidTune.speed);
        return;
    }
    ChassisBodyVelocity command = {};
    if (mode == ChassisMode::Manual)
    {
        command = manualMode_.Update(remote);
    }
    else
    {
        const PoseFeedback& pose = localization_.CurrentPose();
        const AutoModeOutput output =
            autoMode_.Update(pose.pose, chassis_.MeasuredVelocity(), dt, pose.valid && chassis_.HasMeasuredVelocity());
        if (!output.enabled)
        {
            chassis_.Stop();
            return;
        }
        command = output.velocity;
    }
    if (!chassis_.SetVelocity(command))
    {
        ++status_.invalidOutputCount;
        autoMode_.Fault();
    }
}

void ChassisApp::RunOnce()
{
    const float dt = timer_.GetDeltaT(lastControlCycles_);
    status_.dt = dt;
    chassis_.UpdateFeedback();
    localization_.UpdateLidar();//只要接收到雷达数据无论什么模式下都会更新雷达位姿
    // dt 只在应用入口检查一次，异常周期不推进任何积分或规划
    if (!isfinite(dt) || dt <= 0.0f || dt > RobotConfig::MAX_CONTROL_DT_S)
    {
        ++status_.invalidPeriodCount;
        localization_.InvalidateOdometry();
        autoMode_.Fault();
        chassis_.Stop();
        chassis_.UpdateControl(RobotConfig::PID_DT_MIN_MS);
        UpdateDebug();
        return;
    }
    UpdateLocalization(dt);
    RunMode(remote_.State(), dt);
    //保证周期时间2ms<=dt<=5ms
    const float pidDtMs = fminf(RobotConfig::PID_DT_MAX_MS, fmaxf(RobotConfig::PID_DT_MIN_MS, dt * 1000.0f));
    chassis_.UpdateControl(pidDtMs);
    if (!chassis_.OutputValid())
    {
        ++status_.invalidOutputCount;
        autoMode_.Fault();
    }
    if (!chassis_.TransmitOk())
    {
        ++status_.transmitFailureCount;
        autoMode_.Fault();
        chassis_.Stop();
        chassis_.UpdateControl(pidDtMs);
    }
    UpdateDebug();
}

void ChassisApp::UpdateDebug()
{
    const uint32_t now = HAL_GetTick();
    if (now - lastDebugTick_ >= DebugSnapshot::UPDATE_PERIOD_MS)
    {
        status_.stackFreeWords = static_cast<uint32_t>(uxTaskGetStackHighWaterMark(nullptr));
#if DEBUG_SNAPSHOT_ENABLE
        DebugSnapshot::Update(*this);
#endif
        lastDebugTick_ = now;
    }
}
