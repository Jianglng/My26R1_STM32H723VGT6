/**
 * @file chassis_app.h
 * @brief 底盘应用对象，唯一拥有业务设备和控制流程
 */
#pragma once

#include "auto_mode_controller.h"
#include "bsp_dwt.h"
#include "chaohe_imu.h"
#include "chassis.h"
#include "manual_mode_controller.h"
#include "pose_source.h"
#include "remote_input.h"
#include "tune_params.h"

enum class ChassisMode : uint8_t
{
    Stop,
    Manual,
    Auto
};

struct ChassisAppStatus
{
    HAL_StatusTypeDef remoteInit = HAL_ERROR;
    HAL_StatusTypeDef imuInit = HAL_OK;  // 雷达模式无需启动 IMU
    bool canInitialized = false;
    bool initialized = false;
    uint32_t invalidPeriodCount = 0U;
    uint32_t transmitFailureCount = 0U;
    uint32_t invalidOutputCount = 0U;
    uint32_t stackFreeWords = 0U;
    float dt = 0.0f;  // s
    ChassisMode mode = ChassisMode::Stop;
};

class ChassisApp
{
public:
    ChassisApp() = default;
    ChassisApp(const ChassisApp&) = delete;
    ChassisApp& operator=(const ChassisApp&) = delete;
    void Init();
    /** @brief 仅由底盘任务调用，测量并验证一次实际周期 */
    void RunOnce();
    const Chassis& ChassisDevice() const { return chassis_; }
    const PoseSourceManager& Localization() const { return localization_; }
    const AutoModeController& AutoMode() const { return autoMode_; }
    const ChassisAppStatus& Status() const { return status_; }

private:
    ChassisMode SelectMode(const RemoteState& remote) const;
    void ApplyAutoTune();
    void UpdateLocalization(float dt);
    void RunMode(const RemoteState& remote, float dt);
    void UpdateDebug();

    Chassis chassis_;
    DwtTimer timer_;
    RemoteInput remote_;
    ChaoheImu imu_;
    PoseSourceManager localization_;
    ManualModeController manualMode_;
    AutoModeController autoMode_;
    AutoMoveTune activeAutoTune_ = {};  // 上一次通过边界校验的参数，不依赖可写调试回显
    ChassisAppStatus status_ = {};
    uint32_t lastControlCycles_ = 0U;
    uint32_t lastDebugTick_ = 0U;
};
