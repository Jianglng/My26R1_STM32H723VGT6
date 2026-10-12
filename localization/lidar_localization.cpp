#include "lidar_localization.h"

#include "bsp_usb.h"
#include "math_utils.h"
#include "main.h"

#include <math.h>

void LidarLocalization::Reset()
{
    lidarProtocol_.Reset();
    lidarFeedback_ = PoseFeedback{};
    lidarFeedback_.source = PoseSource::Lidar;
}

void LidarLocalization::Update()
{
    bool streamReset = false;
    const uint16_t count = BspUsb::Read(usbRxBytes_, usbRxTicks_,
                                      LocalizationConfig::USB_READ_BUDGET_BYTES, streamReset);
    if (streamReset)
    {
        lidarProtocol_.ResetStream();
        lidarFeedback_.valid = false;
    }

    /* 单拍只处理本次读到的有限字节；多帧时保留最后一帧，半帧留待下拍。 */
    bool receivedPose = false;
    for (uint16_t i = 0U; i < count; ++i)
    {
        if (lidarProtocol_.FeedByte(usbRxBytes_[i], usbRxTicks_[i]))
        {
            receivedPose = true;
        }
    }
    if (receivedPose)
    {
        UpdateChassisPose();
    }

    const LidarPoseProtocolState &lidarState = lidarProtocol_.State();
    /* 读取和解析后再取当前时间，避免中途到达的新帧时间晚于本拍起始时间。 */
    const uint32_t nowTick = HAL_GetTick();
    /* 用原始 USB 接收时间判断新鲜度，而不是把解析积压数据的时间当作接收时间。
     * 无符号减法允许 HAL 毫秒计数正常回绕；超时不切回里程计。 */
    lidarFeedback_.valid = BspUsb::Configured() && lidarState.hasFrame &&
                           (nowTick - lidarState.receiveTick) <= LocalizationConfig::LIDAR_TIMEOUT_MS &&
                           isfinite(lidarFeedback_.pose.x) && isfinite(lidarFeedback_.pose.y) &&
                           isfinite(lidarFeedback_.pose.yaw);
}

void LidarLocalization::UpdateChassisPose()
{
    const LidarPoseProtocolState &lidarState = lidarProtocol_.State();
    const ChassisPose2D &radarPose = lidarState.radarPose;
    const float chassisYawRad = MathUtils::WrapPeriodicError(
        radarPose.yaw - MathUtils::DegToRad(LocalizationConfig::LIDAR_YAW_OFFSET_DEG),
        2.0f * MathUtils::PI);
    const float cosYaw = cosf(chassisYawRad);
    const float sinYaw = sinf(chassisYawRad);

    /* pRadar = pChassis + R(yawChassis) * 安装偏移，因此反解车体中心需减去偏移。
     * 默认偏移 x=-0.08667 m，对应参考工程 center=radar+delta_l*[cos(yaw),sin(yaw)]。 */
    lidarFeedback_.pose.x = radarPose.x - cosYaw * LocalizationConfig::LIDAR_OFFSET_X_M +
                            sinYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    lidarFeedback_.pose.y = radarPose.y - sinYaw * LocalizationConfig::LIDAR_OFFSET_X_M -
                            cosYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    lidarFeedback_.pose.yaw = chassisYawRad;
    lidarFeedback_.updateTick = lidarState.receiveTick;
    lidarFeedback_.source = PoseSource::Lidar;
}
