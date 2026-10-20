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
    lidarOrigin_ = LidarOriginState{};
    centerPoseValid_ = false;
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
        centerPoseValid_ = false;
    }

    /* 逐帧换算：首个新鲜有效帧建立原点，后续帧更新位置；半帧留待下拍。 */
    for (uint16_t i = 0U; i < count; ++i)
    {
        if (lidarProtocol_.FeedByte(usbRxBytes_[i], usbRxTicks_[i]))
        {
            const LidarPoseProtocolState &frame = lidarProtocol_.State();
            const bool frameFresh = BspUsb::Configured() &&
                (HAL_GetTick() - frame.receiveTick) <= LocalizationConfig::LIDAR_TIMEOUT_MS;
            centerPoseValid_ = frameFresh && UpdateChassisPose();
        }
    }
    const LidarPoseProtocolState &lidarState = lidarProtocol_.State();
    /* 读取和解析后再取当前时间，避免中途到达的新帧时间晚于本拍起始时间。 */
    const uint32_t nowTick = HAL_GetTick();
    /* 用原始 USB 接收时间判断新鲜度，而不是把解析积压数据的时间当作接收时间。
     * 无符号减法允许 HAL 毫秒计数正常回绕；超时不切回里程计。 */
    lidarFeedback_.valid = centerPoseValid_ && lidarOrigin_.initialized &&
                           BspUsb::Configured() && lidarState.hasFrame &&
                           (nowTick - lidarState.receiveTick) <= LocalizationConfig::LIDAR_TIMEOUT_MS &&
                           isfinite(lidarFeedback_.pose.x) && isfinite(lidarFeedback_.pose.y) &&
                           isfinite(lidarFeedback_.pose.yaw);
}

bool LidarLocalization::UpdateChassisPose()
{
    const LidarPoseProtocolState &lidarState = lidarProtocol_.State();
    const ChassisPose2D &radarPose = lidarState.radarPose;
    const float chassisYawRad = MathUtils::WrapPeriodicError(
        radarPose.yaw - MathUtils::DegToRad(LocalizationConfig::LIDAR_YAW_OFFSET_DEG),
        2.0f * MathUtils::PI);
    const float cosYaw = cosf(chassisYawRad);
    const float sinYaw = sinf(chassisYawRad);

    /* 安装向量固定在车体上，须按实时航向转到世界系后减去，得到旋转中心。 */
    const float centerWorldX = radarPose.x - cosYaw * LocalizationConfig::LIDAR_OFFSET_X_M +
                                           sinYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    const float centerWorldY = radarPose.y - sinYaw * LocalizationConfig::LIDAR_OFFSET_X_M -
                                           cosYaw * LocalizationConfig::LIDAR_OFFSET_Y_M;
    if (!isfinite(centerWorldX) || !isfinite(centerWorldY) || !isfinite(chassisYawRad))
    {
        return false;
    }

    /* 仅首次记录原点；超时、断连和航点切换后仍沿用，防止路线坐标跳变。 */
    if (!lidarOrigin_.initialized)
    {
        lidarOrigin_.initialCenterWorldX = centerWorldX;
        lidarOrigin_.initialCenterWorldY = centerWorldY;
        lidarOrigin_.initialized = true;
    }

    const float relativeX = centerWorldX - lidarOrigin_.initialCenterWorldX;
    const float relativeY = centerWorldY - lidarOrigin_.initialCenterWorldY;
    if (!isfinite(relativeX) || !isfinite(relativeY))
    {
        return false;
    }

    /* 只减初始位置：保留世界 x/y 方向和世界航向，不按初始 yaw 旋转坐标轴。 */
    lidarFeedback_.pose.x = relativeX;
    lidarFeedback_.pose.y = relativeY;
    lidarFeedback_.pose.yaw = chassisYawRad;
    lidarFeedback_.updateTick = lidarState.receiveTick;
    lidarFeedback_.source = PoseSource::Lidar;
    return true;
}
