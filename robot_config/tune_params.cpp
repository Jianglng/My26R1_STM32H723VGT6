#include "tune_params.h"

/* 非零初值放在 .data 段：默认值存于 Flash，上电时复制到 RAM。 */
AutoMoveTuneSet g_autoMoveTune =
{
    AutoMoveConfig::ODOMETRY_DEFAULT,
    AutoMoveConfig::LIDAR_DEFAULT,
    {},
    0U,
    0U,
    false
};

SteerPidTuneSet g_steerPidTune =
{
    RobotConfig::STEER_ANGLE,
    RobotConfig::STEER_SPEED,
    false
};
