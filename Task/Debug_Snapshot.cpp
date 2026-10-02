#include "Debug_Snapshot.h"

DebugSnapshotData g_debugSnapshot = {};
volatile uint32_t g_debugSnapshotSequence = 0U;

namespace DebugSnapshot
{
    void Update(const RemoteState &remote, const Chassis &chassis, const ChaoheImuState &imu)
    {
        /* 序号用于判断 Watch 是否正好读到更新过程中的中间状态。 */
        ++g_debugSnapshotSequence;

        g_debugSnapshot.remote = remote;
        chassis.CopyFeedbackForDebug(g_debugSnapshot.steerFeedback, g_debugSnapshot.wheelFeedback);
        g_debugSnapshot.imu = imu;
        g_debugSnapshot.measuredVelocity = chassis.MeasuredVelocity();
        g_debugSnapshot.measuredVelocityValid = chassis.HasMeasuredVelocity();
        g_debugSnapshot.updateTick = HAL_GetTick();

        ++g_debugSnapshotSequence;
    }
}
