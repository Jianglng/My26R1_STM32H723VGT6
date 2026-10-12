#include "auto_mode_controller.h"
#include "auto_move_config.h"
#include "bsp_usb.h"
#include "manual_mode_controller.h"
#include "pose_source.h"
#include "robot_config.h"
#include "usb_device.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

uint32_t g_testPrimask = 0U;
static uint32_t testTick = 0U;
extern "C"
{
    USBD_HandleTypeDef hUsbDeviceHS = {};
    uint32_t HAL_GetTick(void) { return testTick; }
    void MX_USB_DEVICE_Init(void)
    {
        hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
        BspUsb_ResetReceive(1U);
    }
}

namespace
{
    using LidarFrame = std::array<uint8_t, 15>;

    void Near(float actual, float expected)
    {
        assert(std::fabs(actual - expected) < 1.0e-5f);
    }

    void SetChecksum(LidarFrame &frame)
    {
        uint8_t checksum = 0U;
        for (unsigned i = 0U; i < frame.size() - 1U; ++i)
        {
            checksum = static_cast<uint8_t>(checksum + frame[i]);
        }
        frame.back() = checksum;
    }

    LidarFrame MakeFrame(float x, float y, float yawRad)
    {
        LidarFrame frame = {{0xAAU, 0x55U}};
        const float values[] = {x, y, yawRad};
        for (unsigned i = 0U; i < 3U; ++i)
        {
            uint32_t bits = 0U;
            std::memcpy(&bits, &values[i], sizeof(bits));
            for (unsigned j = 0U; j < 4U; ++j)
            {
                frame[2U + i * 4U + j] = static_cast<uint8_t>(bits >> (8U * j));
            }
        }
        SetChecksum(frame);
        return frame;
    }

    void Receive(const LidarFrame &frame, uint32_t receiveTick)
    {
        testTick = receiveTick;
        BspUsb_ReceiveBytes(frame.data(), frame.size());
    }

    void TestProtocolRecovery()
    {
        LidarPoseProtocol parser;
        const LidarFrame good = MakeFrame(1.25f, -2.5f, 3.0f * MathUtils::PI);
        parser.FeedByte(0x01U, 1U);
        parser.FeedByte(0xAAU, 1U);
        for (unsigned i = 0U; i < good.size(); ++i)
        {
            assert(parser.FeedByte(good[i], 5U) == (i == good.size() - 1U));
        }
        Near(parser.State().radarPose.x, 1.25f);
        Near(parser.State().radarPose.y, -2.5f);
        assert(std::fabs(parser.State().radarPose.yaw) <= MathUtils::PI);

        LidarFrame bad = good;
        bad.back() ^= 1U;
        for (uint8_t byte : bad) { assert(!parser.FeedByte(byte, 6U)); }
        for (uint8_t byte : good) { parser.FeedByte(byte, 7U); }
        assert(parser.State().frameCount == 2U);
        assert(parser.State().checksumErrorCount > 0U);

        for (float invalid : {std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()})
        {
            const LidarFrame invalidFrame = MakeFrame(invalid, 0.0f, 0.0f);
            for (uint8_t byte : invalidFrame) { assert(!parser.FeedByte(byte, 8U)); }
        }
        assert(parser.State().invalidValueCount == 2U);
        for (uint8_t byte : good) { parser.FeedByte(byte, 9U); }
        assert(parser.State().frameCount == 3U);
        for (unsigned i = 0U; i < 8U; ++i) { parser.FeedByte(good[i], 10U); }
        parser.ResetStream();
        assert(!parser.State().hasFrame);
        for (unsigned i = 8U; i < good.size(); ++i) { parser.FeedByte(good[i], 11U); }
        assert(!parser.State().hasFrame);
        for (uint8_t byte : good) { parser.FeedByte(byte, 12U); }
        assert(parser.State().frameCount == 4U);
    }

    void TestUsbRingWrap()
    {
        BspUsb::Init();
        std::vector<uint8_t> first(900U);
        std::vector<uint8_t> second(200U);
        for (unsigned i = 0U; i < first.size(); ++i) { first[i] = static_cast<uint8_t>(i); }
        for (unsigned i = 0U; i < second.size(); ++i) { second[i] = static_cast<uint8_t>(255U - i); }
        uint8_t bytes[128] = {};
        uint32_t ticks[128] = {};
        bool reset = false;
        testTick = 10U;
        BspUsb_ReceiveBytes(first.data(), first.size());
        for (unsigned batch = 0U; batch < 6U; ++batch)
        {
            const uint16_t count = BspUsb::Read(bytes, ticks, 128U, reset);
            assert(count == 128U);
            for (unsigned i = 0U; i < count; ++i)
            {
                assert(bytes[i] == first[batch * 128U + i]);
                assert(ticks[i] == 10U);
            }
        }
        testTick = 11U;
        BspUsb_ReceiveBytes(second.data(), second.size());
        unsigned consumed = 0U;
        while (BspUsb::RxState().queuedBytes != 0U)
        {
            const uint16_t count = BspUsb::Read(bytes, ticks, 128U, reset);
            for (unsigned i = 0U; i < count; ++i, ++consumed)
            {
                const bool fromFirst = consumed < 132U;
                assert(bytes[i] == (fromFirst ? first[768U + consumed] : second[consumed - 132U]));
                assert(ticks[i] == (fromFirst ? 10U : 11U));
            }
        }
        assert(consumed == 332U);
        g_testPrimask = 1U;
        assert(BspUsb::Read(bytes, ticks, 128U, reset) == 0U);
        assert(g_testPrimask == 1U);
        g_testPrimask = 0U;
    }

    void TestUsbAndLocalization()
    {
        BspUsb::Init();
        LidarLocalization lidar;
        lidar.Reset();
        const LidarFrame frame = MakeFrame(1.0f, 2.0f, MathUtils::PI / 2.0f);

        // Split frame across control cycles; publish only after a complete frame.
        testTick = 100U;
        BspUsb_ReceiveBytes(frame.data(), 7U);
        lidar.Update();
        assert(!lidar.Feedback().valid);
        BspUsb_ReceiveBytes(frame.data() + 7U, frame.size() - 7U);
        lidar.Update();
        assert(lidar.Feedback().valid);
        Near(lidar.Feedback().pose.x, 1.0f);
        Near(lidar.Feedback().pose.y, 2.08667f);
        Near(lidar.Feedback().pose.yaw, MathUtils::PI / 2.0f);

        testTick = 100U + LocalizationConfig::LIDAR_TIMEOUT_MS;
        lidar.Update();
        assert(lidar.Feedback().valid);
        ++testTick;
        lidar.Update();
        assert(!lidar.Feedback().valid);

        // Processing delayed bytes must not refresh their original receive time.
        Receive(frame, 400U);
        testTick = 700U;
        lidar.Update();
        assert(!lidar.Feedback().valid);
        assert(lidar.Feedback().updateTick == 400U);

        std::vector<uint8_t> burst;
        for (unsigned i = 0U; i < 9U; ++i) { burst.insert(burst.end(), frame.begin(), frame.end()); }
        testTick = 800U;
        BspUsb_ReceiveBytes(burst.data(), burst.size());
        const uint32_t oldFrames = lidar.ProtocolState().frameCount;
        lidar.Update();
        assert(BspUsb::RxState().queuedBytes == burst.size() - LocalizationConfig::USB_READ_BUDGET_BYTES);
        assert(lidar.ProtocolState().frameCount == oldFrames + 8U);
        lidar.Update();
        assert(lidar.ProtocolState().frameCount == oldFrames + 9U);

        // Overflow discards the interrupted stream, invalidates old pose, then recovers.
        const std::vector<uint8_t> overflow(1025U, 0xAAU);
        BspUsb_ReceiveBytes(overflow.data(), overflow.size());
        lidar.Update();
        assert(!lidar.Feedback().valid);
        assert(BspUsb::RxState().overflowCount > 0U);
        Receive(frame, 810U);
        lidar.Update();
        assert(lidar.Feedback().valid);
        BspUsb_ResetReceive(0U);
        lidar.Update();
        assert(!lidar.Feedback().valid);
        BspUsb_ResetReceive(1U);
        Receive(frame, 820U);
        lidar.Update();
        assert(lidar.Feedback().valid);

        Receive(frame, 0xFFFFFFF0U);
        testTick = 0x20U;
        lidar.Update();
        assert(lidar.Feedback().valid);
        testTick = 0x100U;
        lidar.Update();
        assert(!lidar.Feedback().valid);
        assert(g_testPrimask == 0U);
    }

    void TestSourceAndAutoParameters()
    {
        PoseSourceManager localization;
        localization.Reset(PoseSource::Lidar);
        PoseFeedback feedback = {};
        const float nan = std::numeric_limits<float>::quiet_NaN();
        localization.UpdateOdometry(ChassisBodyVelocity{1.0f, 0.0f, 0.0f}, nan, 0.01f, false, 900U);
        Receive(MakeFrame(2.0f, 3.0f, 0.0f), 900U);
        localization.UpdateLidar();
        assert(localization.GetCurrentPose(feedback));
        assert(feedback.source == PoseSource::Lidar);
        Near(feedback.pose.x, 2.08667f);
        assert(!localization.GetOdometryPose(feedback));
        testTick = 1200U;
        localization.UpdateLidar();
        assert(!localization.GetCurrentPose(feedback));
        assert(feedback.source == PoseSource::Lidar); // Never fall back to odometry.

        localization.Reset(PoseSource::Odometry);
        const ChassisBodyVelocity forward = {1.0f, 0.0f, 0.0f};
        localization.UpdateOdometry(forward, 0.0f, 0.01f, true, 1300U);
        localization.UpdateOdometry(forward, 0.0f, 0.01f, true, 1310U);
        assert(localization.GetCurrentPose(feedback));
        Near(feedback.pose.x, 0.01f);
        localization.UpdateOdometry(forward, 0.0f, 0.01f, false, 1320U);
        assert(!localization.GetCurrentPose(feedback));

        // A new segment must actually use the selected profile and feedback parameters.
        for (PoseSource source : {PoseSource::Odometry, PoseSource::Lidar})
        {
            AutoModeController autoMode;
            autoMode.Reset(source);
            ChassisPose2D pose = {};
            ChassisBodyVelocity command = {};
            const ChassisBodyVelocity measured = {0.2f, 0.0f, 0.0f};
            assert(!autoMode.Update(pose, ChassisBodyVelocity{}, 0.01f, true, command));
            assert(autoMode.Update(pose, measured, 0.01f, true, command));
            const AutoMoveParam &param = AutoMoveConfig::ForSource(source);
            const AutoMoveState &state = autoMode.Motion().State();
            Near(state.referenceSpeed, param.acceleration * 0.01f);
            Near(state.alongSpeedCorrection, param.alongKd * (state.referenceSpeed - measured.vx));
            assert(autoMode.RouteState().poseSource == source);
            assert(!autoMode.Update(pose, measured, 0.01f, false, command));
            assert(autoMode.Motion().Faulted());
            Near(command.vx, 0.0f);
            Near(command.vy, 0.0f);
            Near(command.wz, 0.0f);
        }
    }

    void TestManualMapping()
    {
        ManualModeController manual;
        RemoteState remote = {};
        remote.leftY = 660;
        Near(manual.Update(remote).vx, 0.0f);
        remote.online = true;
        Near(manual.Update(remote).vx, RobotConfig::MAX_VX);
        remote.leftY = 0;
        remote.leftX = 660;
        Near(manual.Update(remote).vy, RobotConfig::MAX_VY);
        remote.leftX = 0;
        remote.rightX = 660;
        Near(manual.Update(remote).wz, -MathUtils::DegToRad(RobotConfig::MAX_WZ_DEG_PER_SEC));
    }

    void TestWaypointSequence()
    {
        for (PoseSource source : {PoseSource::Odometry, PoseSource::Lidar})
        {
            AutoModeController autoMode;
            autoMode.Reset(source);
            ChassisPose2D pose = {};
            ChassisBodyVelocity measured = {};
            bool reachedSecondPoint = false;
            for (unsigned step = 0U; step < 10000U && !autoMode.RouteState().finished; ++step)
            {
                ChassisBodyVelocity command = {};
                autoMode.Update(pose, measured, 0.01f, true, command);
                assert(!autoMode.Motion().Faulted());
                measured = command; // Ideal chassis follows the requested body velocity.
                pose.x += (std::cos(pose.yaw) * measured.vx - std::sin(pose.yaw) * measured.vy) * 0.01f;
                pose.y += (std::sin(pose.yaw) * measured.vx + std::cos(pose.yaw) * measured.vy) * 0.01f;
                pose.yaw += measured.wz * 0.01f;
                reachedSecondPoint = reachedSecondPoint || autoMode.RouteState().currentPointIndex == 1U;
            }
            assert(reachedSecondPoint);
            assert(autoMode.RouteState().finished);
            assert(autoMode.RouteState().currentPointIndex == AutoRouteConfig::POINT_COUNT);
        }
    }
}

int main()
{
    TestProtocolRecovery();
    TestUsbRingWrap();
    TestUsbAndLocalization();
    TestSourceAndAutoParameters();
    TestManualMapping();
    TestWaypointSequence();
    std::puts("PASS: protocol recovery, USB budget/overflow/disconnect, pose transform/timeout, source isolation, auto parameters and manual mapping");
}
