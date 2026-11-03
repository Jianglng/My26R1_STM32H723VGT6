/**
 * @file motion_tests.cpp
 * @brief 使用真实控制与协议代码验证运动、停机、坐标和接收时效
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

#include "auto_mode_controller.h"
#include "auto_move_config.h"
#include "auto_route_config.h"
#include "bsp_can.h"
#include "bsp_usb.h"
#include "chassis.h"
#include "chassis_odometry.h"
#include "lidar_localization.h"

namespace
{
    uint32_t nowTick = 0U;
    int failures = 0;
    int checks = 0;
    bool usbConfigured = true;
    bool resetStream = false;
    uint8_t usbBytes[256] = {};
    uint32_t usbTicks[256] = {};
    uint16_t usbSize = 0U;
    bool sendOk = true;
    struct SentFrame
    {
        CanBus bus;
        uint32_t id;
        uint8_t bytes[8];
    };
    SentFrame sent[32] = {};
    uint32_t sentCount = 0U;

    void Check(bool condition, const char* expression, int line)
    {
        ++checks;
        if (!condition)
        {
            ++failures;
            std::printf("FAIL line %d: %s\n", line, expression);
        }
    }

#define CHECK(expression) Check((expression), #expression, __LINE__)

    bool Near(float actual, float expected, float tolerance = 1.0e-4f)
    {
        return std::fabs(actual - expected) < tolerance;
    }

    void MakeFrame(uint8_t (&frame)[15], float x, float y, float yaw)
    {
        frame[0] = 0xAAU;
        frame[1] = 0x55U;
        const float values[] = {x, y, yaw};
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            uint32_t bits = 0U;
            std::memcpy(&bits, &values[i], sizeof(bits));
            for (uint32_t j = 0U; j < 4U; ++j)
            {
                frame[2U + i * 4U + j] = static_cast<uint8_t>(bits >> (8U * j));
            }
        }
        uint8_t checksum = 0U;
        for (uint32_t i = 0U; i < 14U; ++i)
        {
            checksum = static_cast<uint8_t>(checksum + frame[i]);
        }
        frame[14] = checksum;
    }

    void QueuePose(float x, float y, float yaw, uint32_t tick)
    {
        uint8_t frame[15];
        MakeFrame(frame, x, y, yaw);
        std::memcpy(usbBytes, frame, sizeof(frame));
        for (uint32_t i = 0U; i < sizeof(frame); ++i)
        {
            usbTicks[i] = tick;
        }
        usbSize = sizeof(frame);
    }

    void TestPidAndProfile()
    {
        Pid pid;
        PidParam parameter = RobotConfig::STEER_ANGLE;
        parameter.kp = 1.0f;
        parameter.ki = 0.0f;
        parameter.kd = 2.0f;
        pid.Init(parameter);
        CHECK(Near(pid.CalculateEncoder(8190.0f, 2.0f, 8192.0f, 2.0f), 8.0f));
        CHECK(Near(pid.CalculateEncoder(8190.0f, 2.0f, 8192.0f, 2.0f), 4.0f));
        parameter.kp = 0.0f;
        parameter.ki = 1.0f;
        parameter.kd = 0.0f;
        parameter.integralLimit = 3.0f;
        pid.Init(parameter);
        CHECK(Near(pid.Calculate(0.0f, 1.0f, 2.0f), 2.0f));
        CHECK(Near(pid.Calculate(0.0f, 1.0f, 2.0f), 3.0f));
        pid.Reset();
        CHECK(Near(pid.State().iOut, 0.0f));

        TrapezoidalProfile profile;
        profile.Start(10.0f, 2.0f, 1.0f, 1.0f);
        profile.Update(1.0f);
        CHECK(Near(profile.Position(), 0.5f) && Near(profile.Velocity(), 1.0f));
        profile.Update(2.0f);
        CHECK(profile.State().phase == TrapezoidalPhase::Cruising);
        CHECK(Near(profile.Position(), 4.0f));
        profile.Update(100.0f);
        CHECK(profile.Finished() && Near(profile.Position(), 10.0f) && Near(profile.Velocity(), 0.0f));
        profile.Start(1.0f, 2.0f, 1.0f, 1.0f);
        profile.Update(1.0f);
        CHECK(Near(profile.Position(), 0.5f) && Near(profile.Velocity(), 1.0f));
        profile.Update(1.0f);
        CHECK(profile.Finished() && Near(profile.Position(), 1.0f));
        profile.Start(0.0f, 2.0f, 1.0f, 1.0f);
        CHECK(profile.Finished());
    }

    void TestKinematics()
    {
        ChassisKinematics model(RobotConfig::GEOMETRY);
        int16_t encoder[3] = {6922, 7180, 5077};
        const ChassisBodyVelocity inputs[] = {
            {1.0f, 0.0f, 0.0f}, {-1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.7f, -0.4f, 0.8f}};
        for (const ChassisBodyVelocity& input : inputs)
        {
            const ChassisWheelCommand command = model.Inverse(input, encoder);
            for (uint32_t i = 0U; i < 3U; ++i)
            {
                const float error = MathUtils::WrapPeriodicError(command.steerEncoder[i] - encoder[i], 8192.0f);
                CHECK(std::fabs(error) <= 2048.01f);
                encoder[i] = static_cast<int16_t>(command.steerEncoder[i]);
            }
            const ChassisBodyVelocity output = model.Forward(encoder, command.wheelRpm);
            CHECK(Near(output.vx, input.vx, 0.002f));
            CHECK(Near(output.vy, input.vy, 0.002f));
            CHECK(Near(output.wz, input.wz, 0.003f));
            const ChassisWheelCommand stopped = model.Inverse({}, encoder);
            for (uint32_t i = 0U; i < 3U; ++i)
            {
                CHECK(stopped.steerEncoder[i] == command.steerEncoder[i]);
                CHECK(stopped.wheelRpm[i] == 0.0f);
            }
        }
    }

    void TestAuto()
    {
        const AutoMoveTune tune = AutoMoveConfig::LIDAR_DEFAULT;
        AutoMove motion;
        motion.Start({}, {0.0f, 0.0f, MathUtils::PI / 2.0f}, tune);
        CHECK(motion.State().phase == AutoMovePhase::Settling);
        motion.Update({}, {}, 0.01f, true);
        CHECK(Near(motion.Velocity().vx, 0.0f) && motion.Velocity().wz > 0.0f);
        motion.Update({}, {}, 0.01f, false);
        CHECK(motion.Faulted() && motion.Velocity().wz == 0.0f);
        motion.Update({}, {}, 0.01f, true);
        CHECK(motion.Faulted());
        motion.Start({}, {}, tune);
        for (uint32_t i = 0U; i < 19U; ++i)
        {
            motion.Update({}, {}, 0.01f, true);
        }
        CHECK(!motion.Finished());
        motion.Update({}, {}, 0.02f, true);
        CHECK(motion.Finished());

        AutoModeController route;
        route.Reset(tune);
        CHECK(!route.Update({}, {}, 0.01f, false).enabled);
        CHECK(!route.Update({}, {}, 0.01f, true).enabled);
        CHECK(route.Update({}, {}, 0.01f, true).enabled);
        CHECK(!route.Update({}, {}, 0.01f, false).enabled);
        CHECK(!route.Update({}, {}, 0.01f, true).enabled);
        route.Reset(tune);
        for (uint32_t pointIndex = 0U; pointIndex < AutoRouteConfig::POINT_COUNT; ++pointIndex)
        {
            const AutoRoutePoint& point = AutoRouteConfig::POINTS[pointIndex];
            const ChassisPose2D pose = {point.x, point.y, MathUtils::DegToRad(point.yawDeg)};
            CHECK(!route.Update(pose, {}, 0.01f, true).enabled);
            for (uint32_t i = 0U; i < 25U; ++i)
            {
                const AutoModeOutput output = route.Update(pose, {}, 0.01f, true);
                if (!output.enabled)
                {
                    break;
                }
            }
            CHECK(route.RouteState().currentPointIndex == pointIndex + 1U);
        }
        CHECK(route.RouteState().finished && !route.Update({}, {}, 0.01f, true).enabled);
    }

    void TestAutoTracking()
    {
        const AutoMoveTune tune = AutoMoveConfig::LIDAR_DEFAULT;
        AutoMove motion;
        ChassisPose2D pose = {0.0f, 0.0f, MathUtils::PI / 2.0f};
        ChassisBodyVelocity measured = {};
        motion.Start(pose, {3.0f, 0.0f, 0.0f}, tune);
        bool bounded = true;
        bool rateLimited = true;
        float lastWorldVx = 0.0f;
        float lastWorldVy = 0.0f;
        constexpr float DT = 0.01f;
        for (uint32_t i = 0U; i < 6000U && !motion.Finished(); ++i)
        {
            motion.Update(pose, measured, DT, true);
            measured = motion.Velocity();
            const float worldVx = std::cos(pose.yaw) * measured.vx - std::sin(pose.yaw) * measured.vy;
            const float worldVy = std::sin(pose.yaw) * measured.vx + std::cos(pose.yaw) * measured.vy;
            bounded = bounded && std::hypot(measured.vx, measured.vy) <= tune.maxOutputVelocity + 0.0001f &&
                      std::fabs(measured.wz) <= MathUtils::DegToRad(tune.maxYawVelocityDegPerSec) + 0.0001f;
            rateLimited = rateLimited && std::hypot(worldVx - lastWorldVx, worldVy - lastWorldVy) <=
                                             std::fmax(tune.acceleration, tune.deceleration) * DT + 0.0001f;
            pose.x += worldVx * DT;
            pose.y += worldVy * DT;
            pose.yaw += measured.wz * DT;
            lastWorldVx = worldVx;
            lastWorldVy = worldVy;
        }
        CHECK(motion.Finished());
        CHECK(bounded && rateLimited);
        CHECK(Near(pose.x, 3.0f, tune.positionTolerance) && Near(pose.y, 0.0f, tune.positionTolerance));
        CHECK(std::fabs(pose.yaw) <= MathUtils::DegToRad(tune.yawToleranceDeg));
        AutoMoveTune invalid = tune;
        invalid.maxVelocity = std::numeric_limits<float>::infinity();
        CHECK(!AutoMoveTuneValid(invalid));
        invalid = tune;
        invalid.maxOutputVelocity = tune.maxVelocity * 0.5f;
        CHECK(!AutoMoveTuneValid(invalid));
    }

    void TestLidar()
    {
        LidarPoseProtocol protocol;
        uint8_t frame[15];
        MakeFrame(frame, 1.0f, 2.0f, 0.0f);
        CHECK(!protocol.FeedByte(0xAAU, 1U));
        for (uint32_t i = 0U; i < 14U; ++i)
        {
            CHECK(!protocol.FeedByte(frame[i], 2U));
        }
        CHECK(protocol.FeedByte(frame[14], 3U));
        CHECK(protocol.State().receiveTick == 3U && Near(protocol.State().radarPose.y, 2.0f));
        frame[14] ^= 1U;
        for (uint8_t byte : frame)
        {
            CHECK(!protocol.FeedByte(byte, 4U));
        }
        MakeFrame(frame, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
        for (uint8_t byte : frame)
        {
            CHECK(!protocol.FeedByte(byte, 5U));
        }
        CHECK(protocol.State().invalidValueCount == 1U);
        CHECK(protocol.State().frameCount == 1U);

        LidarLocalization lidar;
        lidar.Reset();
        nowTick = 1000U;
        QueuePose(0.0f, 0.0f, 0.0f, nowTick);
        lidar.Update();
        CHECK(lidar.Feedback().valid && Near(lidar.Feedback().pose.x, 0.0f));
        CHECK(Near(lidar.OriginState().initialCenterWorldX, -0.4636f));
        const float quarterTurn = MathUtils::PI / 2.0f;
        QueuePose(-0.4796f, 0.4476f, quarterTurn, nowTick);
        lidar.Update();
        CHECK(Near(lidar.Feedback().pose.x, 0.0f) && Near(lidar.Feedback().pose.y, 0.0f));
        CHECK(Near(lidar.Feedback().pose.yaw, quarterTurn));
        nowTick += LocalizationConfig::LIDAR_TIMEOUT_MS + 1U;
        lidar.Update();
        CHECK(!lidar.Feedback().valid);
        QueuePose(1.0f, 0.0f, 0.0f, 1000U);
        lidar.Update();
        CHECK(!lidar.Feedback().valid);
        resetStream = true;
        usbConfigured = false;
        lidar.Update();
        CHECK(!lidar.Feedback().valid && lidar.OriginState().initialized);
        usbConfigured = true;
        QueuePose(2.0f, 0.0f, 0.0f, nowTick);
        lidar.Update();
        CHECK(lidar.Feedback().valid && Near(lidar.Feedback().pose.x, 2.0f));
        lidar.Reset();
        QueuePose(5.0f, 7.0f, quarterTurn, nowTick);
        lidar.Update();
        CHECK(Near(lidar.Feedback().pose.x, 0.0f) && Near(lidar.Feedback().pose.yaw, quarterTurn));
        nowTick = 5U;
        QueuePose(6.0f, 7.0f, quarterTurn, 0xFFFFFFFBU);
        lidar.Update();
        CHECK(lidar.Feedback().valid && Near(lidar.Feedback().pose.x, 1.0f));
    }

    void TestOdometry()
    {
        ChassisOdometry odometry;
        odometry.Reset();
        OdometryInput input = {{1.0f, 0.0f, 0.2f}, 179.0f, 0.01f, 1U, true, true};
        odometry.Update(input);
        CHECK(odometry.Feedback().valid && odometry.Feedback().pose.x == 0.0f);
        input.imuYawDeg = -179.0f;
        odometry.Update(input);
        CHECK(Near(odometry.Feedback().pose.yaw, MathUtils::DegToRad(2.0f)));
        input.imuOnline = false;
        odometry.Update(input);
        const float reconnectYaw = odometry.Feedback().pose.yaw + 0.002f;
        input.imuOnline = true;
        input.imuYawDeg = 90.0f;
        odometry.Update(input);
        CHECK(Near(odometry.Feedback().pose.yaw, reconnectYaw));
        input.wheelFeedbackValid = false;
        odometry.Update(input);
        CHECK(!odometry.Feedback().valid);
        input.wheelFeedbackValid = true;
        odometry.Update(input);
        CHECK(!odometry.Feedback().valid);
        odometry.Reset();
        odometry.Update(input);
        CHECK(odometry.Feedback().valid && odometry.Feedback().pose.x == 0.0f);
    }

    void Feedback(uint32_t index, uint32_t tick)
    {
        const uint16_t encoder = static_cast<uint16_t>(RobotConfig::GEOMETRY.zeroEncoder[index]);
        const uint8_t steer[] = {
            static_cast<uint8_t>(encoder >> 8U), static_cast<uint8_t>(encoder), 0U, 0U, 0U, 0U, 30U, 0U};
        Dji6020Bus::ParseFeedback(0x204U + RobotConfig::DJI_MOTOR_ID[index], steer, tick);
        const uint8_t wheel[8] = {};
        VescMotor::ParseCanFeedback((9U << 8U) | RobotConfig::VESC_NODE_ID[index], wheel, tick);
    }

    bool AllSentZero()
    {
        for (uint32_t i = 0U; i < sentCount; ++i)
        {
            for (uint8_t byte : sent[i].bytes)
            {
                if (byte != 0U)
                {
                    return false;
                }
            }
        }
        return true;
    }

    void TestChassisAndCanProtocols()
    {
        static Chassis chassis;
        CHECK(chassis.Init());
        nowTick = 10U;
        for (uint32_t i = 0U; i < 3U; ++i)
        {
            Feedback(i, nowTick);
        }
        chassis.UpdateFeedback();
        CHECK(chassis.HasMeasuredVelocity());
        CHECK(chassis.SetVelocity({1.0f, 0.0f, 0.0f}));
        sentCount = 0U;
        chassis.UpdateControl(1.0f);
        CHECK(!AllSentZero() && chassis.TransmitOk());
        chassis.Stop();
        sentCount = 0U;
        chassis.UpdateControl(1.0f);
        CHECK(AllSentZero() && sentCount == 4U);

        const uint8_t invalidEncoder[8] = {0x7FU, 0xFFU, 0U, 0U, 0U, 0U, 0U, 0U};
        Dji6020Bus::ParseFeedback(0x205U, invalidEncoder, nowTick);
        chassis.UpdateFeedback();
        CHECK(Near(chassis.MeasuredVelocity().vx, 0.0f));
        nowTick += 101U;
        // 只刷新两个电机，任何一个轮组离线都必须停止整车
        Feedback(0U, nowTick);
        Feedback(1U, nowTick);
        chassis.UpdateFeedback();
        CHECK(!chassis.HasMeasuredVelocity());
        CHECK(chassis.SetVelocity({1.0f, 0.0f, 0.0f}));
        sentCount = 0U;
        chassis.UpdateControl(1.0f);
        CHECK(AllSentZero());
        CHECK(!chassis.SetVelocity({std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}));
        sendOk = false;
        sentCount = 0U;
        chassis.UpdateControl(1.0f);
        CHECK(!chassis.TransmitOk() && sentCount == 4U);
        sendOk = true;

        for (uint32_t i = 0U; i < 3U; ++i)
        {
            Feedback(i, nowTick);
        }
        chassis.UpdateFeedback();
        PidParam angle = RobotConfig::STEER_ANGLE;
        angle.kp = 1.0f;
        angle.kd = 0.0f;
        PidParam speed = RobotConfig::STEER_SPEED;
        speed.kp = std::numeric_limits<float>::max();
        speed.ki = 0.0f;
        speed.kd = std::numeric_limits<float>::max();
        CHECK(chassis.ApplySteerPidParam(angle, speed));
        CHECK(chassis.SetVelocity({0.0f, 1.0f, 0.0f}));
        chassis.UpdateControl(1.0f);
        CHECK(chassis.SetVelocity({1.0f, 1.0f, 0.0f}));
        sentCount = 0U;
        chassis.UpdateControl(1.0f);
        CHECK(!chassis.OutputValid() && chassis.TransmitOk() && AllSentZero());

        static Dji6020Motor steerMotor;
        steerMotor.Init(2U);
        const uint8_t validSteer[8] = {0x1FU, 0xFEU, 0xFFU, 0x9CU, 0U, 0U, 30U, 0U};
        Dji6020Bus::ParseFeedback(0x206U, validSteer, 0xFFFFFFFBU);
        Dji6020Bus::ParseFeedback(0x206U, invalidEncoder, 5U);
        CHECK(steerMotor.GetRxData(5U).encoder == 8190 && steerMotor.GetRxData(5U).speedRpm == -100);
        CHECK(steerMotor.GetRxData(5U).lastRxTick == 0xFFFFFFFBU);
        CHECK(steerMotor.GetRxData(5U).online && !steerMotor.GetRxData(200U).online);

        static VescMotor motor;
        motor.Init(CanBus::Wheel, 12U, 14U);
        sentCount = 0U;
        CHECK(motor.SetRpm(-100));
        CHECK(sent[0].id == 0x30CU && sent[0].bytes[0] == 0xFFU && sent[0].bytes[3] == 0x88U);
        const uint8_t status[8] = {0xFFU, 0xFFU, 0xFAU, 0x88U, 0xFFU, 0x9CU, 0U, 0U};
        VescMotor::ParseCanFeedback(0x90CU, status, 0xFFFFFFFBU);
        CHECK(Near(motor.GetRxData(5U).rpm, -100.0f));
        CHECK(Near(motor.GetRxData(5U).totalCurrent, -10.0f));
        CHECK(motor.GetRxData(5U).speedOnline && !motor.GetRxData(200U).speedOnline);
    }
}  // namespace

uint32_t HAL_GetTick()
{
    return nowTick;
}
bool BspCan::Init()
{
    return true;
}
void BspCan::RegisterReceiver(CanBus, Receiver)
{
}
void BspCan::ProcessReceive()
{
}
bool BspCan::Send(CanBus bus, uint32_t id, CanIdType, const uint8_t (&data)[8])
{
    if (sentCount < 32U)
    {
        sent[sentCount].bus = bus;
        sent[sentCount].id = id;
        std::memcpy(sent[sentCount].bytes, data, sizeof(data));
        ++sentCount;
    }
    return sendOk;
}
bool BspUsb::Configured()
{
    return usbConfigured;
}
uint16_t BspUsb::Read(uint8_t* bytes, uint32_t* ticks, uint16_t capacity, bool& reset)
{
    reset = resetStream;
    resetStream = false;
    const uint16_t size = usbSize < capacity ? usbSize : capacity;
    std::memcpy(bytes, usbBytes, size);
    std::memcpy(ticks, usbTicks, size * sizeof(uint32_t));
    std::memmove(usbBytes, usbBytes + size, usbSize - size);
    std::memmove(usbTicks, usbTicks + size, (usbSize - size) * sizeof(uint32_t));
    usbSize -= size;
    return size;
}

int main()
{
    TestPidAndProfile();
    TestKinematics();
    TestAuto();
    TestAutoTracking();
    TestLidar();
    TestOdometry();
    TestChassisAndCanProtocols();
    std::printf("%d checks, %d failures\n", checks, failures);
    return failures == 0 ? 0 : 1;
}
