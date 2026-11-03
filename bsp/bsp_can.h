/**
 * @file bsp_can.h
 * @brief CAN 发送与接收队列，中断只缓存原始帧，任务负责协议分发
 */
#pragma once

#include <stdint.h>

constexpr uint32_t CAN_CLASSIC_PAYLOAD_SIZE = 8U;

enum class CanBus : uint8_t
{
    Wheel,
    Steer
};
enum class CanIdType : uint8_t
{
    Standard,
    Extended
};

struct CanBusState
{
    uint32_t transmitErrorCount = 0U;
    uint32_t recoveryCount = 0U;
    uint32_t receiveOverflowCount = 0U;
    uint32_t receiveErrorCount = 0U;
};

class BspCan
{
public:
    using Receiver = void (*)(uint32_t id, const uint8_t* data, uint32_t receiveTick);
    /** @brief 初始化应用配置的两路 CAN，失败返回 false，调用者保持停机 */
    static bool Init();
    /** @brief 初始化阶段登记协议接收器，不涉及硬件启动 */
    static void RegisterReceiver(CanBus bus, Receiver receiver);
    /** @brief 仅底盘任务调用，所有负载固定为经典 CAN 的 8 字节 */
    static bool Send(CanBus bus, uint32_t id, CanIdType type,
                     const uint8_t (&data)[CAN_CLASSIC_PAYLOAD_SIZE]);
    /** @brief 每拍限量分发接收队列，不等待新数据 */
    static void ProcessReceive();
    static CanBusState State(CanBus bus);
};
