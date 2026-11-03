/**
 * @file bsp_can.cpp
 * @brief 硬件句柄封装、BUS_OFF 恢复和接收帧缓存
 */
#include "bsp_can.h"

#include <string.h>

#include "fdcan.h"

#include "bsp_critical.h"

namespace
{
    constexpr uint32_t RX_CAPACITY = 32U;
    constexpr uint32_t RX_BUDGET = 32U;
    constexpr uint32_t BUS_COUNT = 2U;
    constexpr uint32_t MAX_CAN_PAYLOAD_SIZE = 64U;

    struct RxFrame
    {
        uint32_t id;
        uint32_t receiveTick;
        CanBus bus;
        uint8_t data[CAN_CLASSIC_PAYLOAD_SIZE];
    };

    RxFrame g_rxFrames[RX_CAPACITY] = {};  // CAN 中断写，底盘任务读取
    // 两路 CAN 中断同为抢占优先级 5，生产者互不嵌套，任务在临界区取帧
    volatile uint32_t g_readIndex = 0U;
    volatile uint32_t g_writeIndex = 0U;
    volatile uint32_t g_queuedFrames = 0U;
    BspCan::Receiver g_receivers[BUS_COUNT] = {};      // 初始化写，底盘任务读
    volatile CanBusState g_busStates[BUS_COUNT] = {};  // 中断和任务更新，读取时保护

    FDCAN_HandleTypeDef& Handle(CanBus bus)
    {
        return bus == CanBus::Wheel ? hfdcan1 : hfdcan3;
    }

    bool ConfigureBus(CanBus bus)
    {
        FDCAN_HandleTypeDef& handle = Handle(bus);
        const bool wheel = bus == CanBus::Wheel;
        const uint32_t interrupt = wheel ? FDCAN_IT_RX_FIFO1_NEW_MESSAGE : FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
        FDCAN_FilterTypeDef filter = {};
        filter.IdType = wheel ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
        filter.FilterIndex = 0U;
        filter.FilterType = FDCAN_FILTER_MASK;
        filter.FilterConfig = wheel ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0;
        return HAL_FDCAN_ConfigFilter(&handle, &filter) == HAL_OK &&
               HAL_FDCAN_ConfigGlobalFilter(&handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE,
                                            FDCAN_REJECT_REMOTE) == HAL_OK &&
               HAL_FDCAN_ConfigInterruptLines(&handle, interrupt,
                                              wheel ? FDCAN_INTERRUPT_LINE1 : FDCAN_INTERRUPT_LINE0) == HAL_OK &&
               HAL_FDCAN_ActivateNotification(&handle, interrupt, 0U) == HAL_OK && HAL_FDCAN_Start(&handle) == HAL_OK;
    }

    bool Recover(CanBus bus)
    {
        FDCAN_HandleTypeDef& handle = Handle(bus);
        if ((handle.Instance->IR & FDCAN_FLAG_BUS_OFF) == 0U)
        {
            return true;
        }
        if (HAL_FDCAN_Stop(&handle) != HAL_OK)
        {
            return false;
        }
        handle.Instance->IR = FDCAN_FLAG_BUS_OFF;
        ++g_busStates[static_cast<uint32_t>(bus)].recoveryCount;
        return HAL_FDCAN_Start(&handle) == HAL_OK;
    }

    void Receive(CanBus bus, uint32_t fifo)
    {
        // FIFO 深度有限，取出本次中断已有的帧，避免单帧回调遗漏积压数据
        FDCAN_HandleTypeDef& handle = Handle(bus);
        const uint32_t count = HAL_FDCAN_GetRxFifoFillLevel(&handle, fifo);
        for (uint32_t i = 0U; i < count; ++i)
        {
            FDCAN_RxHeaderTypeDef header = {};
            RxFrame frame = {};
            // HAL 按收到的 DLC 写入，先容纳完整 CAN FD 负载，再筛选经典帧
            uint8_t payload[MAX_CAN_PAYLOAD_SIZE] = {};
            if (HAL_FDCAN_GetRxMessage(&handle, fifo, &header, payload) != HAL_OK)
            {
                ++g_busStates[static_cast<uint32_t>(bus)].receiveErrorCount;
                return;
            }
            const uint32_t expectedType = bus == CanBus::Wheel ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
            if (header.IdType != expectedType || header.RxFrameType != FDCAN_DATA_FRAME ||
                header.DataLength != FDCAN_DLC_BYTES_8)
            {
                continue;
            }
            if (g_queuedFrames == RX_CAPACITY)
            {
                ++g_busStates[static_cast<uint32_t>(bus)].receiveOverflowCount;
                continue;
            }
            frame.id = header.Identifier;
            frame.receiveTick = HAL_GetTick();
            frame.bus = bus;
            memcpy(frame.data, payload, CAN_CLASSIC_PAYLOAD_SIZE);
            g_rxFrames[g_writeIndex] = frame;
            g_writeIndex = (g_writeIndex + 1U) % RX_CAPACITY;
            ++g_queuedFrames;
        }
    }

    bool Pop(RxFrame& frame)
    {
        CriticalSection lock;
        if (g_queuedFrames == 0U)
        {
            return false;
        }
        frame = g_rxFrames[g_readIndex];
        g_readIndex = (g_readIndex + 1U) % RX_CAPACITY;
        --g_queuedFrames;
        return true;
    }
}  // namespace

bool BspCan::Init()
{
    return ConfigureBus(CanBus::Wheel) && ConfigureBus(CanBus::Steer);
}

void BspCan::RegisterReceiver(CanBus bus, Receiver receiver)
{
    g_receivers[static_cast<uint32_t>(bus)] = receiver;
}

bool BspCan::Send(CanBus bus, uint32_t id, CanIdType type, const uint8_t (&data)[CAN_CLASSIC_PAYLOAD_SIZE])
{
    FDCAN_TxHeaderTypeDef header = {};
    header.Identifier = id;
    header.IdType = type == CanIdType::Extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    const bool sent = Recover(bus) && HAL_FDCAN_AddMessageToTxFifoQ(&Handle(bus), &header, data) == HAL_OK;
    if (!sent)
    {
        ++g_busStates[static_cast<uint32_t>(bus)].transmitErrorCount;
    }
    return sent;
}

void BspCan::ProcessReceive()
{
    for (uint32_t i = 0U; i < RX_BUDGET; ++i)
    {
        RxFrame frame = {};
        if (!Pop(frame))
        {
            break;
        }
        Receiver receiver = g_receivers[static_cast<uint32_t>(frame.bus)];
        if (receiver != nullptr)
        {
            receiver(frame.id, frame.data, frame.receiveTick);
        }
    }
}

CanBusState BspCan::State(CanBus bus)
{
    CriticalSection lock;
    const volatile CanBusState& source = g_busStates[static_cast<uint32_t>(bus)];
    CanBusState state = {};
    state.transmitErrorCount = source.transmitErrorCount;
    state.recoveryCount = source.recoveryCount;
    state.receiveOverflowCount = source.receiveOverflowCount;
    state.receiveErrorCount = source.receiveErrorCount;
    return state;
}

extern "C" void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* handle, uint32_t events)
{
    if (handle == &hfdcan1 && (events & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U)
    {
        Receive(CanBus::Wheel, FDCAN_RX_FIFO1);
    }
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* handle, uint32_t events)
{
    if (handle == &hfdcan3 && (events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        Receive(CanBus::Steer, FDCAN_RX_FIFO0);
    }
}
