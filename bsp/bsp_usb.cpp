/**
 * @file bsp_usb.cpp
 * @brief USB 字节缓存与接收时间记录
 */
#include "bsp_usb.h"

#include "main.h"
#include "usb_device.h"

#include "bsp_critical.h"

namespace
{
    constexpr uint16_t USB_RX_RING_SIZE = 1024U;

    // USB 中断写入，底盘任务在临界区读取字节和接收时间
    uint8_t g_usbRxBytes[USB_RX_RING_SIZE] = {};
    uint32_t g_usbRxTicks[USB_RX_RING_SIZE] = {};
    volatile uint16_t g_usbRxReadIndex = 0U;
    volatile uint16_t g_usbRxWriteIndex = 0U;
    volatile uint16_t g_usbRxUsed = 0U;
    volatile bool g_usbRxStreamReset = false;
    volatile BspUsbRxState g_usbRxState = {};
}  // namespace

extern "C" void BspUsb_ResetReceive(uint8_t configured)
{
    /* USB 初始化、反初始化均经此入口，不能将跨连接的半帧拼在一起 */
    CriticalSection lock;
    g_usbRxReadIndex = 0U;
    g_usbRxWriteIndex = 0U;
    g_usbRxUsed = 0U;
    g_usbRxStreamReset = true;
    g_usbRxState.configured = configured;
    __DMB();
}

extern "C" void BspUsb_ReceiveBytes(const uint8_t* bytes, uint32_t length)
{
    if (bytes == nullptr || length == 0U)
    {
        return;
    }

    /* 本入口只在 USB 接收回调执行；任务读取时用短临界区保护缓存索引 */
    ++g_usbRxState.packetCount;
    g_usbRxState.byteCount += length;
    g_usbRxState.lastPacketLength = length;
    if (length > static_cast<uint32_t>(USB_RX_RING_SIZE - g_usbRxUsed))
    {
        ++g_usbRxState.overflowCount;
        g_usbRxState.droppedByteCount += g_usbRxUsed + length;
        g_usbRxReadIndex = 0U;
        g_usbRxWriteIndex = 0U;
        g_usbRxUsed = 0U;
        g_usbRxStreamReset = true;
        return;
    }

    const uint32_t receiveTick = HAL_GetTick();
    for (uint32_t i = 0U; i < length; ++i)
    {
        g_usbRxBytes[g_usbRxWriteIndex] = bytes[i];
        g_usbRxTicks[g_usbRxWriteIndex] = receiveTick;
        g_usbRxWriteIndex = (g_usbRxWriteIndex + 1U) % USB_RX_RING_SIZE;
    }
    g_usbRxUsed += static_cast<uint16_t>(length);
}

namespace BspUsb
{
    void Init()
    {
        BspUsb_ResetReceive(0U);
        MX_USB_DEVICE_Init();
    }

    uint16_t Read(uint8_t* bytes, uint32_t* receiveTicks, uint16_t capacity, bool& streamReset)
    {
        streamReset = false;
        if (bytes == nullptr || receiveTicks == nullptr)
        {
            return 0U;
        }

        CriticalSection lock;
        streamReset = g_usbRxStreamReset;
        g_usbRxStreamReset = false;
        const uint16_t count = g_usbRxUsed < capacity ? g_usbRxUsed : capacity;
        for (uint16_t i = 0U; i < count; ++i)
        {
            bytes[i] = g_usbRxBytes[g_usbRxReadIndex];
            receiveTicks[i] = g_usbRxTicks[g_usbRxReadIndex];
            g_usbRxReadIndex = (g_usbRxReadIndex + 1U) % USB_RX_RING_SIZE;
        }
        g_usbRxUsed -= count;
        __DMB();
        return count;
    }

    bool Configured()
    {
        CriticalSection lock;
        const bool configured = g_usbRxState.configured != 0U && hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
        return configured;
    }

    BspUsbRxState RxState()
    {
        CriticalSection lock;
        BspUsbRxState state = {};
        state.packetCount = g_usbRxState.packetCount;
        state.byteCount = g_usbRxState.byteCount;
        state.lastPacketLength = g_usbRxState.lastPacketLength;
        state.overflowCount = g_usbRxState.overflowCount;
        state.droppedByteCount = g_usbRxState.droppedByteCount;
        state.queuedBytes = g_usbRxUsed;
        state.configured = Configured() ? 1U : 0U;
        return state;
    }
}  // namespace BspUsb
