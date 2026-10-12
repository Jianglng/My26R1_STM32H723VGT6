#include "bsp_usb.h"

#include "main.h"
#include "usb_device.h"

extern "C" USBD_HandleTypeDef hUsbDeviceHS;

namespace
{
    constexpr uint16_t USB_RX_RING_SIZE = 1024U;

    uint8_t usbRxBytes[USB_RX_RING_SIZE] = {};
    uint32_t usbRxTicks[USB_RX_RING_SIZE] = {};
    volatile uint16_t usbRxReadIndex = 0U;
    volatile uint16_t usbRxWriteIndex = 0U;
    volatile uint16_t usbRxUsed = 0U;
    volatile bool usbRxStreamReset = false;
    volatile BspUsbRxState usbRxState = {};
}

extern "C" void BspUsb_ResetReceive(uint8_t configured)
{
    /* USB 初始化、反初始化均经此入口，不能将跨连接的半帧拼在一起。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    usbRxReadIndex = 0U;
    usbRxWriteIndex = 0U;
    usbRxUsed = 0U;
    usbRxStreamReset = true;
    usbRxState.configured = configured;
    __DMB();
    __set_PRIMASK(primask);
}

extern "C" void BspUsb_ReceiveBytes(const uint8_t *bytes, uint32_t length)
{
    if (bytes == nullptr || length == 0U)
    {
        return;
    }

    /* 本入口只在 USB 接收回调执行；任务读取时用短临界区保护缓存索引。 */
    ++usbRxState.packetCount;
    usbRxState.byteCount += length;
    usbRxState.lastPacketLength = length;
    if (length > static_cast<uint32_t>(USB_RX_RING_SIZE - usbRxUsed))
    {
        ++usbRxState.overflowCount;
        usbRxState.droppedByteCount += usbRxUsed + length;
        usbRxReadIndex = 0U;
        usbRxWriteIndex = 0U;
        usbRxUsed = 0U;
        usbRxStreamReset = true;
        return;
    }

    const uint32_t receiveTick = HAL_GetTick();
    for (uint32_t i = 0U; i < length; ++i)
    {
        usbRxBytes[usbRxWriteIndex] = bytes[i];
        usbRxTicks[usbRxWriteIndex] = receiveTick;
        usbRxWriteIndex = (usbRxWriteIndex + 1U) % USB_RX_RING_SIZE;
    }
    usbRxUsed += static_cast<uint16_t>(length);
}

namespace BspUsb
{
    void Init()
    {
        BspUsb_ResetReceive(0U);
        MX_USB_DEVICE_Init();
    }

    uint16_t Read(uint8_t *bytes, uint32_t *receiveTicks,
                  uint16_t capacity, bool &streamReset)
    {
        streamReset = false;
        if (bytes == nullptr || receiveTicks == nullptr)
        {
            return 0U;
        }

        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        streamReset = usbRxStreamReset;
        usbRxStreamReset = false;
        const uint16_t count = usbRxUsed < capacity ? usbRxUsed : capacity;
        for (uint16_t i = 0U; i < count; ++i)
        {
            bytes[i] = usbRxBytes[usbRxReadIndex];
            receiveTicks[i] = usbRxTicks[usbRxReadIndex];
            usbRxReadIndex = (usbRxReadIndex + 1U) % USB_RX_RING_SIZE;
        }
        usbRxUsed -= count;
        __DMB();
        __set_PRIMASK(primask);
        return count;
    }

    bool Configured()
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        const bool configured = usbRxState.configured != 0U &&
                                hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED;
        __set_PRIMASK(primask);
        return configured;
    }

    BspUsbRxState RxState()
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        BspUsbRxState state = {};
        state.packetCount = usbRxState.packetCount;
        state.byteCount = usbRxState.byteCount;
        state.lastPacketLength = usbRxState.lastPacketLength;
        state.overflowCount = usbRxState.overflowCount;
        state.droppedByteCount = usbRxState.droppedByteCount;
        state.queuedBytes = usbRxUsed;
        state.configured = Configured() ? 1U : 0U;
        __set_PRIMASK(primask);
        return state;
    }
}
