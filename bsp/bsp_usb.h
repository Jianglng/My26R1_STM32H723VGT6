#pragma once

#include <stdint.h>

/** @brief USB 传输层统计，不包含任何雷达协议含义。 */
struct BspUsbRxState
{
    uint32_t packetCount;
    uint32_t byteCount;
    uint32_t lastPacketLength;
    uint32_t overflowCount;
    uint32_t droppedByteCount;
    uint16_t queuedBytes;
    uint8_t configured;
};

#ifdef __cplusplus
extern "C"
{
#endif
    /* 只由 CubeMX USB 接口调用；回调只缓存字节，不解析协议。 */
    void BspUsb_ResetReceive(uint8_t configured);
    void BspUsb_ReceiveBytes(const uint8_t *bytes, uint32_t length);
#ifdef __cplusplus
}

namespace BspUsb
{
    /** @brief 在底盘任务初始化时调用一次，代替 defaultTask 中的 USB 初始化。 */
    void Init();

    /**
     * @brief 非阻塞读取已有字节，仅允许底盘任务一个读取者。
     * @param receiveTicks 每个字节的实际 USB 接收时间，避免积压数据被误判为新数据。
     * @param streamReset 断连、重新枚举或溢出后为 true，调用者须清除协议半帧。
     */
    uint16_t Read(uint8_t *bytes, uint32_t *receiveTicks,
                  uint16_t capacity, bool &streamReset);
    bool Configured();
    BspUsbRxState RxState();
}
#endif
