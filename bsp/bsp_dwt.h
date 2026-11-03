/**
 * @file bsp_dwt.h
 * @brief DWT 周期计数器，应用层每拍测量一次周期
 */
#pragma once

#include <stdint.h>

class DwtTimer
{
public:
    /** @brief 时钟初始化后调用，cpuFreqMhz 必须为实际非零 CPU 主频 */
    void Init(uint32_t cpuFreqMhz);
    /** @brief 更新调用者快照并返回秒，快照间隔须小于一次 32 位 CYCCNT 回绕 */
    float GetDeltaT(uint32_t& lastCount) const;

private:
    uint32_t cpuFreqHz_ = 0U;
};
