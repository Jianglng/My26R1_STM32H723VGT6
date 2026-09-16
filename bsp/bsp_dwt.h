#pragma once

#include "main.h"
#include <stdint.h>

/**
 * @brief DWT 拆出来的日历时间，仅作调试观察。
 */
struct DWT_Time_t
{
    uint32_t s;    ///< 秒。
    uint16_t ms;   ///< 当前秒内的毫秒，0~999。
    uint16_t us;   ///< 当前毫秒内的微秒，0~999。
};

/**
 * @brief 用 Cortex-M7 的 DWT 周期计数器做微秒级计时。
 *
 * CYCCNT 按 CPU 主频计数，本工程为 480 MHz。
 * 必须在 SystemClock_Config() 之后调用 init()，PID 才能得到正确的 dt。
 * delay_s() 是死等，不要在 FreeRTOS 任务里调用。
 */
class DWT_Timer
{
public:
    DWT_Time_t sysTime;   ///< 可读的秒/毫秒/微秒，调用 getTimeline_* 后更新。

    /**
     * @brief 打开 CYCCNT，并记录 CPU 主频。
     * @param cpu_freq_mhz CPU 频率，单位 MHz。本工程传入 SystemCoreClock / 1000000。
     */
    void init(uint32_t cpu_freq_mhz);

    /**
     * @brief 距上次调用该计数器快照的时间差。
     * @param cnt_last 调用方自己保存的 CYCCNT，会被更新成当前值。
     * @return 时间差，单位秒。未初始化时返回 0.001。
     */
    float getDeltaT(uint32_t *cnt_last);

    /** @brief 与 getDeltaT 相同，返回双精度秒。 */
    double getDeltaT64(uint32_t *cnt_last);

    /** @brief 刷新 sysTime。 */
    void sysTimeUpdate();

    float getTimeline_s();
    float getTimeline_ms();
    uint64_t getTimeline_us();

    /** @brief 忙等延时，单位秒。仅供启动阶段使用，任务里不要调用。 */
    void delay_s(float delay_s);

private:
    uint32_t cpu_freq_hz_ = 0;
    uint32_t cpu_freq_hz_ms_ = 0;
    uint32_t cpu_freq_hz_us_ = 0;
    uint32_t cyccnt_round_count_ = 0;
    uint32_t cyccnt_last_ = 0;
    uint64_t cyccnt64_ = 0;

    void cntUpdate();
};

extern DWT_Timer DWT_;
