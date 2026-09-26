#pragma once

#include "main.h"
#include <stdint.h>

/**
 * @brief DWT 拆出来的日历时间，仅作调试观察。
 */
struct DwtTime
{
    uint32_t s;    ///< 秒。
    uint16_t ms;   ///< 当前秒内的毫秒，0~999。
    uint16_t us;   ///< 当前毫秒内的微秒，0~999。
};

/**
 * @brief 用 Cortex-M7 的 DWT 周期计数器做微秒级计时。
 *
 * CYCCNT 按 CPU 主频计数，本工程为 480 MHz。
 * 必须在 SystemClock_Config() 之后调用 Init()，PID 才能得到正确的 dt。
 * DelayS() 是死等，不要在 FreeRTOS 任务里调用。
 */
class DwtTimer
{
public:
    DwtTime sysTime_;   ///< 可读的秒/毫秒/微秒，调用 getTimeline_* 后更新。

    /**
     * @brief 打开 CYCCNT，并记录 CPU 主频。
     * @param cpu_freq_mhz CPU 频率，单位 MHz。本工程传入 SystemCoreClock / 1000000。
     */
    void Init(uint32_t cpu_freq_mhz);

    /**
     * @brief 距上次调用该计数器快照的时间差。
     * @param cnt_last 调用方自己保存的 CYCCNT，会被更新成当前值。
     * @return 时间差，单位秒。未初始化时返回 0.001。
     */
    float GetDeltaT(uint32_t *cnt_last);

    /** @brief 与 GetDeltaT 相同，返回双精度秒。 */
    double GetDeltaT64(uint32_t *cnt_last);

    /** @brief 刷新 sysTime。 */
    void SysTimeUpdate();

    float GetTimelineS();
    float GetTimelineMs();
    uint64_t GetTimelineUs();

    /** @brief 忙等延时，单位秒。仅供启动阶段使用，任务里不要调用。 */
    void DelayS(float seconds);

private:
    uint32_t cpuFreqHz_ = 0;
    uint32_t cpuFreqHzMs_ = 0;
    uint32_t cpuFreqHzUs_ = 0;
    uint32_t cyccntRoundCount_ = 0;
    uint32_t cyccntLast_ = 0;
    uint64_t cyccnt64_ = 0;

    void CntUpdate();
};

