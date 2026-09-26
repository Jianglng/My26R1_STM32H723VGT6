#include "bsp_dwt.h"


void DwtTimer::Init(uint32_t cpu_freq_mhz)
{
    if (cpu_freq_mhz == 0U)
    {
        cpu_freq_mhz = 480U;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cpuFreqHz_ = cpu_freq_mhz * 1000000U;
    cpuFreqHzMs_ = cpuFreqHz_ / 1000U;
    cpuFreqHzUs_ = cpuFreqHz_ / 1000000U;

    cyccntRoundCount_ = 0U;
    cyccntLast_ = DWT->CYCCNT;
    cyccnt64_ = 0U;

    sysTime_.s = 0U;
    sysTime_.ms = 0U;
    sysTime_.us = 0U;
}

float DwtTimer::GetDeltaT(uint32_t *cnt_last)
{
    if (cnt_last == nullptr || cpuFreqHz_ == 0U)
    {
        return 0.001f;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    const float dt = static_cast<float>(cnt_now - *cnt_last)
                   / static_cast<float>(cpuFreqHz_);
    *cnt_last = cnt_now;
    CntUpdate();
    return dt;
}

double DwtTimer::GetDeltaT64(uint32_t *cnt_last)
{
    if (cnt_last == nullptr || cpuFreqHz_ == 0U)
    {
        return 0.001;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    const double dt = static_cast<double>(cnt_now - *cnt_last)
                    / static_cast<double>(cpuFreqHz_);
    *cnt_last = cnt_now;
    CntUpdate();
    return dt;
}

void DwtTimer::SysTimeUpdate()
{
    if (cpuFreqHz_ == 0U)
    {
        return;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    CntUpdate();

    cyccnt64_ = static_cast<uint64_t>(cyccntRoundCount_) * 4294967296ULL + static_cast<uint64_t>(cnt_now);

    const uint64_t sec_part = cyccnt64_ / cpuFreqHz_;
    const uint64_t rem_part = cyccnt64_ - sec_part * cpuFreqHz_;
    const uint64_t ms_part = rem_part / cpuFreqHzMs_;
    const uint64_t rem_part2 = rem_part - ms_part * cpuFreqHzMs_;
    const uint64_t us_part = rem_part2 / cpuFreqHzUs_;

    sysTime_.s = static_cast<uint32_t>(sec_part);
    sysTime_.ms = static_cast<uint16_t>(ms_part);
    sysTime_.us = static_cast<uint16_t>(us_part);
}

float DwtTimer::GetTimelineS()
{
    SysTimeUpdate();
    return static_cast<float>(sysTime_.s)
         + static_cast<float>(sysTime_.ms) * 0.001f
         + static_cast<float>(sysTime_.us) * 0.000001f;
}

float DwtTimer::GetTimelineMs()
{
    SysTimeUpdate();
    return static_cast<float>(sysTime_.s) * 1000.0f
         + static_cast<float>(sysTime_.ms)
         + static_cast<float>(sysTime_.us) * 0.001f;
}

uint64_t DwtTimer::GetTimelineUs()
{
    SysTimeUpdate();
    return static_cast<uint64_t>(sysTime_.s) * 1000000ULL
         + static_cast<uint64_t>(sysTime_.ms) * 1000ULL
         + static_cast<uint64_t>(sysTime_.us);
}

void DwtTimer::DelayS(float seconds)
{
    if (cpuFreqHz_ == 0U || seconds <= 0.0f)
    {
        return;
    }

    const uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_ticks = static_cast<uint32_t>(seconds * static_cast<float>(cpuFreqHz_));
    if (wait_ticks == 0U)
    {
        wait_ticks = 1U;
    }

    while (static_cast<uint32_t>(DWT->CYCCNT - tickstart) < wait_ticks)
    {
    }
}

void DwtTimer::CntUpdate()
{
    const uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < cyccntLast_)
    {
        cyccntRoundCount_++;
    }
    cyccntLast_ = cnt_now;
}
