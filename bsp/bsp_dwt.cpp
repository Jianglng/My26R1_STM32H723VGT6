#include "bsp_dwt.h"

DWT_Timer DWT_;

void DWT_Timer::init(uint32_t cpu_freq_mhz)
{
    if (cpu_freq_mhz == 0U)
    {
        cpu_freq_mhz = 480U;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    cpu_freq_hz_ = cpu_freq_mhz * 1000000U;
    cpu_freq_hz_ms_ = cpu_freq_hz_ / 1000U;
    cpu_freq_hz_us_ = cpu_freq_hz_ / 1000000U;

    cyccnt_round_count_ = 0U;
    cyccnt_last_ = DWT->CYCCNT;
    cyccnt64_ = 0U;

    sysTime.s = 0U;
    sysTime.ms = 0U;
    sysTime.us = 0U;
}

float DWT_Timer::getDeltaT(uint32_t *cnt_last)
{
    if (cnt_last == nullptr || cpu_freq_hz_ == 0U)
    {
        return 0.001f;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    const float dt = static_cast<float>(cnt_now - *cnt_last)
                   / static_cast<float>(cpu_freq_hz_);
    *cnt_last = cnt_now;
    cntUpdate();
    return dt;
}

double DWT_Timer::getDeltaT64(uint32_t *cnt_last)
{
    if (cnt_last == nullptr || cpu_freq_hz_ == 0U)
    {
        return 0.001;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    const double dt = static_cast<double>(cnt_now - *cnt_last)
                    / static_cast<double>(cpu_freq_hz_);
    *cnt_last = cnt_now;
    cntUpdate();
    return dt;
}

void DWT_Timer::sysTimeUpdate()
{
    if (cpu_freq_hz_ == 0U)
    {
        return;
    }

    const uint32_t cnt_now = DWT->CYCCNT;
    cntUpdate();

    cyccnt64_ = (uint64_t)cyccnt_round_count_ * 4294967296ULL + (uint64_t)cnt_now;

    const uint64_t sec_part = cyccnt64_ / cpu_freq_hz_;
    const uint64_t rem_part = cyccnt64_ - sec_part * cpu_freq_hz_;
    const uint64_t ms_part = rem_part / cpu_freq_hz_ms_;
    const uint64_t rem_part2 = rem_part - ms_part * cpu_freq_hz_ms_;
    const uint64_t us_part = rem_part2 / cpu_freq_hz_us_;

    sysTime.s = (uint32_t)sec_part;
    sysTime.ms = (uint16_t)ms_part;
    sysTime.us = (uint16_t)us_part;
}

float DWT_Timer::getTimeline_s()
{
    sysTimeUpdate();
    return (float)sysTime.s
         + (float)sysTime.ms * 0.001f
         + (float)sysTime.us * 0.000001f;
}

float DWT_Timer::getTimeline_ms()
{
    sysTimeUpdate();
    return (float)sysTime.s * 1000.0f
         + (float)sysTime.ms
         + (float)sysTime.us * 0.001f;
}

uint64_t DWT_Timer::getTimeline_us()
{
    sysTimeUpdate();
    return (uint64_t)sysTime.s * 1000000ULL
         + (uint64_t)sysTime.ms * 1000ULL
         + (uint64_t)sysTime.us;
}

void DWT_Timer::delay_s(float delay_s)
{
    if (cpu_freq_hz_ == 0U || delay_s <= 0.0f)
    {
        return;
    }

    const uint32_t tickstart = DWT->CYCCNT;
    uint32_t wait_ticks = (uint32_t)(delay_s * (float)cpu_freq_hz_);
    if (wait_ticks == 0U)
    {
        wait_ticks = 1U;
    }

    while ((uint32_t)(DWT->CYCCNT - tickstart) < wait_ticks)
    {
    }
}

void DWT_Timer::cntUpdate()
{
    const uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < cyccnt_last_)
    {
        cyccnt_round_count_++;
    }
    cyccnt_last_ = cnt_now;
}
