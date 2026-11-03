/**
 * @file bsp_dwt.cpp
 * @brief CYCCNT 无符号差值支持单次计数回绕
 */
#include "bsp_dwt.h"

#include "stm32h7xx.h"

void DwtTimer::Init(uint32_t cpuFreqMhz)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cpuFreqHz_ = cpuFreqMhz * 1000000U;
}

float DwtTimer::GetDeltaT(uint32_t& lastCount) const
{
    const uint32_t now = DWT->CYCCNT;
    const float dt = static_cast<float>(now - lastCount) / static_cast<float>(cpuFreqHz_);
    lastCount = now;
    return dt;
}
