/**
 * @file bsp_critical.h
 * @brief 保存并恢复 PRIMASK，支持中断和任务上下文中的短临界区
 */
#pragma once

#include "stm32h7xx.h"

class CriticalSection
{
public:
    CriticalSection() : primask_(__get_PRIMASK())
    {
        __disable_irq();
        __DMB();
    }
    ~CriticalSection()
    {
        __DMB();
        __set_PRIMASK(primask_);
    }
    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;

private:
    uint32_t primask_;
};
