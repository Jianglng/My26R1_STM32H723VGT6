#pragma once

#include <stdint.h>

/* 主机测试只替代时钟和中断屏蔽，USB 缓存及定位模块仍编译实际工程代码。 */
extern uint32_t g_testPrimask;
extern "C" uint32_t HAL_GetTick(void);
inline uint32_t __get_PRIMASK() { return g_testPrimask; }
inline void __disable_irq() { g_testPrimask = 1U; }
inline void __set_PRIMASK(uint32_t primask) { g_testPrimask = primask; }
inline void __DMB() {}
