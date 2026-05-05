#ifndef LIFETRAC_MURATA_L072_PLATFORM_H
#define LIFETRAC_MURATA_L072_PLATFORM_H

#include <stdint.h>

uint32_t platform_core_hz(void);
void platform_clock_init_hsi16(void);
void platform_systick_init_1ms(void);
uint32_t platform_now_ms(void);
uint32_t platform_now_us(void);
void platform_delay_ms(uint32_t delay_ms);
void platform_irq_enable(uint32_t irqn, uint8_t priority);
void platform_system_reset(void);

#endif /* LIFETRAC_MURATA_L072_PLATFORM_H */
