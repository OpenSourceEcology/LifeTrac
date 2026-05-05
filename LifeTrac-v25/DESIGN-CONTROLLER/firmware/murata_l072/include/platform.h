#ifndef LIFETRAC_MURATA_L072_PLATFORM_H
#define LIFETRAC_MURATA_L072_PLATFORM_H

#include "platform_fault.h"
#include "reset_cause.h"

#include <stdint.h>

#define PLATFORM_CLOCK_SOURCE_HSE_OK        0U
#define PLATFORM_CLOCK_SOURCE_HSI_FALLBACK  1U
#define PLATFORM_CLOCK_SOURCE_MSI_FALLBACK  2U

uint32_t platform_core_hz(void);
void platform_reset_cause_capture_early(void);
reset_cause_t platform_reset_cause_take(void);
void platform_clock_init_hsi16(void);
uint8_t platform_clock_source_id(void);
void platform_systick_init_1ms(void);
uint32_t platform_now_ms(void);
uint32_t platform_now_us(void);
void platform_delay_ms(uint32_t delay_ms);
void platform_irq_enable(uint32_t irqn, uint8_t priority);
void platform_system_reset(void);

#endif /* LIFETRAC_MURATA_L072_PLATFORM_H */
