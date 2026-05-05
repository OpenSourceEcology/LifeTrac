#include "platform.h"
#include "stm32l072_regs.h"

static volatile uint32_t s_tick_ms;
static uint32_t s_core_hz = 16000000UL;

uint32_t platform_core_hz(void) {
    return s_core_hz;
}

void platform_clock_init_hsi16(void) {
    RCC_CR |= RCC_CR_HSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    s_core_hz = 16000000UL;
}

void platform_systick_init_1ms(void) {
    const uint32_t reload = (s_core_hz / 1000UL) - 1UL;

    SYST_RVR = reload;
    SYST_CVR = 0U;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT | SYST_CSR_ENABLE;
}

uint32_t platform_now_ms(void) {
    return s_tick_ms;
}

uint32_t platform_now_us(void) {
    uint32_t ms_before;
    uint32_t ms_after;
    uint32_t systick_val;
    uint32_t elapsed_ticks;

    do {
        ms_before = s_tick_ms;
        systick_val = SYST_CVR;
        ms_after = s_tick_ms;
    } while (ms_before != ms_after);

    elapsed_ticks = (SYST_RVR + 1U) - systick_val;
    return (ms_before * 1000U) + (elapsed_ticks / (s_core_hz / 1000000U));
}

void platform_delay_ms(uint32_t delay_ms) {
    const uint32_t start = platform_now_ms();
    while ((uint32_t)(platform_now_ms() - start) < delay_ms) {
    }
}

void platform_irq_enable(uint32_t irqn, uint8_t priority) {
    nvic_enable_irq(irqn, priority);
}

void platform_system_reset(void) {
    SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ;
    for (;;) {
    }
}

void SysTick_Handler(void) {
    s_tick_ms++;
}
