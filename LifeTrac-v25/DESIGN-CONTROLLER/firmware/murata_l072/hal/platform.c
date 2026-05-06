#include "platform.h"
#include "stm32l072_regs.h"

#include <stddef.h>

#define PLATFORM_FAULT_MAGIC        0xFA075DECUL
#define PLATFORM_HSE_READY_TIMEOUT   800000UL

typedef struct platform_fault_retained_s {
    uint32_t magic;
    uint8_t code;
    uint8_t sub;
    uint16_t reserved;
    uint32_t pc;
    uint32_t lr;
    uint32_t psr;
    uint32_t bfar;
} platform_fault_retained_t;

static volatile platform_fault_retained_t s_fault_retained __attribute__((section(".noinit")));
static volatile uint32_t s_tick_ms;
static uint32_t s_core_hz = 16000000UL;
static reset_cause_t s_reset_cause = RESET_CAUSE_UNKNOWN;
static uint8_t s_reset_cause_captured;
static uint8_t s_clock_source_id = PLATFORM_CLOCK_SOURCE_HSI_FALLBACK;

static reset_cause_t decode_reset_cause(uint32_t csr) {
    if ((csr & RCC_CSR_IWDGRSTF) != 0U) {
        return RESET_CAUSE_IWDG;
    }
    if ((csr & RCC_CSR_WWDGRSTF) != 0U) {
        return RESET_CAUSE_WWDG;
    }
    if ((csr & RCC_CSR_SFTRSTF) != 0U) {
        return RESET_CAUSE_SOFT;
    }
    if ((csr & RCC_CSR_PINRSTF) != 0U) {
        return RESET_CAUSE_PIN;
    }
    if ((csr & RCC_CSR_PORRSTF) != 0U) {
        return RESET_CAUSE_POR_BOR;
    }
    if ((csr & RCC_CSR_OBLRSTF) != 0U) {
        return RESET_CAUSE_OBL;
    }
    if ((csr & RCC_CSR_FWRSTF) != 0U) {
        return RESET_CAUSE_FW;
    }
    if ((csr & RCC_CSR_LPWRRSTF) != 0U) {
        return RESET_CAUSE_LPWR;
    }
    return RESET_CAUSE_UNKNOWN;
}

uint32_t platform_core_hz(void) {
    return s_core_hz;
}

void platform_reset_cause_capture_early(void) {
    if (s_reset_cause_captured != 0U) {
        return;
    }

    s_reset_cause = decode_reset_cause(RCC_CSR);
    RCC_CSR |= RCC_CSR_RMVF;
    s_reset_cause_captured = 1U;
}

reset_cause_t platform_reset_cause_take(void) {
    if (s_reset_cause_captured == 0U) {
        platform_reset_cause_capture_early();
    }
    return s_reset_cause;
}

void platform_clock_init_hsi16(void) {
    uint32_t timeout = 0U;

    RCC_CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    while (((RCC_CR & RCC_CR_HSERDY) == 0U) && timeout < PLATFORM_HSE_READY_TIMEOUT) {
        timeout++;
    }

    if ((RCC_CR & RCC_CR_HSERDY) != 0U) {
        RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSE;
        while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSE) {
        }

        RCC_CR &= ~RCC_CR_HSION;
        s_core_hz = 32000000UL;
        s_clock_source_id = PLATFORM_CLOCK_SOURCE_HSE_OK;
        return;
    }

    RCC_CR |= RCC_CR_HSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    s_core_hz = 16000000UL;
    s_clock_source_id = PLATFORM_CLOCK_SOURCE_HSI_FALLBACK;
}

uint8_t platform_clock_source_id(void) {
    return s_clock_source_id;
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

void platform_fault_record(uint8_t code,
                           uint8_t sub,
                           uint32_t pc,
                           uint32_t lr,
                           uint32_t psr,
                           uint32_t bfar) {
    s_fault_retained.magic = PLATFORM_FAULT_MAGIC;
    s_fault_retained.code = code;
    s_fault_retained.sub = sub;
    s_fault_retained.reserved = 0U;
    s_fault_retained.pc = pc;
    s_fault_retained.lr = lr;
    s_fault_retained.psr = psr;
    s_fault_retained.bfar = bfar;
}

bool platform_fault_take_replay(platform_fault_record_t *out_record) {
    if (out_record == NULL || s_fault_retained.magic != PLATFORM_FAULT_MAGIC) {
        return false;
    }

    out_record->code = s_fault_retained.code;
    out_record->sub = s_fault_retained.sub;
    out_record->pc = s_fault_retained.pc;
    out_record->lr = s_fault_retained.lr;
    out_record->psr = s_fault_retained.psr;
    out_record->bfar = s_fault_retained.bfar;

    s_fault_retained.magic = 0U;
    return true;
}

void SysTick_Handler(void) {
    s_tick_ms++;
}
