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

    /*
     * 2026-05-11 W1-7 RX-fix: assert TCXO_VCC enable before probing HSE.
     * The Murata CMWX1ZZABZ-078 module's 32 MHz TCXO is powered via the
     * MCU GPIO PA12 on the B-L072Z-LRWAN1 reference design (X-CUBE-LRWAN
     * BSP `HW_TCXO_ON`).  Without this assert, HSEBYP probe times out
     * silently and the firmware falls back to HSI16 16 MHz (clock_source
     * id == 1), which is too imprecise (±1 %) to support 921 600 baud
     * COBS-framed RX without framing errors.  Cost: ~5 ms TCXO settle
     * one-shot at boot + ~600 uA when running.
     *
     * Sequence:
     *   1. Enable GPIOA bus clock (idempotent if already on).
     *   2. Set PA12 MODER = 01 (general purpose output).
     *   3. Set PA12 OTYPER = 0 (push-pull, default).
     *   4. Drive PA12 HIGH via BSRR.
     *   5. Spin ~5 ms (well above the 2 ms TCXO startup spec).
     *   6. Then proceed to HSE BYPASS probe below.
     *
     * If HSE still fails after this assert, the carrier wiring routes
     * TCXO_VCC differently (R12 / VDD_TCXO rail per BRINGUP_MAX_CARRIER.md);
     * fall back is unchanged (HSI16 16 MHz with reduced host baud).
     */
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~(3UL << (12U * 2U)))
                             | (1UL << (12U * 2U));        /* PA12 = output */
    GPIO_OTYPER(GPIOA_BASE) &= ~(1UL << 12U);              /* push-pull */
    GPIO_BSRR(GPIOA_BASE) = (1UL << 12U);                  /* PA12 = HIGH */

    /*
     * 2026-05-12 W1-9d: also drive PB6 HIGH for TCXO_VCC enable.
     * ABX00043 carrier straps R12 0R (PB6 -> VDD_TCX0) with R86 DNP, so
     * PA12 above does nothing on this board.  hardwario/lora-modem's
     * TCXO_PIN==2 build variant documents PB6 as a Murata-SiP-supported
     * TCXO control pin.  Risk: zero -- PB6 currently floats; HIGH cannot
     * disturb a pin already 0R-strapped to a +3V3-tied rail.
     * Acceptance: BOOT_URC.clock_source_id should report 0 (HSE_OK)
     * after this edit; previously always 1 (HSI_FALLBACK).
     */
    RCC_IOPENR |= RCC_IOPENR_GPIOBEN;
    GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~(3UL << (6U * 2U)))
                             | (1UL << (6U * 2U));         /* PB6 = output */
    GPIO_OTYPER(GPIOB_BASE) &= ~(1UL << 6U);               /* push-pull */
    GPIO_BSRR(GPIOB_BASE) = (1UL << 6U);                   /* PB6 = HIGH */

    /* Settle delay: ~5 ms at the MSI default ~2.1 MHz boot clock.
     * 2.1e6 cycles/s × 5e-3 s ≈ 10 500 cycles; multiply by ~3 for the
     * cycles-per-iteration of this empty loop = ~3 500 iterations. */
    for (volatile uint32_t i = 0; i < 4000U; ++i) { __asm__ volatile(""); }

    RCC_CR |= RCC_CR_HSEBYP | RCC_CR_HSEON;
    while (((RCC_CR & RCC_CR_HSERDY) == 0U) && timeout < PLATFORM_HSE_READY_TIMEOUT) {
        timeout++;
    }

    /*
     * 2026-05-12 W1-9d follow-up: this function's contract is "init HSI16"
     * (per its name + the lora_ping.c "Force HSI16 (proven path)" comment).
     * Switching SYSCLK to HSE 32 MHz on success silently doubled the USART1
     * kernel clock (USART1 uses SYSCLK by default per RCC_CCIPR[1:0]=00),
     * halving the effective baud and breaking the 921 600 host transport.
     * Pre-W1-9d this never tripped because HSE never locked (TCXO unpowered);
     * once PB6 powered the TCXO and HSERDY started asserting, the long-dormant
     * SYSCLK switch surfaced as TRANSPORT_FAIL on the W1-9b T2 probe.
     *
     * New behavior: probe HSE for diagnostic only; report HSE_OK in BOOT_URC.
     * clock_source_id when HSERDY asserts, but always keep SYSCLK on HSI16.
     * SX1276 still benefits from PB6 powering TCXO_VCC -- that is independent
     * of which clock the L072 SYSCLK runs on.
     */
    RCC_CR |= RCC_CR_HSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    s_core_hz = 16000000UL;
    s_clock_source_id = ((RCC_CR & RCC_CR_HSERDY) != 0U)
                        ? PLATFORM_CLOCK_SOURCE_HSE_OK
                        : PLATFORM_CLOCK_SOURCE_HSI_FALLBACK;
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

void platform_diag_trace(const char *text) {
    if (text == NULL) {
        return;
    }

    if ((LPUART1_CR1 & (USART_CR1_UE | USART_CR1_TE)) != (USART_CR1_UE | USART_CR1_TE)) {
        return;
    }

    while (*text != '\0') {
        while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {
        }
        LPUART1_TDR = (uint8_t)(*text++);
    }

    while ((LPUART1_ISR & USART_ISR_TC) == 0U) {
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
