/*
 * lp_ticker_override.c
 *
 * Full lp_ticker HAL override for Portenta X8 (STM32H747).
 * The X8 has no LSE crystal; the mbed H7_M7 lp_ticker_init tries to
 * init LSE via HAL_RCC_OscConfig, consuming ~256KB of stack before faulting.
 *
 * This file provides ALL lp_ticker_* symbols using direct register writes
 * against LPTIM1 clocked from internal LSI (~32kHz, 16-bit counter).
 *
 * All register offsets match STM32H747 RM0433.
 *
 * Because sketch .o files are linked before libmbed.a, these definitions
 * shadow the archive's lp_ticker.o entirely — no archive symbols are pulled.
 */

#include <stdint.h>

/* ─── RCC register map (RM0433) ─── */
#define RCC_BASE       0x58024400UL
#define RCC_CSR        (*(volatile uint32_t *)(RCC_BASE + 0x074))
#define RCC_APB1LENR   (*(volatile uint32_t *)(RCC_BASE + 0x0E8))
#define RCC_D2CCIP2R   (*(volatile uint32_t *)(RCC_BASE + 0x054))

/* ─── LPTIM1 register map (RM0433 §37.7) ─── */
#define LPTIM1_BASE    0x40002400UL
#define LPTIM1_ISR     (*(volatile uint32_t *)(LPTIM1_BASE + 0x00))
#define LPTIM1_ICR     (*(volatile uint32_t *)(LPTIM1_BASE + 0x04))
#define LPTIM1_IER     (*(volatile uint32_t *)(LPTIM1_BASE + 0x08))
#define LPTIM1_CFGR    (*(volatile uint32_t *)(LPTIM1_BASE + 0x0C))
#define LPTIM1_CR      (*(volatile uint32_t *)(LPTIM1_BASE + 0x10))
#define LPTIM1_CMP     (*(volatile uint32_t *)(LPTIM1_BASE + 0x14))
#define LPTIM1_ARR     (*(volatile uint32_t *)(LPTIM1_BASE + 0x18))
#define LPTIM1_CNT     (*(volatile uint32_t *)(LPTIM1_BASE + 0x1C))

/* ─── NVIC registers ─── */
#define NVIC_ISER(n)   (*(volatile uint32_t *)(0xE000E100UL + (n)*4))
#define NVIC_ISPR(n)   (*(volatile uint32_t *)(0xE000E200UL + (n)*4))
#define NVIC_ICPR(n)   (*(volatile uint32_t *)(0xE000E280UL + (n)*4))

/* RCC_CSR bit 0 = LSION, bit 1 = LSIRDY */
#define RCC_CSR_LSION  (1UL << 0)
#define RCC_CSR_LSIRDY (1UL << 1)

/* D2CCIP2R LPTIM1SEL bits [30:28] — 100 = LSI */
#define D2CCIP2R_LPTIM1SEL_LSI  (0x4UL << 28)
#define D2CCIP2R_LPTIM1SEL_MASK (0x7UL << 28)

/* APB1LENR bit 9 = LPTIM1EN */
#define APB1LENR_LPTIM1EN (1UL << 9)

/* LPTIM1 NVIC IRQ = 93 (in ISER[2], bit 29) */
#define LPTIM1_IRQn 93
#define LPTIM_ISR_CMPM      (1UL << 0)
#define LPTIM_ISR_ARRM      (1UL << 1)
#define LPTIM_IER_CMPMIE    (1UL << 0)

/* ─── ticker_info_t struct (matches mbed Ticker::ticker_info_t layout) ─── */
typedef struct {
    uint32_t frequency;  /* Hz */
    uint32_t bits;       /* counter width */
} ticker_info_t;

/* LSI-based ticker info: 32000 Hz, 16-bit counter */
static const ticker_info_t lp_ticker_info_lsi = {
    .frequency = 32000UL,
    .bits      = 16UL,
};

/* Initialization guard */
static volatile uint32_t lp_ticker_initialized = 0;

/* mbed RTOS upper-layer callback — defined in libmbed, we just call it */
extern void lp_ticker_irq_handler(void);

/*
 * LPTIM1_IRQHandler — forward compare match to mbed RTOS.
 */
void LPTIM1_IRQHandler(void)
{
    LPTIM1_IER &= ~LPTIM_IER_CMPMIE;
    LPTIM1_ICR = LPTIM_ISR_CMPM | LPTIM_ISR_ARRM;
    NVIC_ICPR(2) = (1UL << (LPTIM1_IRQn - 64));
    lp_ticker_irq_handler();
}

/*
 * lp_ticker_init — configure LPTIM1 from LSI; no HAL, no LSE.
 */
void lp_ticker_init(void)
{
    if (lp_ticker_initialized) {
        return;
    }
    lp_ticker_initialized = 1U;

    /* 1. Enable LSI */
    RCC_CSR |= RCC_CSR_LSION;
    uint32_t t = 200000UL;
    while (!(RCC_CSR & RCC_CSR_LSIRDY) && --t) {}

    /* 2. Route LSI to LPTIM1 (D2CCIP2R[30:28] = 100) */
    RCC_D2CCIP2R = (RCC_D2CCIP2R & ~D2CCIP2R_LPTIM1SEL_MASK) | D2CCIP2R_LPTIM1SEL_LSI;

    /* 3. Enable LPTIM1 APB1 clock */
    RCC_APB1LENR |= APB1LENR_LPTIM1EN;
    (void)RCC_APB1LENR; /* read-back sync */

    /* 4. Reset LPTIM1 */
    LPTIM1_CR = 0;

    /* 5. Configure: no prescaler, internal clock */
    LPTIM1_CFGR = 0;

    /* 6. Enable LPTIM1 peripheral */
    LPTIM1_CR = 1UL; /* ENABLE */

    /* 7. Set ARR to max (0xFFFF) for free-running wraparound */
    LPTIM1_ARR = 0xFFFFUL;

    /* 8. Leave compare interrupt disabled until lp_ticker_set_interrupt(). */
    LPTIM1_ICR = 0xFFUL; /* clear all flags first */
    LPTIM1_IER = 0UL;

    /* 9. Enable NVIC for LPTIM1 (IRQ 93 = ISER[2] bit 29) */
    NVIC_ICPR(2) = (1UL << (LPTIM1_IRQn - 64));
    NVIC_ISER(2) = (1UL << (LPTIM1_IRQn - 64));

    /* 10. Start counter in continuous mode */
    LPTIM1_CR |= (1UL << 2); /* CNTSTRT */
}

/*
 * lp_ticker_free — stop LPTIM1 and reset init flag.
 */
void lp_ticker_free(void)
{
    lp_ticker_disable_interrupt();
    lp_ticker_initialized = 0U;
    LPTIM1_CR = 0;
}

/*
 * lp_ticker_read — stable double-read of LPTIM1 CNT.
 */
uint32_t lp_ticker_read(void)
{
    uint32_t previous = LPTIM1_CNT;
    for (uint32_t attempts = 0; attempts < 4; ++attempts) {
        uint32_t current = LPTIM1_CNT;
        if (current == previous) {
            return current;
        }
        previous = current;
    }
    return LPTIM1_CNT;
}

/*
 * lp_ticker_set_interrupt — schedule compare at 'timestamp' (16-bit ticks).
 */
void lp_ticker_set_interrupt(uint32_t timestamp)
{
    /* Must write CMP while LPTIM1 is enabled and running */
    LPTIM1_IER &= ~LPTIM_IER_CMPMIE;
    LPTIM1_CMP = timestamp & 0xFFFFUL;
    LPTIM1_ICR = LPTIM_ISR_CMPM | LPTIM_ISR_ARRM;
    NVIC_ICPR(2) = (1UL << (LPTIM1_IRQn - 64));
    LPTIM1_IER |= LPTIM_IER_CMPMIE;
}

/*
 * lp_ticker_fire_interrupt — force an immediate IRQ by pending LPTIM1 in NVIC.
 */
void lp_ticker_fire_interrupt(void)
{
    NVIC_ISPR(2) = (1UL << (LPTIM1_IRQn - 64));
}

/*
 * lp_ticker_disable_interrupt — mask compare-match interrupt.
 */
void lp_ticker_disable_interrupt(void)
{
    LPTIM1_IER &= ~LPTIM_IER_CMPMIE;
    LPTIM1_ICR = LPTIM_ISR_CMPM;
    NVIC_ICPR(2) = (1UL << (LPTIM1_IRQn - 64));
}

/*
 * lp_ticker_clear_interrupt — clear compare-match flag.
 */
void lp_ticker_clear_interrupt(void)
{
    LPTIM1_ICR = LPTIM_ISR_CMPM;
}

/*
 * lp_ticker_get_info — return LSI-based ticker_info_t.
 */
const ticker_info_t *lp_ticker_get_info(void)
{
    return &lp_ticker_info_lsi;
}

/*
 * Stubs for the reconfigure functions (called by mbed in some error paths).
 * On X8 we're always in LSI mode; ignore LSE reconfigure requests.
 */
void lp_ticker_reconfigure_with_lsi(void)
{
    lp_ticker_free();
    lp_ticker_init();
}

void lp_ticker_reconfigure_with_lse(void)
{
    /* X8 has no LSE — silently fall back to LSI */
    lp_ticker_reconfigure_with_lsi();
}

/* ─────────────────────────────────────────────────────────────────────────────
 * HAL_GetREVID / HAL_GetDEVID overrides
 *
 * The STM32H747 DBGMCU at 0x5C001000 is in the SRD (D3) domain. On the X8,
 * this peripheral may be power-gated or inaccessible, causing a BusFault when
 * HAL_RCC_OscConfig calls HAL_GetREVID to check silicon revision.
 *
 * Return fixed values matching STM32H747 rev Y (production silicon):
 *   REVID = 0x1003  (H7 rev Y)
 *   DEVID = 0x450   (STM32H747/57 device ID)
 * ─────────────────────────────────────────────────────────────────────────── */
uint32_t HAL_GetREVID(void)
{
    return 0x1003UL;   /* H747 rev Y — "not greater than 0x1003" path in HAL */
}

uint32_t HAL_GetDEVID(void)
{
    return 0x450UL;    /* STM32H747/57 device ID */
}
