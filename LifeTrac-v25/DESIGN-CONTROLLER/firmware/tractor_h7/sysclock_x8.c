/**
 * sysclock_x8.c — Clock overrides for Portenta X8 H747 co-processor.
 *
 * Two overrides:
 *  1. SetSysClock()           — switches clock to HSI-PLL (480 MHz) instead of HSE
 *  2. SystemCoreClockUpdate() — sets SystemCoreClock = 480 MHz after PLL lock
 *
 * These replace the STRONG symbols in libmbed.a (patched H7_M7 variant where
 * the original SetSysClock was renamed to SetSysClock_HSE_disabled).
 *
 * Clock plan: HSI(64 MHz) / DIVM1=4 = 16 MHz → PLL1 DIVN1=60 → VCO=960 MHz
 *             DIVP1=2 → SYSCLK=480 MHz, DIVQ1=4 → 240 MHz, DIVR1=2 → 480 MHz
 *             AHB = SYSCLK/2 = 240 MHz, APB1/2/3/4 = AHB/2 = 120 MHz
 */

#include <stdint.h>

/* ── register map ─────────────────────────────────────────────────────────── */
#define RCC_BASE        0x58024400UL
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR        (*(volatile uint32_t *)(RCC_BASE + 0x10))
#define RCC_D1CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x18))
#define RCC_D2CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x1C))
#define RCC_D3CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x20))
#define RCC_PLLCKSELR   (*(volatile uint32_t *)(RCC_BASE + 0x28))
#define RCC_PLLCFGR     (*(volatile uint32_t *)(RCC_BASE + 0x2C))
#define RCC_PLL1DIVR    (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_PLL1FRACR   (*(volatile uint32_t *)(RCC_BASE + 0x34))
#define RCC_D1CCIPR     (*(volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_D2CCIP1R    (*(volatile uint32_t *)(RCC_BASE + 0x50))
#define RCC_D3AMR       (*(volatile uint32_t *)(RCC_BASE + 0xA8))
#define RCC_AHB4LPENR   (*(volatile uint32_t *)(RCC_BASE + 0x108))

#define FLASH_BASE      0x52002000UL
#define FLASH_ACR       (*(volatile uint32_t *)(FLASH_BASE + 0x00))

#define PWR_BASE        0x58024800UL
#define PWR_D3CR        (*(volatile uint32_t *)(PWR_BASE + 0x18))

#define SYSCFG_BASE     0x58000400UL
#define SYSCFG_PWRCR    (*(volatile uint32_t *)(SYSCFG_BASE + 0x04))

/* ── RCC_BDCR (backup domain control — LSE) ───────────────────────────────── */
#define RCC_BDCR        (*(volatile uint32_t *)(RCC_BASE + 0x70))
#define RCC_BDCR_LSEON  (1UL << 0)
#define RCC_BDCR_LSERDY (1UL << 1)

/* ── PWR_CR1 (needed to enable access to backup domain) ──────────────────── */
#define PWR_CR1         (*(volatile uint32_t *)(PWR_BASE + 0x00))
#define PWR_CR1_DBP     (1UL << 0)   /* Disable backup domain write protection */

/* ── RCC_APB4ENR ──────────────────────────────────────────────────────────── */
#define RCC_APB4ENR     (*(volatile uint32_t *)(RCC_BASE + 0xF4))
#define RCC_APB4ENR_SYSCFGEN (1UL << 1)
#define RCC_D3AMR_SRAM4AMEN   (1UL << 29)
#define RCC_AHB4LPENR_SRAM4LPEN (1UL << 29)

/* ── SRAM4 boot trace ────────────────────────────────────────────────────── */
#define LIFETRAC_SHARED_BOOT_TRACE ((volatile uint32_t *)0x38000000UL)

/* ── RCC_CR ───────────────────────────────────────────────────────────────── */
#define RCC_CR_HSION        (1UL << 0)
#define RCC_CR_HSIRDY       (1UL << 2)
#define RCC_CR_LSION        (1UL << 0)  /* in RCC_CSR */
#define RCC_CR_PLL1ON       (1UL << 24)
#define RCC_CR_PLL1RDY      (1UL << 25)

/* ── RCC_CSR ──────────────────────────────────────────────────────────────── */
#define RCC_CSR         (*(volatile uint32_t *)(RCC_BASE + 0x74))
#define RCC_CSR_LSION   (1UL << 0)
#define RCC_CSR_LSIRDY  (1UL << 1)

/* ── RCC_CFGR ─────────────────────────────────────────────────────────────── */
#define RCC_CFGR_SW_PLL1    (0x3UL)
#define RCC_CFGR_SWS_SHIFT  3
#define RCC_CFGR_SWS_PLL1   (0x3UL << 3)

/* ── RCC_PLLCKSELR ────────────────────────────────────────────────────────── */
#define RCC_PLLCKSELR_PLLSRC_HSI   0x0UL
#define RCC_PLLCKSELR_DIVM1(x)     ((uint32_t)(x) << 4)

/* ── RCC_PLLCFGR ──────────────────────────────────────────────────────────── */
#define RCC_PLLCFGR_DIVP1EN    (1UL << 16)
#define RCC_PLLCFGR_DIVQ1EN    (1UL << 17)
#define RCC_PLLCFGR_DIVR1EN    (1UL << 18)
/* PLL1RGE: VCO input range; 10 = 8–16 MHz */
#define RCC_PLLCFGR_PLL1RGE_8_16  (0x2UL << 2)
/* PLL1VCOSEL: 0 = wide VCO (192–836 MHz) — we target 768 MHz */
#define RCC_PLLCFGR_PLL1VCOSEL_WIDE  0UL

/* ── D1CFGR / D2CFGR / D3CFGR ────────────────────────────────────────────── */
#define RCC_D1CFGR_D1CPRE_1    (0x0UL << 8)   /* CPU = SYSCLK (/1) */
#define RCC_D1CFGR_HPRE_DIV2   (0x8UL << 0)   /* AHB = SYSCLK/2 */
#define RCC_D1CFGR_D1PPRE_DIV2 (0x4UL << 4)   /* APB3 = AHB/2 */
#define RCC_D2CFGR_D2PPRE1_DIV2 (0x4UL << 4)
#define RCC_D2CFGR_D2PPRE2_DIV2 (0x4UL << 8)
#define RCC_D3CFGR_D3PPRE_DIV2  (0x4UL << 4)

/* ── D1CCIPR / D2CCIP1R peripheral kernels ──────────────────────────────── */
#define RCC_D1CCIPR_CKPERSEL_MASK       (0x3UL << 28)
#define RCC_D1CCIPR_CKPERSEL_HSI        (0x0UL << 28)
#define RCC_D2CCIP1R_SPI123SEL_MASK     (0x7UL << 12)
#define RCC_D2CCIP1R_SPI123SEL_CLKP     (0x4UL << 12)

/* ── FLASH ACR ────────────────────────────────────────────────────────────── */
#define FLASH_ACR_LATENCY_4WS   0x4UL
#define FLASH_ACR_LATENCY_MASK  0xFUL

/* ── PWR_D3CR / SYSCFG_PWRCR ──────────────────────────────────────────────── */
/* VOS1 = 0x3<<14 (default max on H7 rev Y / rev V) */
#define PWR_D3CR_VOS_SCALE1     (0x3UL << 14)
/* ODEN bit in SYSCFG_PWRCR enables VOS0 overdrive for 480 MHz */
#define SYSCFG_PWRCR_ODEN       (1UL << 0)

/* ── SystemCoreClock global (defined in CMSIS/startup) ───────────────────── */
extern uint32_t SystemCoreClock;

/* ── Cortex-M7 startup registers ─────────────────────────────────────────── */
#define SCB_BASE        0xE000ED00UL
#define SCB_VTOR        (*(volatile uint32_t *)(SCB_BASE + 0x08))
#define SCB_ICSR        (*(volatile uint32_t *)(SCB_BASE + 0x04))
#define SCB_SCR         (*(volatile uint32_t *)(SCB_BASE + 0x10))
#define SCB_CPACR       (*(volatile uint32_t *)(SCB_BASE + 0x88))
#define SYST_CSR        (*(volatile uint32_t *)0xE000E010UL)
#define NVIC_ICER_BASE  ((volatile uint32_t *)0xE000E180UL)
#define NVIC_ICPR_BASE  ((volatile uint32_t *)0xE000E280UL)

void SystemInit(void)
{
    __asm volatile("cpsid i" ::: "memory");

    RCC_D3AMR |= RCC_D3AMR_SRAM4AMEN;
    RCC_AHB4LPENR |= RCC_AHB4LPENR_SRAM4LPEN;
    LIFETRAC_SHARED_BOOT_TRACE[0] = 2UL;
    LIFETRAC_SHARED_BOOT_TRACE[1] = 0x00000180UL;
    LIFETRAC_SHARED_BOOT_TRACE[2] = 0xB00700C0UL;
    LIFETRAC_SHARED_BOOT_TRACE[3] = 0UL;
    LIFETRAC_SHARED_BOOT_TRACE[4] = 0xC0UL;

    SYST_CSR = 0;
    for (uint32_t i = 0; i < 8; ++i) {
        NVIC_ICER_BASE[i] = 0xFFFFFFFFUL;
        NVIC_ICPR_BASE[i] = 0xFFFFFFFFUL;
    }
    SCB_ICSR = (1UL << 25) | (1UL << 27);
    LIFETRAC_SHARED_BOOT_TRACE[2] = 0xB00700C1UL;
    LIFETRAC_SHARED_BOOT_TRACE[4] = 0xC1UL;

    SCB_CPACR |= 0x00F00000UL;
    SCB_SCR   |= (1UL << 4);
    SCB_VTOR   = 0x08040000UL;

    RCC_D1CCIPR = (RCC_D1CCIPR & ~RCC_D1CCIPR_CKPERSEL_MASK) | RCC_D1CCIPR_CKPERSEL_HSI;
    RCC_D2CCIP1R = (RCC_D2CCIP1R & ~RCC_D2CCIP1R_SPI123SEL_MASK) | RCC_D2CCIP1R_SPI123SEL_CLKP;

    LIFETRAC_SHARED_BOOT_TRACE[2] = 0xB00700C2UL;
    LIFETRAC_SHARED_BOOT_TRACE[4] = 0xC2UL;

    __asm volatile("dsb 0xF" ::: "memory");
    __asm volatile("isb 0xF" ::: "memory");
    __asm volatile("cpsie i" ::: "memory");
}

/* ── tiny spin-delay (used for voltage regulator settle time) ─────────────── */
static void _spin(volatile uint32_t n) { while (n--) { __asm volatile("nop"); } }

/* ═══════════════════════════════════════════════════════════════════════════
 * SetSysClock()
 *
 * Replaces the H7_M7 libmbed HSE-based version (renamed SetSysClock_HSE_disabled
 * via objcopy).  Configures 400 MHz from HSI to stay inside the wide-VCO range
 * (192–836 MHz) without requiring VOS0 overdrive:
 *
 *   HSI(64) / DIVM1=4 = 16 MHz  →  PLL1 DIVN1=50 → VCO=800 MHz
 *   DIVP1=2 → SYSCLK = 400 MHz
 *   DIVQ1=4 → Q = 200 MHz
 *   DIVR1=2 → R = 400 MHz
 *   AHB = 200 MHz (HPRE=/2), APB1/2/3/4 = 100 MHz (PPRE=/2)
 *
 * VOS1 (default) supports up to 400 MHz — no ODEN needed.
 * ═══════════════════════════════════════════════════════════════════════════ */
void SetSysClock(void)
{
    /* 1. Enable HSI and wait ready */
    RCC_CR |= RCC_CR_HSION;
    while (!(RCC_CR & RCC_CR_HSIRDY)) {}

    /* 2. Ensure VOS1 (already default, explicit for clarity) */
    PWR_D3CR = (PWR_D3CR & ~(0x3UL << 14)) | PWR_D3CR_VOS_SCALE1;
    _spin(500);

    /* 3. Flash latency: 400 MHz @VOS1 → 2 WS (RM0433 Table 17) */
    FLASH_ACR = (FLASH_ACR & ~FLASH_ACR_LATENCY_MASK) | 0x2UL;

    /* 4. Bus dividers (set BEFORE enabling PLL) */
    RCC_D1CFGR = RCC_D1CFGR_D1CPRE_1 | RCC_D1CFGR_HPRE_DIV2 | RCC_D1CFGR_D1PPRE_DIV2;
    RCC_D2CFGR = RCC_D2CFGR_D2PPRE1_DIV2 | RCC_D2CFGR_D2PPRE2_DIV2;
    RCC_D3CFGR = RCC_D3CFGR_D3PPRE_DIV2;

    /* 5. PLL1 source: HSI / DIVM1=4 → ref=16 MHz */
    RCC_PLLCKSELR = (RCC_PLLCKSELR & ~((0x3UL << 0) | (0x3FUL << 4)))
                  | RCC_PLLCKSELR_PLLSRC_HSI
                  | RCC_PLLCKSELR_DIVM1(4);

    /* 6. PLL1 config: wide VCO, 8–16 MHz input range, enable P/Q/R */
    RCC_PLLCFGR = (RCC_PLLCFGR & ~((0x3UL << 2) | (1UL << 1) | (0x7UL << 16)))
                | RCC_PLLCFGR_PLL1RGE_8_16
                | RCC_PLLCFGR_PLL1VCOSEL_WIDE
                | RCC_PLLCFGR_DIVP1EN
                | RCC_PLLCFGR_DIVQ1EN
                | RCC_PLLCFGR_DIVR1EN;

    /* 7. PLL1 dividers: DIVN1=50, DIVP1=2, DIVQ1=4, DIVR1=2
     *    VCO = 16 * 50 = 800 MHz (within 192–836)
     *    P   = 800 / 2 = 400 MHz (SYSCLK)
     *    Q   = 800 / 4 = 200 MHz
     *    R   = 800 / 2 = 400 MHz
     */
    RCC_PLL1DIVR = ((50 - 1) << 0)   /* DIVN1 [8:0]  = 49 */
                 | ((2  - 1) << 9)   /* DIVP1 [14:9] = 1  */
                 | ((4  - 1) << 16)  /* DIVQ1 [22:16]= 3  */
                 | ((2  - 1) << 24); /* DIVR1 [30:24]= 1  */
    RCC_PLL1FRACR = 0;

    /* 8. Enable PLL1 and wait lock */
    RCC_CR |= RCC_CR_PLL1ON;
    while (!(RCC_CR & RCC_CR_PLL1RDY)) {}

    /* 9. Switch SYSCLK source to PLL1 */
    RCC_CFGR = (RCC_CFGR & ~0x7UL) | RCC_CFGR_SW_PLL1;
    while ((RCC_CFGR & (0x7UL << 3)) != RCC_CFGR_SWS_PLL1) {}

    /* 9b. Keep SPI1/2/3 on a simple HSI-backed kernel clock for X8 bring-up. */
    RCC_D1CCIPR = (RCC_D1CCIPR & ~RCC_D1CCIPR_CKPERSEL_MASK) | RCC_D1CCIPR_CKPERSEL_HSI;
    RCC_D2CCIP1R = (RCC_D2CCIP1R & ~RCC_D2CCIP1R_SPI123SEL_MASK) | RCC_D2CCIP1R_SPI123SEL_CLKP;

    /* 10. Update global */
    SystemCoreClock = 400000000UL;

    /* 11. LSI already started by lp_ticker_override.c when lp_ticker_init()
     *     runs. Nothing more needed here for LP timer clock. */
}

/* ═══════════════════════════════════════════════════════════════════════════
 * SystemCoreClockUpdate()
 *
 * Called by mbed after SetSysClock() and by HAL_InitTick().
 * We always return the fixed value because we know exactly what we set.
 * ═══════════════════════════════════════════════════════════════════════════ */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = 400000000UL;
}
