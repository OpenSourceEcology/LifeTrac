/*
 * safe_mode.c - N-22 UART safe-mode listener.
 *
 * Runs before normal app init and checks for a fixed compile-time magic
 * sequence over USART2 at three baud rates. On a match, control jumps to the
 * STM32 ROM bootloader in system memory.
 */

#include "config.h"
#include "memory_map.h"
#include "stm32l072_regs.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MM_SAFE_MODE_WINDOW_MS       500U
#define SAFE_MODE_BAUD_COUNT         3U
#define SAFE_MODE_SLOT_MS            (MM_SAFE_MODE_WINDOW_MS / SAFE_MODE_BAUD_COUNT)

static const uint32_t kSafeModeBauds[SAFE_MODE_BAUD_COUNT] __attribute__((section(".boot_rodata"))) = {
    921600UL,
    115200UL,
    9600UL
};

static const uint8_t kSafeModeMagic[8] __attribute__((section(".boot_rodata"))) = {
    0xA5, 0x5A, 0xA5, 0x5A, 0x5A, 0xA5, 0x5A, 0xA5
};

__attribute__((section(".boot_text"), noreturn, noinline))
void jump_to_bootloader(void);

static void boot_switch_to_hsi16(void) {
    RCC_CR |= RCC_CR_HSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }
}

static void boot_start_systick_1ms(void) {
    SYST_RVR = (16000000UL / 1000UL) - 1UL;
    SYST_CVR = 0U;
    SYST_CSR = SYST_CSR_CLKSOURCE | SYST_CSR_ENABLE;
}

static void boot_stop_systick(void) {
    SYST_CSR = 0U;
    SYST_RVR = 0U;
    SYST_CVR = 0U;
}

static bool boot_tick_elapsed(void) {
    return (SYST_CSR & SYST_CSR_COUNTFLAG) != 0U;
}

static void boot_usart2_gpio_init(void) {
    uint32_t moder;
    uint32_t afrl;

    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;

    moder = GPIO_MODER(GPIOA_BASE);
    moder &= ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)));
    moder |= (2UL << (2U * 2U)) | (2UL << (3U * 2U));
    GPIO_MODER(GPIOA_BASE) = moder;

    GPIO_OTYPER(GPIOA_BASE) &= ~((1UL << 2U) | (1UL << 3U));

    afrl = GPIO_AFRL(GPIOA_BASE);
    afrl &= ~((0xFUL << (2U * 4U)) | (0xFUL << (3U * 4U)));
    afrl |= (4UL << (2U * 4U)) | (4UL << (3U * 4U));
    GPIO_AFRL(GPIOA_BASE) = afrl;
}

static void boot_usart2_init(uint32_t baud) {
    RCC_APB1ENR |= RCC_APB1ENR_USART2EN;
    boot_usart2_gpio_init();

    USART2_CR1 = 0U;
    USART2_CR2 = 0U;
    USART2_CR3 = 0U;
    USART2_BRR = (16000000UL + (baud / 2UL)) / baud;
    USART2_ICR = USART_ICR_PECF |
                 USART_ICR_FECF |
                 USART_ICR_NCF |
                 USART_ICR_ORECF |
                 USART_ICR_IDLECF;
    USART2_CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
}

static void boot_usart2_deinit(void) {
    USART2_CR1 = 0U;
    USART2_CR2 = 0U;
    USART2_CR3 = 0U;
}

static bool boot_try_magic(uint32_t timeout_ms) {
    uint32_t elapsed_ms = 0U;
    size_t matched = 0U;

    while (elapsed_ms < timeout_ms) {
        if (boot_tick_elapsed()) {
            elapsed_ms++;
        }

        if ((USART2_ISR & USART_ISR_RXNE) == 0U) {
            continue;
        }

        const uint8_t rx = (uint8_t)USART2_RDR;

        if (rx == kSafeModeMagic[matched]) {
            matched++;
            if (matched == sizeof(kSafeModeMagic)) {
                return true;
            }
            continue;
        }

        matched = (rx == kSafeModeMagic[0]) ? 1U : 0U;
    }

    return false;
}

__attribute__((section(".boot_text"), noinline))
void safe_mode_listen(void) {
    uint32_t remaining_ms = MM_SAFE_MODE_WINDOW_MS;

    boot_switch_to_hsi16();
    boot_start_systick_1ms();

    for (size_t i = 0U; i < SAFE_MODE_BAUD_COUNT; ++i) {
        const uint32_t slot_ms = (i == (SAFE_MODE_BAUD_COUNT - 1U)) ? remaining_ms : SAFE_MODE_SLOT_MS;
        remaining_ms -= slot_ms;

        boot_usart2_init(kSafeModeBauds[i]);
        if (boot_try_magic(slot_ms)) {
            boot_stop_systick();
            jump_to_bootloader();
        }
        boot_usart2_deinit();
    }

    boot_stop_systick();
}

__attribute__((section(".boot_text"), noreturn, noinline))
void jump_to_bootloader(void) {
    typedef void (*boot_entry_fn_t)(void);

    const uint32_t boot_sp = MMIO32(MM_SYSMEM_BASE + 0x00UL);
    const uint32_t boot_pc = MMIO32(MM_SYSMEM_BASE + 0x04UL);

    cpu_irq_disable();
    boot_usart2_deinit();
    boot_stop_systick();

    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG_CFGR1 = (SYSCFG_CFGR1 & ~SYSCFG_CFGR1_MEM_MODE_MASK) |
                   SYSCFG_CFGR1_MEM_MODE_SYSTEM_FLASH;

    SCB_VTOR = MM_SYSMEM_BASE;

    __asm__ volatile ("msr msp, %0" : : "r"(boot_sp) : "memory");
    ((boot_entry_fn_t)boot_pc)();

    for (;;) {
        cpu_wfi();
    }
}
