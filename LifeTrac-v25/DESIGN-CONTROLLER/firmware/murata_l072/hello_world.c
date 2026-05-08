/*
 * hello_world.c - Minimal STM32L072 hello-world for Method G bring-up.
 *
 * Sends text at 19200 8N1 on both USART1 (PA9/PA10) and LPUART1 (PA2/PA3).
 * This mirrors the previously bench-validated capture path on /dev/ttymxc3.
 */

#include <stdint.h>

#include "stm32l072_regs.h"

#define USART1_BRR_19200_HSI16   833U
#define LPUART1_BRR_19200_HSI16  213333U

static void delay_loops(volatile uint32_t n) {
    while (n-- != 0U) {
        __asm__("nop");
    }
}

static void clocks_init(void) {
    /* Enable HSI16 and switch SYSCLK to HSI16 for deterministic UART baud. */
    RCC_CR |= (1UL << 0);              /* HSION */
    while ((RCC_CR & (1UL << 2)) == 0U) {
    }                                   /* HSIRDY */

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    /* LPUART1 clock source = HSI16: CCIPR[11:10] = 10b. */
    RCC_CCIPR = (RCC_CCIPR & ~(0x3UL << 10)) | (0x2UL << 10);
}

static void gpio_init(void) {
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;

    /* PA9/PA10 -> AF4 (USART1). */
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((0x3UL << 18) | (0x3UL << 20))) |
                              ((0x2UL << 18) | (0x2UL << 20));
    GPIO_OSPEEDR(GPIOA_BASE) |= (0x3UL << 18) | (0x3UL << 20);
    GPIO_AFRH(GPIOA_BASE) = (GPIO_AFRH(GPIOA_BASE) & ~((0xFUL << 4) | (0xFUL << 8))) |
                             ((0x4UL << 4) | (0x4UL << 8));

    /* PA2/PA3 -> AF6 (LPUART1). */
    GPIO_MODER(GPIOA_BASE) = (GPIO_MODER(GPIOA_BASE) & ~((0x3UL << 4) | (0x3UL << 6))) |
                              ((0x2UL << 4) | (0x2UL << 6));
    GPIO_OSPEEDR(GPIOA_BASE) |= (0x3UL << 4) | (0x3UL << 6);
    GPIO_AFRL(GPIOA_BASE) = (GPIO_AFRL(GPIOA_BASE) & ~((0xFUL << 8) | (0xFUL << 12))) |
                             ((0x6UL << 8) | (0x6UL << 12));
}

static void usart1_init(void) {
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1_CR1 = 0U;
    USART1_CR2 = 0U;
    USART1_CR3 = 0U;
    USART1_BRR = USART1_BRR_19200_HSI16;
    USART1_CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void lpuart1_init(void) {
    RCC_APB1ENR |= RCC_APB1ENR_LPUART1EN;
    LPUART1_CR1 = 0U;
    LPUART1_CR2 = 0U;
    LPUART1_CR3 = 0U;
    LPUART1_BRR = LPUART1_BRR_19200_HSI16;
    LPUART1_CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_putc_both(char c) {
    while ((USART1_ISR & USART_ISR_TXE) == 0U) {
    }
    USART1_TDR = (uint8_t)c;

    while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {
    }
    LPUART1_TDR = (uint8_t)c;
}

static void uart_puts(const char *s) {
    while (*s != '\0') {
        uart_putc_both(*s++);
    }
}

int main(void) {
    clocks_init();
    gpio_init();
    usart1_init();
    lpuart1_init();
    delay_loops(200000U);

    uart_puts("\r\n=== LIFETRAC L072 hello v0.2 (USART1+LPUART1) ===\r\n");
    uart_puts("If you can read this, the X8 -> L072 flash pipeline works.\r\n");

    uint32_t counter = 0U;
    for (;;) {
        uart_puts("LIFETRAC L072 tick=0x");

        uint32_t n = counter++;
        for (int shift = 28; shift >= 0; shift -= 4) {
            uint32_t nib = (n >> (uint32_t)shift) & 0xFU;
            uart_putc_both((char)(nib < 10U ? ('0' + nib) : ('A' + nib - 10U)));
        }
        uart_putc_both('\r');
        uart_putc_both('\n');

        delay_loops(1600000U);
    }
}
