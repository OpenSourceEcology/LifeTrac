/* main.c — minimal STM32L072 hello-world for Murata CMWX1ZZABZ on Portenta X8.
 *
 * Boots from POR (no FW update from MKRWAN), switches to HSI16 for accurate
 * UART baud, then prints "LIFETRAC L072 v0.1\r\n" forever at 19200 8N1 on
 * BOTH USART1 (PA9 TX / PA10 RX) and LPUART1 (PA2 TX / PA3 RX). One of them
 * lands on /dev/ttymxc3 from the Portenta X8 i.MX side; we don't yet know
 * which, so emit on both. The unused one harmlessly drives an idle line.
 *
 * No CMSIS, no HAL — bare register pokes only. Cortex-M0+ thumb.
 */

#include <stdint.h>

/* ---------- register addresses ---------- */
#define RCC_BASE      0x40021000UL
#define RCC_CR        (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR      (*(volatile uint32_t *)(RCC_BASE + 0x0C))
#define RCC_CCIPR     (*(volatile uint32_t *)(RCC_BASE + 0x4C))
#define RCC_IOPENR    (*(volatile uint32_t *)(RCC_BASE + 0x2C))
#define RCC_APB1ENR   (*(volatile uint32_t *)(RCC_BASE + 0x38))
#define RCC_APB2ENR   (*(volatile uint32_t *)(RCC_BASE + 0x34))

#define GPIOA_BASE    0x50000000UL
#define GPIOA_MODER   (*(volatile uint32_t *)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR (*(volatile uint32_t *)(GPIOA_BASE + 0x08))
#define GPIOA_AFRL    (*(volatile uint32_t *)(GPIOA_BASE + 0x20))
#define GPIOA_AFRH    (*(volatile uint32_t *)(GPIOA_BASE + 0x24))

#define USART1_BASE   0x40013800UL
#define LPUART1_BASE  0x40004800UL

#define UART_CR1(b)   (*(volatile uint32_t *)((b) + 0x00))
#define UART_BRR(b)   (*(volatile uint32_t *)((b) + 0x0C))
#define UART_ISR(b)   (*(volatile uint32_t *)((b) + 0x1C))
#define UART_TDR(b)   (*(volatile uint32_t *)((b) + 0x28))

#define ISR_TXE       (1U << 7)

/* ---------- delay helpers ---------- */
static void delay_loops(volatile uint32_t n) { while (n--) { __asm__("nop"); } }

/* ---------- UART setup ---------- */
static void clocks_init(void)
{
    /* Enable HSI16, wait ready, switch SYSCLK to HSI16 */
    RCC_CR |= (1U << 0);              /* HSION */
    while (!(RCC_CR & (1U << 2))) { } /* HSIRDY */
    RCC_CFGR = (RCC_CFGR & ~0x3U) | 0x1U;
    while (((RCC_CFGR >> 2) & 0x3U) != 0x1U) { } /* SWS = HSI16 */

    /* Select HSI16 as LPUART1 clock source: CCIPR[11:10] = 10 */
    RCC_CCIPR = (RCC_CCIPR & ~(0x3U << 10)) | (0x2U << 10);
}

static void gpio_init(void)
{
    RCC_IOPENR  |= (1U << 0);  /* IOPAEN */

    /* PA9, PA10 -> AF mode (10), AF4 (USART1) */
    GPIOA_MODER = (GPIOA_MODER & ~((0x3U << 18) | (0x3U << 20)))
                                |  ((0x2U << 18) | (0x2U << 20));
    GPIOA_OSPEEDR |= (0x3U << 18) | (0x3U << 20);
    /* AFRH bits 7:4 (PA9), 11:8 (PA10) -> 4 */
    GPIOA_AFRH = (GPIOA_AFRH & ~((0xFU << 4) | (0xFU << 8)))
                              |  ((0x4U << 4) | (0x4U << 8));

    /* PA2, PA3 -> AF mode (10), AF6 (LPUART1) */
    GPIOA_MODER = (GPIOA_MODER & ~((0x3U << 4)  | (0x3U << 6)))
                                |  ((0x2U << 4)  | (0x2U << 6));
    GPIOA_OSPEEDR |= (0x3U << 4) | (0x3U << 6);
    /* AFRL bits 11:8 (PA2), 15:12 (PA3) -> 6 */
    GPIOA_AFRL = (GPIOA_AFRL & ~((0xFU << 8) | (0xFU << 12)))
                              |  ((0x6U << 8) | (0x6U << 12));
}

static void usart1_init(void)
{
    RCC_APB2ENR |= (1U << 14); /* USART1EN */
    UART_BRR(USART1_BASE) = 833;             /* 16e6 / 19200 ≈ 833 */
    UART_CR1(USART1_BASE) = (1U << 3) | (1U << 0); /* TE | UE */
}

static void lpuart1_init(void)
{
    RCC_APB1ENR |= (1U << 18); /* LPUART1EN */
    /* LPUART BRR = 256 * fck / baud. 256 * 16e6 / 19200 = 213333 (0x34155) */
    UART_BRR(LPUART1_BASE) = 213333;
    UART_CR1(LPUART1_BASE) = (1U << 3) | (1U << 0); /* TE | UE */
}

static void uart_putc_both(char c)
{
    while (!(UART_ISR(USART1_BASE)  & ISR_TXE)) { }
    UART_TDR(USART1_BASE)  = (uint8_t)c;
    while (!(UART_ISR(LPUART1_BASE) & ISR_TXE)) { }
    UART_TDR(LPUART1_BASE) = (uint8_t)c;
}

static void uart_puts(const char *s) { while (*s) uart_putc_both(*s++); }

/* ---------- main ---------- */
int main(void)
{
    clocks_init();
    gpio_init();
    usart1_init();
    lpuart1_init();
    delay_loops(200000);

    uart_puts("\r\n=== LIFETRAC L072 hello v0.2 (USART1+LPUART1) ===\r\n");
    uart_puts("If you can read this, the X8 -> L072 flash pipeline works.\r\n");

    uint32_t counter = 0;
    for (;;) {
        uart_puts("LIFETRAC L072 tick=0x");
        uint32_t n = counter++;
        for (int shift = 28; shift >= 0; shift -= 4) {
            uint32_t nib = (n >> shift) & 0xFU;
            uart_putc_both((char)(nib < 10 ? ('0' + nib) : ('A' + nib - 10)));
        }
        uart_putc_both('\r'); uart_putc_both('\n');
        delay_loops(1600000);
    }
}

/* ---------- minimal startup ---------- */
extern uint32_t _estack;
extern uint32_t _sdata, _edata, _sidata;
extern uint32_t _sbss, _ebss;

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;
    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

__attribute__((section(".isr_vector"), used))
const void *vector_table[] = {
    (void *)&_estack,
    (void *)Reset_Handler,
    Default_Handler, /* NMI */
    Default_Handler, /* HardFault */
};
