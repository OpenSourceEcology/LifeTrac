/* Minimal Reset_Handler for UART test only */
#include "stm32l072_regs.h"
#include "platform.h"

extern uint32_t _estack;

/* Minimal USART2 initialization */
static void init_usart2(void) {
    /* Enable GPIOA clock */
    RCC_IOPENR |= (1U << 0);
    
    /* Enable USART2 clock */
    RCC_APB1ENR |= (1U << 17);
    
    /* Enable HSI16 */
    RCC_CR |= (1U << 4);
    while ((RCC_CR & (1U << 5)) == 0) {
    }
    
    /* Switch to HSI16 */
    RCC_CFGR = (RCC_CFGR & ~0x3) | 0x1;
    while ((RCC_CFGR & 0xC) != 0x4) {
    }
    
    /* PA2=TX (AF4), PA3=RX (AF4) */
    GPIOA_MODER = (GPIOA_MODER & ~0xF0) | 0xA0;  /* PA2, PA3 as alternate */
    GPIOA_AFRL = (GPIOA_AFRL & ~0xFF00) | 0x4400;  /* AF4 for both */
    
    /* USART2 config: 19200 8E1 */
    USART2_BRR = 416;  /* 16MHz / 19200 = 833, divided by 2 for oversampling = 416 */
    USART2_CR1 = 0x0C;  /* TE, RE */
    USART2_CR1 |= 0x01;  /* Enable USART */
}

static void send_char(char c) {
    while ((USART2_ISR & 0x80) == 0) {  /* Wait for TXE */
    }
    USART2_TDR = (uint8_t)c;
}

__attribute__((section(".boot_text"), noreturn))
void Reset_Handler(void) {
    init_usart2();
    send_char('T');
    send_char('\r');
    send_char('\n');
    
    /* Hang */
    for (;;) {
        __asm("wfi");
    }
}

__attribute__((weak))
void NMI_Handler(void) {
    while (1) {
    }
}

__attribute__((weak))
void HardFault_Handler(void) {
    while (1) {
    }
}

__attribute__((section(".isr_vector")))
void (* const g_pfnVectors[48])(void) = {
    (void (*)(void))(intptr_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0
};
