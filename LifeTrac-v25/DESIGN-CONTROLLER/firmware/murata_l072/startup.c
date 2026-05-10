#include "stm32l072_regs.h"
#include "host_types.h"
#include "platform.h"

#include <stdint.h>

typedef void (*isr_handler_t)(void);

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void NMI_Handler(void);
void HardFault_Handler(void);
void HardFault_Handler_C(uint32_t *stack_ptr);

static void boot_diag_uart_init_19200(void) {
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC_APB1ENR |= RCC_APB1ENR_LPUART1EN;

    /* Route LPUART1 clock from HSI16 for deterministic 19200 baud. */
    RCC_CCIPR = (RCC_CCIPR & ~(0x3UL << 10U)) | (0x2UL << 10U);

    GPIO_MODER(GPIOA_BASE) =
        (GPIO_MODER(GPIOA_BASE) & ~((3UL << (2U * 2U)) | (3UL << (3U * 2U)))) |
        (2UL << (2U * 2U)) |
        (2UL << (3U * 2U));
    GPIO_AFRL(GPIOA_BASE) =
        (GPIO_AFRL(GPIOA_BASE) & ~((0xFUL << (2U * 4U)) | (0xFUL << (3U * 4U)))) |
        (6UL << (2U * 4U)) |
        (6UL << (3U * 4U));

    LPUART1_CR1 = 0U;
    LPUART1_CR2 = 0U;
    LPUART1_CR3 = 0U;
    LPUART1_BRR = 213333UL;
    LPUART1_CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void boot_diag_uart_putc(char c) {
    while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {
    }
    LPUART1_TDR = (uint8_t)c;
}

static void boot_diag_uart_puts(const char *s) {
    while (*s != '\0') {
        boot_diag_uart_putc(*s++);
    }
}

static void boot_diag_switch_hsi16(void) {
    RCC_CR |= RCC_CR_HSION;
    while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }
}

static void __attribute__((noreturn))
fault_reset_and_reboot(uint8_t code, uint8_t sub, uint32_t pc, uint32_t lr, uint32_t psr) {
    boot_diag_uart_init_19200();
    platform_diag_trace("FAULT:RESET\r\n");
    platform_fault_record(code, sub, pc, lr, psr, 0U);
    platform_system_reset();
    for (;;) {
        cpu_wfi();
    }
}

void __attribute__((noreturn)) NMI_Handler(void) {
    fault_reset_and_reboot(HOST_FAULT_CODE_NMI, 0U, 0U, 0U, 0U);
}

void __attribute__((naked)) HardFault_Handler(void) {
    __asm__ volatile (
        "movs r1, #4\n"
        "mov r2, lr\n"
        "tst r2, r1\n"
        "bne 1f\n"
        "mrs r0, msp\n"
        "b 2f\n"
        "1:\n"
        "mrs r0, psp\n"
        "2:\n"
        "b HardFault_Handler_C\n"
    );
}

void __attribute__((noreturn)) HardFault_Handler_C(uint32_t *stack_ptr) {
    uint32_t stacked_pc = 0U;
    uint32_t stacked_lr = 0U;
    uint32_t stacked_psr = 0U;

    if (stack_ptr != (uint32_t *)0U) {
        stacked_lr = stack_ptr[5];
        stacked_pc = stack_ptr[6];
        stacked_psr = stack_ptr[7];
    }

    fault_reset_and_reboot(HOST_FAULT_CODE_HARDFAULT,
                           0U,
                           stacked_pc,
                           stacked_lr,
                           stacked_psr);
}

void __attribute__((weak)) SystemInit(void) {
}

#define WEAK_ALIAS(handler_name) __attribute__((weak, alias(#handler_name)))

void SVC_Handler(void) WEAK_ALIAS(Default_Handler);
void PendSV_Handler(void) WEAK_ALIAS(Default_Handler);
void SysTick_Handler(void) WEAK_ALIAS(Default_Handler);

void WWDG_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void PVD_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void RTC_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void FLASH_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void RCC_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void EXTI0_1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void EXTI2_3_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void EXTI4_15_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TSC_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void DMA1_Channel1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void DMA1_Channel2_3_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void DMA1_Channel4_5_6_7_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void ADC1_COMP_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void LPTIM1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void USART4_5_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM2_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM3_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM6_DAC_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM7_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM21_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void I2C3_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void TIM22_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void I2C1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void I2C2_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void SPI1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void SPI2_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void USART1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void USART2_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void RNG_LPUART1_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void LCD_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void USB_IRQHandler(void) WEAK_ALIAS(Default_Handler);
void CRS_IRQHandler(void) WEAK_ALIAS(Default_Handler);

__attribute__((section(".isr_vector")))
const isr_handler_t g_pfnVectors[] = {
    (isr_handler_t)&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SVC_Handler,
    0,
    0,
    PendSV_Handler,
    SysTick_Handler,

    WWDG_IRQHandler,
    PVD_IRQHandler,
    RTC_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_1_IRQHandler,
    EXTI2_3_IRQHandler,
    EXTI4_15_IRQHandler,
    TSC_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_3_IRQHandler,
    DMA1_Channel4_5_6_7_IRQHandler,
    ADC1_COMP_IRQHandler,
    LPTIM1_IRQHandler,
    USART4_5_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM6_DAC_IRQHandler,
    TIM7_IRQHandler,
    TIM21_IRQHandler,
    I2C3_IRQHandler,
    TIM22_IRQHandler,
    I2C1_IRQHandler,
    I2C2_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    RNG_LPUART1_IRQHandler,
    LCD_IRQHandler,
    USB_IRQHandler,
    CRS_IRQHandler
};

__attribute__((noreturn))
void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    boot_diag_switch_hsi16();
    boot_diag_uart_init_19200();
    boot_diag_uart_puts("EARLY:RESET\\r\\n");
    platform_diag_trace("RST:START\r\n");

    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0U;
    }

    platform_diag_trace("RST:MAIN\r\n");
    boot_diag_uart_puts("EARLY:MAIN\\r\\n");

    SystemInit();
    (void)main();

    for (;;) {
        cpu_wfi();
    }
}

void __attribute__((noreturn)) Default_Handler(void) {
    for (;;) {
        cpu_wfi();
    }
}
