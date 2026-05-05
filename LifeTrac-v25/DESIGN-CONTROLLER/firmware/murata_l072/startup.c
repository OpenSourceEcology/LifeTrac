#include "stm32l072_regs.h"

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

void __attribute__((weak)) SystemInit(void) {
}

#define WEAK_ALIAS(handler_name) __attribute__((weak, alias(#handler_name)))

void NMI_Handler(void) WEAK_ALIAS(Default_Handler);
void HardFault_Handler(void) WEAK_ALIAS(Default_Handler);
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

__attribute__((section(".boot_text"), noreturn))
void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0U;
    }

    SystemInit();
    (void)main();

    for (;;) {
        cpu_wfi();
    }
}

void __attribute__((section(".boot_text"), noreturn)) Default_Handler(void) {
    for (;;) {
        cpu_wfi();
    }
}
