#ifndef LIFETRAC_MURATA_L072_REGS_H
#define LIFETRAC_MURATA_L072_REGS_H

#include <stdint.h>

#define MMIO32(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#define MMIO16(addr) (*(volatile uint16_t *)(uintptr_t)(addr))
#define MMIO8(addr)  (*(volatile uint8_t *)(uintptr_t)(addr))

/* RCC */
#define RCC_BASE                   0x40021000UL
#define RCC_CR                     MMIO32(RCC_BASE + 0x00UL)
#define RCC_CFGR                   MMIO32(RCC_BASE + 0x0CUL)
#define RCC_CCIPR                  MMIO32(RCC_BASE + 0x4CUL)
#define RCC_IOPENR                 MMIO32(RCC_BASE + 0x2CUL)
#define RCC_AHBENR                 MMIO32(RCC_BASE + 0x30UL)
#define RCC_APB2ENR                MMIO32(RCC_BASE + 0x34UL)
#define RCC_APB1ENR                MMIO32(RCC_BASE + 0x38UL)
#define RCC_CSR                    MMIO32(RCC_BASE + 0x74UL)

#define RCC_CR_HSION               (1UL << 8)
#define RCC_CR_HSIRDY              (1UL << 10)
#define RCC_CR_HSEON               (1UL << 16)
#define RCC_CR_HSERDY              (1UL << 17)
#define RCC_CR_HSEBYP              (1UL << 18)

#define RCC_CFGR_SW_MASK           (3UL << 0)
#define RCC_CFGR_SW_HSI16          (1UL << 0)
#define RCC_CFGR_SW_HSE            (2UL << 0)
#define RCC_CFGR_SWS_MASK          (3UL << 2)
#define RCC_CFGR_SWS_HSI16         (1UL << 2)
#define RCC_CFGR_SWS_HSE           (2UL << 2)

#define RCC_IOPENR_GPIOAEN         (1UL << 0)
#define RCC_IOPENR_GPIOBEN         (1UL << 1)
#define RCC_IOPENR_GPIOCEN         (1UL << 2)

#define RCC_AHBENR_DMA1EN          (1UL << 0)

#define RCC_APB2ENR_SYSCFGEN       (1UL << 0)
#define RCC_APB2ENR_SPI1EN         (1UL << 12)
#define RCC_APB2ENR_USART1EN       (1UL << 14)

#define RCC_APB1ENR_USART2EN       (1UL << 17)
#define RCC_APB1ENR_LPUART1EN      (1UL << 18)

#define RCC_CSR_RMVF               (1UL << 23)
#define RCC_CSR_FWRSTF             (1UL << 24)
#define RCC_CSR_OBLRSTF            (1UL << 25)
#define RCC_CSR_PINRSTF            (1UL << 26)
#define RCC_CSR_PORRSTF            (1UL << 27)
#define RCC_CSR_SFTRSTF            (1UL << 28)
#define RCC_CSR_IWDGRSTF           (1UL << 29)
#define RCC_CSR_WWDGRSTF           (1UL << 30)
#define RCC_CSR_LPWRRSTF           (1UL << 31)

/* GPIO */
#define GPIOA_BASE                 0x50000000UL
#define GPIOB_BASE                 0x50000400UL
#define GPIOC_BASE                 0x50000800UL

#define GPIO_MODER(port)           MMIO32((port) + 0x00UL)
#define GPIO_OTYPER(port)          MMIO32((port) + 0x04UL)
#define GPIO_OSPEEDR(port)         MMIO32((port) + 0x08UL)
#define GPIO_PUPDR(port)           MMIO32((port) + 0x0CUL)
#define GPIO_IDR(port)             MMIO32((port) + 0x10UL)
#define GPIO_ODR(port)             MMIO32((port) + 0x14UL)
#define GPIO_BSRR(port)            MMIO32((port) + 0x18UL)
#define GPIO_AFRL(port)            MMIO32((port) + 0x20UL)
#define GPIO_AFRH(port)            MMIO32((port) + 0x24UL)

/* DMA1 */
#define DMA1_BASE                  0x40020000UL
#define DMA1_ISR                   MMIO32(DMA1_BASE + 0x00UL)
#define DMA1_IFCR                  MMIO32(DMA1_BASE + 0x04UL)

#define DMA1_CCR(ch)               MMIO32(DMA1_BASE + 0x08UL + (((ch) - 1UL) * 0x14UL))
#define DMA1_CNDTR(ch)             MMIO32(DMA1_BASE + 0x0CUL + (((ch) - 1UL) * 0x14UL))
#define DMA1_CPAR(ch)              MMIO32(DMA1_BASE + 0x10UL + (((ch) - 1UL) * 0x14UL))
#define DMA1_CMAR(ch)              MMIO32(DMA1_BASE + 0x14UL + (((ch) - 1UL) * 0x14UL))

#define DMA_ISR_GIF(ch)            (1UL << ((((ch) - 1UL) * 4UL) + 0UL))
#define DMA_ISR_TCIF(ch)           (1UL << ((((ch) - 1UL) * 4UL) + 1UL))
#define DMA_ISR_HTIF(ch)           (1UL << ((((ch) - 1UL) * 4UL) + 2UL))
#define DMA_ISR_TEIF(ch)           (1UL << ((((ch) - 1UL) * 4UL) + 3UL))

#define DMA_IFCR_CGIF(ch)          DMA_ISR_GIF(ch)
#define DMA_IFCR_CTCIF(ch)         DMA_ISR_TCIF(ch)
#define DMA_IFCR_CHTIF(ch)         DMA_ISR_HTIF(ch)
#define DMA_IFCR_CTEIF(ch)         DMA_ISR_TEIF(ch)

#define DMA_CCR_EN                 (1UL << 0)
#define DMA_CCR_TCIE               (1UL << 1)
#define DMA_CCR_HTIE               (1UL << 2)
#define DMA_CCR_TEIE               (1UL << 3)
#define DMA_CCR_DIR                (1UL << 4)
#define DMA_CCR_CIRC               (1UL << 5)
#define DMA_CCR_PINC               (1UL << 6)
#define DMA_CCR_MINC               (1UL << 7)
#define DMA_CCR_PL_HIGH            (2UL << 12)

/* USART1 */
#define USART1_BASE                0x40013800UL
#define USART1_CR1                 MMIO32(USART1_BASE + 0x00UL)
#define USART1_CR2                 MMIO32(USART1_BASE + 0x04UL)
#define USART1_CR3                 MMIO32(USART1_BASE + 0x08UL)
#define USART1_BRR                 MMIO32(USART1_BASE + 0x0CUL)
#define USART1_RQR                 MMIO32(USART1_BASE + 0x18UL)
#define USART1_ISR                 MMIO32(USART1_BASE + 0x1CUL)
#define USART1_ICR                 MMIO32(USART1_BASE + 0x20UL)
#define USART1_RDR                 MMIO32(USART1_BASE + 0x24UL)
#define USART1_TDR                 MMIO32(USART1_BASE + 0x28UL)

/* USART2 */
#define USART2_BASE                0x40004400UL
#define USART2_CR1                 MMIO32(USART2_BASE + 0x00UL)
#define USART2_CR2                 MMIO32(USART2_BASE + 0x04UL)
#define USART2_CR3                 MMIO32(USART2_BASE + 0x08UL)
#define USART2_BRR                 MMIO32(USART2_BASE + 0x0CUL)
#define USART2_RQR                 MMIO32(USART2_BASE + 0x18UL)
#define USART2_ISR                 MMIO32(USART2_BASE + 0x1CUL)
#define USART2_ICR                 MMIO32(USART2_BASE + 0x20UL)
#define USART2_RDR                 MMIO32(USART2_BASE + 0x24UL)
#define USART2_TDR                 MMIO32(USART2_BASE + 0x28UL)

/* LPUART1 */
#define LPUART1_BASE               0x40004800UL
#define LPUART1_CR1                MMIO32(LPUART1_BASE + 0x00UL)
#define LPUART1_CR2                MMIO32(LPUART1_BASE + 0x04UL)
#define LPUART1_CR3                MMIO32(LPUART1_BASE + 0x08UL)
#define LPUART1_BRR                MMIO32(LPUART1_BASE + 0x0CUL)
#define LPUART1_ISR                MMIO32(LPUART1_BASE + 0x1CUL)
#define LPUART1_ICR                MMIO32(LPUART1_BASE + 0x20UL)
#define LPUART1_RDR                MMIO32(LPUART1_BASE + 0x24UL)
#define LPUART1_TDR                MMIO32(LPUART1_BASE + 0x28UL)

#define USART_CR1_UE               (1UL << 0)
#define USART_CR1_RE               (1UL << 2)
#define USART_CR1_TE               (1UL << 3)
#define USART_CR1_IDLEIE           (1UL << 4)

#define USART_CR3_DMAR             (1UL << 6)

#define USART_ISR_PE               (1UL << 0)
#define USART_ISR_FE               (1UL << 1)
#define USART_ISR_NE               (1UL << 2)
#define USART_ISR_ORE              (1UL << 3)
#define USART_ISR_IDLE             (1UL << 4)
#define USART_ISR_RXNE             (1UL << 5)
#define USART_ISR_TC               (1UL << 6)
#define USART_ISR_TXE              (1UL << 7)

#define USART_ICR_PECF             (1UL << 0)
#define USART_ICR_FECF             (1UL << 1)
#define USART_ICR_NCF              (1UL << 2)
#define USART_ICR_ORECF            (1UL << 3)
#define USART_ICR_IDLECF           (1UL << 4)

/* SPI1 */
#define SPI1_BASE                  0x40013000UL
#define SPI1_CR1                   MMIO32(SPI1_BASE + 0x00UL)
#define SPI1_CR2                   MMIO32(SPI1_BASE + 0x04UL)
#define SPI1_SR                    MMIO32(SPI1_BASE + 0x08UL)
#define SPI1_DR8                   MMIO8(SPI1_BASE + 0x0CUL)

#define SPI_CR1_CPHA               (1UL << 0)
#define SPI_CR1_CPOL               (1UL << 1)
#define SPI_CR1_MSTR               (1UL << 2)
#define SPI_CR1_BR_DIV4            (1UL << 3)
#define SPI_CR1_BR_DIV8            (2UL << 3)
#define SPI_CR1_SPE                (1UL << 6)
#define SPI_CR1_LSBFIRST           (1UL << 7)
#define SPI_CR1_SSI                (1UL << 8)
#define SPI_CR1_SSM                (1UL << 9)

#define SPI_CR2_DS_8BIT            (7UL << 8)
#define SPI_CR2_FRXTH              (1UL << 12)

#define SPI_SR_RXNE                (1UL << 0)
#define SPI_SR_TXE                 (1UL << 1)
#define SPI_SR_BSY                 (1UL << 7)

/* SYSCFG + EXTI */
#define SYSCFG_BASE                0x40010000UL
#define SYSCFG_CFGR1               MMIO32(SYSCFG_BASE + 0x00UL)
#define SYSCFG_EXTICR1             MMIO32(SYSCFG_BASE + 0x08UL)
#define SYSCFG_EXTICR2             MMIO32(SYSCFG_BASE + 0x0CUL)
#define SYSCFG_EXTICR3             MMIO32(SYSCFG_BASE + 0x10UL)
#define SYSCFG_EXTICR4             MMIO32(SYSCFG_BASE + 0x14UL)
#define SYSCFG_EXTICR(group)       MMIO32(SYSCFG_BASE + 0x08UL + (((group) - 1UL) * 4UL))

#define SYSCFG_CFGR1_MEM_MODE_MASK           (3UL << 0)
#define SYSCFG_CFGR1_MEM_MODE_SYSTEM_FLASH   (1UL << 0)

#define EXTI_BASE                  0x40010400UL
#define EXTI_IMR                   MMIO32(EXTI_BASE + 0x00UL)
#define EXTI_EMR                   MMIO32(EXTI_BASE + 0x04UL)
#define EXTI_RTSR                  MMIO32(EXTI_BASE + 0x08UL)
#define EXTI_FTSR                  MMIO32(EXTI_BASE + 0x0CUL)
#define EXTI_SWIER                 MMIO32(EXTI_BASE + 0x10UL)
#define EXTI_PR                    MMIO32(EXTI_BASE + 0x14UL)

/* SysTick + SCB */
#define SYST_CSR                   MMIO32(0xE000E010UL)
#define SYST_RVR                   MMIO32(0xE000E014UL)
#define SYST_CVR                   MMIO32(0xE000E018UL)

#define SYST_CSR_ENABLE            (1UL << 0)
#define SYST_CSR_TICKINT           (1UL << 1)
#define SYST_CSR_CLKSOURCE         (1UL << 2)
#define SYST_CSR_COUNTFLAG         (1UL << 16)

#define SCB_VTOR                   MMIO32(0xE000ED08UL)
#define SCB_AIRCR                  MMIO32(0xE000ED0CUL)

#define SCB_AIRCR_VECTKEY          (0x5FAUL << 16)
#define SCB_AIRCR_SYSRESETREQ      (1UL << 2)

/* NVIC */
#define NVIC_ISER0                 MMIO32(0xE000E100UL)
#define NVIC_ICER0                 MMIO32(0xE000E180UL)
#define NVIC_IPR_BASE              0xE000E400UL
#define NVIC_IPR(irqn)             MMIO8(NVIC_IPR_BASE + (irqn))

/* IRQ numbers for STM32L072 family. */
#define EXTI0_1_IRQn               5U
#define EXTI4_15_IRQn              7U
#define DMA1_Channel4_5_6_7_IRQn   11U
#define USART2_IRQn                27U

static inline void nvic_enable_irq(uint32_t irqn, uint8_t priority) {
    NVIC_IPR(irqn) = (uint8_t)(priority << 4);
    NVIC_ISER0 = (1UL << irqn);
}

static inline uint32_t cpu_irq_save(void) {
    uint32_t primask;
    __asm__ volatile ("mrs %0, primask" : "=r"(primask));
    __asm__ volatile ("cpsid i" : : : "memory");
    return primask;
}

static inline void cpu_irq_restore(uint32_t primask) {
    __asm__ volatile ("msr primask, %0" : : "r"(primask) : "memory");
}

static inline void cpu_irq_disable(void) {
    __asm__ volatile ("cpsid i" : : : "memory");
}

static inline void cpu_irq_enable(void) {
    __asm__ volatile ("cpsie i" : : : "memory");
}

static inline void cpu_wfi(void) {
    __asm__ volatile ("wfi");
}

#endif /* LIFETRAC_MURATA_L072_REGS_H */
