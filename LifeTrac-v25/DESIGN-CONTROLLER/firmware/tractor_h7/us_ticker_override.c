/*
 * us_ticker_override.c
 *
 * Portenta X8-safe mbed us_ticker override for STM32H747 M7.
 * The archive us_ticker.o spins in init_32bit_timer() because it waits for a
 * CM4 HSEM core-id value (0x80000100) while the M7 hardware returns 0x80000300.
 * This implementation drives TIM2 directly as a 1 MHz 32-bit ticker.
 */

#include <stdint.h>

#define RCC_BASE       0x58024400UL
#define RCC_APB1LRSTR  (*(volatile uint32_t *)(RCC_BASE + 0x090))
#define RCC_APB1LENR   (*(volatile uint32_t *)(RCC_BASE + 0x0E8))

#define TIM2_BASE      0x40000000UL
#define TIM2_CR1       (*(volatile uint32_t *)(TIM2_BASE + 0x00))
#define TIM2_DIER      (*(volatile uint32_t *)(TIM2_BASE + 0x0C))
#define TIM2_SR        (*(volatile uint32_t *)(TIM2_BASE + 0x10))
#define TIM2_EGR       (*(volatile uint32_t *)(TIM2_BASE + 0x14))
#define TIM2_CNT       (*(volatile uint32_t *)(TIM2_BASE + 0x24))
#define TIM2_PSC       (*(volatile uint32_t *)(TIM2_BASE + 0x28))
#define TIM2_ARR       (*(volatile uint32_t *)(TIM2_BASE + 0x2C))
#define TIM2_CCR1      (*(volatile uint32_t *)(TIM2_BASE + 0x34))

#define TIM5_BASE      0x40000C00UL
#define TIM5_CR1       (*(volatile uint32_t *)(TIM5_BASE + 0x00))
#define TIM5_DIER      (*(volatile uint32_t *)(TIM5_BASE + 0x0C))
#define TIM5_SR        (*(volatile uint32_t *)(TIM5_BASE + 0x10))
#define TIM5_EGR       (*(volatile uint32_t *)(TIM5_BASE + 0x14))
#define TIM5_CNT       (*(volatile uint32_t *)(TIM5_BASE + 0x24))
#define TIM5_PSC       (*(volatile uint32_t *)(TIM5_BASE + 0x28))
#define TIM5_ARR       (*(volatile uint32_t *)(TIM5_BASE + 0x2C))

#define NVIC_ISER0     (*(volatile uint32_t *)0xE000E100UL)
#define NVIC_ISPR0     (*(volatile uint32_t *)0xE000E200UL)

#define APB1LENR_TIM2EN    (1UL << 0)
#define APB1LRSTR_TIM2RST  (1UL << 0)
#define APB1LENR_TIM5EN    (1UL << 3)
#define APB1LRSTR_TIM5RST  (1UL << 3)
#define TIM_CR1_CEN        (1UL << 0)
#define TIM_DIER_CC1IE     (1UL << 1)
#define TIM_SR_CC1IF       (1UL << 1)
#define TIM_EGR_UG         (1UL << 0)
#define TIM2_IRQn          28U

typedef struct {
    uint32_t frequency;
    uint32_t bits;
} ticker_info_t;

static const ticker_info_t us_ticker_info_tim2 = {
    .frequency = 1000000UL,
    .bits = 32UL,
};

static volatile uint32_t us_ticker_initialized = 0;

extern void us_ticker_irq_handler(void);
void us_ticker_init(void);

void init_32bit_timer(void)
{
    us_ticker_init();
}

void save_timer_ctx(void)
{
}

void restore_timer_ctx(void)
{
    us_ticker_init();
}

void TIM2_IRQHandler(void)
{
    TIM2_SR &= ~TIM_SR_CC1IF;
    us_ticker_irq_handler();
}

void us_ticker_init(void)
{
    if (us_ticker_initialized) {
        return;
    }
    us_ticker_initialized = 1U;

    RCC_APB1LENR |= APB1LENR_TIM2EN | APB1LENR_TIM5EN;
    (void)RCC_APB1LENR;

    RCC_APB1LRSTR |= APB1LRSTR_TIM2RST;
    RCC_APB1LRSTR &= ~APB1LRSTR_TIM2RST;
    RCC_APB1LRSTR |= APB1LRSTR_TIM5RST;
    RCC_APB1LRSTR &= ~APB1LRSTR_TIM5RST;

    TIM2_CR1 = 0;
    TIM2_DIER = 0;
    TIM2_PSC = 199UL;
    TIM2_ARR = 0xFFFFFFFFUL;
    TIM2_CNT = 0;
    TIM2_EGR = TIM_EGR_UG;
    TIM2_SR = 0;
    TIM2_CR1 = TIM_CR1_CEN;

    TIM5_CR1 = 0;
    TIM5_DIER = 0;
    TIM5_PSC = 199UL;
    TIM5_ARR = 0xFFFFFFFFUL;
    TIM5_CNT = 0;
    TIM5_EGR = TIM_EGR_UG;
    TIM5_SR = 0;
    TIM5_CR1 = TIM_CR1_CEN;

    NVIC_ISER0 = (1UL << TIM2_IRQn);
}

void us_ticker_free(void)
{
    us_ticker_disable_interrupt();
    TIM2_CR1 = 0;
    TIM5_CR1 = 0;
    us_ticker_initialized = 0U;
}

uint32_t us_ticker_read(void)
{
    return TIM2_CNT;
}

void us_ticker_set_interrupt(uint32_t timestamp)
{
    TIM2_CCR1 = timestamp;
    TIM2_SR &= ~TIM_SR_CC1IF;
    TIM2_DIER |= TIM_DIER_CC1IE;
}

void us_ticker_fire_interrupt(void)
{
    NVIC_ISPR0 = (1UL << TIM2_IRQn);
}

void us_ticker_disable_interrupt(void)
{
    TIM2_DIER &= ~TIM_DIER_CC1IE;
}

void us_ticker_clear_interrupt(void)
{
    TIM2_SR &= ~TIM_SR_CC1IF;
}

const ticker_info_t *us_ticker_get_info(void)
{
    return &us_ticker_info_tim2;
}
