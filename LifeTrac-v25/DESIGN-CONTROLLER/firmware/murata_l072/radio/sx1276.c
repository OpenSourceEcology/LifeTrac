#include "sx1276.h"
#include "sx1276_modes.h"
#include "platform.h"
#include "stm32l072_regs.h"

#include <stddef.h>

/* Default Murata CMWX1ZZABZ pin map used in this first increment. */
#define SX1276_NSS_PORT               GPIOA_BASE
#define SX1276_NSS_PIN                15U

#define SX1276_RESET_PORT             GPIOC_BASE
#define SX1276_RESET_PIN              0U

#define SX1276_DIO0_PORT              GPIOB_BASE
#define SX1276_DIO0_PIN               4U
#define SX1276_DIO1_PORT              GPIOB_BASE
#define SX1276_DIO1_PIN               1U
#define SX1276_DIO2_PORT              GPIOB_BASE
#define SX1276_DIO2_PIN               0U
#define SX1276_DIO3_PORT              GPIOC_BASE
#define SX1276_DIO3_PIN               13U

#define SX1276_RF_SW_TXRX_PORT        GPIOA_BASE
#define SX1276_RF_SW_TXRX_PIN         1U
#define SX1276_RF_SW_RX_PORT          GPIOC_BASE
#define SX1276_RF_SW_RX_PIN           1U
#define SX1276_RF_SW_TX_BOOST_PORT    GPIOC_BASE
#define SX1276_RF_SW_TX_BOOST_PIN     2U

#define SX1276_REG_FRF_MSB            0x06U
#define SX1276_REG_FRF_MID            0x07U
#define SX1276_REG_FRF_LSB            0x08U
#define SX1276_REG_PA_CONFIG          0x09U
#define SX1276_REG_MODEM_CONFIG1      0x1DU
#define SX1276_REG_MODEM_CONFIG2      0x1EU
#define SX1276_REG_MODEM_CONFIG3      0x26U
#define SX1276_REG_DETECT_OPTIMIZE    0x31U
#define SX1276_REG_DETECTION_THRESH   0x37U
#define SX1276_REG_VERSION            0x42U

static volatile uint32_t s_irq_events;

static uint32_t exti_port_code(uint32_t port) {
    if (port == GPIOB_BASE) {
        return 1U;
    }
    if (port == GPIOC_BASE) {
        return 2U;
    }
    return 0U;
}

static void gpio_mode_output(uint32_t port, uint8_t pin) {
    uint32_t moder = GPIO_MODER(port);
    moder &= ~(3UL << (pin * 2U));
    moder |= (1UL << (pin * 2U));
    GPIO_MODER(port) = moder;

    GPIO_OTYPER(port) &= ~(1UL << pin);

    uint32_t speed = GPIO_OSPEEDR(port);
    speed &= ~(3UL << (pin * 2U));
    speed |= (2UL << (pin * 2U));
    GPIO_OSPEEDR(port) = speed;

    uint32_t pupd = GPIO_PUPDR(port);
    pupd &= ~(3UL << (pin * 2U));
    GPIO_PUPDR(port) = pupd;
}

static void gpio_mode_input(uint32_t port, uint8_t pin) {
    uint32_t moder = GPIO_MODER(port);
    moder &= ~(3UL << (pin * 2U));
    GPIO_MODER(port) = moder;

    uint32_t pupd = GPIO_PUPDR(port);
    pupd &= ~(3UL << (pin * 2U));
    GPIO_PUPDR(port) = pupd;
}

static void gpio_mode_alt(uint32_t port, uint8_t pin, uint8_t af) {
    uint32_t moder = GPIO_MODER(port);
    moder &= ~(3UL << (pin * 2U));
    moder |= (2UL << (pin * 2U));
    GPIO_MODER(port) = moder;

    GPIO_OTYPER(port) &= ~(1UL << pin);

    uint32_t speed = GPIO_OSPEEDR(port);
    speed &= ~(3UL << (pin * 2U));
    speed |= (2UL << (pin * 2U));
    GPIO_OSPEEDR(port) = speed;

    uint32_t pupd = GPIO_PUPDR(port);
    pupd &= ~(3UL << (pin * 2U));
    GPIO_PUPDR(port) = pupd;

    if (pin < 8U) {
        uint32_t afrl = GPIO_AFRL(port);
        afrl &= ~(0xFUL << (pin * 4U));
        afrl |= ((uint32_t)af << (pin * 4U));
        GPIO_AFRL(port) = afrl;
    } else {
        const uint8_t pin_hi = (uint8_t)(pin - 8U);
        uint32_t afrh = GPIO_AFRH(port);
        afrh &= ~(0xFUL << (pin_hi * 4U));
        afrh |= ((uint32_t)af << (pin_hi * 4U));
        GPIO_AFRH(port) = afrh;
    }
}

static void gpio_write(uint32_t port, uint8_t pin, uint8_t value) {
    if (value != 0U) {
        GPIO_BSRR(port) = (1UL << pin);
    } else {
        GPIO_BSRR(port) = (1UL << (pin + 16U));
    }
}

static void sx1276_select(uint8_t selected) {
    gpio_write(SX1276_NSS_PORT, SX1276_NSS_PIN, (uint8_t)(selected == 0U));
}

static uint8_t spi1_transfer(uint8_t value) {
    while ((SPI1_SR & SPI_SR_TXE) == 0U) {
    }

    SPI1_DR8 = value;

    while ((SPI1_SR & SPI_SR_RXNE) == 0U) {
    }

    while ((SPI1_SR & SPI_SR_BSY) != 0U) {
    }

    return SPI1_DR8;
}

static void exti_route_line(uint8_t line, uint32_t port) {
    const uint32_t group = (line / 4U) + 1U;
    const uint32_t shift = (uint32_t)(line % 4U) * 4U;
    uint32_t exticr = SYSCFG_EXTICR(group);
    exticr &= ~(0xFUL << shift);
    exticr |= (exti_port_code(port) << shift);
    SYSCFG_EXTICR(group) = exticr;
}

static uint8_t bw_to_reg_bits(uint16_t bw_khz) {
    if (bw_khz >= 500U) {
        return 9U;
    }
    if (bw_khz >= 250U) {
        return 8U;
    }
    return 7U;
}

static void sx1276_set_rf_switch_tx(uint8_t tx_mode) {
    if (tx_mode != 0U) {
        gpio_write(SX1276_RF_SW_TXRX_PORT, SX1276_RF_SW_TXRX_PIN, 1U);
        gpio_write(SX1276_RF_SW_RX_PORT, SX1276_RF_SW_RX_PIN, 0U);
        gpio_write(SX1276_RF_SW_TX_BOOST_PORT, SX1276_RF_SW_TX_BOOST_PIN, 1U);
    } else {
        gpio_write(SX1276_RF_SW_TXRX_PORT, SX1276_RF_SW_TXRX_PIN, 0U);
        gpio_write(SX1276_RF_SW_RX_PORT, SX1276_RF_SW_RX_PIN, 1U);
        gpio_write(SX1276_RF_SW_TX_BOOST_PORT, SX1276_RF_SW_TX_BOOST_PIN, 0U);
    }
}

static void sx1276_gpio_init(void) {
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN |
                  RCC_IOPENR_GPIOBEN |
                  RCC_IOPENR_GPIOCEN;

    /* W1-8 fix (2026-05-11): SX1276 SCK is wired to PB3 inside the Murata
     * CMWX1ZZABZ-078 SiP.  PA5 is wired to SX1276 DIO4.  Driving PA5 as
     * SPI1_SCK clocks DIO4 instead of SCK and the SX1276 SPI engine never
     * shifts -- RegVersion (0x42) reads 0x00.  Reference: B-L072Z-LRWAN1
     * schematic; lora_ping.c gpio_spi_init() also uses PB3=SCK and proves
     * SPI works on this hardware unit. */
    gpio_mode_alt(GPIOB_BASE, 3U, 0U);   /* SCK  = PB3 (AF0 = SPI1_SCK)  */
    gpio_mode_alt(GPIOA_BASE, 6U, 0U);   /* MISO = PA6 (AF0 = SPI1_MISO) */
    gpio_mode_alt(GPIOA_BASE, 7U, 0U);   /* MOSI = PA7 (AF0 = SPI1_MOSI) */

    /* Pull-up on PA6 (MISO) -- prevents floating reads if SX1276 ever
     * tristates MISO mid-frame.  lora_ping.c uses the same pull-up. */
    {
        uint32_t pupd = GPIO_PUPDR(GPIOA_BASE);
        pupd &= ~(3UL << (6U * 2U));
        pupd |= (1UL << (6U * 2U));      /* PUPDR = 01 = pull-up */
        GPIO_PUPDR(GPIOA_BASE) = pupd;
    }

    gpio_mode_output(SX1276_NSS_PORT, SX1276_NSS_PIN);
    sx1276_select(0U);

    gpio_mode_output(SX1276_RESET_PORT, SX1276_RESET_PIN);
    gpio_write(SX1276_RESET_PORT, SX1276_RESET_PIN, 1U);

    gpio_mode_output(SX1276_RF_SW_TXRX_PORT, SX1276_RF_SW_TXRX_PIN);
    gpio_mode_output(SX1276_RF_SW_RX_PORT, SX1276_RF_SW_RX_PIN);
    gpio_mode_output(SX1276_RF_SW_TX_BOOST_PORT, SX1276_RF_SW_TX_BOOST_PIN);
    sx1276_set_rf_switch_tx(0U);

    gpio_mode_input(SX1276_DIO0_PORT, SX1276_DIO0_PIN);
    gpio_mode_input(SX1276_DIO1_PORT, SX1276_DIO1_PIN);
    gpio_mode_input(SX1276_DIO2_PORT, SX1276_DIO2_PIN);
    gpio_mode_input(SX1276_DIO3_PORT, SX1276_DIO3_PIN);
}

static void sx1276_spi_init(void) {
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC_APB2ENR;   /* ensure APB2 clock has propagated to SPI1 */

    /* W1-8 (2026-05-11): if the boot ROM or a previous init left SPI1
     * active, wait for idle before clearing CR1 -- same precaution as
     * lora_ping.c gpio_spi_init(). */
    while ((SPI1_SR & SPI_SR_BSY) != 0U) {
    }

    SPI1_CR1 = 0U;
    SPI1_CR2 = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH;

    SPI1_CR1 = SPI_CR1_MSTR |
               SPI_CR1_SSM |
               SPI_CR1_SSI |
               SPI_CR1_BR_DIV64;
    SPI1_CR1 |= SPI_CR1_SPE;
}

static void sx1276_exti_init(void) {
    const uint32_t dio_mask = (1UL << SX1276_DIO0_PIN) |
                              (1UL << SX1276_DIO1_PIN) |
                              (1UL << SX1276_DIO2_PIN) |
                              (1UL << SX1276_DIO3_PIN);

    RCC_APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    exti_route_line(SX1276_DIO0_PIN, SX1276_DIO0_PORT);
    exti_route_line(SX1276_DIO1_PIN, SX1276_DIO1_PORT);
    exti_route_line(SX1276_DIO2_PIN, SX1276_DIO2_PORT);
    exti_route_line(SX1276_DIO3_PIN, SX1276_DIO3_PORT);

    EXTI_IMR |= dio_mask;
    EXTI_RTSR |= dio_mask;
    EXTI_FTSR &= ~dio_mask;
    EXTI_PR = dio_mask;

    platform_irq_enable(EXTI0_1_IRQn, 3U);
    platform_irq_enable(EXTI4_15_IRQn, 3U);
}

bool sx1276_init(void) {
    sx1276_gpio_init();
    sx1276_spi_init();
    sx1276_exti_init();
    sx1276_radio_reset();

    if (!sx1276_modes_init()) {
        return false;
    }

    const uint8_t version = sx1276_read_version();
    if (version == 0x00U || version == 0xFFU) {
        return false;
    }

    sx1276_set_frequency_hz(915000000UL);
    sx1276_set_sf_bw_cr(7U, 250U, 5U);
    sx1276_set_tx_power_dbm(14U);

    return true;
}

void sx1276_radio_reset(void) {
    /* W1-9c (2026-05-12): match Semtech LoRaMac-node SX1276Reset() pattern.
     * Reference: Lora-net/LoRaMac-node src/boards/B-L072Z-LRWAN1/sx1276-board.c
     *
     *   GpioInit(Reset, OUTPUT, PUSH_PULL, NO_PULL, 0); // drive LOW
     *   DelayMs(1);
     *   GpioInit(Reset, INPUT, PUSH_PULL, NO_PULL, 1);  // Hi-Z release
     *   DelayMs(6);
     *
     * Releasing NRESET as Hi-Z (and letting the SX1276's internal pull-up
     * raise the line) avoids the MCU's push-pull HIGH driver fighting the
     * chip's internal pull-up during the brief window when the SX1276
     * digital block is sampling NRESET.  Active push-pull HIGH release is
     * a known cause of the chip latching into a state where SPI register
     * writes succeed but RegOpMode mode-bit transitions are silently
     * ignored -- exactly the W1-9 OPMODE-stuck symptom.
     */
    gpio_mode_output(SX1276_RESET_PORT, SX1276_RESET_PIN);
    gpio_write(SX1276_RESET_PORT, SX1276_RESET_PIN, 0U);
    platform_delay_ms(1U);
    gpio_mode_input(SX1276_RESET_PORT, SX1276_RESET_PIN);
    platform_delay_ms(6U);
}

uint8_t sx1276_read_reg(uint8_t reg_addr) {
    uint8_t value;

    sx1276_select(1U);
    (void)spi1_transfer((uint8_t)(reg_addr & 0x7FU));
    value = spi1_transfer(0x00U);
    sx1276_select(0U);

    return value;
}

void sx1276_write_reg(uint8_t reg_addr, uint8_t value) {
    sx1276_select(1U);
    (void)spi1_transfer((uint8_t)(reg_addr | 0x80U));
    (void)spi1_transfer(value);
    sx1276_select(0U);
}

void sx1276_read_burst(uint8_t start_reg, uint8_t *dst, size_t len) {
    sx1276_select(1U);
    (void)spi1_transfer((uint8_t)(start_reg & 0x7FU));
    for (size_t i = 0U; i < len; ++i) {
        dst[i] = spi1_transfer(0x00U);
    }
    sx1276_select(0U);
}

void sx1276_write_burst(uint8_t start_reg, const uint8_t *src, size_t len) {
    sx1276_select(1U);
    (void)spi1_transfer((uint8_t)(start_reg | 0x80U));
    for (size_t i = 0U; i < len; ++i) {
        (void)spi1_transfer(src[i]);
    }
    sx1276_select(0U);
}

uint8_t sx1276_read_version(void) {
    return sx1276_read_reg(SX1276_REG_VERSION);
}

void sx1276_set_frequency_hz(uint32_t freq_hz) {
    const uint64_t frf = (((uint64_t)freq_hz) << 19) / 32000000ULL;

    sx1276_write_reg(SX1276_REG_FRF_MSB, (uint8_t)((frf >> 16) & 0xFFU));
    sx1276_write_reg(SX1276_REG_FRF_MID, (uint8_t)((frf >> 8) & 0xFFU));
    sx1276_write_reg(SX1276_REG_FRF_LSB, (uint8_t)(frf & 0xFFU));
}

void sx1276_set_tx_power_dbm(uint8_t dbm) {
    uint8_t clipped = dbm;
    if (clipped < 2U) {
        clipped = 2U;
    }
    if (clipped > 17U) {
        clipped = 17U;
    }

    sx1276_write_reg(SX1276_REG_PA_CONFIG, (uint8_t)(0x80U | (clipped - 2U)));
}

void sx1276_set_sf_bw_cr(uint8_t sf, uint16_t bw_khz, uint8_t cr_den) {
    uint8_t sf_use = sf;
    uint8_t cr_use = cr_den;
    const uint8_t bw_bits = bw_to_reg_bits(bw_khz);
    uint8_t cfg3;

    if (sf_use < 6U) {
        sf_use = 6U;
    }
    if (sf_use > 12U) {
        sf_use = 12U;
    }

    if (cr_use < 5U) {
        cr_use = 5U;
    }
    if (cr_use > 8U) {
        cr_use = 8U;
    }

    sx1276_write_reg(SX1276_REG_MODEM_CONFIG1,
                     (uint8_t)((bw_bits << 4) | ((cr_use - 4U) << 1) | 0U));
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG2,
                     (uint8_t)((sf_use << 4) | (1U << 2)));

    cfg3 = sx1276_read_reg(SX1276_REG_MODEM_CONFIG3);
    cfg3 |= (1U << 2);
    if (sf_use >= 11U && bw_khz <= 125U) {
        cfg3 |= (1U << 3);
    } else {
        cfg3 &= ~(1U << 3);
    }
    sx1276_write_reg(SX1276_REG_MODEM_CONFIG3, cfg3);

    if (sf_use == 6U) {
        sx1276_write_reg(SX1276_REG_DETECT_OPTIMIZE,
                         (uint8_t)((sx1276_read_reg(SX1276_REG_DETECT_OPTIMIZE) & 0xF8U) | 0x05U));
        sx1276_write_reg(SX1276_REG_DETECTION_THRESH, 0x0CU);
    } else {
        sx1276_write_reg(SX1276_REG_DETECT_OPTIMIZE,
                         (uint8_t)((sx1276_read_reg(SX1276_REG_DETECT_OPTIMIZE) & 0xF8U) | 0x03U));
        sx1276_write_reg(SX1276_REG_DETECTION_THRESH, 0x0AU);
    }
}

void sx1276_apply_profile_full(const sx1276_profile_t *profile) {
    if (profile == NULL) {
        return;
    }

    if (!sx1276_modes_to_standby()) {
        return;
    }
    sx1276_set_frequency_hz(profile->freq_hz);
    sx1276_set_sf_bw_cr(profile->sf, profile->bw_khz, profile->cr_den);
    sx1276_set_tx_power_dbm(profile->tx_power_dbm);
}

uint32_t sx1276_take_irq_events(void) {
    uint32_t irq_state = cpu_irq_save();
    uint32_t events = s_irq_events;
    s_irq_events = 0U;
    cpu_irq_restore(irq_state);
    return events;
}

bool sx1276_reg_dump(uint8_t *out_regs, size_t out_len) {
    if (out_regs == NULL || out_len < 0x43U) {
        return false;
    }

    for (uint8_t reg = 0U; reg <= SX1276_REG_VERSION; ++reg) {
        out_regs[reg] = sx1276_read_reg(reg);
    }

    return true;
}

void EXTI0_1_IRQHandler(void) {
    const uint32_t line_mask = (1UL << SX1276_DIO1_PIN) | (1UL << SX1276_DIO2_PIN);
    const uint32_t pending = EXTI_PR & line_mask;

    if (pending == 0U) {
        return;
    }

    EXTI_PR = pending;

    if ((pending & (1UL << SX1276_DIO1_PIN)) != 0U) {
        s_irq_events |= SX1276_EVT_DIO1;
    }
    if ((pending & (1UL << SX1276_DIO2_PIN)) != 0U) {
        s_irq_events |= SX1276_EVT_DIO2;
    }
}

void EXTI4_15_IRQHandler(void) {
    const uint32_t line_mask = (1UL << SX1276_DIO0_PIN) | (1UL << SX1276_DIO3_PIN);
    const uint32_t pending = EXTI_PR & line_mask;

    if (pending == 0U) {
        return;
    }

    EXTI_PR = pending;

    if ((pending & (1UL << SX1276_DIO0_PIN)) != 0U) {
        s_irq_events |= SX1276_EVT_DIO0;
    }
    if ((pending & (1UL << SX1276_DIO3_PIN)) != 0U) {
        s_irq_events |= SX1276_EVT_DIO3;
    }
}
