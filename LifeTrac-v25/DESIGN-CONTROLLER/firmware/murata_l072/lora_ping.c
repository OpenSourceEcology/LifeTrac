/*
 * lora_ping.c  — Minimal LoRa connection test for two Portenta X8 Max Carriers.
 *
 * Compile with -DPING_ROLE=0 for the TX pinger (Unit A).
 * Compile with -DPING_ROLE=1 for the RX ponger (Unit B).
 *
 *   Unit A sends "PING" + 2-byte counter every ~2 s, then listens 1 s for "PONG".
 *   Unit B listens continuously, prints RSSI/SNR, and replies with "PONG".
 *
 * PHY:  SF7 / BW 125 kHz / CR 4/5 / 915.0 MHz / +14 dBm PA_BOOST / sync 0x12
 * UART: LPUART1 (PA2/PA3 AF6) + USART1 (PA9/PA10 AF4) @ 19200 8N1 on HSI16
 *       Both outputs active so /dev/ttymxc3 (LPUART1) is captured by the X8.
 *
 * Build sources: startup.c hal/platform.c lora_ping.c include/static_asserts.c
 */

#include <stddef.h>
#include <stdint.h>

#include "dev_build_policy.h"
#include "platform.h"
#include "stm32l072_regs.h"

#ifndef PING_ROLE
#define PING_ROLE 0   /* 0 = TX pinger, 1 = RX ponger */
#endif

/* ------------------------------------------------------------------
 * SX1276 register addresses
 * ------------------------------------------------------------------ */
#define SX_FIFO                0x00U
#define SX_OP_MODE             0x01U
#define SX_FRF_MSB             0x06U
#define SX_FRF_MID             0x07U
#define SX_FRF_LSB             0x08U
#define SX_PA_CONFIG           0x09U
#define SX_FIFO_ADDR_PTR       0x0DU
#define SX_FIFO_TX_BASE        0x0EU
#define SX_FIFO_RX_BASE        0x0FU
#define SX_FIFO_RX_CUR         0x10U
#define SX_IRQ_FLAGS           0x12U
#define SX_RX_NB_BYTES         0x13U
#define SX_PKT_SNR             0x19U
#define SX_PKT_RSSI            0x1AU
#define SX_MODEM_CFG1          0x1DU
#define SX_MODEM_CFG2          0x1EU
#define SX_PREAMBLE_MSB        0x20U
#define SX_PREAMBLE_LSB        0x21U
#define SX_PAYLOAD_LEN         0x22U
#define SX_MODEM_CFG3          0x26U
#define SX_DETECT_OPTIMIZE     0x31U
#define SX_DETECTION_THRESH    0x37U
#define SX_SYNC_WORD           0x39U
#define SX_DIO_MAPPING1        0x40U
#define SX_VERSION             0x42U

/* LoRa op-mode constants (MSB=1 selects LoRa) */
#define OPMODE_LORA_SLEEP      0x80U
#define OPMODE_LORA_STDBY      0x81U
#define OPMODE_LORA_TX         0x83U
#define OPMODE_LORA_RX_CONT    0x85U

/* IRQ flag bits */
#define IRQ_TX_DONE            (1U << 3)
#define IRQ_RX_DONE            (1U << 6)
#define IRQ_CRC_ERR            (1U << 5)

/* ------------------------------------------------------------------
 * Pin map — Murata CMWX1ZZABZ-078 on Max Carrier
 * ------------------------------------------------------------------ */
/* SPI1 */
#define SX_NSS_PORT            GPIOA_BASE
#define SX_NSS_PIN             15U
/* SPI SCK=PA5 AF0, MISO=PA6 AF0, MOSI=PA7 AF0 (configured below) */

/* Hard reset */
#define SX_RST_PORT            GPIOC_BASE
#define SX_RST_PIN             0U

/* RF front-end switch */
#define RF_TXRX_PORT           GPIOA_BASE   /* PA1: HIGH=TX, LOW=RX */
#define RF_TXRX_PIN            1U
#define RF_RX_PORT             GPIOC_BASE   /* PC1: LOW=TX, HIGH=RX */
#define RF_RX_PIN              1U
#define RF_TXBOOST_PORT        GPIOC_BASE   /* PC2: HIGH=TX, LOW=RX */
#define RF_TXBOOST_PIN         2U

/* ------------------------------------------------------------------
 * GPIO helpers (minimal, no external dependencies)
 * ------------------------------------------------------------------ */

static void pin_output(uint32_t port, uint8_t pin)
{
    GPIO_MODER(port) = (GPIO_MODER(port) & ~(3UL << (pin * 2U)))
                      | (1UL << (pin * 2U));          /* output mode */
    GPIO_OTYPER(port) &= ~(1UL << pin);               /* push-pull   */
    GPIO_OSPEEDR(port) = (GPIO_OSPEEDR(port) & ~(3UL << (pin * 2U)))
                       | (2UL << (pin * 2U));          /* fast speed  */
    GPIO_PUPDR(port) &= ~(3UL << (pin * 2U));         /* no pull     */
}

static void pin_set(uint32_t port, uint8_t pin, uint8_t val)
{
    if (val != 0U) {
        GPIO_BSRR(port) = (1UL << pin);
    } else {
        GPIO_BSRR(port) = (1UL << (pin + 16U));
    }
}

/* ------------------------------------------------------------------
 * SPI1 raw helpers (matches sx1276.c spi1_transfer exactly)
 * ------------------------------------------------------------------ */

static void sx_nss(uint8_t assert)
{
    /* assert=1 → drive NSS low (active) */
    pin_set(SX_NSS_PORT, SX_NSS_PIN, (uint8_t)(assert == 0U));
}

static uint8_t spi_xfer(uint8_t byte)
{
    while ((SPI1_SR & SPI_SR_TXE) == 0U) {
    }
    SPI1_DR8 = byte;
    while ((SPI1_SR & SPI_SR_RXNE) == 0U) {
    }
    while ((SPI1_SR & SPI_SR_BSY) != 0U) {
    }
    return SPI1_DR8;
}

static uint8_t sx_rd(uint8_t reg)
{
    sx_nss(1U);
    (void)spi_xfer((uint8_t)(reg & 0x7FU));
    const uint8_t v = spi_xfer(0x00U);
    sx_nss(0U);
    return v;
}

static void sx_wr(uint8_t reg, uint8_t val)
{
    sx_nss(1U);
    (void)spi_xfer((uint8_t)(reg | 0x80U));
    (void)spi_xfer(val);
    sx_nss(0U);
}

static void sx_wr_burst(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    sx_nss(1U);
    (void)spi_xfer((uint8_t)(reg | 0x80U));
    for (uint8_t i = 0U; i < len; i++) {
        (void)spi_xfer(buf[i]);
    }
    sx_nss(0U);
}

static void sx_rd_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    sx_nss(1U);
    (void)spi_xfer((uint8_t)(reg & 0x7FU));
    for (uint8_t i = 0U; i < len; i++) {
        buf[i] = spi_xfer(0x00U);
    }
    sx_nss(0U);
}

/* ------------------------------------------------------------------
 * UART output (LPUART1 + USART1 simultaneously, same as hello_world.c)
 * ------------------------------------------------------------------ */

static void uart_putc(char c)
{
    while ((USART1_ISR & USART_ISR_TXE) == 0U) {
    }
    USART1_TDR = (uint8_t)c;

    while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {
    }
    LPUART1_TDR = (uint8_t)c;
}

static void uart_puts(const char *s)
{
    while (*s != '\0') {
        uart_putc(*s++);
    }
}

static void uart_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[(v >> 4) & 0xFU]);
    uart_putc(hex[v & 0xFU]);
}

static void uart_hex32(uint32_t v)
{
    uart_hex8((uint8_t)(v >> 24U));
    uart_hex8((uint8_t)(v >> 16U));
    uart_hex8((uint8_t)(v >>  8U));
    uart_hex8((uint8_t)(v));
}

static void uart_dec(int32_t v)
{
    char buf[12];
    uint8_t i = 0U;

    if (v < 0) {
        uart_putc('-');
        v = -v;
    }
    if (v == 0) {
        uart_putc('0');
        return;
    }
    while ((v > 0) && (i < (uint8_t)sizeof(buf))) {
        buf[i++] = (char)('0' + (v % 10));
        v /= 10;
    }
    while (i > 0U) {
        uart_putc(buf[--i]);
    }
}

/* ------------------------------------------------------------------
 * RF switch helpers
 * ------------------------------------------------------------------ */

static void rf_tx_path(void)
{
    pin_set(RF_TXRX_PORT,   RF_TXRX_PIN,   1U);
    pin_set(RF_RX_PORT,     RF_RX_PIN,     0U);
    pin_set(RF_TXBOOST_PORT, RF_TXBOOST_PIN, 1U);
}

static void rf_rx_path(void)
{
    pin_set(RF_TXRX_PORT,   RF_TXRX_PIN,   0U);
    pin_set(RF_RX_PORT,     RF_RX_PIN,     1U);
    pin_set(RF_TXBOOST_PORT, RF_TXBOOST_PIN, 0U);
}

/* ------------------------------------------------------------------
 * SX1276 hardware reset
 * ------------------------------------------------------------------ */

static void sx_hw_reset(void)
{
    pin_set(SX_RST_PORT, SX_RST_PIN, 0U);
    platform_delay_ms(2U);
    pin_set(SX_RST_PORT, SX_RST_PIN, 1U);
    platform_delay_ms(8U);
}

/* ------------------------------------------------------------------
 * SX1276 radio init
 * Returns version register (0x12 expected).  Returns 0 on fail.
 * ------------------------------------------------------------------ */

static uint8_t sx_radio_init(void)
{
    /* --- SPI diagnostic: read VERSION and OP_MODE before any writes --- */
    /* Print SPI peripheral + GPIO register states */
    uart_puts("DBG: SPI1_CR1=0x"); uart_hex32((uint32_t)MMIO16(SPI1_BASE + 0x00UL));
    uart_puts(" CR2=0x"); uart_hex32((uint32_t)MMIO16(SPI1_BASE + 0x04UL)); uart_puts("\r\n");
    uart_puts("DBG: GPIOA_MODER=0x"); uart_hex32(GPIO_MODER(GPIOA_BASE)); uart_puts("\r\n");
    uart_puts("DBG: GPIOA_AFRL=0x"); uart_hex32(GPIO_AFRL(GPIOA_BASE));
    uart_puts(" PUPDR=0x"); uart_hex32(GPIO_PUPDR(GPIOA_BASE)); uart_puts("\r\n");

    /* Normal read */
    const uint8_t ver0 = sx_rd(SX_VERSION);
    const uint8_t opm0 = sx_rd(SX_OP_MODE);
    uart_puts("DBG: pre-init  ver=0x");
    uart_hex8(ver0);
    uart_puts(" opm=0x");
    uart_hex8(opm0);
    uart_puts("\r\n");

    /* MOSI-MISO loopback test: use 0xA5 dummy — if result==0xA5 then MOSI=MISO shorted */
    uint8_t lb_result;
    sx_nss(1U);
    (void)spi_xfer((uint8_t)(SX_VERSION & 0x7FU));
    lb_result = spi_xfer(0xA5U);
    sx_nss(0U);
    uart_puts("DBG: loopback(A5)=0x"); uart_hex8(lb_result);
    uart_puts(" (A5=short,12=SX1276,00=SX stuck)\r\n");

    /* Switch to LoRa sleep so all config registers are writable */
    sx_wr(SX_OP_MODE, OPMODE_LORA_SLEEP);
    platform_delay_ms(5U);

    const uint8_t opm1 = sx_rd(SX_OP_MODE);
    const uint8_t ver  = sx_rd(SX_VERSION);
    uart_puts("DBG: post-sleep ver=0x");
    uart_hex8(ver);
    uart_puts(" opm=0x");
    uart_hex8(opm1);
    uart_puts("\r\n");

    if ((ver == 0x00U) || (ver == 0xFFU)) {
        return 0U;
    }

    /* Standby */
    sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
    platform_delay_ms(1U);

    /* Frequency: 915.0 MHz
     *   frf = (freq_hz * 2^19) / 32_000_000
     *       = (915_000_000 * 524288) / 32_000_000 = 0xE4C000
     */
    sx_wr(SX_FRF_MSB, 0xE4U);
    sx_wr(SX_FRF_MID, 0xC0U);
    sx_wr(SX_FRF_LSB, 0x00U);

    /* PA_CONFIG: PA_BOOST (bit7=1), +14 dBm
     *   Pout = 2 + OutputPower(bits[3:0])  =>  14 dBm => OutputPower = 12 = 0x0C
     */
    sx_wr(SX_PA_CONFIG, 0x8CU);

    /* MODEM_CONFIG1: BW=125 kHz (0x7<<4), CR=4/5 (0x1<<1), explicit header */
    sx_wr(SX_MODEM_CFG1, 0x72U);

    /* MODEM_CONFIG2: SF=7 (0x7<<4), TxCont=0, CRC=1 (bit2) */
    sx_wr(SX_MODEM_CFG2, 0x74U);

    /* MODEM_CONFIG3: AGC auto on (bit2) */
    sx_wr(SX_MODEM_CFG3, 0x04U);

    /* Preamble length = 8 symbols */
    sx_wr(SX_PREAMBLE_MSB, 0x00U);
    sx_wr(SX_PREAMBLE_LSB, 0x08U);

    /* Private LoRa network sync word */
    sx_wr(SX_SYNC_WORD, 0x12U);

    /* SF >= 7: detection optimiser = 0x03, threshold = 0x0A */
    sx_wr(SX_DETECT_OPTIMIZE,  0x03U);
    sx_wr(SX_DETECTION_THRESH, 0x0AU);

    /* FIFO base addresses: TX and RX both at 0x00 */
    sx_wr(SX_FIFO_TX_BASE, 0x00U);
    sx_wr(SX_FIFO_RX_BASE, 0x00U);

    return ver;
}

static void sx_radio_minimize_idle(void)
{
    rf_rx_path();
    sx_wr(SX_IRQ_FLAGS, 0xFFU);
    sx_wr(SX_OP_MODE, OPMODE_LORA_SLEEP);
}

/* ------------------------------------------------------------------
 * TX one packet.  Returns 1 on TX_DONE, 0 on timeout.
 * ------------------------------------------------------------------ */

static uint8_t sx_transmit(const uint8_t *payload, uint8_t len)
{
    sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
    platform_delay_ms(1U);

    /* Load FIFO */
    sx_wr(SX_FIFO_ADDR_PTR, 0x00U);
    sx_wr_burst(SX_FIFO, payload, len);
    sx_wr(SX_PAYLOAD_LEN, len);

    /* Clear all IRQ flags */
    sx_wr(SX_IRQ_FLAGS, 0xFFU);

    /* Map DIO0 → TxDone */
    sx_wr(SX_DIO_MAPPING1, 0x40U);

    /* Switch RF path */
    rf_tx_path();

    /* Start TX */
    sx_wr(SX_OP_MODE, OPMODE_LORA_TX);

    /* Poll IRQ_FLAGS for TX_DONE; timeout = 500 ms */
    const uint32_t deadline = platform_now_ms() + 500U;
    while ((int32_t)(platform_now_ms() - (uint32_t)deadline) < 0) {
        if ((sx_rd(SX_IRQ_FLAGS) & (uint8_t)IRQ_TX_DONE) != 0U) {
            sx_wr(SX_IRQ_FLAGS, (uint8_t)IRQ_TX_DONE);
            sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
            return 1U;
        }
    }

    /* Timeout — back to standby */
    sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
    return 0U;
}

/* ------------------------------------------------------------------
 * RX: arm continuous RX, block until a valid packet arrives or
 * timeout_ms elapses.
 * Returns payload length (0 = timeout or CRC error).
 * ------------------------------------------------------------------ */

static uint8_t sx_receive(uint8_t *buf, uint8_t bufsize,
                           int16_t *out_rssi, int8_t *out_snr,
                           uint32_t timeout_ms)
{
    sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
    platform_delay_ms(1U);

    /* Clear flags, map DIO0 → RxDone */
    sx_wr(SX_IRQ_FLAGS, 0xFFU);
    sx_wr(SX_DIO_MAPPING1, 0x00U);

    /* Switch RF path */
    rf_rx_path();

    /* Enter continuous RX */
    sx_wr(SX_OP_MODE, OPMODE_LORA_RX_CONT);

    const uint32_t deadline = platform_now_ms() + timeout_ms;
    while ((int32_t)(platform_now_ms() - (uint32_t)deadline) < 0) {
        const uint8_t irq = sx_rd(SX_IRQ_FLAGS);

        if ((irq & (uint8_t)IRQ_RX_DONE) != 0U) {
            if ((irq & (uint8_t)IRQ_CRC_ERR) != 0U) {
                sx_wr(SX_IRQ_FLAGS, 0xFFU);
                sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
                return 0U;
            }

            const uint8_t nb  = sx_rd(SX_RX_NB_BYTES);
            const uint8_t ptr = sx_rd(SX_FIFO_RX_CUR);
            const uint8_t len = (nb < bufsize) ? nb : bufsize;

            sx_wr(SX_FIFO_ADDR_PTR, ptr);
            sx_rd_burst(SX_FIFO, buf, len);

            if (out_snr != NULL) {
                *out_snr = (int8_t)((int8_t)sx_rd(SX_PKT_SNR) / 4);
            }
            if (out_rssi != NULL) {
                *out_rssi = (int16_t)((int16_t)sx_rd(SX_PKT_RSSI) - 157);
            }

            sx_wr(SX_IRQ_FLAGS, 0xFFU);
            sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
            return len;
        }
    }

    /* Timeout */
    sx_wr(SX_OP_MODE, OPMODE_LORA_STDBY);
    return 0U;
}

/* ------------------------------------------------------------------
 * Clock init — force HSI16 (proven path from hello_world.c)
 * Also sets LPUART1 clock source to HSI16 via CCIPR.
 * Does NOT use platform_clock_init_hsi16() to avoid HSE probe hang.
 * ------------------------------------------------------------------ */

#define HSI16_CLOCK_HZ   16000000UL
#define USART1_BRR_VAL   (HSI16_CLOCK_HZ / 19200UL)           /* 833 */
#define LPUART1_BRR_VAL  ((256UL * HSI16_CLOCK_HZ) / 19200UL) /* 213333 */

static void clocks_hsi16_init(void)
{
    /* Enable HSI16 */
    RCC_CR |= (1UL << 0U);                                     /* HSION bit0 */
    while ((RCC_CR & (1UL << 2U)) == 0U) {                     /* HSIRDY bit2 */
    }

    /* Switch SYSCLK to HSI16 */
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW_MASK) | RCC_CFGR_SW_HSI16;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI16) {
    }

    /* LPUART1 clock source = HSI16: CCIPR[11:10] = 10b */
    RCC_CCIPR = (RCC_CCIPR & ~(3UL << 10U)) | (2UL << 10U);
}

/* ------------------------------------------------------------------
 * SysTick 1 ms — uses known HSI16 = 16 MHz (reload = 15999)
 * platform.c SysTick_Handler increments s_tick_ms; we just init here.
 * ------------------------------------------------------------------ */

static void systick_1ms_init(void)
{
    /* platform_systick_init_1ms() reads s_core_hz which defaults to
     * 16000000 in platform.c — matches HSI16, so call is safe here. */
    platform_systick_init_1ms();
}

/* ------------------------------------------------------------------
 * Peripheral init
 * ------------------------------------------------------------------ */

static void gpio_spi_init(void)
{
    /* Enable GPIO clocks (GPIOB/C for RF switch / reset) */
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;

    /* PB3=SCK, PA6=MISO, PA7=MOSI — AF0 (SPI1) */
    GPIO_MODER(GPIOA_BASE) =
        (GPIO_MODER(GPIOA_BASE) & ~((3UL << 12U) | (3UL << 14U)))
        | ((2UL << 12U) | (2UL << 14U));    /* AF mode PA6/7 */
    GPIO_MODER(GPIOB_BASE) =
        (GPIO_MODER(GPIOB_BASE) & ~(3UL << 6U))
        | (2UL << 6U);                      /* AF mode PB3 */
        
    /* OTYPER: push-pull (reset default); OSPEEDR: high speed for SCK+MOSI outputs */
    GPIO_OTYPER(GPIOA_BASE) &= ~((1UL << 6U) | (1UL << 7U));
    GPIO_OTYPER(GPIOB_BASE) &= ~(1UL << 3U);
    GPIO_OSPEEDR(GPIOA_BASE) = (GPIO_OSPEEDR(GPIOA_BASE)
        & ~((3UL << 12U) | (3UL << 14U)))
        | ((2UL << 12U) | (2UL << 14U));   /* high speed */
    GPIO_OSPEEDR(GPIOB_BASE) = (GPIO_OSPEEDR(GPIOB_BASE)
        & ~(3UL << 6U))
        | (2UL << 6U);                     /* high speed */
        
    /* Pull-up on MISO (PA6) — prevents floating 0x00 if slave tristates */
    GPIO_PUPDR(GPIOA_BASE) = (GPIO_PUPDR(GPIOA_BASE) & ~(3UL << 12U))
        | (1UL << 12U);
        
    /* AF0 in AFRL for PA6 [bits 27:24], PA7 [bits 31:28] */
    GPIO_AFRL(GPIOA_BASE) &= ~((15UL << 24U) | (15UL << 28U));
    /* AF0 in AFRL for PB3 [bits 15:12] */
    GPIO_AFRL(GPIOB_BASE) &= ~(15UL << 12U);

    /* NSS = PA15 output, deasserted (HIGH) */
    pin_output(SX_NSS_PORT, SX_NSS_PIN);
    sx_nss(0U);

    /* Reset = PC0 output, released (HIGH) */
    pin_output(SX_RST_PORT, SX_RST_PIN);
    pin_set(SX_RST_PORT, SX_RST_PIN, 1U);

    /* RF switch outputs, default RX path */
    pin_output(RF_TXRX_PORT,   RF_TXRX_PIN);
    pin_output(RF_RX_PORT,     RF_RX_PIN);
    pin_output(RF_TXBOOST_PORT, RF_TXBOOST_PIN);
    rf_rx_path();

    /* SPI1: master, 8-bit, CPOL=0 CPHA=0, BR/8 (~2 MHz at 16 MHz, ~4 MHz at 32 MHz) */
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
    /* Barrier read ensures APB2 clock has propagated to SPI1 before any access */
    (void)RCC_APB2ENR;
    /* If boot ROM left SPI active, wait for idle before touching CR2. */
    while ((SPI1_SR & SPI_SR_BSY) != 0U) { }
    /* Disable SPI with halfword access; do NOT write DS=0 (reserved) to CR2. */
    MMIO16(SPI1_BASE + 0x00UL) = 0U;
    (void)MMIO16(SPI1_BASE + 0x00UL);                  /* flush write */
    /* Configure CR2 first (before enabling SPI): DS=8-bit frame, FRXTH=8-bit RXNE */
    MMIO16(SPI1_BASE + 0x04UL) = (uint16_t)(SPI_CR2_DS_8BIT | SPI_CR2_FRXTH);
    (void)MMIO16(SPI1_BASE + 0x04UL);                  /* flush write */
    /* Configure CR1: master, SW NSS, BR/8 (~2 MHz @ 16 MHz), MSB first, CPOL=0 CPHA=0 */
    MMIO16(SPI1_BASE + 0x00UL) = (uint16_t)(SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_DIV8);
    (void)MMIO16(SPI1_BASE + 0x00UL);                  /* flush write */
    MMIO16(SPI1_BASE + 0x00UL) = (uint16_t)(MMIO16(SPI1_BASE + 0x00UL) | SPI_CR1_SPE);
}

static void uart_init(void)
{
    RCC_IOPENR  |= RCC_IOPENR_GPIOAEN;
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC_APB1ENR |= RCC_APB1ENR_LPUART1EN;

    /* PA9=TX, PA10=RX — AF4 (USART1) */
    GPIO_MODER(GPIOA_BASE) =
        (GPIO_MODER(GPIOA_BASE) & ~((3UL << 18U) | (3UL << 20U)))
        | ((2UL << 18U) | (2UL << 20U));
    GPIO_OSPEEDR(GPIOA_BASE) |= (3UL << 18U) | (3UL << 20U);
    GPIO_AFRH(GPIOA_BASE) =
        (GPIO_AFRH(GPIOA_BASE) & ~((0xFUL << 4U) | (0xFUL << 8U)))
        | ((4UL << 4U) | (4UL << 8U));

    /* PA2=TX, PA3=RX — AF6 (LPUART1) */
    GPIO_MODER(GPIOA_BASE) =
        (GPIO_MODER(GPIOA_BASE) & ~((3UL << 4U) | (3UL << 6U)))
        | ((2UL << 4U) | (2UL << 6U));
    GPIO_OSPEEDR(GPIOA_BASE) |= (3UL << 4U) | (3UL << 6U);
    GPIO_AFRL(GPIOA_BASE) =
        (GPIO_AFRL(GPIOA_BASE) & ~((0xFUL << 8U) | (0xFUL << 12U)))
        | ((6UL << 8U) | (6UL << 12U));

    /* USART1 at 19200 8N1, HSI16 = 16 MHz */
    USART1_CR1 = 0U;
    USART1_CR2 = 0U;
    USART1_CR3 = 0U;
    USART1_BRR = USART1_BRR_VAL;
    USART1_CR1 = USART_CR1_TE | USART_CR1_UE;

    /* LPUART1 at 19200 8N1 (CCIPR already set to HSI16 by clocks_hsi16_init) */
    LPUART1_CR1 = 0U;
    LPUART1_CR2 = 0U;
    LPUART1_CR3 = 0U;
    LPUART1_BRR = LPUART1_BRR_VAL;
    LPUART1_CR1 = USART_CR1_TE | USART_CR1_UE;
}

/* ------------------------------------------------------------------
 * main
 * ------------------------------------------------------------------ */

int main(void)
{
    /* Force HSI16 (proven path, avoids HSE/TCXO probe issues) */
    clocks_hsi16_init();

    /* SysTick @ 1 ms using HSI16 default in platform.c */
    systick_1ms_init();

    /* UART before first print */
    uart_init();

    /* SPI + GPIO (no delays needed here) */
    gpio_spi_init();

    /* Short settle before printing / probing radio */
    platform_delay_ms(200U);

    uart_puts("\r\n=== LIFETRAC L072 lora_ping v0.1");
#if PING_ROLE == 0
    uart_puts(" [PINGER/TX]");
#else
    uart_puts(" [PONGER/RX]");
#endif
    uart_puts(" ===\r\n");
    uart_puts("PHY: SF7/BW125/CR4-5  915.0 MHz  +14 dBm  sync=0x12\r\n");

    /* Hardware reset the radio */
    sx_hw_reset();

    /* Configure radio */
    const uint8_t ver = sx_radio_init();
    if (ver == 0U) {
        uart_puts("ERR: SX1276 not found (version 0x00 or 0xFF) — halted\r\n");
        for (;;) {
        }
    }

    uart_puts("SX1276 version=0x");
    uart_hex8(ver);
    uart_puts("  ready.\r\n");

#if RADIO_MINIMIZER_DEFAULT && !RADIO_ACTIVE_TEST_DEFAULT
    sx_radio_minimize_idle();
    uart_puts("RADIO: MINIMIZED (RADIO_ACTIVE_TEST_DEFAULT=0)\r\n");
    uart_puts("Enable active RF testing only in an explicit bench/test build.\r\n");
    for (;;) {
        platform_delay_ms(1000U);
    }
#endif

    /* ----------------------------------------------------------------
     * Main loop
     * -------------------------------------------------------------- */
    uint8_t  pkt[32];
    int16_t  rssi = 0;
    int8_t   snr  = 0;
    uint32_t tx_count = 0U;
    uint32_t rx_count = 0U;

#if PING_ROLE == 0
    /* ---- TX PINGER ---- */
    uart_puts("Role: PINGER — sending PING every ~2 s, listening 1 s for PONG\r\n");

    for (;;) {
        /* Packet: "PING" + big-endian 2-byte counter */
        pkt[0] = (uint8_t)'P';
        pkt[1] = (uint8_t)'I';
        pkt[2] = (uint8_t)'N';
        pkt[3] = (uint8_t)'G';
        pkt[4] = (uint8_t)(tx_count >> 8U);
        pkt[5] = (uint8_t)(tx_count & 0xFFU);
        tx_count++;

        uart_puts("TX PING #");
        uart_dec((int32_t)tx_count);
        uart_puts(" ... ");

        if (sx_transmit(pkt, 6U) != 0U) {
            uart_puts("OK\r\n");
        } else {
            uart_puts("TIMEOUT\r\n");
        }

        /* Listen for PONG for up to 1 s */
        const uint8_t rxlen = sx_receive(pkt, (uint8_t)sizeof(pkt),
                                         &rssi, &snr, 1000U);
        if ((rxlen >= 4U)
            && (pkt[0] == (uint8_t)'P')
            && (pkt[1] == (uint8_t)'O')
            && (pkt[2] == (uint8_t)'N')
            && (pkt[3] == (uint8_t)'G')) {
            rx_count++;
            uart_puts("RX PONG  rssi=");
            uart_dec((int32_t)rssi);
            uart_puts("dBm snr=");
            uart_dec((int32_t)snr);
            uart_puts("dB  ok=");
            uart_dec((int32_t)rx_count);
            uart_puts("\r\n");
        } else if (rxlen > 0U) {
            uart_puts("RX unknown (");
            uart_dec((int32_t)rxlen);
            uart_puts(" bytes):");
            for (uint8_t i = 0U; i < rxlen; i++) {
                uart_putc(' ');
                uart_hex8(pkt[i]);
            }
            uart_puts("\r\n");
        } else {
            uart_puts("RX timeout (no PONG)\r\n");
        }

        /* Pace TX cadence to ~2 s total (TX airtime ~100 ms + RX window 1 s + 900 ms here) */
        platform_delay_ms(900U);
    }

#else /* PING_ROLE == 1 */

    /* ---- RX PONGER ---- */
    uart_puts("Role: PONGER — listening for PING, replying PONG\r\n");

    for (;;) {
        const uint8_t rxlen = sx_receive(pkt, (uint8_t)sizeof(pkt),
                                         &rssi, &snr, 5000U);

        if ((rxlen >= 4U)
            && (pkt[0] == (uint8_t)'P')
            && (pkt[1] == (uint8_t)'I')
            && (pkt[2] == (uint8_t)'N')
            && (pkt[3] == (uint8_t)'G')) {
            rx_count++;
            uart_puts("RX PING  rssi=");
            uart_dec((int32_t)rssi);
            uart_puts("dBm snr=");
            uart_dec((int32_t)snr);
            uart_puts("dB  count=");
            uart_dec((int32_t)rx_count);
            uart_puts("\r\n");

            /* Reply: "PONG" + big-endian 2-byte counter */
            pkt[0] = (uint8_t)'P';
            pkt[1] = (uint8_t)'O';
            pkt[2] = (uint8_t)'N';
            pkt[3] = (uint8_t)'G';
            pkt[4] = (uint8_t)(rx_count >> 8U);
            pkt[5] = (uint8_t)(rx_count & 0xFFU);

            uart_puts("TX PONG ... ");
            if (sx_transmit(pkt, 6U) != 0U) {
                uart_puts("OK\r\n");
            } else {
                uart_puts("TIMEOUT\r\n");
            }

        } else if (rxlen > 0U) {
            uart_puts("RX unknown (");
            uart_dec((int32_t)rxlen);
            uart_puts(" bytes):");
            for (uint8_t i = 0U; i < rxlen; i++) {
                uart_putc(' ');
                uart_hex8(pkt[i]);
            }
            uart_puts("\r\n");
        } else {
            uart_puts("RX timeout (listening...)\r\n");
        }
    }

#endif /* PING_ROLE */
}
