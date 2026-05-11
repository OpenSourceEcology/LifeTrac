#include "sx1276_modes.h"

#include "config.h"
#include "host_cmd.h"
#include "platform.h"
#include "sx1276.h"
#include "stm32l072_regs.h"

#include <stddef.h>

#define SX1276_REG_OP_MODE            0x01U
#define SX1276_REG_DIO_MAPPING1       0x40U
#define SX1276_REG_DIO_MAPPING2       0x41U

#define SX1276_OPMODE_LORA_SLEEP      0x80U
#define SX1276_OPMODE_LORA_STDBY      0x81U
#define SX1276_OPMODE_LORA_TX         0x83U
#define SX1276_OPMODE_LORA_RX_CONT    0x85U
#define SX1276_OPMODE_LORA_RX_SINGLE  0x86U
#define SX1276_OPMODE_LORA_CAD        0x87U

#define SX1276_DIO_MAP_STBY           0x00U
#define SX1276_DIO_MAP_TX             0x40U
#define SX1276_DIO_MAP_RX             0x00U
#define SX1276_DIO_MAP_CAD            0xA0U

#define SX1276_RF_SW_TXRX_PORT        GPIOA_BASE
#define SX1276_RF_SW_TXRX_PIN         1U
#define SX1276_RF_SW_RX_PORT          GPIOC_BASE
#define SX1276_RF_SW_RX_PIN           1U
#define SX1276_RF_SW_TX_BOOST_PORT    GPIOC_BASE
#define SX1276_RF_SW_TX_BOOST_PIN     2U

typedef struct sx1276_mode_desc_s {
    sx1276_state_t state;
    uint8_t opmode;
    uint8_t dio_mapping1;
    uint8_t dio_mapping2;
    uint8_t tx_path;
} sx1276_mode_desc_t;

static const sx1276_mode_desc_t k_mode_descs[] = {
    { SX1276_STATE_SLEEP,     SX1276_OPMODE_LORA_SLEEP,     SX1276_DIO_MAP_STBY, 0x00U, 0U },
    { SX1276_STATE_STANDBY,   SX1276_OPMODE_LORA_STDBY,     SX1276_DIO_MAP_STBY, 0x00U, 0U },
    { SX1276_STATE_TX,        SX1276_OPMODE_LORA_TX,        SX1276_DIO_MAP_TX,   0x00U, 1U },
    { SX1276_STATE_RX_CONT,   SX1276_OPMODE_LORA_RX_CONT,   SX1276_DIO_MAP_RX,   0x00U, 0U },
    { SX1276_STATE_RX_SINGLE, SX1276_OPMODE_LORA_RX_SINGLE, SX1276_DIO_MAP_RX,   0x00U, 0U },
    { SX1276_STATE_CAD,       SX1276_OPMODE_LORA_CAD,       SX1276_DIO_MAP_CAD,  0x00U, 0U }
};

static sx1276_state_t s_state = SX1276_STATE_UNINIT;

static void sx1276_delay_us(uint32_t delay_us) {
    const uint32_t start = platform_now_us();
    while ((uint32_t)(platform_now_us() - start) < delay_us) {
    }
}

static void rf_switch_set_tx(uint8_t tx_mode) {
    if (tx_mode != 0U) {
        GPIO_BSRR(SX1276_RF_SW_TXRX_PORT) = (1UL << SX1276_RF_SW_TXRX_PIN);
        GPIO_BSRR(SX1276_RF_SW_RX_PORT) = (1UL << (SX1276_RF_SW_RX_PIN + 16U));
        GPIO_BSRR(SX1276_RF_SW_TX_BOOST_PORT) = (1UL << SX1276_RF_SW_TX_BOOST_PIN);
    } else {
        GPIO_BSRR(SX1276_RF_SW_TXRX_PORT) = (1UL << (SX1276_RF_SW_TXRX_PIN + 16U));
        GPIO_BSRR(SX1276_RF_SW_RX_PORT) = (1UL << SX1276_RF_SW_RX_PIN);
        GPIO_BSRR(SX1276_RF_SW_TX_BOOST_PORT) = (1UL << (SX1276_RF_SW_TX_BOOST_PIN + 16U));
    }
}

static const sx1276_mode_desc_t *mode_desc_for_state(sx1276_state_t state) {
    for (size_t i = 0U; i < (sizeof(k_mode_descs) / sizeof(k_mode_descs[0])); ++i) {
        if (k_mode_descs[i].state == state) {
            return &k_mode_descs[i];
        }
    }
    return NULL;
}

static bool sx1276_modes_apply(const sx1276_mode_desc_t *desc) {
    if (desc == NULL) {
        s_state = SX1276_STATE_FAULT;
        return false;
    }

    rf_switch_set_tx(desc->tx_path);
    sx1276_delay_us(10U);

    sx1276_write_reg(SX1276_REG_DIO_MAPPING1, desc->dio_mapping1);
    sx1276_write_reg(SX1276_REG_DIO_MAPPING2, desc->dio_mapping2);
    sx1276_write_reg(SX1276_REG_OP_MODE, desc->opmode);

    /* W1-9 (2026-05-11): On the bench unit, RegOpMode mode-bit
     * transitions are silently ignored after wake-from-SLEEP unless the
     * chip's digital clock is fed by the external TCXO (RegTcxo bit 4).
     * Even with TcxoInputOn=1 set, some transitions still need a brief
     * settle.  This soft-retry loop pokes the OPMODE write up to 5 ms;
     * if the chip still hasn't honoured the requested mode we keep the
     * firmware-tracked s_state in sync with what the chip actually has
     * (rather than failing init).  Returning false here would prevent
     * sx1276_init() from running set_frequency_hz / set_sf_bw_cr /
     * set_tx_power_dbm, which still latch correctly even when the
     * digital block is asleep, and would stop host_cmd / stats / RX
     * paths that don't actually need the modem to be running (W1-7
     * UART regression coverage).
     */
    {
        const uint32_t deadline_us = platform_now_us() + 5000U;
        uint8_t opmode_rb = sx1276_read_reg(SX1276_REG_OP_MODE);
        while ((opmode_rb & 0x07U) != (desc->opmode & 0x07U)) {
            if ((int32_t)(platform_now_us() - deadline_us) >= 0) {
                break;
            }
            sx1276_delay_us(200U);
            sx1276_write_reg(SX1276_REG_OP_MODE, desc->opmode);
            sx1276_delay_us(50U);
            opmode_rb = sx1276_read_reg(SX1276_REG_OP_MODE);
        }
        if ((opmode_rb & 0x07U) != (desc->opmode & 0x07U)) {
            /* W1-9 deferred: emit fault for visibility but accept the
             * mismatch -- chip is stuck in its current mode (typically
             * LoRa SLEEP).  Subsequent TX/RX FSMs will time out cleanly
             * via their own deadlines.
             */
            host_cmd_emit_fault(HOST_FAULT_CODE_RADIO_OPMODE_DRIFT, opmode_rb);
        }
    }

#if HOST_DEBUG_OPMODE_GUARD
    {
        const uint8_t opmode_rb = sx1276_read_reg(SX1276_REG_OP_MODE);
        if ((opmode_rb & 0x87U) != (desc->opmode & 0x87U)) {
            host_cmd_emit_fault(HOST_FAULT_CODE_RADIO_OPMODE_DRIFT, opmode_rb);
            s_state = SX1276_STATE_FAULT;
            return false;
        }
    }
#endif

    s_state = desc->state;
    return true;
}

sx1276_state_t sx1276_modes_get_state(void) {
    return s_state;
}

bool sx1276_modes_to_sleep(void) {
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_SLEEP));
}

bool sx1276_modes_to_standby(void) {
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_STANDBY));
}

bool sx1276_modes_to_tx(void) {
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_TX));
}

bool sx1276_modes_to_rx_cont(void) {
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_RX_CONT));
}

bool sx1276_modes_to_rx_single(uint16_t timeout_symbols) {
    (void)timeout_symbols;
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_RX_SINGLE));
}

bool sx1276_modes_to_cad(void) {
    if (!sx1276_modes_to_standby()) {
        return false;
    }
    return sx1276_modes_apply(mode_desc_for_state(SX1276_STATE_CAD));
}

bool sx1276_modes_init(void) {
    s_state = SX1276_STATE_UNINIT;

    /* W1-9 fix #2 (2026-05-11): Set RegTcxo (0x4B) bit 4 (TcxoInputOn=1)
     * BEFORE any mode transition.  Murata CMWX1ZZABZ-078 supplies a
     * 32 MHz TCXO whose output drives the SX1276 XTA pin.  The chip's
     * default RegTcxo = 0x09 enables the internal crystal oscillator,
     * which expects a real crystal across XTA/XTB; without TcxoInputOn
     * the digital block has no clock and OPMODE mode-bit transitions
     * are silently ignored (configuration register writes still latch
     * because they don't need the digital state machine).  Set this
     * first while the chip is still in its reset state (FSK STDBY = 0x01,
     * register write protection minimal) so the TCXO-derived clock is
     * available for the first OPMODE-mode transition.
     *
     * Bench-confirmed (Stage 2 W1-9): with default RegTcxo, RegOpMode
     * read-back stays at 0x80 / 0x01 forever even via direct
     * REG_WRITE_REQ from host; with RegTcxo = 0x10, mode transitions
     * complete normally.
     */
    sx1276_write_reg(0x4BU, 0x10U);
    platform_delay_ms(2U);

    /* W1-9 fix #1 (2026-05-11): Per SX1276 datasheet §4.1.6, the
     * LongRangeMode bit (bit 7) of RegOpMode (0x01) can only be modified
     * while the chip is in SLEEP mode of the *current* modem.  After
     * NRESET the chip boots in FSK STDBY; go to FSK SLEEP first so the
     * subsequent LoRa-SLEEP write can flip the LongRange bit cleanly.
     */
    sx1276_write_reg(SX1276_REG_OP_MODE, 0x00U);
    platform_delay_ms(1U);

    if (!sx1276_modes_to_sleep()) {
        return false;
    }
    platform_delay_ms(1U);

    return sx1276_modes_to_standby();
}
