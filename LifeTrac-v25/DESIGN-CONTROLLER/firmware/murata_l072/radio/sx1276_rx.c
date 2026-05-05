#include "sx1276_rx.h"

#include "host_stats.h"
#include "platform.h"
#include "sx1276.h"
#include "sx1276_modes.h"

#define SX1276_REG_FIFO                   0x00U
#define SX1276_REG_FIFO_ADDR_PTR          0x0DU
#define SX1276_REG_FIFO_RX_CURRENT_ADDR   0x10U
#define SX1276_REG_IRQ_FLAGS              0x12U
#define SX1276_REG_RX_NB_BYTES            0x13U
#define SX1276_REG_PKT_SNR_VALUE          0x19U
#define SX1276_REG_PKT_RSSI_VALUE         0x1AU
#define SX1276_REG_MODEM_CONFIG2          0x1EU

#define SX1276_IRQ_VALID_HEADER           0x10U
#define SX1276_IRQ_PAYLOAD_CRC_ERROR      0x20U
#define SX1276_IRQ_RX_DONE                0x40U

_Static_assert(sizeof(sx1276_rx_frame_t) <= 280U,
               "sx1276_rx_frame_t must stay within main-loop stack budget");

bool sx1276_rx_arm(void) {
    if (!sx1276_modes_to_rx_cont()) {
        return false;
    }

    if ((sx1276_read_reg(SX1276_REG_MODEM_CONFIG2) & (1U << 2)) == 0U) {
        (void)sx1276_modes_to_standby();
        return false;
    }

    return true;
}

void sx1276_rx_disarm(void) {
    (void)sx1276_modes_to_standby();
}

bool sx1276_rx_service(uint32_t events, sx1276_rx_frame_t *out_frame) {
    uint8_t irq_flags;

    if ((events & SX1276_EVT_DIO0) == 0U || out_frame == NULL) {
        return false;
    }

    out_frame->timestamp_us = platform_now_us();

    irq_flags = sx1276_read_reg(SX1276_REG_IRQ_FLAGS);

    if ((irq_flags & SX1276_IRQ_PAYLOAD_CRC_ERROR) != 0U) {
        host_stats_radio_crc_err();
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return false;
    }

    if ((irq_flags & SX1276_IRQ_RX_DONE) != 0U) {
        const uint8_t rx_len = sx1276_read_reg(SX1276_REG_RX_NB_BYTES);
        const uint8_t fifo_addr = sx1276_read_reg(SX1276_REG_FIFO_RX_CURRENT_ADDR);
        const int8_t snr_q4 = (int8_t)sx1276_read_reg(SX1276_REG_PKT_SNR_VALUE);
        const uint8_t pkt_rssi = sx1276_read_reg(SX1276_REG_PKT_RSSI_VALUE);

        sx1276_write_reg(SX1276_REG_FIFO_ADDR_PTR, fifo_addr);
        if (rx_len > 0U) {
            sx1276_read_burst(SX1276_REG_FIFO, out_frame->payload, rx_len);
        }

        out_frame->length = rx_len;
        out_frame->snr_db = (int8_t)(snr_q4 / 4);
        out_frame->rssi_dbm = (int16_t)pkt_rssi - 157;

        host_stats_radio_rx_ok();
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return true;
    }

    if ((irq_flags & SX1276_IRQ_VALID_HEADER) != 0U) {
        sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
        return false;
    }

    sx1276_write_reg(SX1276_REG_IRQ_FLAGS, irq_flags);
    return false;
}
