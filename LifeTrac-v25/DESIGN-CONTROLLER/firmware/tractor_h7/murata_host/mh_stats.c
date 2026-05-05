#include "mh_stats.h"

#include "mh_wire.h"

#include <stddef.h>
#include <string.h>

static uint32_t read_u32_le(const uint8_t *in) {
    return (uint32_t)in[0] |
           ((uint32_t)in[1] << 8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

bool mh_stats_parse(const uint8_t *payload, uint16_t payload_len, mh_stats_t *out) {
    if (payload == NULL || out == NULL) {
        return false;
    }

    if (payload_len < HOST_STATS_LEGACY_PAYLOAD_LEN) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    out->host_dropped = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_DROPPED]);
    out->host_errors = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_ERRORS]);
    out->host_queue_full = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_QUEUE_FULL]);
    out->host_irq_idle = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_IRQ_IDLE]);
    out->host_irq_ht = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_IRQ_HT]);
    out->host_irq_tc = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_IRQ_TC]);
    out->host_irq_te = read_u32_le(&payload[HOST_STATS_OFFSET_HOST_IRQ_TE]);
    out->radio_dio0 = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_DIO0]);
    out->radio_dio1 = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_DIO1]);
    out->radio_dio2 = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_DIO2]);
    out->radio_dio3 = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_DIO3]);
    out->radio_crc_err = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_CRC_ERR]);
    out->radio_rx_ok = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_RX_OK]);
    out->radio_tx_ok = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_TX_OK]);
    out->radio_tx_abort_lbt = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_TX_ABORT_LBT]);

    if (payload_len >= HOST_STATS_PAYLOAD_LEN) {
        out->radio_tx_abort_airtime = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_TX_ABORT_AIRTIME]);
        out->radio_state = read_u32_le(&payload[HOST_STATS_OFFSET_RADIO_STATE]);
        out->has_tx_abort_airtime = true;
    } else {
        out->radio_tx_abort_airtime = 0U;
        out->radio_state = read_u32_le(&payload[HOST_STATS_LEGACY_OFFSET_RADIO_STATE]);
        out->has_tx_abort_airtime = false;
    }

    return true;
}
