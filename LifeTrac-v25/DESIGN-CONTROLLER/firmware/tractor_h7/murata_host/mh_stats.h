#ifndef LIFETRAC_TRACTOR_H7_MH_STATS_H
#define LIFETRAC_TRACTOR_H7_MH_STATS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mh_stats_s {
    uint32_t host_dropped;
    uint32_t host_errors;
    uint32_t host_queue_full;
    uint32_t host_irq_idle;
    uint32_t host_irq_ht;
    uint32_t host_irq_tc;
    uint32_t host_irq_te;
    uint32_t radio_dio0;
    uint32_t radio_dio1;
    uint32_t radio_dio2;
    uint32_t radio_dio3;
    uint32_t radio_crc_err;
    uint32_t radio_rx_ok;
    uint32_t radio_tx_ok;
    uint32_t radio_tx_abort_lbt;
    uint32_t radio_tx_abort_airtime;
    uint32_t radio_state;
    bool has_tx_abort_airtime;
} mh_stats_t;

bool mh_stats_parse(const uint8_t *payload, uint16_t payload_len, mh_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LIFETRAC_TRACTOR_H7_MH_STATS_H */
