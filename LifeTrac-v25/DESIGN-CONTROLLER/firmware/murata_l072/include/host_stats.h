#ifndef LIFETRAC_MURATA_L072_HOST_STATS_H
#define LIFETRAC_MURATA_L072_HOST_STATS_H

#include <stdint.h>

#define HOST_STATS_PAYLOAD_LEN       64U

void host_stats_reset(void);
void host_stats_note_radio_events(uint32_t events);
void host_stats_radio_rx_ok(void);
void host_stats_radio_crc_err(void);
void host_stats_radio_tx_ok(void);
void host_stats_radio_tx_abort_lbt(void);
uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap);

#endif /* LIFETRAC_MURATA_L072_HOST_STATS_H */
