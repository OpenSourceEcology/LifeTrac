#ifndef LIFETRAC_MURATA_L072_HOST_STATS_H
#define LIFETRAC_MURATA_L072_HOST_STATS_H

#include "host_types.h"

#include <stdbool.h>
#include <stdint.h>

void host_stats_reset(void);
void host_stats_note_radio_events(uint32_t events);

/* RS-11.5 TX FIFO readback discriminator (sx1276_tx.c). */
void host_stats_tx_fifo_rb_ok(void);
void host_stats_tx_fifo_rb_bad(void);
void host_stats_tx_done_early(void);
void host_stats_radio_rx_ok(void);
void host_stats_radio_crc_err(void);
void host_stats_radio_tx_ok(void);
void host_stats_radio_tx_abort_lbt(void);
void host_stats_radio_tx_abort_airtime(void);
/* RS-12 (2026-09-12) URC-path loss accounting; see host_types.h tail. */
void host_stats_rx_urc_lost_add(uint32_t n);
void host_stats_rx_pretx_drained_add(uint32_t n);
/* RS-12.10 (2026-09-12): FIFO-skip detector + TX deaf-window timers. */
void host_stats_rx_fifo_skip(void);
void host_stats_tx_deaf_note(uint32_t deaf_us, uint32_t done_to_rearm_us,
                             bool done_valid);
/* RS-12.15 v2 (2026-09-14): FHSS clock-authority counters (see the
 * host_types.h tail). dec_idx is the sx1276_fhss_snap_decision_t value. */
void host_stats_fhss_dec_note(uint8_t dec_idx);
void host_stats_clk_demotion_note(bool reset);
void host_stats_tx_first_anchor(void);
void host_stats_tx_stream_streak_note(uint32_t streak);
uint16_t host_stats_serialize(uint8_t *out, uint16_t out_cap);

#endif /* LIFETRAC_MURATA_L072_HOST_STATS_H */
