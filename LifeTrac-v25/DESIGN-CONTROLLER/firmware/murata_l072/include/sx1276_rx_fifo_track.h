/*
 * RS-12.10 (2026-09-12): FIFO-address tracker for the LoRa RX path.
 *
 * In RXCONTINUOUS the SX1276 appends each demodulated packet to the
 * 256-byte data buffer right after the previous one (wrapping mod 256)
 * and RegFifoRxCurrentAddr points at the START of the most recent
 * packet. When two packets complete before the main loop services the
 * first, DIO0 (RxDone) is still high from the first — no second rising
 * edge is produced — and the service reads only the LAST packet. The
 * flash session of 2026-09-12 showed the edge counter (rx_urc_lost)
 * is blind to exactly that case. The address is not: a packet that was
 * never read leaves a hole between the previous packet's end and the
 * next packet's start. This pure TU turns that hole into a count.
 *
 * Contract:
 *   reset() after every (re)arm of RX — the first packet after an arm
 *            only teaches the tracker where the buffer pointer is.
 *   note(start, len) on every serviced packet (RX_DONE and payload-CRC-
 *            error alike; both advance the buffer pointer). Returns true
 *            when `start` is not where the previous packet ended.
 * Bench-pinned by bench/host_proto/rx_fifo_track.c (check-rx-fifo-track).
 */
#ifndef LIFETRAC_MURATA_L072_SX1276_RX_FIFO_TRACK_H
#define LIFETRAC_MURATA_L072_SX1276_RX_FIFO_TRACK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct sx1276_rx_fifo_track_s {
    uint8_t valid;     /* 1 once a packet has been noted since the last reset */
    uint8_t expected;  /* buffer address the next packet must start at */
} sx1276_rx_fifo_track_t;

void sx1276_rx_fifo_track_reset(sx1276_rx_fifo_track_t *t);
bool sx1276_rx_fifo_track_note(sx1276_rx_fifo_track_t *t,
                               uint8_t start_addr,
                               uint8_t len);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_FIFO_TRACK_H */
