/*
 * RS-12.10 (2026-09-12): pure FIFO-address tracker — see the header.
 * No hardware, no statics: the caller owns the state so the bench can
 * drive it with plain integers.
 */
#include "sx1276_rx_fifo_track.h"

#include <stddef.h>

void sx1276_rx_fifo_track_reset(sx1276_rx_fifo_track_t *t) {
    if (t == NULL) {
        return;
    }
    t->valid = 0U;
    t->expected = 0U;
}

bool sx1276_rx_fifo_track_note(sx1276_rx_fifo_track_t *t,
                               uint8_t start_addr,
                               uint8_t len) {
    bool skipped = false;

    if (t == NULL) {
        return false;
    }
    if (t->valid != 0U && start_addr != t->expected) {
        skipped = true;
    }
    /* The modem's write pointer is 8-bit: wrap is the cast. */
    t->expected = (uint8_t)(start_addr + len);
    t->valid = 1U;
    return skipped;
}
