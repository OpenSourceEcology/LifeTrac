/*
 * rx_fifo_track — pins sx1276_rx_fifo_track_{reset,note}(), the RS-12.10
 * detector for a packet that completed while the previous one was still
 * unserviced (DIO0 level-held, FifoRxCurrentAddr jumps past a hole).
 *
 * Cases are the ones the bench produces:
 *   - first packet after an arm only teaches the pointer      -> no skip
 *   - contiguous 255 B fragments, wrapping the 256 B buffer   -> no skip
 *   - a hole the size of one unserviced packet                -> skip
 *   - a hole that itself wraps past 255                       -> skip
 *   - reset in the middle (re-arm) forgives the next jump     -> no skip
 *   - zero-length packet keeps the pointer where it is         -> no skip
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "sx1276_rx_fifo_track.h"

static int s_fail;
static int s_cases;

static void expect(bool got, bool want, const char *what) {
    s_cases++;
    if (got != want) {
        s_fail++;
        printf("[FAIL] %s: got %d want %d\n", what, (int)got, (int)want);
    }
}

int main(void) {
    sx1276_rx_fifo_track_t t;

    sx1276_rx_fifo_track_reset(&t);
    expect(sx1276_rx_fifo_track_note(&t, 0U, 255U), false, "first packet teaches");
    expect(sx1276_rx_fifo_track_note(&t, 255U, 255U), false, "second packet contiguous at 255");
    expect(sx1276_rx_fifo_track_note(&t, 254U, 255U), false, "third packet contiguous after wrap (255+255 mod 256 = 254)");
    expect(sx1276_rx_fifo_track_note(&t, 253U, 100U), false, "short packet contiguous");
    expect(t.expected == (uint8_t)(253U + 100U), true, "expected advances by len");

    /* One 255 B packet completed unserviced between the last one and this. */
    expect(sx1276_rx_fifo_track_note(&t, (uint8_t)(253U + 100U + 255U), 255U), true, "hole of one packet -> skip");
    expect(sx1276_rx_fifo_track_note(&t, (uint8_t)(253U + 100U + 255U + 255U), 40U), false, "contiguous again after the skip");

    /* Hole that wraps the buffer. */
    sx1276_rx_fifo_track_reset(&t);
    (void)sx1276_rx_fifo_track_note(&t, 200U, 255U);           /* ends at 199 */
    expect(sx1276_rx_fifo_track_note(&t, 10U, 20U), true, "start 10 != expected 199 -> skip");

    /* Re-arm forgives whatever the modem does with its pointer. */
    sx1276_rx_fifo_track_reset(&t);
    expect(sx1276_rx_fifo_track_note(&t, 77U, 5U), false, "after reset the first packet never counts");
    expect(sx1276_rx_fifo_track_note(&t, 82U, 0U), false, "zero-length packet contiguous");
    expect(sx1276_rx_fifo_track_note(&t, 82U, 3U), false, "zero-length packet did not move the pointer");

    /* NULL is a no-op. */
    expect(sx1276_rx_fifo_track_note(NULL, 1U, 1U), false, "NULL tracker never skips");
    sx1276_rx_fifo_track_reset(NULL);

    if (s_fail != 0) {
        printf("[FAIL] rx_fifo_track: %d of %d cases\n", s_fail, s_cases);
        return EXIT_FAILURE;
    }
    printf("[PASS] rx_fifo_track: %d cases\n", s_cases);
    return EXIT_SUCCESS;
}
