#ifndef LIFETRAC_MURATA_L072_SX1276_RX_GRID_POLICY_H
#define LIFETRAC_MURATA_L072_SX1276_RX_GRID_POLICY_H

/*
 * RS-12.15 v2 (2026-09-14, PR #125 review round 3): the RX side's grid
 * policy -- WHICH health tier to hand sx1276_fhss_consider_remote(),
 * WHETHER an accepted header re-anchors the local clock, and WHAT a scan
 * demotion does to the clock -- extracted from sx1276_rx.c into a HW-free
 * TU so the bench can drive the whole adopt -> demote -> duplex-TX ->
 * remote-frame sequence against the REAL consider_remote()
 * (check-rx-grid-policy). sx1276_rx.c keeps only the hardware: timestamps,
 * the ToA estimate, counters, and the modem.
 *
 * State owned here: s_grid_adopted (F6) -- has ANY remote grid been
 * accepted since the last acquisition reset? The clock and the authority
 * streak live in their own HW-free TUs and are driven through this one.
 *
 * Rules (see sx1276_fhss_authority.h for the originator discriminator):
 *   consider():  originator := authority(now, clock_valid, adopted)
 *                adopt      := originator ? rx_leads(...) : 1
 *                health     := (!clock_valid || !adopted)
 *                                ? (originator ? (adopt ? STALE : FRESH)
 *                                              : UNANCHORED)
 *                                : (age <= FRESH_MS ? FRESH : STALE)
 *                An originator hands FRESH to consider_remote to make it
 *                REFUSE a lagging follower's snap, and STALE (never
 *                UNANCHORED) for a leading grid so the +/-1 epoch-drift
 *                barrier -- the only spoof/replay guard on the
 *                unauthenticated header -- stays in force. A follower or
 *                a recovering node gets the pre-v2 tiers unchanged.
 *   adopt():     anchor_rx + adopted = 1 + authority cleared.
 *   demotion():  an ADOPTED clock is reset (fresh acquisition, as before);
 *                a self-anchored clock is KEPT -- the streaming node's grid
 *                never depended on hearing the base, and resetting it was
 *                the RS-12.12/14 lock-loss mechanism.
 *   reset():     scan reset -- clock, authority and adoption all cleared.
 */

#include <stdint.h>

#include "sx1276_fhss.h"

typedef struct {
    sx1276_fhss_clock_health_t health;   /* tier for consider_remote()   */
    uint8_t adopt;                       /* re-anchor on SNAPPED/ALIGNED */
    uint8_t originator;                  /* telemetry / tests            */
} sx1276_rx_grid_verdict_t;

/* Scan reset: forget clock, authority and adoption (fresh acquisition). */
void sx1276_rx_grid_reset(void);

/* 1 when a remote grid has been adopted since the last reset. */
uint8_t sx1276_rx_grid_adopted(void);

/* Decide how to treat a parsed A6a header received at now_ms with the
 * given ToA / slot_offset / absolute slot. Pure w.r.t. this TU's state
 * (mutates nothing). */
sx1276_rx_grid_verdict_t sx1276_rx_grid_consider(uint32_t now_ms,
                                                 uint32_t toa_us,
                                                 uint8_t  slot_offset_ms,
                                                 uint32_t remote_abs);

/* The caller accepted the header (SNAPPED/ALIGNED) and the verdict said
 * adopt: re-anchor the clock from it and become a follower. */
void sx1276_rx_grid_adopt(uint32_t now_ms, uint32_t toa_us,
                          uint8_t slot_offset_ms, uint32_t remote_abs);

/* LOCKED -> SCANNING loss demotion. Returns 1 if an adopted clock was
 * reset, 0 if a self-anchored clock was kept. Adoption is cleared either
 * way (the lock is gone). */
uint8_t sx1276_rx_grid_on_demotion(void);

#endif /* LIFETRAC_MURATA_L072_SX1276_RX_GRID_POLICY_H */
