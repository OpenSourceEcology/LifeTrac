#ifndef LIFETRAC_MURATA_L072_SX1276_FHSS_AUTHORITY_H
#define LIFETRAC_MURATA_L072_SX1276_FHSS_AUTHORITY_H

/*
 * RS-12.15 v2 (2026-09-14): who owns the FHSS grid on a duplex node?
 *
 * The slot-clock design (sx1276_fhss_clock.h) gives a node two ways to
 * hold a grid: ADOPT one from a received A6a header (a follower) or
 * self-anchor one at its own first FHSS TX (an originator). The RS-12.12
 * / RS-12.14 lock losses came from treating those two the same:
 *
 *   * the RX health gate read a self-anchored grid as UNANCHORED (no
 *     refusal authority), so the streaming tractor snapped / re-anchored
 *     to the base's lagged FOLLOWER copy of its own grid; and
 *   * the LOCKED->SCANNING demotion reset the clock unconditionally, so
 *     a received command LOCKed the tractor's scan machine, the 2 s
 *     loss timer demoted it, and the demotion wiped the tractor's OWN
 *     streaming clock -- the next TX re-anchored "slot k+1 starts now",
 *     renumbering the shared grid under the base's follower.
 *
 * Both fixes need the same discriminator and it must NOT be
 * "clock valid + no adopted grid": that is also the documented
 * post-demotion recovery state (PR #125 review), where a single duplex
 * TX re-validates a stale grid and must NOT confer authority, or the F6
 * recovery tier becomes unreachable again.
 *
 * The discriminator is SUSTAINED STREAMING: a node earns originator
 * authority only after MIN_STREAK consecutive own FHSS transmissions
 * each within STREAK_GAP_MS of the previous one. An image stream (a
 * fragment every ~40-200 ms) clears that inside one train; a command
 * sender (>= 1 s apart under the RS-12.14 gate, or two copies 37 ms
 * apart at most) never does, and a node that just recovered from a
 * demotion starts at zero. Adopting a remote grid clears the streak.
 *
 *   originator := clock_valid && !grid_adopted && streak >= MIN_STREAK
 *                 && (now - last_own_tx) < STREAK_GAP_MS
 *
 * Two boundary rules (PR #125 review round 3): the chain test is STRICT
 * (a sender spaced exactly at the gap -- the RS-12.14 stream gate admits
 * commands at >= 1.0 s -- never chains), and authority DECAYS one gap
 * after the last own transmission, so a node that bursts a few queued
 * commands and goes quiet cannot keep refusing a peer's grid.
 *
 * Consumers (sx1276_rx.c):
 *   - adoption gate: an originator adopts a remote grid only when it
 *     LEADS its own (sx1276_fhss_clock_rx_leads); everyone else adopts
 *     as before.
 *   - demotion edge: a clock that was ADOPTED is reset (fresh acquisition
 *     as before); a self-anchored clock survives its owner's demotion --
 *     the streaming node's grid never depended on hearing the base.
 *
 * HW-free, module-static state (same shape as sx1276_fhss_clock.c) so
 * the bench pins it: check-fhss-authority.
 */

#include <stdint.h>

/* Two own transmissions further apart than this end the streak. */
#define SX1276_FHSS_AUTHORITY_STREAK_GAP_MS 1000U
/* Consecutive own transmissions (each within the gap) that make a node
 * an originator. 8 x 200 ms slots = 1.6 s of streaming. */
#define SX1276_FHSS_AUTHORITY_MIN_STREAK    8U

/* Forget all streaming history (boot, scan reset, follower demotion). */
void sx1276_fhss_authority_reset(void);

/* Note one own FHSS transmission that actually went on air (called from
 * the TX_DONE path in sx1276_tx_poll, NOT at admission -- PR #125 review:
 * an aborted attempt must not count as streaming). now_ms = local time
 * of the TX_DONE. */
void sx1276_fhss_authority_note_tx(uint32_t now_ms);

/* Note that a remote grid was adopted: the node is a follower now. */
void sx1276_fhss_authority_note_adopt(void);

/* Current streak (saturating) and the largest streak seen since reset. */
uint32_t sx1276_fhss_authority_streak(void);
uint32_t sx1276_fhss_authority_streak_max(void);

/* 1 when the node holds originator authority AT now_ms: valid clock, no
 * adopted grid, streak >= MIN_STREAK, and its last own TX less than
 * STREAK_GAP_MS ago (authority decays when streaming stops). */
uint8_t sx1276_fhss_authority_is_originator(uint32_t now_ms,
                                            uint8_t clock_valid,
                                            uint8_t grid_adopted);

#endif /* LIFETRAC_MURATA_L072_SX1276_FHSS_AUTHORITY_H */
