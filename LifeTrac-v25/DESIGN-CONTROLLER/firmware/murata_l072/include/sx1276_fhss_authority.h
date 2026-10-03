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
 * fragment every ~40-200 ms, or one frame per second at the slowest
 * rate) clears that within MIN_STREAK frames; a node that just
 * recovered from a demotion starts at zero. Adopting a remote grid
 * clears the streak.
 *
 *   originator := clock_valid && !grid_adopted && streak >= MIN_STREAK
 *                 && (now - last_own_tx) < STREAK_GAP_MS
 *
 * Two boundary rules (PR #125 review round 3): the chain test is STRICT
 * (a sender spaced exactly at the gap never chains), and authority DECAYS
 * one gap after the last own transmission, so a node that bursts a few
 * queued commands and goes quiet cannot keep refusing a peer's grid.
 *
 * RS-13.1 A11 (2026-10-03) -- the gap is 1500 ms, not 1000. With 1000 and
 * the strict test, a ONE-FRAME-PER-SECOND stream never chained: on air the
 * tractor's 1 fps VECTOR legs logged TX gaps of 993/999/1005 ms
 * (p10/p50/p90) and tx_stream_streak_max 2 and 3, so the tractor never
 * held authority, adopted every base command it heard (an ALIGNED echo of
 * its own grid sets grid_adopted), and its next scan demotion reset that
 * adopted clock -- the RS-12.12/14 lock-loss exposure RS-12.15 closed was
 * open again at 1 fps. 1500 = 1.5 x the slowest supported stream cadence
 * (SX1276_FHSS_AUTHORITY_SLOWEST_STREAM_MS): a 1 fps stream chains with
 * ~490 ms of headroom for host jitter, ToA spread and a one-slot (200 ms)
 * TX deferral; a skipped frame (a 2 s gap) does not chain; and 2 s of
 * silence always ends authority (pinned < SX1276_FHSS_CLOCK_FRESH_MS in
 * sx1276_rx_grid_policy.c: an originator never refuses a peer for longer
 * than a fresh follower would).
 *
 * ONE constant does both jobs on purpose. Decay shorter than the chain gap
 * would let a live streak flicker out of authority between two frames, and
 * a base command heard in that window is ADOPTED -- grid_adopted is set and
 * the streak cleared, i.e. the streamer becomes a follower and its next
 * demotion resets its clock (the exposure above). Decay longer than the
 * chain gap would let authority outlive a broken streak.
 *
 * What the wider gap gives up, and why that is safe: base commands under
 * the RS-12.14 stream gate (>= 1.0 s apart) now chain ON CADENCE -- a 1 Hz
 * command sender and a 1 fps stream are indistinguishable by spacing, and
 * the stream is the one that must win. A command sender is kept out by
 * ADOPTION, not by the gap: every header a non-originator accepts
 * (SNAPPED/ALIGNED) clears its streak (note_adopt), and the base's gate is
 * >= 1.0 s only while fragments are flowing (IDLE_DRAIN_QUIET_S = 1.5 s
 * after the last one), i.e. while it is adopting the tractor's headers
 * between its own sends. A base that has heard nothing for 1.5 s drops to
 * the 0.12 s idle gate, which chained under the old 1000 ms rule as well;
 * there the decay above is the bound, unchanged in kind (1.5 s instead of
 * 1 s after its last TX). Bench-pinned in check-fhss-authority (1 fps /
 * 2 fps / silence / command-sender fns) and check-rx-grid-policy
 * (sequences 8b and 10).
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

/* The slowest own-TX cadence that must earn and KEEP authority: the 1 fps
 * image stream (RS-13.1 A11). A design bound, not consulted at run time --
 * it pins STREAK_GAP_MS through the _Static_asserts in
 * sx1276_fhss_authority.c. */
#define SX1276_FHSS_AUTHORITY_SLOWEST_STREAM_MS 1000U
/* Two own transmissions this far apart or further end the streak (strict
 * chain test), and authority decays this long after the last own TX.
 * 1.5 x SLOWEST_STREAM_MS; see the RS-13.1 A11 note above. */
#define SX1276_FHSS_AUTHORITY_STREAK_GAP_MS 1500U
/* Consecutive own transmissions (each within the gap) that make a node
 * an originator. 8 x 200 ms slots = 1.6 s of dense streaming; 8 frames =
 * 7 s at 1 fps, 3.5 s at 2 fps. */
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
