/*
 * RS-12.15 v2 (2026-09-14): originator-authority unit test (host-cc).
 *
 * Pins the discriminator that separates a STREAMING originator from a
 * post-demotion duplex node: authority needs MIN_STREAK consecutive own
 * transmissions each STRICTLY within STREAK_GAP_MS of the previous one,
 * it DECAYS one gap after the last own TX, adopting a remote grid clears
 * it, and neither an invalid clock nor an adopted grid can ever hold it.
 * The policy-level demotion -> TX -> RX regression (with the real
 * consider_remote) lives in rx_grid_policy.c; this file pins the pure
 * streak arithmetic.
 *
 * RS-13.1 A11 (2026-10-03): the gap is 1.5 s so a ONE-FRAME-PER-SECOND
 * stream (on-air TX gaps 993/999/1005 ms p10/p50/p90; the 1 s gap gave
 * tx_stream_streak_max 2-3) earns authority and keeps it across every
 * inter-frame interval, 2 fps likewise, while 2 s of silence (a skipped
 * 1 fps frame) still ends it. test_one_fps_stream_* pin the cadences;
 * test_command_sender_* pin what keeps a 1 Hz command sender out now that
 * cadence alone cannot (adoption), and the accepted, decaying case where
 * it cannot hear its peer at all.
 */

#include <stdio.h>
#include <stdint.h>

#include "sx1276_fhss_authority.h"

static int g_failures = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

#define GAP  SX1276_FHSS_AUTHORITY_STREAK_GAP_MS
#define MINS SX1276_FHSS_AUTHORITY_MIN_STREAK

static uint32_t stream(uint32_t start_ms, uint32_t n, uint32_t cadence_ms) {
    uint32_t t = start_ms;
    for (uint32_t i = 0U; i < n; ++i) {
        t = start_ms + i * cadence_ms;
        sx1276_fhss_authority_note_tx(t);
    }
    return t;   /* time of the last TX */
}

static void test_reset_has_no_authority(void) {
    sx1276_fhss_authority_reset();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(1) reset streak 0");
    CHECK(sx1276_fhss_authority_is_originator(1000U, 1U, 0U) == 0U,
          "(1) valid clock + no adopted grid but NO streak -> not originator");
}

static void test_streaming_earns_authority(void) {
    sx1276_fhss_authority_reset();
    uint32_t last = stream(1000U, MINS - 1U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 0U,
          "(2) MIN_STREAK-1 transmissions: not yet");
    last = stream(1000U + (MINS - 1U) * 200U, 1U, 200U);
    CHECK(sx1276_fhss_authority_streak() == MINS, "(2) streak == MIN_STREAK");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U,
          "(2) MIN_STREAK transmissions: originator");
    /* 40 ms fragment pacing inside a train clears it even faster. */
    sx1276_fhss_authority_reset();
    last = stream(5000U, 8U, 40U);
    CHECK(sx1276_fhss_authority_is_originator(last + 30U, 1U, 0U) == 1U,
          "(2) 8 fragments at 40 ms: originator");
    /* Sparse 2-fragment trains at 1.5 fps (the leg Q/R instrument):
     * 40 ms inside the pair, ~627 ms between pairs -- both chain. */
    sx1276_fhss_authority_reset();
    uint32_t t = 7000U;
    for (uint32_t train = 0U; train < 5U; ++train) {
        sx1276_fhss_authority_note_tx(t);
        sx1276_fhss_authority_note_tx(t + 40U);
        t += 667U;
    }
    CHECK(sx1276_fhss_authority_streak() == 10U, "(2) sparse trains chain (10)");
    CHECK(sx1276_fhss_authority_is_originator(t - 667U + 400U, 1U, 0U) == 1U,
          "(2) 1.5 fps 2-fragment stream: originator between trains");
}

/* Host jitter around a nominal cadence, ms: covers the on-air 1 fps spread
 * (993..1005 ms p10..p90) with the +/-10 ms the task bound names. */
static const int32_t k_jitter[10] = { 0, 10, -10, 5, -7, 10, -10, 3, 0, -5 };

/* n own TXs from t0 at cadence_ms + k_jitter[i % 10] (i >= 1). After each
 * TX it checks the authority one ms before the NEXT TX is due -- the end
 * of the interval, where the old 1 s decay opened a window. Returns the
 * time of the last TX; *held_from = the 1-based index from which every
 * end-of-interval check (and every check after it) held authority, or 0
 * if authority was ever lost after being gained. */
static uint32_t jittered_stream(uint32_t t0, uint32_t n, uint32_t cadence_ms,
                                uint32_t *held_from) {
    uint32_t t = t0;
    uint32_t first_held = 0U;
    uint8_t  lost_after_gain = 0U;
    for (uint32_t i = 0U; i < n; ++i) {
        if (i != 0U) {
            t = (uint32_t)((int32_t)t + (int32_t)cadence_ms + k_jitter[i % 10U]);
        }
        sx1276_fhss_authority_note_tx(t);
        const uint32_t next_gap =
            (uint32_t)((int32_t)cadence_ms + k_jitter[(i + 1U) % 10U]);
        const uint8_t held =
            sx1276_fhss_authority_is_originator(t + next_gap - 1U, 1U, 0U);
        if (held != 0U && first_held == 0U) {
            first_held = i + 1U;
        } else if (held == 0U && first_held != 0U) {
            lost_after_gain = 1U;
        }
    }
    if (held_from != NULL) {
        *held_from = (lost_after_gain != 0U) ? 0U : first_held;
    }
    return t;
}

static void test_one_fps_stream_earns_and_keeps_it(void) {
    /* THE A11 FIX: one frame per second, +/-10 ms jitter. */
    uint32_t held_from = 0U;
    sx1276_fhss_authority_reset();
    uint32_t last = jittered_stream(100000U, MINS - 1U, 1000U, NULL);
    CHECK(sx1276_fhss_authority_streak() == MINS - 1U,
          "(8) 1 fps: every frame chains (streak %u)", sx1276_fhss_authority_streak());
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 0U,
          "(8) 1 fps: MIN_STREAK-1 frames is not yet authority");
    sx1276_fhss_authority_reset();
    last = jittered_stream(100000U, 60U, 1000U, &held_from);
    CHECK(sx1276_fhss_authority_streak() == 60U,
          "(8) 1 fps x 60: streak 60 (got %u; the 1 s gap gave 2-3 on air)",
          sx1276_fhss_authority_streak());
    CHECK(held_from == MINS,
          "(8) 1 fps: authority from frame MIN_STREAK on and NEVER lost between "
          "frames (held_from=%u)", held_from);
    /* Inside the old 1 s decay window: still the originator. */
    CHECK(sx1276_fhss_authority_is_originator(last + 1005U, 1U, 0U) == 1U,
          "(8) 1 fps: a late frame (1005 ms) keeps authority");
    /* Constant worst cases at both ends of the jitter band. */
    sx1276_fhss_authority_reset();
    last = stream(200000U, 30U, 1010U);
    CHECK(sx1276_fhss_authority_streak() == 30U &&
          sx1276_fhss_authority_is_originator(last + 1009U, 1U, 0U) == 1U,
          "(8) constant 1010 ms: chains and holds to the next frame");
    sx1276_fhss_authority_reset();
    last = stream(300000U, 30U, 990U);
    CHECK(sx1276_fhss_authority_streak() == 30U &&
          sx1276_fhss_authority_is_originator(last + 989U, 1U, 0U) == 1U,
          "(8) constant 990 ms: chains and holds to the next frame");
    /* One frame's key-up deferred a whole 200 ms slot (the slot-fit
     * guard): gaps 1200 then 800 -- still one unbroken streak. */
    sx1276_fhss_authority_reset();
    uint32_t t = 400000U;
    for (uint32_t i = 0U; i < 20U; ++i) {
        sx1276_fhss_authority_note_tx(t);
        t += ((i % 2U) == 0U) ? 1200U : 800U;
    }
    CHECK(sx1276_fhss_authority_streak() == 20U,
          "(8) 1 fps with alternate one-slot deferrals: unbroken (got %u)",
          sx1276_fhss_authority_streak());
}

static void test_two_fps_stream_earns_and_keeps_it(void) {
    uint32_t held_from = 0U;
    sx1276_fhss_authority_reset();
    const uint32_t last = jittered_stream(500000U, 40U, 500U, &held_from);
    CHECK(sx1276_fhss_authority_streak() == 40U, "(9) 2 fps x 40: streak 40");
    CHECK(held_from == MINS, "(9) 2 fps: authority from frame MIN_STREAK, never lost "
          "(held_from=%u)", held_from);
    CHECK(sx1276_fhss_authority_is_originator(last + 499U, 1U, 0U) == 1U,
          "(9) 2 fps: holds to the next frame");
}

static void test_silence_ends_it(void) {
    /* A 1 fps originator that goes quiet. */
    sx1276_fhss_authority_reset();
    const uint32_t last = jittered_stream(600000U, 20U, 1000U, NULL);
    CHECK(sx1276_fhss_authority_is_originator(last + GAP - 1U, 1U, 0U) == 1U,
          "(10) one ms before the gap: still");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP, 1U, 0U) == 0U,
          "(10) at the gap (1.5 s): authority gone");
    CHECK(sx1276_fhss_authority_is_originator(last + 2000U, 1U, 0U) == 0U,
          "(10) 2 s of silence: authority gone");
    /* A skipped 1 fps frame: the next TX lands 2 s later and restarts the
     * streak, so authority must be re-earned (MIN_STREAK more frames). */
    sx1276_fhss_authority_note_tx(last + 2000U);
    CHECK(sx1276_fhss_authority_streak() == 1U,
          "(10) skipped frame (2 s gap): streak restarts at 1");
    CHECK(sx1276_fhss_authority_is_originator(last + 2010U, 1U, 0U) == 0U,
          "(10) right after the skipped frame: not an originator");
    const uint32_t again = stream(last + 2000U + 1000U, MINS - 1U, 1000U);
    CHECK(sx1276_fhss_authority_streak() == MINS &&
          sx1276_fhss_authority_is_originator(again + 10U, 1U, 0U) == 1U,
          "(10) MIN_STREAK frames after the skip: originator again");
}

static void test_gap_boundaries(void) {
    /* EXACTLY the gap apart: the chain test is strict (round 3). */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 12U; ++i) {
        sx1276_fhss_authority_note_tx(2000U + i * GAP);
    }
    CHECK(sx1276_fhss_authority_streak() == 1U, "(3) exactly-GAP spacing never chains");
    CHECK(sx1276_fhss_authority_is_originator(2000U + 11U * GAP + 1U, 1U, 0U) == 0U,
          "(3) 12 sends exactly one gap apart: not an originator");
    /* One ms inside the gap chains; the boundary itself does not. */
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(100U);
    sx1276_fhss_authority_note_tx(100U + GAP - 1U);
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) gap == GAP-1 chains");
    sx1276_fhss_authority_note_tx(100U + GAP - 1U + GAP);
    CHECK(sx1276_fhss_authority_streak() == 1U, "(3) gap == GAP restarts");
    /* Senders 2 s apart (0.5 Hz) never chain. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 40U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 2000U);
        CHECK(sx1276_fhss_authority_streak() == 1U,
              "(3) 2 s spaced sends never chain (i=%u)", i);
    }
    CHECK(sx1276_fhss_authority_is_originator(1000U + 39U * 2000U + 5U, 1U, 0U) == 0U,
          "(3) 40 sends 2 s apart: still not an originator");
    /* Two immediate copies (37 ms apart) every 2 s chain to 2, never to 8. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 10U; ++i) {
        sx1276_fhss_authority_note_tx(1000U + i * 2000U);
        sx1276_fhss_authority_note_tx(1000U + i * 2000U + 37U);
    }
    CHECK(sx1276_fhss_authority_streak() == 2U, "(3) copy pairs chain to 2");
    CHECK(sx1276_fhss_authority_is_originator(1000U + 9U * 2000U + 40U, 1U, 0U) == 0U,
          "(3) copy pairs: not an originator");
}

static void test_command_sender_hearing_peer_never_earns_it(void) {
    /* A base under the RS-12.14 stream gate sends >= 1.0 s apart -- inside
     * the 1.5 s gap, so cadence alone no longer keeps it out. What does is
     * adoption: the gate is >= 1.0 s only while fragments flow, and every
     * header the non-originator accepts clears its streak. (Policy-level
     * version with the real consider_remote: rx_grid_policy.c seq. 8b.) */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 40U; ++i) {
        const uint32_t t = 1000U + i * 1000U;
        sx1276_fhss_authority_note_tx(t);               /* command */
        CHECK(sx1276_fhss_authority_streak() == 1U,
              "(3b) 1 Hz commands between heard frames: streak 1 (i=%u)", i);
        CHECK(sx1276_fhss_authority_is_originator(t + 10U, 1U, 0U) == 0U,
              "(3b) ... and never an originator (i=%u)", i);
        sx1276_fhss_authority_note_adopt();             /* tractor frame */
    }
    CHECK(sx1276_fhss_authority_streak_max() == 1U, "(3b) streak_max 1");
    /* Even at the gate's tightest legal spacing with two copies queued
     * behind it, one heard frame per second keeps the streak below 8. */
    sx1276_fhss_authority_reset();
    for (uint32_t i = 0U; i < 20U; ++i) {
        sx1276_fhss_authority_note_tx(50000U + i * 1000U);
        sx1276_fhss_authority_note_tx(50000U + i * 1000U + 37U);
        sx1276_fhss_authority_note_adopt();
    }
    CHECK(sx1276_fhss_authority_streak_max() == 2U,
          "(3b) copy pairs between heard frames: streak_max 2");
}

static void test_command_sender_deaf_case_decays(void) {
    /* The accepted cost of the 1.5 s gap, pinned so it stays visible: a
     * node that sends MIN_STREAK times ~1 s apart WITHOUT hearing its peer
     * is indistinguishable from a 1 fps stream and earns authority. On the
     * base that needs >= 7 s of deafness while its gate stays at 1.0 s --
     * but the gate drops to 0.12 s 1.5 s after the last fragment, which
     * chained under the old 1 s rule too. Either way it is transient: it
     * decays one gap after the last send. */
    sx1276_fhss_authority_reset();
    const uint32_t last = stream(700000U, MINS, 1000U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U,
          "(3c) MIN_STREAK deaf 1 Hz sends: authority (same as a 1 fps stream)");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP, 1U, 0U) == 0U,
          "(3c) ... decayed one gap after the last send");
    /* And the first heard header ends it at once (adopt clears it). */
    sx1276_fhss_authority_reset();
    const uint32_t last2 = stream(800000U, MINS, 1000U);
    sx1276_fhss_authority_note_adopt();
    CHECK(sx1276_fhss_authority_is_originator(last2 + 10U, 1U, 0U) == 0U,
          "(3c) one adopted header ends it");
}

static void test_post_demotion_single_tx(void) {
    /* The pure half of the PR #125 regression (the policy-level version
     * with the real consider_remote is in rx_grid_policy.c). */
    sx1276_fhss_authority_reset();               /* demotion edge */
    sx1276_fhss_authority_note_tx(20000U);       /* one command TX */
    CHECK(sx1276_fhss_authority_is_originator(20010U, 1U, 0U) == 0U,
          "(4) post-demotion single TX: NOT an originator");
    for (uint32_t i = 1U; i < MINS - 1U; ++i) {
        sx1276_fhss_authority_note_tx(20000U + i * 500U);
        CHECK(sx1276_fhss_authority_is_originator(20000U + i * 500U + 10U, 1U, 0U) == 0U,
              "(4) below MIN_STREAK after demotion: still not (i=%u)", i);
    }
    uint32_t last = stream(30000U, 12U, 200U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U, "(4) streaming again");
    sx1276_fhss_authority_note_adopt();
    CHECK(sx1276_fhss_authority_streak() == 0U, "(4) adopt clears the streak");
    CHECK(sx1276_fhss_authority_is_originator(last + 20U, 1U, 0U) == 0U,
          "(4) after adopting: not an originator");
}

static void test_decay_when_streaming_stops(void) {
    /* Round 3: a node that bursts 8 queued commands 250 ms apart (the idle
     * drain's poll cadence) and then goes quiet must lose authority one
     * gap after its last TX -- otherwise a post-demotion base could sit on
     * a stale self-anchored grid refusing the tractor forever. */
    sx1276_fhss_authority_reset();
    uint32_t last = stream(40000U, 8U, 250U);
    CHECK(sx1276_fhss_authority_is_originator(last + 100U, 1U, 0U) == 1U,
          "(5) right after the burst: originator (transient)");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP - 1U, 1U, 0U) == 1U,
          "(5) one ms before the gap: still");
    CHECK(sx1276_fhss_authority_is_originator(last + GAP, 1U, 0U) == 0U,
          "(5) at the gap: authority decayed");
    CHECK(sx1276_fhss_authority_is_originator(last + 60000U, 1U, 0U) == 0U,
          "(5) a minute later: still decayed");
    /* The streak value itself is not consulted while silent: only a new
     * TX can restart it, and it restarts at 1. */
    sx1276_fhss_authority_note_tx(last + 60000U);
    CHECK(sx1276_fhss_authority_streak() == 1U, "(5) next TX after silence restarts at 1");
}

static void test_gates_on_clock_and_adoption(void) {
    sx1276_fhss_authority_reset();
    uint32_t last = stream(1000U, 20U, 100U);
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 0U, 0U) == 0U,
          "(6) invalid clock: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 1U) == 0U,
          "(6) adopted grid: never an originator");
    CHECK(sx1276_fhss_authority_is_originator(last + 10U, 1U, 0U) == 1U,
          "(6) valid + unadopted + streaming: originator");
    CHECK(sx1276_fhss_authority_streak_max() == 20U, "(6) streak_max tracks");
}

static void test_tick_wrap(void) {
    sx1276_fhss_authority_reset();
    sx1276_fhss_authority_note_tx(0xFFFFFF00U);
    sx1276_fhss_authority_note_tx(0x00000050U);   /* 336 ms later, across wrap */
    CHECK(sx1276_fhss_authority_streak() == 2U, "(7) wrap-spanning gap chains");
    for (uint32_t i = 0U; i < 8U; ++i) {
        sx1276_fhss_authority_note_tx(0x00000050U + (i + 1U) * 100U);
    }
    CHECK(sx1276_fhss_authority_is_originator(0x00000050U + 8U * 100U + 50U, 1U, 0U) == 1U,
          "(7) authority across the wrap");
}

int main(void) {
    test_reset_has_no_authority();
    test_streaming_earns_authority();
    test_gap_boundaries();
    test_command_sender_hearing_peer_never_earns_it();
    test_command_sender_deaf_case_decays();
    test_post_demotion_single_tx();
    test_decay_when_streaming_stops();
    test_gates_on_clock_and_adoption();
    test_tick_wrap();
    test_one_fps_stream_earns_and_keeps_it();
    test_two_fps_stream_earns_and_keeps_it();
    test_silence_ends_it();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] fhss_authority: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] fhss_authority: 12 test fns (gap %u ms: 1 fps +/-10 ms and 2 fps hold "
           "authority, 2 s silence ends it, strict gap, decay, adoption keeps 1 Hz "
           "commands out, post-demotion single TX)\n",
           (unsigned)GAP);
    return 0;
}
