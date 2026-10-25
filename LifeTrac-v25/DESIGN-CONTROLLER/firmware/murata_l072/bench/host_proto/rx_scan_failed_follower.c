/*
 * RS-12.21 / FHSS leg 6 (2b_f3, 2026-10-24): ACCEPTANCE CHECK of the scan
 * FAILED fix, system-in-the-loop against the real firmware. The fix is
 * option (b) + L + d2 of firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md
 * (section 5 item 2; adopted by the design owner on 2026-10-24):
 *   (b)  scan FAILED is held only while the node holds originator
 *        authority (it follows its own clock): a final FAIL on any other
 *        node, and the first scan tick of a node in FAILED whose authority
 *        has decayed, re-enter SCANNING for a fresh 30 s attempt;
 *   L    a LOCK starts the LOCKED loss timer at the locking frame;
 *   d2   a 2 s scan dwell, the channel walk continuing across the retries
 *        and (b)'s restarts.
 * It replaces the characterisation of the two defects the fix removes
 * (PR #167, `git log` of this file):
 *   (1) a FOLLOWER (the base) whose cold-start scan ran out into FAILED
 *       never hopped again (leg 6): parked in STANDBY on the walker's last
 *       channel, re-armed there by the daemon's raw RXCONT and later on each
 *       PROBE's channel, it adopted the grid but never LOCKed or followed it
 *       and heard a few percent of the stream;
 *   (2) an ORIGINATOR (the tractor) in FAILED that adopted a grid heard in a
 *       camera stall's decay window lost originator authority, and with it
 *       self-follow, for good: in FAILED nothing LOCKs, so nothing demotes,
 *       so nothing clears the adoption.
 * Every check marked [main fails] below fails against the pre-fix
 * sx1276_rx.c and 500 ms dwell (shown 2026-10-24 in a scratch copy), and
 * every run here also passes the shared simulation's structural checks
 * (rx_reverse_delivery.c check_structure), which include RS-12.21's own:
 * FAILED never survives a scan tick without authority, every FAIL is held
 * or left by (b)'s rule with each exit recorded, the walker steps through
 * the channel table in order, and no LOCK demotes sooner than 2 s after
 * its last anchor.
 *
 * WHY (1). On leg 6 the base heard 5 of the tractor's 606 frames, on
 * channels consistent with a receiver parked on the scan walker's last
 * channel and then on each PROBE's TX channel; base d(fhss_dec_aligned) 0,
 * d(clk_demotion_reset) 0; post radio_state 4 (RX_CONT). Hypothesis H1:
 * the scan used its 4 x 30 s budget without a catch and entered FAILED,
 * which absorbed. The base kept transmitting on hop channels (its PROBEs)
 * while its receiver no longer followed the hop sequence -- the "synced
 * receivers" of the FCC 15.247(a)(1) row in the FCC notes (AI NOTES/
 * 2026-05-19_FCC_Part15_902-928_Compliance_Notes) -- so it was a compliance
 * state as well as a delivery defect.
 * WHY (2). On 2026-10-18 legs W/X the tractor's scan also ran out into
 * FAILED, harmlessly while it streamed: an originator follows its own clock
 * in any scan state. (b) keeps exactly that, and restarts the scan once the
 * authority decays, so a command heard in the decay window LOCKs and the
 * LOCK's demotion clears the adoption, as before FAILED.
 *
 * WHAT. Reuses the simulated SX1276, clock, air and main-loop driver of
 * check-rx-reverse-delivery (rx_reverse_delivery.c is #included with
 * RX_REVERSE_DELIVERY_SIM_ONLY, which leaves out its scenarios and main)
 * against the same REAL sx1276_rx.c / sx1276_tx.c / scan / grid / clock
 * TUs.
 * (1) The firmware is the BASE: a follower on profile 1. Added here:
 *   - originator: the tractor's 2 fps camera, one ~200 B fragment per frame
 *     (VECTOR p50 199 B), keyed 13.4 ms into the next slot of ITS grid --
 *     about 2 slots in 5 -- optionally on a clock running ppm fast;
 *   - rx daemon (image_rx_daemon.py, LIFETRAC_REACTIVE_FIRE=1): a PROBE
 *     after each frame heard (behind the shared 1.0 s send gate), and
 *     _ensure_rxcont -- a raw RegOpMode=RXCONT write, no retune -- after
 *     each own TX and every 5 s while idle.
 * (2) The firmware is the TRACTOR, with that harness's camera daemon and
 *   base model: scenario 8 (no command for 150 s, so the scan runs out into
 *   FAILED) with scenario 7's stream and command plane (2 fps, leg H train
 *   mix, aligned pump at train end behind the 1.0 s gate, idle drain), then
 *   300 s of commands with a 2.5 s camera stall every 9 s; and the same run
 *   without stalls as the control.
 *
 * (1) ASSUMED, as for the characterisation: no frame reaches the base for
 *   the whole 120 s scan budget (the stream starts 1 s after the first
 *   final FAIL); 0 ppm between the boards; fixed 200 B frames. The stream
 *   runs 180 s, and on to 60 s past the LOCK (300 s at most). Default seed
 *   852 (the characterisation's: it reproduced leg 6's first two catches
 *   under the defect); `-s <seed>` runs another -- seeds 1-1000 all pass.
 * (1) ASSERTED -- leg-6 replay:
 *   A1 the scan budget runs out as before -- four RX_SCAN_FAILED faults, sub
 *      0x20 0x21 0x22 0xA3, the final 120 s after activation -- but the
 *      follower does not stay FAILED: in the same pass it is SCANNING again
 *      (the edge recorded), its receiver in RX_CONT on the walker's next
 *      channel, (4 x REDESIGN/DWELL) mod 50 = 10 [main fails: STANDBY on
 *      the walker's last channel];
 *   A2 then the stream: the scan LOCKs on a frame heard on a walker channel
 *      (SNAPPED onto the stream's grid, adopted); every FAIL between the
 *      stream start and the LOCK is final (0xA3), 30 s after the one before
 *      [main fails: FAILED absorbs, no LOCK];
 *   A3 the base follows the hop grid from the LOCK on: still LOCKED at the
 *      end, never demoted, no further FAIL; on every pass in mid-slot its
 *      clock is on the originator's slot and its receiver, when listening,
 *      on that slot's channel; every frame heard after the LOCK was tuned
 *      by the slot follower and is ALIGNED; it hears >= 95 % of the frames
 *      keyed from 1 s after the LOCK; every PROBE is keyed in the
 *      originator's slot its header claims [main fails];
 *   A4 the receiver is never left out of RX (no pass ends outside RX_CONT
 *      with no TX keyed) and the daemon's raw RXCONT never has to re-arm it:
 *      the fix does not lean on a host re-arm [main fails].
 * (1b) ASSERTED -- a silent follower, 305 s, no stream: one RX_SCAN_FAILED
 *   every 30 s (sub 0x20 0x21 0x22, then 0xA3 every time; the warm bit,
 *   sticky for the boot, aside), each a restart
 *   into SCANNING, never FAILED on a scan tick; the walk covers all 50
 *   channels (d2: it continues across the restarts -- the first prototype
 *   restarted it at channel 0 and rescanned channels 0-14 for ever); never
 *   out of RX, never re-armed by the daemon [main fails].
 *
 * (2) ASSERTED -- default seed (scenario 7's); `-e` measures other seeds:
 *   T1 no command for 150 s: the streaming tractor's scan runs out into
 *      FAILED (one final FAIL, no LOCK) and holds it -- it holds authority;
 *      nothing adopted;
 *   T2 control, no stalls: authority on every pass from then on, no
 *      adoption, no LOCK, the own grid never renumbered, self-follow keeps
 *      running, every clean command sent while it held authority received,
 *      and FAILED held throughout (no restart) -- 2026-10-18 legs W/X;
 *   T3 with stalls: the first decay of authority in FAILED restarts the scan
 *      on the next tick, and the walker is back within one dwell of every
 *      decay; a command heard in a decay window LOCKs and is adopted, and a
 *      LOCK's demotion resets the adopted clock [main fails: adopted in
 *      FAILED for good, no LOCK, no demotion];
 *   T4 after the first adoption the tractor holds authority again and
 *      self-follows again, and it receives at least half of the commands
 *      sent from then on [main fails: no authority, no self-follow, fewer
 *      than half received].
 *   What T3/T4 recover to is the decay window before FAILED, which the 2 s
 *   dwell and L move (memo section 5 item 3, accepted with the fix): own-
 *   grid renumbering and phase restarts are not asserted here.
 *
 * `rx_scan_failed_follower -v` prints the runs (catches, PROBEs, counters).
 * `rx_scan_failed_follower -e` skips the checks and runs the evaluation the
 * memo's numbers come from; it prints and never fails (`-e3`: E3 only):
 *   E1  leg-6 replay, seeded trials: silent for the scan budget, the
 *       stream from 1 s after the final FAIL on a random grid phase: does
 *       the base LOCK, how long after the stream starts, and what share of
 *       the frames keyed from 120 s to 300 s after the start it hears;
 *       at 0 ppm and with the originator 2750 ppm fast (10-24: base L072
 *       610-1400 ppm slow vs its host, tractor grid ~+1600 ppm);
 *   E1w E1 with a host watchdog model, option (c): the rx daemon re-sends
 *       REG_PROFILE=1 (profile activation: sx1276_fhss_init +
 *       sx1276_rx_scan_reset) 200 ms after each final RX_SCAN_FAILED fault
 *       (not adopted; kept for comparison with the memo);
 *   E2  cold start, the stream already on at activation (random grid
 *       phase): P(no LOCK within the 120 s budget), the attempt that
 *       catches, the per-attempt catch probability, at 2 fps and 1 fps;
 *   E3  (2) over seeded runs: how many adopt a grid in the command phase,
 *       in how many authority never returns after the first adoption, the
 *       authority share of the phase's passes, commands received, LOCKs,
 *       demotions, phase restarts (tx_first_anchor > 1) and own-grid
 *       renumbering; with stalls, the control without, and the decay
 *       window before FAILED (commands from 5 s, so the scan LOCKs on them
 *       and never fails out: scenario 7's shape);
 *   E3w E3 with option (c)'s watchdog on the TRACTOR too: a re-activation
 *       restarts the originator's own grid at epoch 0 under the base
 *       (renumbered by hundreds of slots) -- why (c) was never proposed
 *       there.
 * Memo numbers, 2026-10-24 (b+L+d2): E1 100/100, median 17.9 s, p90
 * 56.3 s, 99.8 % heard (2750 ppm: 100/100, 99.9 %); E2 1.3 % at 2 fps,
 * 10.7 % at 1 fps; E3 with stalls: authority back in 50/50.
 */

#define RX_REVERSE_DELIVERY_SIM_ONLY 1
/* Some of its helpers (base_cmd_at_key, run_for_ms) are not used here. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include "rx_reverse_delivery.c"
#pragma GCC diagnostic pop

#include <stdlib.h>

/* The acceptance figures and the memo's evaluation are for this dwell; a
 * different one needs `-e` re-run (and check-rx-reverse-delivery scenario
 * 6's 6 s window rescaled for a dwell of 6 s or more). */
_Static_assert(SX1276_RX_SCAN_DWELL_MS == 2000U,
               "RS-12.21 d2: the acceptance check is sized for a 2 s scan dwell");

/* ------------------------------------------------------------------ */
/* Originator (tractor) model: a camera stream on its own grid         */
/* ------------------------------------------------------------------ */

#define OS_MAX_FRAMES      4096U
#define OS_KEY_US          ORIG_KEY_US   /* 12 ms head-start + 1.4 ms   */
#define OS_LOOKAHEAD_US    2000ULL
#define OS_FRAME_LEN       200U          /* 208 B on air: ~164 ms        */

typedef struct {
    uint32_t abs;
    uint32_t hz;
    uint64_t key_us;
    uint8_t  heard;
} os_frame_t;

typedef struct {
    uint8_t  on;
    uint64_t t0_us;            /* boundary of slot abs0, base (sim) time */
    uint32_t abs0;
    int32_t  ppm;              /* originator clock fast by ppm vs the base */
    uint32_t next_n;           /* next slot (from abs0) to consider       */
    uint64_t next_frame_us;    /* camera: next frame ready                */
    uint32_t period_ms, jitter_ms;
    uint8_t  len;
    uint32_t sent;
    os_frame_t log[OS_MAX_FRAMES];
} ostream_t;
static ostream_t g_os;

static uint64_t os_boundary_us(uint32_t n) {
    return g_os.t0_us + ((uint64_t)n * SLOT_US * (uint64_t)(1000000 - g_os.ppm))
                        / 1000000ULL;
}

/* The originator's absolute slot at sim time t (t >= t0). */
static uint32_t os_abs_at(uint64_t t_us) {
    return g_os.abs0 + (uint32_t)(((t_us - g_os.t0_us) * 1000000ULL)
                                  / (SLOT_US * (uint64_t)(1000000 - g_os.ppm)));
}

static void os_start(uint64_t t0_us, uint32_t abs0, int32_t ppm,
                     uint32_t period_ms, uint8_t len) {
    g_os.on = 1U;
    g_os.t0_us = t0_us;
    g_os.abs0 = abs0;
    g_os.ppm = ppm;
    g_os.next_n = 0U;
    g_os.period_ms = period_ms;
    g_os.jitter_ms = 40U;
    g_os.len = len;
    g_os.next_frame_us = t0_us + (uint64_t)rnd_range(0U, period_ms) * 1000ULL;
}

/* Key every slot whose boundary is (about to be) due: a camera frame that
 * is ready by a boundary goes out 13.4 ms into that slot; extra frames
 * ready in the same slot are dropped (pipeline). */
static void os_tick(void) {
    if (g_os.on == 0U) {
        return;
    }
    for (;;) {
        const uint64_t b = os_boundary_us(g_os.next_n);
        if (b > g_now_us + OS_LOOKAHEAD_US) {
            break;
        }
        if (g_os.next_frame_us <= b) {
            if (b + OS_KEY_US > g_now_us && g_os.sent < OS_MAX_FRAMES) {
                uint8_t payload[OS_FRAME_LEN];
                const uint16_t seq = (uint16_t)g_os.sent;
                const uint32_t abs = g_os.abs0 + g_os.next_n;
                memset(payload, 0x5A, sizeof(payload));
                payload[0] = MARK_ORIG;
                payload[1] = (uint8_t)(seq & 0xFFU);
                payload[2] = (uint8_t)(seq >> 8);
                if (air_schedule(PKT_FRAME, seq, b + OS_KEY_US, abs,
                                 (uint8_t)(OS_KEY_US / 1000ULL), payload,
                                 g_os.len) != NULL) {
                    g_os.log[seq].abs = abs;
                    g_os.log[seq].hz = hop_hz(abs);
                    g_os.log[seq].key_us = b + OS_KEY_US;
                    g_os.log[seq].heard = 0U;
                    ++g_os.sent;
                    ++g_sc.frames_sent;
                }
            }
            while (g_os.next_frame_us <= b) {
                g_os.next_frame_us += (uint64_t)rnd_range(
                    g_os.period_ms - g_os.jitter_ms,
                    g_os.period_ms + g_os.jitter_ms) * 1000ULL;
            }
        }
        ++g_os.next_n;
    }
}

/* ------------------------------------------------------------------ */
/* rx daemon model (image_rx_daemon.py, reactive fire)                 */
/* ------------------------------------------------------------------ */

#define RXD_PROBE_LAT_US   30000ULL    /* frame complete -> TX_FRAME_REQ  */
#define RXD_TXDONE_LAT_US  3000ULL     /* wait_for_tx_done -> _ensure_rxcont */
#define RXD_IDLE_CHECK_US  5000000ULL  /* idle-loop _ensure_rxcont throttle */
#define RXD_GATE_US        1000000ULL  /* shared send gate while streaming  */
#define RXD_WATCHDOG_US    200000ULL   /* final FAULT -> REG_PROFILE=1 (c)  */

typedef struct {
    uint8_t  reactive;
    uint8_t  watchdog;          /* option (c) model */
    uint64_t next_idle_us;
    uint64_t post_tx_us;        /* 0 = none pending */
    uint64_t last_send_us;      /* 0 = never        */
    uint64_t reactivate_us;     /* 0 = none pending */
    uint64_t seen_final_us;
    uint32_t frames_seen, tx_seen;
    uint32_t probes;
    uint32_t arms_idle, arms_after_tx;
    uint32_t reactivations;
} rxd_t;
static rxd_t g_rxd;

static uint8_t rxd_ensure_rxcont(void) {
    if (g_radio.mode == SX1276_STATE_RX_CONT || sx1276_tx_busy()) {
        return 0U;
    }
    {
        const ctx_t saved = g_ctx;
        g_ctx = CTX_HOST;
        host_raw_rxcont();
        g_ctx = saved;
    }
    return 1U;
}

/* REG_PROFILE=1 re-sent: host_cfg_profile_activate()'s profile-1 branch as
 * scenario_begin() models it (scheduler init + scan, clock, authority and
 * adoption reset). */
static void rxd_reactivate(void) {
    const ctx_t saved = g_ctx;
    g_ctx = CTX_HOST;
    (void)sx1276_fhss_init(0ULL, 0ULL, 0U);
    sx1276_rx_scan_reset();
    g_scan_mirror = SX1276_RX_SCAN_STATE_BOOT;
    g_ctx = saved;
    ++g_rxd.reactivations;
}

static void rxd_tick(void) {
    if (g_sc.frames_recv != g_rxd.frames_seen) {
        g_rxd.frames_seen = g_sc.frames_recv;
        if (g_rxd.reactive != 0U && g_bh.burst_left == 0U &&
            (g_rxd.last_send_us == 0U ||
             g_now_us >= g_rxd.last_send_us + RXD_GATE_US)) {
            g_bh.burst_left = 1U;
            g_bh.burst_gap_ms = 0U;
            g_bh.burst_at_us = g_now_us + RXD_PROBE_LAT_US;
            g_rxd.last_send_us = g_bh.burst_at_us;
            ++g_rxd.probes;
        }
    }
    if (g_sc.fw_tx_ok != g_rxd.tx_seen) {
        g_rxd.tx_seen = g_sc.fw_tx_ok;
        g_rxd.post_tx_us = g_now_us + RXD_TXDONE_LAT_US;
        g_rxd.last_send_us = g_now_us;
    }
    if (g_rxd.post_tx_us != 0U && g_now_us >= g_rxd.post_tx_us) {
        g_rxd.post_tx_us = 0U;
        g_rxd.arms_after_tx += rxd_ensure_rxcont();
    }
    if (g_now_us >= g_rxd.next_idle_us) {
        g_rxd.next_idle_us += RXD_IDLE_CHECK_US;
        g_rxd.arms_idle += rxd_ensure_rxcont();
    }
    if (g_rxd.watchdog != 0U) {
        if (g_final_fail_us != 0U && g_final_fail_us != g_rxd.seen_final_us) {
            g_rxd.seen_final_us = g_final_fail_us;
            g_rxd.reactivate_us = g_now_us + RXD_WATCHDOG_US;
        }
        if (g_rxd.reactivate_us != 0U && g_now_us >= g_rxd.reactivate_us &&
            !sx1276_tx_busy()) {
            g_rxd.reactivate_us = 0U;
            rxd_reactivate();
        }
    }
}

/* ------------------------------------------------------------------ */
/* One run: activation, scan budget, stream                            */
/* ------------------------------------------------------------------ */

#define L6_MAX_CATCH 1024U   /* every frame of a 300 s stream at 2 fps */
#define L6_DEC_DIM   SX1276_RX_CONSIDER_REMOTE_COUNT_DIM
/* Mid-slot window for the on-grid check: both clocks are within a few ms
 * of each other (ms-quantised anchors), so stay clear of the boundary. */
#define L6_GRID_GUARD_US 10000ULL

typedef struct {
    uint64_t t_us;
    uint32_t abs;
    uint32_t hz;
    uint8_t  dec;
    uint8_t  tuner;            /* who had tuned the synth for it          */
    uint8_t  locked_before;    /* scan LOCKED before the frame            */
} l6_catch_t;

typedef struct {
    uint32_t seed;
    int32_t  ppm;
    uint32_t period_ms;
    uint8_t  stream_at_activation;   /* E2: stream on from activation */
    uint32_t start_abs;              /* E1: 0 = random                */
    uint32_t run_after_start_ms;     /* E1 */
    uint32_t after_lock_ms;          /* and at least this past the LOCK
                                      * (no LOCK: up to L6_STREAM_CAP_MS) */
    uint32_t cap_ms;                 /* E2: give up after this        */
    uint32_t silent_ms;              /* (1b): no stream at all        */
    uint8_t  watchdog;
    uint8_t  stop_at_lock;
} l6_cfg_t;

typedef struct {
    uint32_t act_ms;
    uint64_t act_us;
    uint64_t start_us;          /* stream start (0 = not yet)              */
    uint64_t final_pass_us;     /* pass that saw the first final FAIL      */
    uint8_t  final_mode;        /* modem state right after it              */
    uint32_t final_hz;
    uint8_t  final_tuner;
    uint8_t  final_scan;        /* scan state (mirror) right after it      */
    uint64_t lock_us;           /* first LOCK (0 = none)                   */
    uint32_t fails_before_lock;
    uint8_t  lock_dec, lock_tuner;
    uint32_t adoptions_at_lock;
    uint32_t ncatch;
    l6_catch_t catches[L6_MAX_CATCH];
    uint32_t dec[L6_DEC_DIM];
    uint32_t tx_done, tx_on_grid;
    uint32_t tx_hz[L6_MAX_CATCH];
    uint32_t tx_abs[L6_MAX_CATCH];
    uint32_t pass_not_rx;       /* passes ending outside RX, no TX keyed   */
    uint32_t grid_checked, grid_off;  /* after the LOCK, mid-slot passes   */
} l6_t;
static l6_t g_l6;

static void l6_on_catch(uint8_t locked_before) {
    const uint16_t seq = (uint16_t)(g_frame.payload[1] |
                                    ((uint16_t)g_frame.payload[2] << 8));
    if (seq >= g_os.sent) {
        CHECK(0, "[%s] heard an unknown frame (seq %u)", g_sc.name, seq);
        return;
    }
    g_os.log[seq].heard = 1U;
    if (g_last_dec < L6_DEC_DIM) {
        ++g_l6.dec[g_last_dec];
    }
    if (g_l6.ncatch < L6_MAX_CATCH) {
        l6_catch_t *k = &g_l6.catches[g_l6.ncatch];
        k->t_us = g_now_us;
        k->abs = g_os.log[seq].abs;
        k->hz = g_os.log[seq].hz;
        k->dec = g_last_dec;
        k->tuner = (uint8_t)g_heard_tuner;
        k->locked_before = locked_before;
    }
    ++g_l6.ncatch;
}

static void l6_on_tx_done(void) {
    lora_pkt_hdr_t h;
    if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) !=
        LORA_PKT_HDR_OK) {
        return;
    }
    {
        const uint32_t abs = sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx);
        if (g_l6.tx_done < L6_MAX_CATCH) {
            g_l6.tx_hz[g_l6.tx_done] = hop_hz(abs);
            g_l6.tx_abs[g_l6.tx_done] = abs;
        }
        ++g_l6.tx_done;
        /* Keyed in the originator's slot its header claims. */
        if (g_os.on != 0U && g_radio.tx_start_us >= g_os.t0_us &&
            os_abs_at(g_radio.tx_start_us) == abs) {
            ++g_l6.tx_on_grid;
        }
    }
}

static void l6_pass(void) {
    const uint32_t frames0 = g_sc.frames_recv;
    const uint32_t txok0 = g_sc.fw_tx_ok;
    const uint32_t locks0 = g_sc.locks;
    const uint8_t  locked0 = (uint8_t)(g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED);

    os_tick();
    fw_pass();

    if (g_sc.frames_recv != frames0) {
        l6_on_catch(locked0);
    }
    if (g_sc.fw_tx_ok != txok0) {
        l6_on_tx_done();
    }
    if (g_sc.locks != locks0 && g_l6.lock_us == 0U) {
        g_l6.lock_us = g_now_us;
        g_l6.fails_before_lock = g_sc.scan_fail_urc;
        g_l6.lock_dec = g_last_dec;
        g_l6.lock_tuner = (uint8_t)g_heard_tuner;
        g_l6.adoptions_at_lock = g_sc.adoptions;
    }
    if (g_final_fail_us != 0U && g_l6.final_pass_us == 0U) {
        g_l6.final_pass_us = g_now_us;
        g_l6.final_mode = (uint8_t)g_radio.mode;
        g_l6.final_hz = g_radio.freq_hz;
        g_l6.final_tuner = (uint8_t)g_tuner;
        g_l6.final_scan = (uint8_t)g_scan_mirror;
    }
    if (g_radio.mode != SX1276_STATE_RX_CONT && !sx1276_tx_busy()) {
        ++g_l6.pass_not_rx;
    }
    /* From the LOCK on, in mid-slot: the clock on the originator's slot and
     * the receiver, when listening, on that slot's channel. */
    if (g_l6.lock_us != 0U && g_os.on != 0U && g_now_us >= g_os.t0_us &&
        g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED) {
        const uint32_t abs = os_abs_at(g_now_us);
        const uint64_t in_slot = g_now_us - os_boundary_us(abs - g_os.abs0);
        if (in_slot >= L6_GRID_GUARD_US && in_slot + L6_GRID_GUARD_US < SLOT_US) {
            ++g_l6.grid_checked;
            if (sx1276_fhss_clock_valid() == 0U ||
                sx1276_fhss_clock_abs_slot(platform_now_ms()) != abs ||
                (g_radio.mode == SX1276_STATE_RX_CONT &&
                 g_radio.freq_hz != hop_hz(abs))) {
                ++g_l6.grid_off;
            }
        }
    }
    rxd_tick();
}

static void l6_start_stream(const l6_cfg_t *c, uint64_t t_us) {
    const uint32_t abs0 = (c->start_abs != 0U)
        ? c->start_abs
        : rnd_range(0U, 999U) * SX1276_FHSS_CHANNEL_COUNT
          + rnd_range(0U, SX1276_FHSS_CHANNEL_COUNT - 1U);
    os_start(t_us + (uint64_t)rnd_range(0U, (uint32_t)SLOT_US - 1U), abs0,
             c->ppm, c->period_ms, (uint8_t)OS_FRAME_LEN);
    g_l6.start_us = g_now_us;
}

#define L6_STREAM_CAP_MS 300000U

/* Keep the stream running? run_after_start_ms after its start, then (with
 * after_lock_ms) until that long after the LOCK, or the cap without one. */
static uint8_t l6_more(const l6_cfg_t *c) {
    const uint64_t t = g_now_us - g_l6.start_us;
    if (c->stop_at_lock != 0U && g_l6.lock_us != 0U) {
        return 0U;
    }
    if (t < (uint64_t)c->run_after_start_ms * 1000ULL) {
        return 1U;
    }
    if (c->after_lock_ms == 0U) {
        return 0U;
    }
    if (g_l6.lock_us == 0U) {
        return (uint8_t)(t < (uint64_t)L6_STREAM_CAP_MS * 1000ULL);
    }
    return (uint8_t)(g_now_us < g_l6.lock_us + (uint64_t)c->after_lock_ms * 1000ULL);
}

static void l6_run(const char *name, const l6_cfg_t *c) {
    scenario_begin(name, ROLE_BASE, c->seed);
    memset(&g_os, 0, sizeof(g_os));
    memset(&g_rxd, 0, sizeof(g_rxd));
    memset(&g_l6, 0, sizeof(g_l6));
    g_rxd.reactive = 1U;
    g_rxd.watchdog = c->watchdog;
    g_rxd.next_idle_us = g_now_us + (uint64_t)rnd_range(0U, 4999U) * 1000ULL;
    g_l6.act_us = g_now_us;
    g_l6.act_ms = platform_now_ms();
    if (c->silent_ms != 0U) {
        while (g_now_us < g_l6.act_us + (uint64_t)c->silent_ms * 1000ULL) {
            l6_pass();
        }
        return;
    }
    if (c->stream_at_activation != 0U) {
        l6_start_stream(c, g_now_us);
        while (g_now_us < g_l6.act_us + (uint64_t)c->cap_ms * 1000ULL &&
               !(c->stop_at_lock != 0U && g_l6.lock_us != 0U)) {
            l6_pass();
        }
        return;
    }
    /* Silent for the scan budget; the stream starts 1 s after the first
     * final FAIL (or at +121 s if a variant never reports one). */
    while (g_l6.start_us == 0U) {
        l6_pass();
        if ((g_final_fail_us != 0U && g_now_us >= g_final_fail_us + 1000000ULL) ||
            g_now_us >= g_l6.act_us + 121000000ULL) {
            l6_start_stream(c, g_now_us);
        }
    }
    while (l6_more(c) != 0U) {
        l6_pass();
    }
}

/* ------------------------------------------------------------------ */
/* The acceptance check, base: leg 6 replayed, and a silent follower   */
/* ------------------------------------------------------------------ */

static const char *dec_name(uint8_t d) {
    switch (d) {
        case SX1276_FHSS_SNAP_DEC_ALIGNED: return "ALIGNED";
        case SX1276_FHSS_SNAP_DEC_SNAPPED: return "SNAPPED";
        case SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT: return "EPOCH_DRIFT";
        case SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT: return "LOCKED_OUT";
        default: return "other";
    }
}

#define L6_LEG6_SEED 852U
static uint32_t g_l6_seed = L6_LEG6_SEED;

/* Scan attempts in the budget, and the walker channel after them: each
 * attempt dwells on REDESIGN / DWELL channels and the walk continues. */
#define L6_ATTEMPTS        (SX1276_RX_SCAN_MAX_RETRIES + 1U)
#define L6_DWELLS_PER_TRY  (SX1276_RX_SCAN_REDESIGN_MS / SX1276_RX_SCAN_DWELL_MS)

static void test_leg6_recovers(void) {
    /* Leg 6's first catch was (epoch 13, hop 17); start the grid at epoch
     * 13 as the characterisation did. */
    const l6_cfg_t c = {
        .seed = g_l6_seed, .ppm = 0, .period_ms = 500U,
        .stream_at_activation = 0U,
        .start_abs = 13U * SX1276_FHSS_CHANNEL_COUNT,
        .run_after_start_ms = 180000U, .after_lock_ms = 60000U,
        .cap_ms = 0U, .silent_ms = 0U,
        .watchdog = 0U, .stop_at_lock = 0U,
    };
    const uint8_t walker_next = (uint8_t)((L6_ATTEMPTS * L6_DWELLS_PER_TRY)
                                          % SX1276_FHSS_CHANNEL_COUNT);
    uint32_t after_follow = 0U, after_aligned = 0U, after_n = 0U;
    uint32_t late_sent = 0U, late_heard = 0U;
    uint32_t fails_unlocked_bad = 0U;

    l6_run("leg 6 replay: follower recovers", &c);
    /* A PROBE may be on air when the run ends: let it finish. */
    for (int i = 0; i < 1000 && sx1276_tx_busy(); ++i) {
        l6_pass();
    }
    print_scenario();
    check_structure();

    /* A1  the budget runs out as before, but FAILED does not hold. */
    CHECK(g_sc.scan_fail_urc >= 4U && g_fail_sub[0] == 0x20U &&
          g_fail_sub[1] == 0x21U && g_fail_sub[2] == 0x22U &&
          g_fail_sub[3] == 0xA3U,
          "[%s] scan faults: %u, sub %02x %02x %02x %02x (expect >= 4: 20 21 22"
          " a3)", g_sc.name, g_sc.scan_fail_urc, g_fail_sub[0], g_fail_sub[1],
          g_fail_sub[2], g_fail_sub[3]);
    CHECK((uint32_t)(g_fail_ms[3] - g_l6.act_ms) >= 120000U &&
          (uint32_t)(g_fail_ms[3] - g_l6.act_ms) < 120100U,
          "[%s] final FAIL %u ms after activation (expect 120 s)", g_sc.name,
          (unsigned)(g_fail_ms[3] - g_l6.act_ms));
    CHECK(g_l6.final_scan == (uint8_t)SX1276_RX_SCAN_STATE_SCANNING &&
          g_l6.final_mode == (uint8_t)SX1276_STATE_RX_CONT &&
          g_l6.final_tuner == (uint8_t)CTX_SCAN &&
          chan_of_hz(g_l6.final_hz) == walker_next,
          "[%s] after the final FAIL: scan state %u (expect SCANNING %u), mode"
          " %u (expect RX_CONT %u), on ch %u (expect the walker's next, %u),"
          " last tuned by ctx %u", g_sc.name, g_l6.final_scan,
          (unsigned)SX1276_RX_SCAN_STATE_SCANNING, g_l6.final_mode,
          (unsigned)SX1276_STATE_RX_CONT, chan_of_hz(g_l6.final_hz),
          walker_next, g_l6.final_tuner);

    /* A2  the stream: a LOCK on a walker channel; FAILs before it final. */
    for (uint32_t i = 4U; i < g_l6.fails_before_lock && i < FAIL_LOG; ++i) {
        if (g_fail_sub[i] != 0xA3U ||
            (uint32_t)(g_fail_ms[i] - g_fail_ms[i - 1U]) <
                SX1276_RX_SCAN_REDESIGN_MS ||
            (uint32_t)(g_fail_ms[i] - g_fail_ms[i - 1U]) >=
                SX1276_RX_SCAN_REDESIGN_MS + 100U) {
            ++fails_unlocked_bad;
        }
    }
    CHECK(g_l6.lock_us != 0U && g_l6.lock_us >= g_l6.start_us &&
          g_l6.lock_tuner == (uint8_t)CTX_SCAN &&
          g_l6.lock_dec == (uint8_t)SX1276_FHSS_SNAP_DEC_SNAPPED &&
          g_l6.adoptions_at_lock == 1U && fails_unlocked_bad == 0U,
          "[%s] no LOCK on the stream: LOCK %s, on a channel tuned by ctx %u"
          " (expect the walker, %u), decision %s (expect SNAPPED), adoptions"
          " %u (expect 1); %u of the FAILs before it not final 30 s apart",
          g_sc.name, (g_l6.lock_us != 0U) ? "yes" : "never", g_l6.lock_tuner,
          (unsigned)CTX_SCAN, dec_name(g_l6.lock_dec), g_l6.adoptions_at_lock,
          fails_unlocked_bad);

    /* A3  from the LOCK on, the base follows the hop grid. */
    for (uint32_t i = 0U; i < g_l6.ncatch && i < L6_MAX_CATCH; ++i) {
        const l6_catch_t *k = &g_l6.catches[i];
        if (k->locked_before == 0U) {
            continue;
        }
        ++after_n;
        after_follow += (uint32_t)(k->tuner == (uint8_t)CTX_FOLLOW);
        after_aligned += (uint32_t)(k->dec == (uint8_t)SX1276_FHSS_SNAP_DEC_ALIGNED);
    }
    for (uint32_t i = 0U; i < g_os.sent; ++i) {
        if (g_l6.lock_us != 0U && g_os.log[i].key_us >= g_l6.lock_us + 1000000ULL) {
            ++late_sent;
            late_heard += g_os.log[i].heard;
        }
    }
    CHECK(g_l6.lock_us != 0U && g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED &&
          g_sc.demotions == 0U && g_sc.scan_fail_urc == g_l6.fails_before_lock,
          "[%s] after the LOCK: scan state %u (expect LOCKED), demotions %u,"
          " FAILs %u after it (expect 0 0)", g_sc.name, (unsigned)g_scan_mirror,
          g_sc.demotions, g_sc.scan_fail_urc - g_l6.fails_before_lock);
    CHECK(g_l6.grid_checked > 1000U && g_l6.grid_off == 0U,
          "[%s] off the originator's hop grid on %u of %u mid-slot passes after"
          " the LOCK (clock slot, or the receiver's channel)", g_sc.name,
          g_l6.grid_off, g_l6.grid_checked);
    CHECK(after_n >= 100U && after_follow == after_n && after_aligned == after_n,
          "[%s] frames heard after the LOCK: %u, tuned by the slot follower %u,"
          " ALIGNED %u (expect all)", g_sc.name, after_n, after_follow,
          after_aligned);
    CHECK(late_sent != 0U && late_heard * 100U >= late_sent * 95U,
          "[%s] heard %u of the %u frames keyed from 1 s after the LOCK"
          " (expect >= 95 %%)", g_sc.name, late_heard, late_sent);
    CHECK(g_l6.tx_done >= 10U && g_l6.tx_on_grid == g_l6.tx_done,
          "[%s] PROBEs %u, keyed in the originator's slot they claim %u"
          " (expect all, >= 10)", g_sc.name, g_l6.tx_done, g_l6.tx_on_grid);

    /* A4  never out of RX, never re-armed by the daemon. */
    CHECK(g_l6.pass_not_rx == 0U && g_rxd.arms_idle == 0U &&
          g_rxd.arms_after_tx == 0U,
          "[%s] receiver out of RX at the end of %u passes with no TX keyed;"
          " the daemon's raw RXCONT re-armed it %u times idle, %u after a TX"
          " (expect 0 0 0)", g_sc.name, g_l6.pass_not_rx, g_rxd.arms_idle,
          g_rxd.arms_after_tx);

    if (g_verbose != 0) {
        char lock_buf[32];
        if (g_l6.lock_us != 0U) {
            snprintf(lock_buf, sizeof(lock_buf), "+%.2f s",
                     (double)(g_l6.lock_us - g_l6.start_us) / 1e6);
        } else {
            snprintf(lock_buf, sizeof(lock_buf), "never");
        }
        printf("    activation -> final FAIL %u ms, then SCANNING on ch %u;"
               " stream from +%llu ms, LOCK %s after it (after %u FAILs);"
               " %u frames sent, %u heard; after the LOCK %u heard (%u of %u"
               " keyed from 1 s after it), %u mid-slot passes checked on grid;"
               " PROBEs %u\n",
               (unsigned)(g_fail_ms[3] - g_l6.act_ms), chan_of_hz(g_l6.final_hz),
               (unsigned long long)((g_l6.start_us - g_l6.act_us) / 1000ULL),
               lock_buf, g_l6.fails_before_lock, g_os.sent, g_l6.ncatch, after_n,
               late_heard, late_sent, g_l6.grid_checked, g_l6.tx_done);
        for (uint32_t i = 0U; i < g_l6.ncatch && i < 3U; ++i) {
            const l6_catch_t *k = &g_l6.catches[i];
            printf("    catch %u at +%.2f s: (%u,%u) ch %u -> %s, tuned by ctx %u\n",
                   i + 1U, (double)(k->t_us - g_l6.act_us) / 1e6,
                   sx1276_fhss_clock_epoch_of(k->abs),
                   (unsigned)sx1276_fhss_clock_hop_of(k->abs), chan_of_hz(k->hz),
                   dec_name(k->dec), k->tuner);
        }
        printf("    daemon: probes=%u raw arms idle=%u after-TX=%u\n",
               g_rxd.probes, g_rxd.arms_idle, g_rxd.arms_after_tx);
    }
}

#define SILENT_MS 305000U   /* ten 30 s attempts */

static void test_silent_follower(void) {
    const l6_cfg_t c = {
        .seed = g_l6_seed, .ppm = 0, .period_ms = 500U,
        .stream_at_activation = 0U, .start_abs = 0U,
        .run_after_start_ms = 0U, .after_lock_ms = 0U, .cap_ms = 0U,
        .silent_ms = SILENT_MS,
        .watchdog = 0U, .stop_at_lock = 0U,
    };
    const uint32_t expect_fails = SILENT_MS / SX1276_RX_SCAN_REDESIGN_MS;
    uint32_t bad_sub = 0U, bad_gap = 0U, seen = 0U;

    l6_run("silent follower: report and keep scanning", &c);
    print_scenario();
    check_structure();

    for (uint32_t i = 0U; i < g_sc.scan_fail_urc && i < FAIL_LOG; ++i) {
        /* The warm bit is sticky for the boot (sx1276_rx_scan_reset keeps
         * it), so it is set here when an earlier run in this process
         * LOCKed: compare without it, and require it unchanged. */
        const uint8_t warm = (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_WARM);
        const uint8_t want = (i < L6_ATTEMPTS - 1U)
            ? (uint8_t)(0x20U | i)
            : (uint8_t)(0xA0U | (L6_ATTEMPTS - 1U));
        bad_sub += (uint32_t)((g_fail_sub[i] & (uint8_t)~warm) != want ||
                              (g_fail_sub[i] & warm) != (g_fail_sub[0] & warm));
        if (i != 0U &&
            ((uint32_t)(g_fail_ms[i] - g_fail_ms[i - 1U]) < SX1276_RX_SCAN_REDESIGN_MS ||
             (uint32_t)(g_fail_ms[i] - g_fail_ms[i - 1U]) >=
                 SX1276_RX_SCAN_REDESIGN_MS + 100U)) {
            ++bad_gap;
        }
    }
    for (uint8_t i = 0U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        seen += (uint32_t)((g_sc.walker_seen >> i) & 1ULL);
    }
    CHECK(g_sc.scan_fail_urc == expect_fails && bad_sub == 0U && bad_gap == 0U &&
          g_sc.fail_final == expect_fails - (L6_ATTEMPTS - 1U) &&
          g_sc.final_restarts == g_sc.fail_final && g_sc.tick_restarts == 0U,
          "[%s] FAILs %u (expect %u, one per 30 s: %u off the 20 21 22 a3 a3 ..."
          " sequence, %u not 30 s after the one before), final %u, each a"
          " restart into SCANNING %u, restarts on a tick %u (expect 0)",
          g_sc.name, g_sc.scan_fail_urc, expect_fails, bad_sub, bad_gap,
          g_sc.fail_final, g_sc.final_restarts, g_sc.tick_restarts);
    CHECK(seen == SX1276_FHSS_CHANNEL_COUNT,
          "[%s] the walk covered %u of %u channels in %u s (d2: it must continue"
          " across the restarts)", g_sc.name, seen,
          (unsigned)SX1276_FHSS_CHANNEL_COUNT, SILENT_MS / 1000U);
    CHECK(g_l6.pass_not_rx == 0U && g_rxd.arms_idle == 0U &&
          g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING,
          "[%s] receiver out of RX at the end of %u passes, re-armed by the"
          " daemon %u times, scan state %u at the end (expect 0 0 SCANNING)",
          g_sc.name, g_l6.pass_not_rx, g_rxd.arms_idle, (unsigned)g_scan_mirror);
    if (g_verbose != 0) {
        printf("    FAIL sub-bytes:");
        for (uint32_t i = 0U; i < g_sc.scan_fail_urc && i < FAIL_LOG; ++i) {
            printf(" %02x", g_fail_sub[i]);
        }
        printf("; walker channels %u/%u\n", seen, (unsigned)SX1276_FHSS_CHANNEL_COUNT);
    }
}

/* ------------------------------------------------------------------ */
/* Tractor role: the originator in scan FAILED                         */
/* ------------------------------------------------------------------ */

#define TR_SILENT_MS  150000U   /* no command: the scan runs out (scen. 8) */
#define TR_PHASE_MS   300000U   /* then commands, with or without stalls   */
#define TR_SEED       0x789ABCDU   /* scenario 7's */

typedef struct {
    uint32_t seed;
    uint8_t  stalls;            /* a 2.5 s camera stall every 9 s (scen. 7) */
    uint8_t  watchdog;          /* (c)'s watchdog on the tractor (E3w only) */
    uint32_t silent_ms;         /* no command for this long (0: TR_SILENT_MS) */
} tr_cfg_t;

typedef struct {
    /* at the end of the silent phase */
    uint8_t  failed0, auth0;
    uint32_t adoptions0, locks0, fail_final0;
    uint64_t phase_us;
    uint32_t sent0, recv0, passes0, auth_passes0, follow_own0;
    /* first adoption in the command phase (adopt_us 0 = none) */
    uint64_t adopt_us;
    uint32_t sent_at_adopt, recv_at_adopt, passes_at_adopt, auth_at_adopt;
    uint32_t follow_own_at_adopt;
    /* the tractor-side watchdog model */
    uint64_t seen_final_us, reactivate_us;
    /* own grid renumbered: a TX keyed more than one slot off the grid of
     * the TX before it (a phase restart moves it by less than a slot) */
    uint8_t  have_tx;
    uint32_t last_tx_abs;
    uint64_t last_tx_us;
    uint32_t renumbers, renumber_max;
} tr_t;
static tr_t g_tr;

static void tr_on_tx_done(void) {
    lora_pkt_hdr_t h;
    uint32_t abs;
    if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) !=
        LORA_PKT_HDR_OK) {
        return;
    }
    abs = sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx);
    if (g_tr.have_tx != 0U) {
        const uint32_t expect = g_tr.last_tx_abs + (uint32_t)(
            (g_radio.tx_start_us - g_tr.last_tx_us + SLOT_US / 2ULL) / SLOT_US);
        const int32_t d = (int32_t)(abs - expect);
        const uint32_t ad = (uint32_t)((d < 0) ? -d : d);
        if (ad > 1U) {
            ++g_tr.renumbers;
            if (ad > g_tr.renumber_max) {
                g_tr.renumber_max = ad;
            }
        }
    }
    g_tr.have_tx = 1U;
    g_tr.last_tx_abs = abs;
    g_tr.last_tx_us = g_radio.tx_start_us;
}

static void tr_pass(uint8_t watchdog) {
    const uint32_t txok0 = g_sc.fw_tx_ok;
    fw_pass();
    if (g_sc.fw_tx_ok != txok0) {
        tr_on_tx_done();
    }
    if (watchdog != 0U) {
        if (g_final_fail_us != 0U && g_final_fail_us != g_tr.seen_final_us) {
            g_tr.seen_final_us = g_final_fail_us;
            g_tr.reactivate_us = g_now_us + RXD_WATCHDOG_US;
        }
        if (g_tr.reactivate_us != 0U && g_now_us >= g_tr.reactivate_us &&
            !sx1276_tx_busy()) {
            g_tr.reactivate_us = 0U;
            rxd_reactivate();
        }
    }
}

static void tr_run(const char *name, const tr_cfg_t *c) {
    static const uint8_t trains[4] = { 23U, 45U, 23U, 9U };   /* leg H mix */
    const uint32_t silent_ms = (c->silent_ms != 0U) ? c->silent_ms : TR_SILENT_MS;
    uint64_t until;
    scenario_begin(name, ROLE_TRACTOR, c->seed);
    memset(&g_os, 0, sizeof(g_os));
    memset(&g_rxd, 0, sizeof(g_rxd));
    memset(&g_tr, 0, sizeof(g_tr));
    g_th.period_ms = 500U;
    g_th.jitter_ms = 40U;
    memcpy(g_th.train_w, trains, 4U);
    g_th.next_frame_us = g_now_us + 50000ULL;
    g_th.next_rxcont_us = g_now_us;
    base_model_init();
    g_base.mode = CMD_PUMP_AT_TRAIN_END;
    g_base.min_gap_ms = 1000U;
    g_base.idle_drain = 1U;
    /* Scenario 8: no command until the scan has run out. */
    g_base.next_allowed_us = g_now_us + (uint64_t)silent_ms * 1000ULL;
    until = g_now_us + (uint64_t)silent_ms * 1000ULL;
    while (g_now_us < until) {
        tr_pass(c->watchdog);
    }
    g_tr.failed0 = (uint8_t)(g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED);
    g_tr.auth0 = auth_now_ms(platform_now_ms());
    g_tr.adoptions0 = g_sc.adoptions;
    g_tr.locks0 = g_sc.locks;
    g_tr.fail_final0 = g_sc.fail_final;
    g_tr.phase_us = g_now_us;
    g_tr.sent0 = g_sc.sent;
    g_tr.recv0 = g_sc.recv;
    g_tr.passes0 = g_sc.passes;
    g_tr.auth_passes0 = g_sc.auth_passes;
    g_tr.follow_own0 = g_sc.follow_own;
    if (c->stalls != 0U) {
        g_th.stall_every_ms = 9000U;
        g_th.stall_len_ms = 2500U;
        g_th.stall_from_us = g_now_us;
        g_th.stall_to_us = g_now_us;
    }
    until = g_now_us + (uint64_t)TR_PHASE_MS * 1000ULL;
    while (g_now_us < until) {
        tr_pass(c->watchdog);
        if (g_tr.adopt_us == 0U && g_sc.adoptions != g_tr.adoptions0) {
            g_tr.adopt_us = g_now_us;
            g_tr.sent_at_adopt = g_sc.sent;
            g_tr.recv_at_adopt = g_sc.recv;
            g_tr.passes_at_adopt = g_sc.passes;
            g_tr.auth_at_adopt = g_sc.auth_passes;
            g_tr.follow_own_at_adopt = g_sc.follow_own;
        }
    }
}

static void check_tr_silent_phase(void) {
    /* T1 */
    CHECK(g_tr.failed0 != 0U && g_tr.fail_final0 == 1U && g_tr.locks0 == 0U &&
          g_tr.adoptions0 == 0U && g_tr.auth0 != 0U,
          "[%s] after %u s without a command: scan FAILED %u (final FAILs %u),"
          " locks %u, adoptions %u, authority %u (expect 1 1 0 0 1)", g_sc.name,
          TR_SILENT_MS / 1000U, g_tr.failed0, g_tr.fail_final0, g_tr.locks0,
          g_tr.adoptions0, g_tr.auth0);
}

static void test_tractor_failed_recovers(void) {
    tr_cfg_t c = { .seed = TR_SEED, .stalls = 0U, .watchdog = 0U };

    /* T1 + T2: the control -- FAILED held while the tractor streams. */
    tr_run("tractor in scan FAILED, no stalls (control)", &c);
    print_scenario();
    check_structure();
    check_tr_silent_phase();
    CHECK(g_sc.adoptions == 0U && g_sc.locks == 0U && g_tr.renumbers == 0U &&
          g_sc.auth_passes - g_tr.auth_passes0 == g_sc.passes - g_tr.passes0 &&
          g_sc.follow_own > g_tr.follow_own0 && g_sc.sent - g_tr.sent0 >= 100U &&
          g_sc.covered_clean_recv == g_sc.covered_clean,
          "[%s] control: adoptions %u locks %u, own grid renumbered %u times,"
          " authority on %u of %u passes, self-follow retunes %u, cmds sent"
          " %u, clean while covered %u of which received %u (expect 0 0 0,"
          " all, > 0, >= 100, all)", g_sc.name,
          g_sc.adoptions, g_sc.locks, g_tr.renumbers,
          g_sc.auth_passes - g_tr.auth_passes0,
          g_sc.passes - g_tr.passes0, g_sc.follow_own - g_tr.follow_own0,
          g_sc.sent - g_tr.sent0, g_sc.covered_clean, g_sc.covered_clean_recv);
    CHECK(g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED &&
          g_sc.tick_restarts == 0U && g_sc.final_restarts == 0U,
          "[%s] control: scan state %u at the end (expect FAILED, held under"
          " authority), restarts out of FAILED %u on a tick, %u at the final"
          " FAIL (expect 0 0)", g_sc.name, (unsigned)g_scan_mirror,
          g_sc.tick_restarts, g_sc.final_restarts);
    if (g_verbose != 0) {
        printf("    phase: cmds sent=%u recv=%u, authority on %u/%u passes\n",
               g_sc.sent - g_tr.sent0, g_sc.recv - g_tr.recv0,
               g_sc.auth_passes - g_tr.auth_passes0, g_sc.passes - g_tr.passes0);
    }

    /* T1 + T3 + T4: the same seed with camera stalls. */
    c.stalls = 1U;
    tr_run("tractor in scan FAILED, camera stalls", &c);
    print_scenario();
    check_structure();
    check_tr_silent_phase();
    CHECK(g_sc.tick_restarts >= 1U && g_sc.walker_after_decay_viol == 0U,
          "[%s] restarts out of FAILED on a tick %u (expect >= 1: the first"
          " decay of authority), walker later than one dwell after a decay"
          " %u times (expect 0)", g_sc.name, g_sc.tick_restarts,
          g_sc.walker_after_decay_viol);
    CHECK(g_tr.adopt_us != 0U && g_sc.locks >= 1U && g_sc.demotions >= 1U &&
          g_sc.dem_reset >= 1U,
          "[%s] a command heard in a decay window: adopted %s, locks %u,"
          " demotions %u, adopted clock reset at a demotion %u (expect yes,"
          " >= 1 each)", g_sc.name, (g_tr.adopt_us != 0U) ? "yes" : "never",
          g_sc.locks, g_sc.demotions, g_sc.dem_reset);
    CHECK(g_tr.adopt_us != 0U && g_sc.auth_passes > g_tr.auth_at_adopt &&
          g_sc.follow_own > g_tr.follow_own_at_adopt &&
          (g_sc.recv - g_tr.recv_at_adopt) * 2U >= g_sc.sent - g_tr.sent_at_adopt,
          "[%s] after the first adoption: authority on %u of %u passes,"
          " self-follow retunes %u, cmds received %u of %u (expect > 0, > 0,"
          " at least half)", g_sc.name,
          g_sc.auth_passes - g_tr.auth_at_adopt,
          g_sc.passes - g_tr.passes_at_adopt,
          g_sc.follow_own - g_tr.follow_own_at_adopt,
          g_sc.recv - g_tr.recv_at_adopt, g_sc.sent - g_tr.sent_at_adopt);
    if (g_verbose != 0) {
        printf("    phase: adopted at +%.1f s; before it cmds sent=%u recv=%u,"
               " authority on %u/%u passes; after it cmds sent=%u recv=%u,"
               " authority on %u/%u passes; own grid renumbered %u times (max"
               " %u slots)\n",
               (g_tr.adopt_us != 0U)
                   ? (double)(g_tr.adopt_us - g_tr.phase_us) / 1e6 : -1.0,
               g_tr.sent_at_adopt - g_tr.sent0, g_tr.recv_at_adopt - g_tr.recv0,
               g_tr.auth_at_adopt - g_tr.auth_passes0,
               g_tr.passes_at_adopt - g_tr.passes0,
               g_sc.sent - g_tr.sent_at_adopt, g_sc.recv - g_tr.recv_at_adopt,
               g_sc.auth_passes - g_tr.auth_at_adopt,
               g_sc.passes - g_tr.passes_at_adopt, g_tr.renumbers,
               g_tr.renumber_max);
    }
}

/* ------------------------------------------------------------------ */
/* Evaluation (-e): the memo's measures, SIL only                      */
/* ------------------------------------------------------------------ */

#define EV_MAX 1000U
static uint32_t g_ev[EV_MAX];

static void ev_sort(uint32_t *a, uint32_t n) {
    for (uint32_t i = 1U; i < n; ++i) {
        const uint32_t v = a[i];
        uint32_t j = i;
        while (j > 0U && a[j - 1U] > v) {
            a[j] = a[j - 1U];
            --j;
        }
        a[j] = v;
    }
}

static double ev_pct(const uint32_t *a, uint32_t n, uint32_t p) {
    if (n == 0U) {
        return -1.0;
    }
    return (double)a[((n - 1U) * p) / 100U] / 1000.0;
}

static double ev_sqrt(double x) {
    double r = (x > 1.0) ? x : 1.0;
    if (x <= 0.0) {
        return 0.0;
    }
    for (int i = 0; i < 60; ++i) {
        r = 0.5 * (r + x / r);
    }
    return r;
}

/* Wilson 95 % interval for k of n. */
static void ev_wilson(uint32_t k, uint32_t n, double *lo, double *hi) {
    const double z = 1.959964, p = (double)k / (double)n, nn = (double)n;
    const double den = 1.0 + z * z / nn;
    const double mid = (p + z * z / (2.0 * nn)) / den;
    const double half = z * ev_sqrt(p * (1.0 - p) / nn + z * z / (4.0 * nn * nn)) / den;
    *lo = mid - half;
    *hi = mid + half;
}

static void eval_e1(const char *label, uint32_t n, int32_t ppm, uint8_t watchdog) {
    uint32_t nlock = 0U, sent_late = 0U, heard_late = 0U, catches_unlocked = 0U;
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const l6_cfg_t c = {
            .seed = 0xE1000001U + t * 7919U, .ppm = ppm, .period_ms = 500U,
            .stream_at_activation = 0U, .start_abs = 0U,
            .run_after_start_ms = 300000U, .after_lock_ms = 0U, .cap_ms = 0U,
            .silent_ms = 0U,
            .watchdog = watchdog, .stop_at_lock = 0U,
        };
        l6_run(label, &c);
        if (g_l6.lock_us != 0U && g_l6.lock_us >= g_l6.start_us) {
            g_ev[nlock++] = (uint32_t)((g_l6.lock_us - g_l6.start_us) / 1000ULL);
        } else {
            catches_unlocked += g_l6.ncatch;
        }
        for (uint32_t i = 0U; i < g_os.sent; ++i) {
            if (g_os.log[i].key_us >= g_l6.start_us + 120000000ULL) {
                ++sent_late;
                heard_late += g_os.log[i].heard;
            }
        }
    }
    ev_sort(g_ev, nlock);
    printf("E1 %-34s n=%u ppm=%d: LOCKed %u/%u; T_lock after stream start"
           " median %.1f s p90 %.1f s max %.1f s; heard %u/%u = %.1f%% of the"
           " frames keyed 120-300 s after the start; catches in runs that"
           " never locked %u\n",
           label, n, (int)ppm, nlock, n, ev_pct(g_ev, nlock, 50U),
           ev_pct(g_ev, nlock, 90U), ev_pct(g_ev, nlock, 100U), heard_late,
           sent_late, (sent_late != 0U) ? 100.0 * heard_late / (double)sent_late : 0.0,
           catches_unlocked);
}

static void eval_e2(const char *label, uint32_t n, uint32_t period_ms) {
    uint32_t nlock = 0U, no_lock_120 = 0U, by_attempt[6];
    double lo = 0.0, hi = 0.0;
    memset(by_attempt, 0, sizeof(by_attempt));
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const l6_cfg_t c = {
            .seed = 0xE2000001U + t * 104729U + period_ms, .ppm = 0,
            .period_ms = period_ms, .stream_at_activation = 1U,
            .start_abs = 0U, .run_after_start_ms = 0U, .after_lock_ms = 0U,
            .cap_ms = 300000U,
            .silent_ms = 0U, .watchdog = 0U, .stop_at_lock = 1U,
        };
        l6_run(label, &c);
        if (g_l6.lock_us != 0U) {
            const uint32_t ms = (uint32_t)((g_l6.lock_us - g_l6.act_us) / 1000ULL);
            g_ev[nlock++] = ms;
            if (ms >= 120000U) {
                ++no_lock_120;
            }
            ++by_attempt[(g_l6.fails_before_lock < 5U) ? g_l6.fails_before_lock : 5U];
        } else {
            ++no_lock_120;
        }
    }
    ev_sort(g_ev, nlock);
    ev_wilson(no_lock_120, n, &lo, &hi);
    printf("E2 %-34s n=%u %u fps: no LOCK within 120 s %u/%u = %.1f%%"
           " (95%% %.1f-%.1f%%); LOCKed by 300 s %u/%u; T_lock median %.1f s"
           " p90 %.1f s; first LOCK after 0/1/2/3/4/5+ FAILs:"
           " %u/%u/%u/%u/%u/%u",
           label, n, 1000U / period_ms, no_lock_120, n,
           100.0 * no_lock_120 / (double)n, 100.0 * lo, 100.0 * hi, nlock, n,
           ev_pct(g_ev, nlock, 50U), ev_pct(g_ev, nlock, 90U),
           by_attempt[0], by_attempt[1], by_attempt[2], by_attempt[3],
           by_attempt[4], by_attempt[5]);
    {
        /* per-attempt catch probability, given the attempt was reached */
        uint32_t reached = n;
        printf("; per-attempt P(catch | reached) =");
        for (uint32_t a = 0U; a < 4U; ++a) {
            printf(" %.2f", (reached != 0U) ? (double)by_attempt[a] / reached : 0.0);
            reached -= by_attempt[a];
        }
        printf("\n");
    }
}

static void eval_e3(const char *label, uint32_t n, uint8_t stalls, uint8_t watchdog,
                    uint32_t silent_ms) {
    uint32_t nfailed = 0U, nadopt = 0U, nlost = 0U, nregrid = 0U, nend_adopted = 0U;
    uint32_t sent = 0U, recv = 0U, locks = 0U, dems = 0U, react = 0U;
    uint32_t renum = 0U, nrenum = 0U, renum_max = 0U;
    uint64_t passes = 0U, auth = 0U;
    for (uint32_t t = 0U; t < n && t < EV_MAX; ++t) {
        const tr_cfg_t c = {
            .seed = 0xE3000001U + t * 7919U, .stalls = stalls, .watchdog = watchdog,
            .silent_ms = silent_ms,
        };
        tr_run(label, &c);
        nfailed += g_tr.failed0;
        if (g_tr.adopt_us != 0U) {
            g_ev[nadopt++] = (uint32_t)((g_tr.adopt_us - g_tr.phase_us) / 1000ULL);
            if (g_sc.auth_passes == g_tr.auth_at_adopt) {
                ++nlost;
            }
        }
        nend_adopted += sx1276_rx_grid_adopted();
        if (g_sc.first_anchor > 1U) {
            ++nregrid;
        }
        if (g_tr.renumbers != 0U) {
            ++nrenum;
            renum += g_tr.renumbers;
            if (g_tr.renumber_max > renum_max) {
                renum_max = g_tr.renumber_max;
            }
        }
        passes += g_sc.passes - g_tr.passes0;
        auth += g_sc.auth_passes - g_tr.auth_passes0;
        sent += g_sc.sent - g_tr.sent0;
        recv += g_sc.recv - g_tr.recv0;
        locks += g_sc.locks;
        dems += g_sc.demotions;
        react += g_rxd.reactivations;
    }
    ev_sort(g_ev, nadopt);
    printf("E3 %-34s n=%u: scan FAILED when the commands start %u/%u;"
           " adopted a grid in the phase %u/%u (median %.1f s in), authority"
           " never back after it"
           " %u/%u, adopted at the end %u/%u; authority on %.1f%% of the phase"
           " passes; cmds received %u/%u = %.1f%%; locks %u demotions %u; phase"
           " restarts (tx_first_anchor > 1) in %u/%u runs; own grid renumbered"
           " (a TX > 1 slot off the previous TX's grid) %u times in %u/%u runs,"
           " by up to %u slots; re-activations %u\n",
           label, n, nfailed, n, nadopt, n, ev_pct(g_ev, nadopt, 50U), nlost,
           nadopt, nend_adopted, n,
           (passes != 0U) ? 100.0 * (double)auth / (double)passes : 0.0,
           recv, sent, (sent != 0U) ? 100.0 * recv / (double)sent : 0.0,
           locks, dems, nregrid, n, renum, nrenum, n, renum_max, react);
}

static void run_eval_tractor(void) {
    /* Commands from 5 s: the scan LOCKs on them and never fails out --
     * the decay window before FAILED (scenario 7's shape). */
    eval_e3("tractor SCANNING + stalls", 50U, 1U, 0U, 5000U);
    eval_e3("tractor FAILED, no stalls (control)", 20U, 0U, 0U, 0U);
    eval_e3("tractor FAILED + stalls", 50U, 1U, 0U, 0U);
    eval_e3("tractor FAILED + stalls + watchdog", 50U, 1U, 1U, 0U);
}

static void run_eval(void) {
    printf("[RXSFF] evaluation: scan DWELL %u ms, REDESIGN %u ms, MAX_RETRIES %u\n",
           (unsigned)SX1276_RX_SCAN_DWELL_MS, (unsigned)SX1276_RX_SCAN_REDESIGN_MS,
           (unsigned)SX1276_RX_SCAN_MAX_RETRIES);
    eval_e1("leg-6 replay", 100U, 0, 0U);
    eval_e1("leg-6 replay, drifting clocks", 100U, 2750, 0U);
    eval_e1("leg-6 replay + host watchdog (c)", 100U, 0, 1U);
    eval_e2("cold start, stream on", 600U, 500U);
    eval_e2("cold start, stream on", 600U, 1000U);
    run_eval_tractor();
}

int main(int argc, char **argv) {
    int eval = 0;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) { g_verbose = 1; }
        if (strcmp(argv[i], "-e") == 0) { eval = 1; }
        if (strcmp(argv[i], "-e3") == 0) { eval = 3; }
        if (strcmp(argv[i], "-s") == 0 && i + 1 < argc) {
            g_l6_seed = (uint32_t)strtoul(argv[++i], NULL, 0);
        }
    }
    if (eval == 3) {
        run_eval_tractor();
        return 0;
    }
    if (eval != 0) {
        run_eval();
        return 0;
    }
    test_leg6_recovers();
    test_silent_follower();
    test_tractor_failed_recovers();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_scan_failed_follower: %d failure(s) -- the"
                " RS-12.21 scan fix ((b) + L + d2) does not hold: a node without"
                " originator authority stays in, or does not recover from, scan"
                " FAILED (or an unrelated change to the scan constants, the"
                " walker, the TX path or the shared sim, rx_reverse_delivery.c"
                " -- see which check failed)\n", g_failures);
        return 1;
    }
    printf("[PASS] rx_scan_failed_follower: RS-12.21 acceptance -- a follower"
           " whose scan ran out reports, keeps scanning and LOCKs onto the"
           " stream, then follows the hop grid (leg 6); a silent follower"
           " reports every 30 s and walks all 50 channels; a tractor in scan"
           " FAILED that loses authority scans again, and regains authority"
           " and self-follow after an adoption\n");
    return 0;
}
