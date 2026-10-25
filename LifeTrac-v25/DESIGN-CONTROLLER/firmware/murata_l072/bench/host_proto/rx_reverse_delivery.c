/*
 * RS-12.15 (reverse-delivery half, 2026-10-14): base -> tractor command
 * delivery on the FHSS profile, system-in-the-loop.
 *
 * WHY. On air the base hops every command onto the streaming tractor's
 * grid (277 distinct hop/channel pairs in 281 sends, RS-12.12 leg I), yet
 * the tractor heard 1/17 (leg H) and 49/281 (leg I) on profile 1, and
 * 19/72, 2/43, 7/49 on RS-12.15 v2 (legs R, T, U), vs 56/58 on profile 2.
 * sx1276_rx_slot_follow() ran only while the scan machine was LOCKED, LOCK
 * demotes 2 s after the last valid frame, and a node that mostly transmits
 * rarely receives one -- so between trains the tractor listened on its
 * last TX channel or on the scan walker's, not on its own clock's.
 *
 * WHAT. This harness links the REAL sx1276_rx.c and sx1276_tx.c (and the
 * HW-free TUs they call) against a simulated SX1276 (register file, FIFO,
 * IRQ flags, modes, synthesiser; SPI timed at the firmware's 2 MHz SCK), a
 * simulated clock (time advances inside busy-waits and SPI traffic) and a
 * simulated air, and drives them in main.c's loop order: tx_poll ->
 * rx_service -> TX mailbox -> scan tick -> slot follow -> gamma-1 tick,
 * then WFI to the next 1 ms SysTick or radio IRQ. The peer is a model:
 *   tractor role (firmware = tractor): the camera daemon streams trains
 *     (pipeline depth 2, 40 ms train gap, raw RXCONT re-arm when idle);
 *     the base learns the grid from every tractor header the way its own
 *     firmware does (anchor_rx on its own, drifting ms clock, after the
 *     FIFO read) and keys each command by its slot rule (12 ms head-start,
 *     ToA + 15 ms fit guard, phase byte stamped after the PLL settle) on
 *     the hop channel.
 *   base role (firmware = base): an ideal originator streams one frame
 *     per slot on its own grid; the firmware follows it and sends
 *     commands.
 * The channel is ideal: a command is lost only to a half-duplex collision
 * (the tractor transmitted while it was on air -- not CLEAN) or to the
 * receiver not listening on its channel from its preamble on. It is
 * COVERED when the tractor held originator authority
 * (sx1276_fhss_authority_is_originator) from its first to its last symbol.
 *
 * PINNED (sequence numbers match the output):
 *   1/2 between trains, light and storm command planes: every clean,
 *       covered command is received (before the fix only an in-slot
 *       ALIGNED hit, or the 2 s LOCK after one, was heard);
 *   3/4 camera-shaped stream + aligned pump at train end (leg H / leg I
 *       shapes): every covered miss is a half-duplex collision;
 *   5   follower unchanged: a base-role node never holds authority, the
 *       walker retunes on every BEGIN_SCAN / ADVANCE, the slot follower
 *       runs only while LOCKED on an adopted grid, a demotion resets the
 *       adopted clock, and the node re-acquires a re-gridded originator;
 *   6   authority bound: a base-role node that bursts >= 8 commands < 1 s
 *       apart with no adopted grid follows its own clock only while it
 *       holds authority, and the walker is back within one dwell of the
 *       decay; it then re-acquires the originator;
 *   7   decay window (2026-10-14): the tractor stalls for > 1 s, so its
 *       authority decays while an echo may hold it LOCKED; its receiver
 *       must not follow its own clock there (a frame heard in that window
 *       is handed UNANCHORED and adopted; closing the window itself is a
 *       grid-policy matter, sx1276_rx_grid_policy.c, not this harness's);
 *   8   scan FAILED: with no command for 150 s the tractor's scan machine
 *       runs out of retries into FAILED, which a streaming node holds
 *       (RS-12.21 (b): FAILED is held only under originator authority) --
 *       it never LOCKs there; its own clock still drives the receiver and
 *       every clean, covered command is received;
 *   9   final FAIL mid-command: the streaming tractor's final scan FAIL
 *       fires while a clean command is on air. The fault is reported and
 *       FAILED committed, but the receiver is not parked: the slot
 *       follower would re-arm it only at the next boundary, so the
 *       command must still be received;
 *   10  CRC errors, then a clean retry: commands with payload-CRC errors
 *       early in the first scan attempt, then silence. Each FAIL's
 *       crc_seen bit covers its own window (the last dwell) only, as on
 *       a walking node, so neither of the first two FAILs reports one.
 * In EVERY scenario, per main-loop pass: the walker retunes exactly when
 * the scan machine dispatches BEGIN_SCAN / ADVANCE (or a FAIL, which
 * re-enters SCANNING unless it is held) unless the node holds originator
 * authority; no scan action takes the
 * receiver out of RX while the node holds authority; every slot-follow
 * retune is onto the clock's channel and allowed by the follow rule
 * (LOCKED on an adopted grid, or originator authority); and every own
 * transmission is keyed on the hop channel of the slot its header claims.
 * RS-12.21 (2026-10-24, firmware/murata_l072/FHSS_SCAN_FAILED_OPTIONS.md
 * section 5 item 2, adopted) adds four, in every scenario too:
 *   (b)  the scan machine is FAILED after a scan tick only while the node
 *        holds originator authority (or a TX is keyed: the tick is frozen
 *        then); a FAIL is held exactly when it is final and the node holds
 *        authority, and every exit from FAILED is recorded as
 *        (FAILED, BEGIN_SCAN) -- the mirror below follows the records;
 *   d2   the walker steps through the channel table in order: channel 0
 *        only after a BEGIN_SCAN out of BOOT or LOCKED, the next channel
 *        after a retry or a restart out of FAILED (the walk continues
 *        across attempts);
 *   L    no LOCKED -> SCANNING demotion comes sooner than LOCK_LOSS_MS
 *        after the LOCK or the last accepted frame.
 * In the tractor scenarios 1-4 and 8 the tractor's grid never moves: one
 * phase anchor, no adoption, no clock reset, no SNAP.
 *
 * `rx_reverse_delivery -v` also prints per-scenario counters (misses by
 * cause, consider_remote decisions, adoptions by what had tuned the
 * receiver, scan FAILs) and an FNV-1a trace hash over every
 * radio-affecting call with its time and caller. Scenario 5's hash is the
 * same against the pre-fix sx1276_rx.c: the follower path is
 * call-for-call, microsecond-for-microsecond unchanged. The hashes are
 * printed, not pinned. RS-12.21's 2 s scan dwell changed all ten
 * (2026-10-24; the walker's timing is in every scenario), with every
 * scenario's assertions unchanged; (b) and L alone change none but
 * scenario 7's (L: its LOCKs come out of SCANNING).
 *
 * check-rx-scan-failed-follower (rx_scan_failed_follower.c) reuses this
 * simulation: it #includes this file with RX_REVERSE_DELIVERY_SIM_ONLY
 * defined, which leaves out the scenarios below and main().
 * check-radio-park (radio_park.c, RS-12.21 2026-10-24) does too, with
 * RX_SIM_REAL_HOST also defined: it links the real host_cmd.c, host_cfg.c
 * and host_cfg_profile.c, so the stubs for host_cmd_emit_fault(),
 * host_uart_send_urc() and host_cfg_profile_active() drop out, and the
 * mailbox step below also services host_cmd.c's own TX ring. While the
 * host has the radio parked the scan tick is frozen, which the (b) check
 * allows.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "host_cfg_keys.h"
#include "host_cfg_profile.h"
#include "host_cmd.h"
#include "host_stats.h"
#include "host_types.h"
#include "host_uart.h"
#include "lora_pkt_hdr.h"
#include "platform.h"
#include "sx1276.h"
#include "sx1276_airtime.h"
#include "sx1276_fhss.h"
#include "sx1276_fhss_authority.h"
#include "sx1276_fhss_chantab.h"
#include "sx1276_fhss_clock.h"
#include "sx1276_lbt.h"
#include "sx1276_legal_dwell.h"
#include "sx1276_modes.h"
#include "sx1276_rx.h"
#include "sx1276_rx_grid_policy.h"
#include "sx1276_rx_scan_fail.h"
#include "sx1276_rx_scan_policy.h"
#include "sx1276_tx.h"

static int g_failures = 0;
static int g_verbose = 0;

#define CHECK(cond, ...) do {                                          \
    if (!(cond)) {                                                     \
        ++g_failures;                                                  \
        fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);           \
        fprintf(stderr, __VA_ARGS__);                                  \
        fputc('\n', stderr);                                           \
    }                                                                  \
} while (0)

/* ------------------------------------------------------------------ */
/* Timing model                                                        */
/* ------------------------------------------------------------------ */

#define SLOT_US            ((uint64_t)SX1276_FHSS_SLOT_MS * 1000ULL)
#define HEADSTART_US       ((uint64_t)SX1276_FHSS_SLOT_TX_HEADSTART_MS * 1000ULL)
#define COST_CLOCK_US      1ULL      /* one platform_now_us() read (the busy-
                                       * waits spin on it, so it must cost) */
/* platform_now_ms() reads the SysTick counter: free, so an extra ms read
 * in a firmware path does not perturb the timing it is compared on. */
/* SPI1 runs SCK = 2 MHz (sx1276_spi_init: PCLK2 16 MHz / 8) and
 * spi1_transfer() polls TXE/RXNE/BSY per byte: ~4.5 us a byte, so a
 * register access (address + data) is ~10 us and a 218 B FIFO burst ~1 ms
 * (the RS-3.6 comment's figure). */
#define COST_SPI_US        10ULL     /* one register access                */
#define SPI_BURST_US(len)  ((((uint64_t)(len) + 1ULL) * 9ULL) / 2ULL)
#define COST_MODE_US       40ULL     /* RF switch + DIO map + OpMode write */
#define COST_FREQ_US       30ULL     /* three FRF register writes          */
#define TX_RAMP_US         60ULL     /* OpMode=TX -> first preamble symbol */
#define WAKE_US            15ULL     /* WFI exit -> radio block            */
/* The receiver must be listening on the channel by this long after the
 * first preamble symbol (8-symbol preamble at SF7/BW250 = 4.1 ms; the
 * detector needs ~5 symbols). */
#define LATE_ARM_TOL_US    1500ULL

/* Peer models. The base's L072 runs the same firmware, so its copy of the
 * tractor's grid is quantised the same way: sx1276_fhss_clock_anchor_rx()
 * on ITS OWN ms counter (phase g_base.ms_phase_us against the tractor's,
 * drifting) at its rx_service clock read, which comes after the ~1 ms FIFO
 * burst; and the tractor stamps its phase byte ~1 ms (its FIFO load)
 * before key-up. The copy therefore lags the tractor's grid by ~1-2 ms,
 * spread by the ms floors and the half-up ToA rounding -- exactly what the
 * originator's rx_leads() test sees (an echo that seems to LEAD is
 * adopted). */
#define BASE_HOST_LAT_US   6000ULL   /* frag on air -> base daemon -> base L072 */
#define BASE_SRV_LAT_US    120ULL    /* RxDone -> rx_service clock read: wake +
                                      * six register accesses (+ the FIFO burst,
                                      * + 0..400 us of main-loop work) */
#define BASE_ADMIT_US      50ULL     /* TX_FRAME_REQ / boundary -> admission */
#define BASE_STAMP_US      1100ULL   /* admission -> phase-byte stamp (standby,
                                      * set_freq, 1 ms PLL settle) */
#define BASE_KEY_US        250ULL    /* stamp -> first preamble symbol (24 B
                                      * FIFO load, OpMode=TX, PA ramp) */
#define BASE_DRIFT_PPM     3ULL      /* base clock vs tractor clock: sweeps the
                                      * ms-tick phase ~0.9 ms per 300 s */
#define TRACTOR_HOST_LAT_US 3000ULL  /* TX_DONE URC -> next TX_FRAME_REQ */
#define TRAIN_GAP_US       40000ULL  /* image_tx_daemon TRAIN_GAP_S */
#define RXCONT_POLL_US     250000ULL /* idle queue.get(timeout=0.25) */
#define ORIG_KEY_US        (HEADSTART_US + 1400ULL)   /* originator key-up */

#define CMD_PAYLOAD_LEN    16U
#define FRAG_FULL_LEN      210U      /* 218 B on air: 171.6 ms, fits 12+ToA+15 */
#define ORIG_FRAME_LEN     100U

#define MARK_FRAG          0xA5U
#define MARK_CMD           0xC0U
#define MARK_ORIG          0x0FU
#define MARK_BASE_CMD      0xB0U

static uint64_t g_now_us = 1000000ULL;   /* monotonic across scenarios */
static int      g_in_advance;

static uint32_t g_rng = 1U;
static uint32_t rnd(void) {
    uint32_t x = g_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_rng = x;
    return x;
}
static uint32_t rnd_range(uint32_t lo, uint32_t hi) {
    return (hi <= lo) ? lo : (lo + (rnd() % (hi - lo + 1U)));
}

static uint32_t toa_us(uint8_t on_air_len) {
    return sx1276_airtime_compute_toa_us(7U, 250000UL, 5U, on_air_len,
                                         1U, 0U, 8U, 0U);
}

/* Channel table index of a centre frequency (0xFF: not a table channel). */
static uint8_t chan_of_hz(uint32_t hz) {
    for (uint8_t i = 0U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (sx1276_fhss_chantab_center_hz(i) == hz) {
            return i;
        }
    }
    return 0xFFU;
}

/* Channel of an absolute slot, from the pure scheduler helpers (both ends
 * run farm_id = node_id = 0, the activation default). */
static uint32_t hop_hz(uint32_t abs_slot) {
    static uint32_t cached_epoch = 0xFFFFFFFFU;
    static uint8_t  perm[SX1276_FHSS_CHANNEL_COUNT];
    const uint32_t epoch = sx1276_fhss_clock_epoch_of(abs_slot);
    if (epoch != cached_epoch) {
        sx1276_fhss_compute_permutation(
            sx1276_fhss_compute_seed(0ULL, 0ULL, epoch), perm);
        cached_epoch = epoch;
    }
    return sx1276_fhss_chantab_center_hz(perm[sx1276_fhss_clock_hop_of(abs_slot)]);
}

/* ------------------------------------------------------------------ */
/* Per-scenario bookkeeping                                            */
/* ------------------------------------------------------------------ */

typedef enum {
    CTX_NONE = 0, CTX_BOOT, CTX_HOST, CTX_TX_POLL, CTX_RX_SERVICE,
    CTX_MAILBOX, CTX_SCAN, CTX_FOLLOW, CTX_GAMMA1
} ctx_t;
static ctx_t g_ctx;

#define MAX_CMDS 1024U

typedef struct {
    const char *name;
    /* commands from the peer (tractor role) */
    uint32_t sent, recv, recv_dup;
    uint32_t miss_coll, miss_not_rx, miss_wrong_ch, miss_late_arm;
    uint32_t covered, covered_clean, covered_clean_recv;
    uint32_t dec[SX1276_RX_CONSIDER_REMOTE_COUNT_DIM];
    /* frames from the peer originator (base role) */
    uint32_t frames_sent, frames_recv;
    /* firmware-side counters (host_stats stubs) */
    uint32_t fw_tx_ok, first_anchor, dem_kept, dem_reset, scan_fail_urc;
    uint32_t adoptions;               /* s_grid_adopted 0 -> 1 edges     */
    /* ... by what last tuned the synth the adopted frame came in on */
    uint32_t adopt_via_follow, adopt_via_walker, adopt_via_tx;
    uint32_t fail_final;
    uint32_t crc_sent, crc_err;       /* commands sent corrupt / CRC IRQs */
    /* loop-structure checks */
    uint32_t walker_retunes, walker_suppressed, viol_walker;
    uint32_t viol_scan_park;          /* scan action left RX under authority */
    uint32_t follow_locked, follow_own, viol_follow_gate, viol_follow_chan;
    uint32_t viol_tx_chan;            /* own TX off its header slot's channel */
    uint32_t window_follow;           /* follow retune: LOCKED, !adopted, !auth */
    uint32_t auth_passes, passes, mirror_mismatch;
    uint32_t locks, demotions;
    uint64_t walker_after_decay_us;   /* first walker retune after a decay */
    uint32_t walker_after_decay_viol;
    /* RS-12.21 (2026-10-24) */
    uint32_t viol_fail_rule;          /* (b): FAILED held / left off the rule */
    uint32_t viol_failed_unauth;      /* (b): FAILED after a tick, no authority */
    uint32_t viol_walk_order;         /* d2: walker step not the next channel */
    uint32_t viol_early_demotion;     /* L: demoted < LOCK_LOSS after an anchor */
    uint32_t final_restarts;          /* (b): final FAIL -> SCANNING at once */
    uint32_t tick_restarts;           /* (b): FAILED -> SCANNING on a tick */
    uint64_t walker_seen;             /* channels the walker tuned (bit n) */
    uint32_t trace;                   /* FNV-1a over radio-affecting calls */
    /* first clean covered command missed (0 = none) */
    uint64_t first_miss_us;
    uint32_t first_miss_pkt_hz, first_miss_radio_hz;
    uint32_t first_miss_mode, first_miss_scan, first_miss_tuner;
} scen_t;
static scen_t g_sc;

static uint8_t  g_cmd_recv[MAX_CMDS];
static uint8_t  g_cmd_covered_clean[MAX_CMDS];

static void trace_mix(uint32_t op, uint32_t arg) {
    const uint32_t words[4] = {
        (uint32_t)g_ctx, op, arg, (uint32_t)(g_now_us & 0xFFFFFFFFULL)
    };
    uint32_t h = (g_sc.trace == 0U) ? 0x811C9DC5U : g_sc.trace;
    for (unsigned w = 0; w < 4U; ++w) {
        for (unsigned b = 0; b < 4U; ++b) {
            h ^= (words[w] >> (8U * b)) & 0xFFU;
            h *= 0x01000193U;
        }
    }
    g_sc.trace = h;
}

/* ------------------------------------------------------------------ */
/* Simulated SX1276                                                    */
/* ------------------------------------------------------------------ */

#define REG_FIFO              0x00U
#define REG_OP_MODE           0x01U
#define REG_PA_CONFIG         0x09U
#define REG_FIFO_ADDR_PTR     0x0DU
#define REG_FIFO_RX_CUR_ADDR  0x10U
#define REG_IRQ_FLAGS         0x12U
#define REG_RX_NB_BYTES       0x13U
#define REG_PKT_SNR           0x19U
#define REG_PKT_RSSI          0x1AU
#define REG_MODEM_CONFIG1     0x1DU
#define REG_MODEM_CONFIG2     0x1EU
#define REG_PREAMBLE_MSB      0x20U
#define REG_PREAMBLE_LSB      0x21U
#define REG_PAYLOAD_LENGTH    0x22U
#define REG_MODEM_CONFIG3     0x26U
#define IRQ_TX_DONE           0x08U
#define IRQ_VALID_HEADER      0x10U
#define IRQ_PAYLOAD_CRC_ERROR 0x20U
#define IRQ_RX_DONE           0x40U

typedef struct {
    sx1276_state_t mode;
    uint32_t freq_hz;
    uint64_t rx_since_us;          /* start of the current uninterrupted RX */
    uint8_t  regs[0x80];
    uint8_t  fifo[256];
    uint8_t  fifo_ptr;
    uint8_t  rx_wr;                /* RXCONT appends here (mod 256) */
    uint8_t  irq;
    uint8_t  dio0;
    uint64_t tx_start_us, tx_end_us;
    uint8_t  tx_bytes[256];
    uint8_t  tx_len;
} sim_radio_t;
static sim_radio_t g_radio;

/* Air: packets from the peer. One radio, so at most one in flight, but a
 * command can be scheduled while the previous one is still pending. */
#define AIR_MAX 8U
typedef enum { PKT_CMD = 1, PKT_FRAME = 2 } pkt_kind_t;
typedef struct {
    uint8_t  used, started, collided, auth_start;
    uint8_t  corrupt;              /* heard with a payload-CRC error */
    pkt_kind_t kind;
    uint64_t start_us, end_us;
    uint32_t freq_hz;
    uint16_t id;
    uint8_t  len;
    uint8_t  bytes[256];
} air_pkt_t;
static air_pkt_t g_air[AIR_MAX];

static void sim_advance_to(uint64_t t_us);

static uint8_t auth_now_ms(uint32_t now_ms) {
    return sx1276_fhss_authority_is_originator(now_ms,
                                               sx1276_fhss_clock_valid(),
                                               sx1276_rx_grid_adopted());
}

static void radio_reset(void) {
    memset(&g_radio, 0, sizeof(g_radio));
    g_radio.mode = SX1276_STATE_STANDBY;
    g_radio.freq_hz = 915000000UL;
    g_radio.regs[REG_MODEM_CONFIG1] = 0x82U;   /* BW250, CR4/5, explicit */
    g_radio.regs[REG_MODEM_CONFIG2] = 0x74U;   /* SF7, CRC on            */
    g_radio.regs[REG_MODEM_CONFIG3] = 0x04U;   /* AGC auto, no LDRO      */
    g_radio.regs[REG_PREAMBLE_MSB]  = 0x00U;
    g_radio.regs[REG_PREAMBLE_LSB]  = 0x08U;
    g_radio.regs[REG_PA_CONFIG]     = 0x8FU;
    memset(g_air, 0, sizeof(g_air));
}

/* --- HAL stubs: register file, FIFO, modes, synthesiser --- */

uint8_t sx1276_read_reg(uint8_t reg_addr) {
    sim_advance_to(g_now_us + COST_SPI_US);
    switch (reg_addr) {
        case REG_FIFO: {
            const uint8_t v = g_radio.fifo[g_radio.fifo_ptr];
            g_radio.fifo_ptr = (uint8_t)(g_radio.fifo_ptr + 1U);
            return v;
        }
        case REG_IRQ_FLAGS:
            return g_radio.irq;
        case REG_OP_MODE:
            return (uint8_t)(0x80U | ((g_radio.mode == SX1276_STATE_RX_CONT) ? 0x05U
                              : (g_radio.mode == SX1276_STATE_TX) ? 0x03U
                              : (g_radio.mode == SX1276_STATE_SLEEP) ? 0x00U : 0x01U));
        default:
            return (reg_addr < 0x80U) ? g_radio.regs[reg_addr] : 0U;
    }
}

void sx1276_write_reg(uint8_t reg_addr, uint8_t value) {
    sim_advance_to(g_now_us + COST_SPI_US);
    switch (reg_addr) {
        case REG_FIFO:
            g_radio.fifo[g_radio.fifo_ptr] = value;
            g_radio.fifo_ptr = (uint8_t)(g_radio.fifo_ptr + 1U);
            break;
        case REG_FIFO_ADDR_PTR:
            g_radio.fifo_ptr = value;
            break;
        case REG_IRQ_FLAGS:
            g_radio.irq = (uint8_t)(g_radio.irq & (uint8_t)~value);
            break;
        default:
            if (reg_addr < 0x80U) {
                g_radio.regs[reg_addr] = value;
            }
            break;
    }
}

void sx1276_read_burst(uint8_t start_reg, uint8_t *dst, size_t len) {
    sim_advance_to(g_now_us + SPI_BURST_US(len));
    for (size_t i = 0; i < len; ++i) {
        if (start_reg == REG_FIFO) {
            dst[i] = g_radio.fifo[g_radio.fifo_ptr];
            g_radio.fifo_ptr = (uint8_t)(g_radio.fifo_ptr + 1U);
        } else {
            dst[i] = 0U;
        }
    }
}

void sx1276_write_burst(uint8_t start_reg, const uint8_t *src, size_t len) {
    sim_advance_to(g_now_us + SPI_BURST_US(len));
    for (size_t i = 0; i < len; ++i) {
        if (start_reg == REG_FIFO) {
            g_radio.fifo[g_radio.fifo_ptr] = src[i];
            g_radio.fifo_ptr = (uint8_t)(g_radio.fifo_ptr + 1U);
        }
    }
}

/* Per-call accounting for the loop-structure checks. */
static uint32_t g_freq_calls, g_last_freq;
/* Who last tuned the synth, and who had tuned it for the last frame heard. */
static ctx_t g_tuner = CTX_NONE, g_heard_tuner = CTX_NONE;
/* RS-12.21 d2: the channel the scan walker must tune next (reset to 0 by a
 * BEGIN_SCAN out of BOOT or LOCKED, in the scan-counter hook below). */
static uint8_t g_walk_next;
/* RS-12.21 (2026-10-24): every firmware call into the modem, and every scan
 * record, monotonic and never reset, for check-radio-park's "nothing touched
 * the parked modem, the scan machine stood still". */
static struct {
    uint32_t freq, standby, rx_cont, tx, sleep, scan_rec;
} g_calls;

void sx1276_set_frequency_hz(uint32_t freq_hz) {
    sim_advance_to(g_now_us + COST_FREQ_US);
    ++g_calls.freq;
    g_radio.freq_hz = freq_hz;
    g_radio.rx_since_us = g_now_us;   /* a retune restarts any reception */
    ++g_freq_calls;
    g_tuner = g_ctx;
    g_last_freq = freq_hz;
    /* Every retune in the scan tick is a walker step; so is one in
     * rx_service (a frame event in BOOT dispatches BEGIN_SCAN). */
    if (g_ctx == CTX_SCAN || g_ctx == CTX_RX_SERVICE) {
        const uint8_t ch = chan_of_hz(freq_hz);
        if (ch != g_walk_next) {
            ++g_sc.viol_walk_order;
        }
        if (ch < SX1276_FHSS_CHANNEL_COUNT) {
            g_sc.walker_seen |= (1ULL << ch);
        }
        g_walk_next = (uint8_t)((ch + 1U) % SX1276_FHSS_CHANNEL_COUNT);
    }
    trace_mix(1U, freq_hz);
}

bool sx1276_modes_to_standby(void) {
    sim_advance_to(g_now_us + COST_MODE_US);
    ++g_calls.standby;
    g_radio.mode = SX1276_STATE_STANDBY;
    trace_mix(2U, 0U);
    return true;
}

/* RS-12.21 (2026-10-24): the firmware park (sx1276_rx_park) sleeps the
 * modem. Nothing else in the linked firmware calls it, so the existing
 * scenarios' traces are unchanged. */
bool sx1276_modes_to_sleep(void) {
    sim_advance_to(g_now_us + COST_MODE_US);
    ++g_calls.sleep;
    g_radio.mode = SX1276_STATE_SLEEP;
    trace_mix(6U, 0U);
    return true;
}

bool sx1276_modes_to_rx_cont(void) {
    sim_advance_to(g_now_us + COST_MODE_US);
    ++g_calls.rx_cont;
    g_radio.mode = SX1276_STATE_RX_CONT;
    g_radio.rx_since_us = g_now_us;
    trace_mix(3U, g_radio.freq_hz);
    return true;
}

bool sx1276_modes_to_tx(void) {
    sim_advance_to(g_now_us + COST_MODE_US);
    ++g_calls.tx;
    g_radio.mode    = SX1276_STATE_TX;
    g_radio.tx_len  = g_radio.regs[REG_PAYLOAD_LENGTH];
    memcpy(g_radio.tx_bytes, g_radio.fifo, g_radio.tx_len);   /* TX base 0 */
    g_radio.tx_start_us = g_now_us + TX_RAMP_US;
    g_radio.tx_end_us   = g_radio.tx_start_us + toa_us(g_radio.tx_len);
    /* The slot follower now snaps and consumes the shared scheduler on
     * every slot boundary in more states; the TX path must still key each
     * frame on the hop channel of the slot its header claims. */
    {
        lora_pkt_hdr_t h;
        if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) ==
                LORA_PKT_HDR_OK &&
            g_radio.freq_hz != hop_hz(sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx))) {
            ++g_sc.viol_tx_chan;
        }
    }
    /* Half duplex: every peer packet overlapping this TX is lost to us. */
    for (unsigned i = 0; i < AIR_MAX; ++i) {
        air_pkt_t *p = &g_air[i];
        if (p->used != 0U && p->start_us < g_radio.tx_end_us &&
            p->end_us > g_now_us) {
            p->collided = 1U;
        }
    }
    trace_mix(4U, g_radio.tx_len);
    return true;
}

sx1276_state_t sx1276_modes_get_state(void) {
    return g_radio.mode;
}

sx1276_lbt_result_t sx1276_lbt_check_and_backoff(void) {
    return SX1276_LBT_RESULT_DISABLED;
}

/* A host raw RegOpMode=RXCONT write (image_tx_daemon._ensure_rxcont), as
 * host_cmd.c handles it: register write + mode sync + FIFO-tracker reset. */
static void host_raw_rxcont(void) {
    sim_advance_to(g_now_us + COST_MODE_US);
    g_radio.mode = SX1276_STATE_RX_CONT;
    g_radio.rx_since_us = g_now_us;
    sx1276_rx_note_external_arm();
    trace_mix(5U, g_radio.freq_hz);
}

/* --- platform stubs --- */

uint32_t platform_now_us(void) {
    sim_advance_to(g_now_us + COST_CLOCK_US);
    return (uint32_t)(g_now_us & 0xFFFFFFFFULL);
}

uint32_t platform_now_ms(void) {
    return (uint32_t)((g_now_us / 1000ULL) & 0xFFFFFFFFULL);
}

/* --- host-side stubs (counters the scenarios read back) --- */

static uint8_t g_last_dec = 0xFFU;
static uint8_t g_last_fail_sub;
static uint8_t g_fail_seen;
static uint32_t g_pass_now_ms;     /* the current pass's now_ms */
#define FAIL_LOG 16U               /* RS-12.21 (b): a FAIL every 30 s unacquired */
static uint32_t g_fail_ms[FAIL_LOG];   /* pass now_ms of the first FAILs */
static uint8_t  g_fail_sub[FAIL_LOG];  /* ... and their sub-bytes */
static uint64_t g_final_fail_us;   /* when the LAST final FAIL fired (0 = none) */
static uint64_t g_last_crc_us;     /* last payload-CRC IRQ serviced */

void host_stats_radio_rx_ok(void) {}
void host_stats_radio_crc_err(void) { ++g_sc.crc_err; g_last_crc_us = g_now_us; }
void host_stats_radio_tx_ok(void) { ++g_sc.fw_tx_ok; }
void host_stats_radio_tx_abort_airtime(void) {}
void host_stats_rx_fifo_skip(void) {}
void host_stats_tx_fifo_rb_ok(void) {}
void host_stats_tx_fifo_rb_bad(void) {}
void host_stats_tx_done_early(void) {}
void host_stats_tx_deaf_note(uint32_t deaf_us, uint32_t done_to_rearm_us,
                             bool done_valid) {
    (void)deaf_us; (void)done_to_rearm_us; (void)done_valid;
}
void host_stats_fhss_dec_note(uint8_t dec_idx) { g_last_dec = dec_idx; }
void host_stats_clk_demotion_note(bool reset) {
    if (reset) { ++g_sc.dem_reset; } else { ++g_sc.dem_kept; }
}
void host_stats_tx_first_anchor(void) { ++g_sc.first_anchor; }
void host_stats_tx_stream_streak_note(uint32_t streak) { (void)streak; }

/* Every FAULT the firmware emits. check-radio-park links the real
 * host_cmd.c (RX_SIM_REAL_HOST), whose host_cmd_emit_fault() reaches this
 * through its FAULT_URC; the other harnesses through the stub below. */
static void sim_note_fault(uint8_t code, uint8_t sub) {
    if (code == HOST_FAULT_CODE_RX_SCAN_FAILED) {
        if (g_sc.scan_fail_urc < FAIL_LOG) {
            g_fail_ms[g_sc.scan_fail_urc] = g_pass_now_ms;
            g_fail_sub[g_sc.scan_fail_urc] = sub;
        }
        if ((sub & (1U << SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL)) != 0U) {
            g_final_fail_us = g_now_us;
        }
        ++g_sc.scan_fail_urc;
        g_last_fail_sub = sub;
        g_fail_seen = 1U;
    }
}

/* RX_SIM_REAL_HOST (check-radio-park, 2026-10-24): the harness links the
 * real host_cmd.c and host_cfg_profile.c, and provides host_uart_send_urc()
 * itself, so these three stubs drop out. g_profile stays: scenario_begin()
 * writes it, nothing reads it then (the real active profile is set through
 * the real CFG_SET REG_PROFILE path). */
static host_cfg_profile_req_t g_profile;
#ifndef RX_SIM_REAL_HOST
void host_cmd_emit_fault(uint8_t code, uint8_t sub) {
    sim_note_fault(code, sub);
}

void host_uart_send_urc(uint8_t type, uint16_t seq, uint8_t flags,
                        const uint8_t *payload, uint16_t payload_len) {
    (void)type; (void)seq; (void)flags; (void)payload; (void)payload_len;
}

const host_cfg_profile_req_t *host_cfg_profile_active(void) {
    return &g_profile;
}
#endif /* !RX_SIM_REAL_HOST */

/* --- scan-machine mirror: intercepts the counters TU's record hook --- */

static sx1276_rx_scan_state_t  g_scan_mirror = SX1276_RX_SCAN_STATE_BOOT;
static uint8_t                 g_tick_recorded;   /* records in this scan tick */
static sx1276_rx_scan_state_t  g_tick_state;      /* ... the first one's      */
static sx1276_rx_scan_action_t g_tick_action;
static uint8_t                 g_tick_left_failed; /* then (FAILED, BEGIN_SCAN) */
static uint8_t                 g_tick_extra_bad;   /* then anything else        */
static uint32_t                g_lock_anchor_ms;   /* LOCK / last accepted frame */

void sx1276_rx_scan_counter_record(sx1276_rx_scan_state_t state,
                                   sx1276_rx_scan_action_t action) {
    ++g_calls.scan_rec;
    if (state != g_scan_mirror) {
        ++g_sc.mirror_mismatch;
    }
    if (g_ctx == CTX_SCAN) {
        if (g_tick_recorded == 0U) {
            g_tick_state = state;
            g_tick_action = action;
        } else if (state == SX1276_RX_SCAN_STATE_FAILED &&
                   action == SX1276_RX_SCAN_ACTION_BEGIN_SCAN) {
            g_tick_left_failed = 1U;
        } else {
            g_tick_extra_bad = 1U;
        }
        ++g_tick_recorded;
    }
    switch (action) {
        case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
            if (state == SX1276_RX_SCAN_STATE_LOCKED) {
                ++g_sc.demotions;
                /* RS-12.21 L: the loss timer runs from the LOCK or the last
                 * accepted frame (the demotion is a TICK: the pass's ms). */
                if ((uint32_t)(g_pass_now_ms - g_lock_anchor_ms) <
                    SX1276_RX_SCAN_LOCK_LOSS_MS) {
                    ++g_sc.viol_early_demotion;
                }
            }
            /* RS-12.21 d2: a fresh acquisition (out of BOOT or LOCKED)
             * restarts the walk at channel 0; leaving FAILED continues it. */
            if (state != SX1276_RX_SCAN_STATE_FAILED) {
                g_walk_next = 0U;
            }
            g_scan_mirror = SX1276_RX_SCAN_STATE_SCANNING;
            break;
        case SX1276_RX_SCAN_ACTION_LOCK:
            ++g_sc.locks;
            g_scan_mirror = SX1276_RX_SCAN_STATE_LOCKED;
            /* A frame event: the firmware's now_ms is this same ms. */
            g_lock_anchor_ms = platform_now_ms();
            break;
        case SX1276_RX_SCAN_ACTION_REANCHOR:
            /* A frame event (now) or a tick: a frame read late in a pass
             * carries a fresher ms than the pass's now_ms, so the scan tick
             * after it sees time run backwards and re-anchors to the pass's
             * ms (up to 1 ms earlier). */
            g_lock_anchor_ms = (g_ctx == CTX_SCAN) ? g_pass_now_ms
                                                   : platform_now_ms();
            break;
        case SX1276_RX_SCAN_ACTION_FAIL:
            /* RS-12.21: the dispatcher records (FAILED, BEGIN_SCAN) when it
             * leaves FAILED again -- a non-final retry, or (b) on a node
             * without originator authority -- so the mirror follows the
             * records alone; fw_pass checks the decision against (b). */
            g_scan_mirror = SX1276_RX_SCAN_STATE_FAILED;
            break;
        default:
            break;
    }
}

/* ------------------------------------------------------------------ */
/* Peer models                                                         */
/* ------------------------------------------------------------------ */

typedef enum { ROLE_TRACTOR = 1, ROLE_BASE = 2 } role_t;
static role_t g_role;

/* Base (follower) model, tractor role. */
typedef enum { CMD_PUMP_AT_TRAIN_END = 1, CMD_IN_GAP = 2 } cmd_mode_t;
typedef struct {
    cmd_mode_t mode;
    uint32_t min_gap_ms, extra_gap_ms;      /* spacing between commands */
    uint32_t delay_min_ms, delay_max_ms;    /* CMD_IN_GAP decision delay */
    uint8_t  idle_drain;                    /* drain during >= 1.5 s quiet */
    uint8_t  have_grid;
    uint32_t ms_phase_us;                   /* base ms tick phase at t0     */
    uint64_t t0_us;
    uint64_t anchor_ms;                     /* base-ms value of the anchor */
    uint64_t anchor_us;                     /* the same instant, sim time  */
    uint32_t anchor_abs;
    uint64_t busy_until_us;
    uint64_t next_allowed_us;
    uint64_t decide_at_us;                  /* 0 = none pending */
    uint64_t last_frag_us;
    uint64_t next_drain_us;
    uint16_t next_id;
    uint8_t  corrupt;                       /* commands arrive with bad CRC */
} base_model_t;
static base_model_t g_base;

/* Originator model, base role. */
typedef struct {
    uint8_t  on;
    uint64_t next_slot_us;
    uint32_t next_abs;
    uint64_t silent_from_us, silent_to_us;
    uint16_t seq;
} orig_model_t;
static orig_model_t g_orig;

static air_pkt_t *air_alloc(void) {
    for (unsigned i = 0; i < AIR_MAX; ++i) {
        if (g_air[i].used == 0U) {
            memset(&g_air[i], 0, sizeof(g_air[i]));
            g_air[i].used = 1U;
            return &g_air[i];
        }
    }
    return NULL;
}

static air_pkt_t *air_schedule(pkt_kind_t kind, uint16_t id, uint64_t start_us,
                               uint32_t abs_slot, uint8_t slot_offset_ms,
                               const uint8_t *payload, uint8_t payload_len) {
    air_pkt_t *p = air_alloc();
    lora_pkt_hdr_t h;
    if (p == NULL) {
        CHECK(0, "air table full");
        return NULL;
    }
    h.profile_id     = (uint8_t)REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    h.hop_idx        = sx1276_fhss_clock_hop_of(abs_slot);
    h.epoch          = sx1276_fhss_clock_epoch_of(abs_slot);
    h.slot_offset_ms = slot_offset_ms;
    (void)lora_pkt_hdr_pack(&h, p->bytes);
    memcpy(&p->bytes[LORA_PKT_HDR_LEN], payload, payload_len);
    p->kind     = kind;
    p->id       = id;
    p->len      = (uint8_t)(LORA_PKT_HDR_LEN + payload_len);
    p->start_us = start_us;
    p->end_us   = start_us + toa_us(p->len);
    p->freq_hz  = hop_hz(abs_slot);
    /* Already overlapping our own TX in flight? */
    if (g_radio.mode == SX1276_STATE_TX &&
        p->start_us < g_radio.tx_end_us && p->end_us > g_radio.tx_start_us) {
        p->collided = 1U;
    }
    return p;
}

/* Offset of the base's ms clock against sim time at t (drifting). */
static uint64_t base_off_us(uint64_t t_us) {
    return (uint64_t)g_base.ms_phase_us
         + ((t_us - g_base.t0_us) * BASE_DRIFT_PPM) / 1000000ULL;
}

static void base_model_init(void) {
    g_base.ms_phase_us = rnd_range(0U, 999U);
    g_base.t0_us = g_now_us;
}

/* Key-up of a command the base's L072 admitted at t (inside the slot rule)
 * in `slot` of its copy of the tractor's grid. */
static void base_key_cmd(uint64_t t, uint64_t slot) {
    uint8_t payload[CMD_PAYLOAD_LEN];
    const uint32_t toa = toa_us((uint8_t)(LORA_PKT_HDR_LEN + CMD_PAYLOAD_LEN));
    /* Phase byte: base ms clock at the stamp minus the slot's boundary in
     * base ms (sx1276_tx.c F7: stamped after the PLL settle). */
    const uint64_t stamp = t + BASE_STAMP_US;
    const uint64_t key = stamp + BASE_KEY_US;
    const uint64_t off_ms = (stamp - base_off_us(stamp)) / 1000ULL
        - (g_base.anchor_ms + slot * (uint64_t)SX1276_FHSS_SLOT_MS);
    const uint16_t id = g_base.next_id++;
    memset(payload, 0, sizeof(payload));
    payload[0] = MARK_CMD;
    payload[1] = (uint8_t)(id & 0xFFU);
    payload[2] = (uint8_t)(id >> 8);
    air_pkt_t *p = air_schedule(PKT_CMD, id, key, g_base.anchor_abs + (uint32_t)slot,
                                (uint8_t)off_ms, payload, CMD_PAYLOAD_LEN);
    g_base.busy_until_us = key + toa + 2000ULL;
    if (g_base.corrupt != 0U) {
        if (p != NULL) { p->corrupt = 1U; }
        ++g_sc.crc_sent;
    } else {
        ++g_sc.sent;
    }
}

/* The base's L072 admits a command at t: its own slot rule on its copy of
 * the tractor's grid (sx1276_tx_slot_wait_us + the mailbox), then key-up. */
static void base_send_cmd(uint64_t t_us) {
    if (g_base.have_grid == 0U || g_base.next_id >= MAX_CMDS) {
        return;
    }
    uint64_t t = ((t_us > g_base.busy_until_us) ? t_us : g_base.busy_until_us)
                 + BASE_ADMIT_US;
    const uint32_t toa = toa_us((uint8_t)(LORA_PKT_HDR_LEN + CMD_PAYLOAD_LEN));
    uint64_t el      = t - g_base.anchor_us;
    uint64_t slot    = el / SLOT_US;
    uint64_t in_slot = el % SLOT_US;
    if (in_slot < HEADSTART_US) {
        t += HEADSTART_US - in_slot;
    } else if (in_slot + toa + SX1276_FHSS_SLOT_TX_GUARD_US > SLOT_US) {
        t += (SLOT_US - in_slot) + HEADSTART_US;
        ++slot;
    }
    base_key_cmd(t, slot);
}

/* A command whose first preamble symbol is on air at key_us, if the base's
 * slot rule admits one there. Returns its id, or -1. */
static int base_cmd_at_key(uint64_t key_us) {
    const uint32_t toa = toa_us((uint8_t)(LORA_PKT_HDR_LEN + CMD_PAYLOAD_LEN));
    const uint64_t t = key_us - BASE_KEY_US - BASE_STAMP_US;
    if (g_base.have_grid == 0U || g_base.next_id >= MAX_CMDS ||
        t < g_base.anchor_us) {
        return -1;
    }
    const uint64_t el      = t - g_base.anchor_us;
    const uint64_t in_slot = el % SLOT_US;
    if (in_slot < HEADSTART_US ||
        in_slot + toa + SX1276_FHSS_SLOT_TX_GUARD_US > SLOT_US) {
        return -1;
    }
    const int id = (int)g_base.next_id;
    base_key_cmd(t, el / SLOT_US);
    return id;
}

/* The tractor's TX left the air: the ideal follower decodes it. */
static void base_on_tractor_frame(void) {
    lora_pkt_hdr_t h;
    if (lora_pkt_hdr_unpack(g_radio.tx_bytes, g_radio.tx_len, &h) !=
        LORA_PKT_HDR_OK) {
        return;
    }
    /* sx1276_fhss_clock_anchor_rx() on the base's ms clock: service-time
     * ms - half-up ToA ms - slot_offset. */
    {
        const uint64_t srv = g_radio.tx_end_us + BASE_SRV_LAT_US
                           + SPI_BURST_US(g_radio.tx_len)
                           + (uint64_t)rnd_range(0U, 400U);
        const uint64_t rx_ms = (srv - base_off_us(srv)) / 1000ULL;
        const uint64_t toa_ms = ((uint64_t)toa_us(g_radio.tx_len) + 500ULL) / 1000ULL;
        g_base.anchor_ms = rx_ms - toa_ms - (uint64_t)h.slot_offset_ms;
        g_base.anchor_us = g_base.anchor_ms * 1000ULL + base_off_us(srv);
    }
    g_base.anchor_abs = sx1276_fhss_clock_abs_of(h.epoch, h.hop_idx);
    g_base.have_grid  = 1U;
    g_base.last_frag_us = g_radio.tx_end_us;
    g_base.next_drain_us = g_radio.tx_end_us + 1500000ULL;
    /* Aligned pump / gap command: fires when the train's LAST fragment
     * has been received (the marker byte after the A6a header). */
    if (g_radio.tx_len > LORA_PKT_HDR_LEN + 1U &&
        g_radio.tx_bytes[LORA_PKT_HDR_LEN] == MARK_FRAG &&
        g_radio.tx_bytes[LORA_PKT_HDR_LEN + 1U] == 1U &&
        g_base.decide_at_us == 0U &&
        g_radio.tx_end_us >= g_base.next_allowed_us) {
        const uint64_t delay = (g_base.mode == CMD_PUMP_AT_TRAIN_END)
            ? BASE_HOST_LAT_US
            : (uint64_t)rnd_range(g_base.delay_min_ms, g_base.delay_max_ms) * 1000ULL;
        g_base.decide_at_us = g_radio.tx_end_us + delay;
    }
}

static void base_decide(void) {
    base_send_cmd(g_base.decide_at_us);
    g_base.next_allowed_us = g_base.decide_at_us
        + (uint64_t)g_base.min_gap_ms * 1000ULL
        + (uint64_t)rnd_range(0U, g_base.extra_gap_ms) * 1000ULL;
    g_base.decide_at_us = 0U;
}

static void base_idle_drain(void) {
    /* RS-12.11 idle drain: >= 1.5 s with no fragment, one command per
     * ~0.25 s through the shared gate (0.12 s idle gap). */
    base_send_cmd(g_base.next_drain_us);
    g_base.next_drain_us += 250000ULL + (uint64_t)rnd_range(0U, 150U) * 1000ULL;
}

static void orig_slot(void) {
    const uint64_t b = g_orig.next_slot_us;
    if (!(b >= g_orig.silent_from_us && b < g_orig.silent_to_us)) {
        uint8_t payload[ORIG_FRAME_LEN];
        memset(payload, 0x5A, sizeof(payload));
        payload[0] = MARK_ORIG;
        payload[1] = (uint8_t)(g_orig.seq & 0xFFU);
        payload[2] = (uint8_t)(g_orig.seq >> 8);
        air_schedule(PKT_FRAME, g_orig.seq, b + ORIG_KEY_US, g_orig.next_abs,
                     (uint8_t)(ORIG_KEY_US / 1000ULL), payload, ORIG_FRAME_LEN);
        ++g_orig.seq;
        ++g_sc.frames_sent;
    }
    g_orig.next_slot_us += SLOT_US;
    ++g_orig.next_abs;
}

static void air_start(air_pkt_t *p) {
    p->started = 1U;
    p->auth_start = auth_now_ms((uint32_t)(g_now_us / 1000ULL));
}

static void air_end(air_pkt_t *p) {
    const uint8_t auth_end = auth_now_ms((uint32_t)(g_now_us / 1000ULL));
    const uint8_t covered = (uint8_t)(p->auth_start != 0U && auth_end != 0U);
    const uint8_t is_cmd = (uint8_t)(p->kind == PKT_CMD && p->corrupt == 0U);
    uint8_t heard = 0U;
    if (is_cmd != 0U) {
        if (covered != 0U) {
            ++g_sc.covered;
            if (p->collided == 0U) {
                ++g_sc.covered_clean;
                g_cmd_covered_clean[p->id] = 1U;
            }
        }
    }
    if (p->collided != 0U) {
        if (is_cmd != 0U) { ++g_sc.miss_coll; }
    } else if (g_radio.mode != SX1276_STATE_RX_CONT) {
        if (is_cmd != 0U) { ++g_sc.miss_not_rx; }
    } else if (g_radio.freq_hz != p->freq_hz) {
        if (is_cmd != 0U) { ++g_sc.miss_wrong_ch; }
    } else if (g_radio.rx_since_us > p->start_us + LATE_ARM_TOL_US) {
        if (is_cmd != 0U) { ++g_sc.miss_late_arm; }
    } else {
        heard = 1U;
        g_heard_tuner = g_tuner;
    }
    if (heard == 0U && is_cmd != 0U && covered != 0U && p->collided == 0U &&
        g_sc.first_miss_us == 0U) {
        /* First clean covered miss: where the receiver was, and who put it
         * there (printed with the reverse-delivery CHECK). */
        g_sc.first_miss_us = p->start_us;
        g_sc.first_miss_pkt_hz = p->freq_hz;
        g_sc.first_miss_radio_hz = g_radio.freq_hz;
        g_sc.first_miss_mode = (uint32_t)g_radio.mode;
        g_sc.first_miss_scan = (uint32_t)g_scan_mirror;
        g_sc.first_miss_tuner = (uint32_t)g_tuner;
    }
    if (heard != 0U) {
        /* RXCONT appends to the data buffer; RxDone + ValidHeader on DIO0. */
        const uint8_t start = g_radio.rx_wr;
        for (uint8_t i = 0U; i < p->len; ++i) {
            g_radio.fifo[(uint8_t)(start + i)] = p->bytes[i];
        }
        g_radio.rx_wr = (uint8_t)(start + p->len);
        g_radio.regs[REG_FIFO_RX_CUR_ADDR] = start;
        g_radio.regs[REG_RX_NB_BYTES] = p->len;
        g_radio.regs[REG_PKT_SNR] = 40U;    /* +10 dB */
        g_radio.regs[REG_PKT_RSSI] = 95U;   /* -62 dBm */
        g_radio.irq = (uint8_t)(g_radio.irq | IRQ_RX_DONE | IRQ_VALID_HEADER |
                                ((p->corrupt != 0U) ? IRQ_PAYLOAD_CRC_ERROR : 0U));
        g_radio.dio0 = 1U;
    }
    p->used = 0U;
}

/* Advance simulated time, firing radio and peer events in time order. */
static void sim_advance_to(uint64_t t_us) {
    if (g_in_advance != 0) {
        if (t_us > g_now_us) { g_now_us = t_us; }
        return;
    }
    g_in_advance = 1;
    for (;;) {
        uint64_t next = t_us;
        int what = 0;
        air_pkt_t *pk = NULL;
        if (g_radio.mode == SX1276_STATE_TX && g_radio.tx_end_us <= next) {
            next = g_radio.tx_end_us; what = 1;
        }
        for (unsigned i = 0; i < AIR_MAX; ++i) {
            air_pkt_t *p = &g_air[i];
            if (p->used == 0U) { continue; }
            if (p->started == 0U && p->start_us <= next && (what == 0 || p->start_us < next)) {
                next = p->start_us; what = 2; pk = p;
            }
            if (p->end_us <= next && (what == 0 || p->end_us < next)) {
                next = p->end_us; what = 3; pk = p;
            }
        }
        if (g_role == ROLE_TRACTOR) {
            if (g_base.decide_at_us != 0U && g_base.decide_at_us <= next &&
                (what == 0 || g_base.decide_at_us < next)) {
                next = g_base.decide_at_us; what = 4;
            }
            if (g_base.idle_drain != 0U && g_base.have_grid != 0U &&
                g_base.next_drain_us <= next && (what == 0 || g_base.next_drain_us < next)) {
                next = g_base.next_drain_us; what = 5;
            }
        } else if (g_role == ROLE_BASE && g_orig.on != 0U) {
            if (g_orig.next_slot_us <= next && (what == 0 || g_orig.next_slot_us < next)) {
                next = g_orig.next_slot_us; what = 6;
            }
        }
        if (what == 0) { break; }
        if (next > g_now_us) { g_now_us = next; }
        switch (what) {
            case 1:
                g_radio.irq = (uint8_t)(g_radio.irq | IRQ_TX_DONE);
                g_radio.dio0 = 1U;
                g_radio.mode = SX1276_STATE_STANDBY;   /* chip auto-returns */
                if (g_role == ROLE_TRACTOR) { base_on_tractor_frame(); }
                break;
            case 2: air_start(pk); break;
            case 3: air_end(pk); break;
            case 4: base_decide(); break;
            case 5: base_idle_drain(); break;
            case 6: orig_slot(); break;
            default: break;
        }
    }
    if (t_us > g_now_us) { g_now_us = t_us; }
    g_in_advance = 0;
}

/* ------------------------------------------------------------------ */
/* Firmware host side: TX mailbox (host_cmd.c) and the X8 daemons      */
/* ------------------------------------------------------------------ */

#define TXQ_DEPTH 4U   /* config.h HOST_TXQ_DEPTH */
static sx1276_tx_request_t g_txq[TXQ_DEPTH];
static uint8_t g_txq_head, g_txq_count;
static uint32_t g_submit_fail;

/* handle_tx_frame(): park when anything is parked, the radio is busy, or
 * the frame does not fit the remaining slot. */
static bool fw_submit(const sx1276_tx_request_t *req) {
    if (g_txq_count > 0U || sx1276_tx_busy() ||
        sx1276_tx_slot_wait_us(req->length) > 0U) {
        if (g_txq_count >= TXQ_DEPTH) {
            ++g_submit_fail;
            return false;
        }
        g_txq[(uint8_t)((g_txq_head + g_txq_count) % TXQ_DEPTH)] = *req;
        ++g_txq_count;
        return true;
    }
    if (!sx1276_tx_begin(req)) {
        ++g_submit_fail;
        return false;
    }
    return true;
}

/* host_cmd_service_tx_mailbox(). */
static void fw_service_mailbox(void) {
    while (g_txq_count > 0U) {
        if (sx1276_tx_busy()) { return; }
        if (sx1276_tx_slot_wait_us(g_txq[g_txq_head].length) > 0U) { return; }
        const sx1276_tx_request_t next = g_txq[g_txq_head];
        g_txq_head = (uint8_t)((g_txq_head + 1U) % TXQ_DEPTH);
        --g_txq_count;
        if (!sx1276_tx_begin(&next)) {
            ++g_submit_fail;
            continue;
        }
        return;
    }
}

/* image_tx_daemon (tractor role). */
typedef struct {
    uint32_t period_ms, jitter_ms;
    uint8_t  train_w[4];
    uint64_t next_frame_us;
    uint32_t frames_waiting;
    uint8_t  train_active, frags_left, outstanding;
    uint16_t train_seq;
    uint64_t next_submit_us, last_train_done_us, next_rxcont_us;
    uint64_t stall_from_us, stall_to_us;    /* camera produces nothing */
    uint32_t stall_every_ms, stall_len_ms;  /* periodic stall injection */
} tractor_host_t;
static tractor_host_t g_th;

static uint8_t draw_train_len(void) {
    const uint32_t total = (uint32_t)g_th.train_w[0] + g_th.train_w[1]
                         + g_th.train_w[2] + g_th.train_w[3];
    uint32_t r = rnd() % total;
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (r < g_th.train_w[i]) { return (uint8_t)(i + 1U); }
        r -= g_th.train_w[i];
    }
    return 1U;
}

static void tractor_host_tick(void) {
    while (g_now_us >= g_th.next_frame_us) {
        const uint64_t f = g_th.next_frame_us;
        if (g_th.stall_every_ms != 0U && f >= g_th.stall_to_us &&
            f >= g_th.stall_from_us + (uint64_t)g_th.stall_every_ms * 1000ULL) {
            g_th.stall_from_us = f;
            g_th.stall_to_us   = f + (uint64_t)g_th.stall_len_ms * 1000ULL;
        }
        if (!(f >= g_th.stall_from_us && f < g_th.stall_to_us) &&
            g_th.frames_waiting < 2U) {
            ++g_th.frames_waiting;
        }
        g_th.next_frame_us += (uint64_t)rnd_range(g_th.period_ms - g_th.jitter_ms,
                                                  g_th.period_ms + g_th.jitter_ms) * 1000ULL;
    }
    if (g_th.train_active == 0U && g_th.frames_waiting > 0U &&
        g_now_us >= g_th.last_train_done_us + TRAIN_GAP_US) {
        --g_th.frames_waiting;
        g_th.train_active = 1U;
        g_th.frags_left = draw_train_len();
        g_th.next_submit_us = g_now_us;
    }
    while (g_th.train_active != 0U && g_th.frags_left > 0U &&
           g_th.outstanding < 2U && g_now_us >= g_th.next_submit_us) {
        sx1276_tx_request_t req;
        memset(&req, 0, sizeof(req));
        const uint8_t last = (uint8_t)(g_th.frags_left == 1U);
        req.tx_id  = (uint8_t)(g_th.train_seq & 0xFFU);
        req.length = (last != 0U) ? (uint8_t)rnd_range(40U, FRAG_FULL_LEN)
                                  : (uint8_t)FRAG_FULL_LEN;
        req.payload[0] = MARK_FRAG;
        req.payload[1] = last;
        req.payload[2] = (uint8_t)(g_th.train_seq & 0xFFU);
        if (fw_submit(&req)) {
            ++g_th.outstanding;
        }
        --g_th.frags_left;
        g_th.next_submit_us = g_now_us + 300ULL;
        if (g_th.frags_left == 0U && g_th.outstanding == 0U) {
            g_th.train_active = 0U;       /* every submit refused */
            g_th.last_train_done_us = g_now_us;
        }
    }
    if (g_th.train_active == 0U && g_th.outstanding == 0U &&
        g_now_us >= g_th.next_rxcont_us) {
        if (g_radio.mode != SX1276_STATE_RX_CONT && !sx1276_tx_busy()) {
            host_raw_rxcont();
        }
        g_th.next_rxcont_us = g_now_us + RXCONT_POLL_US;
    }
}

static void tractor_host_on_tx_done(void) {
    if (g_th.outstanding > 0U) { --g_th.outstanding; }
    if (g_th.next_submit_us < g_now_us + TRACTOR_HOST_LAT_US) {
        g_th.next_submit_us = g_now_us + TRACTOR_HOST_LAT_US;
    }
    if (g_th.train_active != 0U && g_th.frags_left == 0U && g_th.outstanding == 0U) {
        g_th.train_active = 0U;
        g_th.last_train_done_us = g_now_us;
        g_th.next_rxcont_us = g_now_us + 5000ULL;
        ++g_th.train_seq;
    }
}

/* base daemon (base role): commands on a schedule, plus an optional burst. */
typedef struct {
    uint64_t next_cmd_us;
    uint32_t gap_min_ms, gap_max_ms;
    uint64_t cmds_from_us, cmds_to_us;
    uint8_t  burst_left;
    uint32_t burst_gap_ms;
    uint64_t burst_at_us;
    uint16_t seq;
} base_host_t;
static base_host_t g_bh;

static void base_host_submit(void) {
    sx1276_tx_request_t req;
    memset(&req, 0, sizeof(req));
    req.tx_id = 0xFEU;
    req.length = CMD_PAYLOAD_LEN;
    req.payload[0] = MARK_BASE_CMD;
    req.payload[1] = (uint8_t)(g_bh.seq & 0xFFU);
    ++g_bh.seq;
    (void)fw_submit(&req);
}

static void base_host_tick(void) {
    if (g_bh.gap_max_ms != 0U && g_now_us >= g_bh.next_cmd_us) {
        if (g_now_us >= g_bh.cmds_from_us && g_now_us < g_bh.cmds_to_us) {
            base_host_submit();
        }
        g_bh.next_cmd_us = g_now_us
            + (uint64_t)rnd_range(g_bh.gap_min_ms, g_bh.gap_max_ms) * 1000ULL;
    }
    if (g_bh.burst_left > 0U && g_now_us >= g_bh.burst_at_us) {
        base_host_submit();
        --g_bh.burst_left;
        g_bh.burst_at_us = g_now_us + (uint64_t)g_bh.burst_gap_ms * 1000ULL;
    }
}

/* ------------------------------------------------------------------ */
/* Firmware main loop (main.c order) + per-pass structural checks      */
/* ------------------------------------------------------------------ */

static sx1276_rx_frame_t g_frame;
static uint8_t  g_prev_adopted;
static uint8_t  g_prev_auth;
static uint64_t g_decay_at_us;     /* authority fell 1 -> 0 here (0 = none) */

static void on_rx_frame(const sx1276_rx_frame_t *f) {
    if (f->length >= 3U && f->payload[0] == MARK_CMD) {
        const uint16_t id = (uint16_t)(f->payload[1] | ((uint16_t)f->payload[2] << 8));
        if (id < MAX_CMDS) {
            if (g_cmd_recv[id] != 0U) {
                ++g_sc.recv_dup;
            } else {
                g_cmd_recv[id] = 1U;
                ++g_sc.recv;
                if (g_cmd_covered_clean[id] != 0U) { ++g_sc.covered_clean_recv; }
                if (g_last_dec < SX1276_RX_CONSIDER_REMOTE_COUNT_DIM) {
                    ++g_sc.dec[g_last_dec];
                }
            }
        }
    } else if (f->length >= 1U && f->payload[0] == MARK_ORIG) {
        ++g_sc.frames_recv;
    }
}

static void fw_pass(void) {
    const uint32_t now_ms = platform_now_ms();
    sx1276_tx_result_t res;
    uint32_t events;
    bool done;

    g_pass_now_ms = now_ms;
    g_ctx = CTX_HOST;
    if (g_role == ROLE_TRACTOR) { tractor_host_tick(); } else { base_host_tick(); }

    events = (g_radio.dio0 != 0U) ? SX1276_EVT_DIO0 : 0U;
    g_radio.dio0 = 0U;

    g_ctx = CTX_TX_POLL;
    done = sx1276_tx_poll(events, &res);
    if (done && g_role == ROLE_TRACTOR) { tractor_host_on_tx_done(); }

    g_ctx = CTX_RX_SERVICE;
    if (((events & SX1276_EVT_DIO0) != 0U) && !sx1276_tx_busy()) {
        g_last_dec = 0xFFU;
        if (sx1276_rx_service(events, &g_frame)) {
            on_rx_frame(&g_frame);
        }
    }

    g_ctx = CTX_MAILBOX;
    fw_service_mailbox();
#ifdef RX_SIM_REAL_HOST
    host_cmd_service_tx_mailbox();   /* the real host_cmd.c TX ring */
#endif

    /* Scan tick: the walker retunes on every BEGIN_SCAN / ADVANCE (and a
     * FAIL that re-enters SCANNING) unless the node follows its own clock,
     * and no scan action takes the receiver out of RX while it does (the
     * slot follower re-arms only at the next boundary). */
    {
        /* Authority as the firmware evaluates it: at a fresh time (the
         * pass's now_ms can be a tick older than the last TX_DONE note). */
        const uint8_t auth = auth_now_ms(platform_now_ms());
        const sx1276_state_t mode_before = g_radio.mode;
        const uint8_t busy_before = (uint8_t)sx1276_tx_busy();
        const uint8_t parked_before = (uint8_t)sx1276_rx_parked();
        g_tick_recorded = 0U;
        g_tick_left_failed = 0U;
        g_tick_extra_bad = 0U;
        g_fail_seen = 0U;
        g_freq_calls = 0U;
        g_ctx = CTX_SCAN;
        sx1276_rx_scan_tick(now_ms);
        if (auth != 0U && mode_before == SX1276_STATE_RX_CONT &&
            g_radio.mode != SX1276_STATE_RX_CONT) {
            ++g_sc.viol_scan_park;
        }
        if (g_tick_recorded != 0U) {
            uint32_t expect = 0U;
            switch (g_tick_action) {
                case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
                    if (g_tick_state == SX1276_RX_SCAN_STATE_FAILED) {
                        /* RS-12.21 (b): FAILED -> SCANNING on a tick, only
                         * once the node has lost originator authority. */
                        ++g_sc.tick_restarts;
                        if (auth != 0U) { ++g_sc.viol_fail_rule; }
                    }
                    /* fallthrough */
                case SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL:
                    expect = (auth != 0U) ? 0U : 1U;
                    break;
                case SX1276_RX_SCAN_ACTION_FAIL: {
                    /* RS-12.21 (b): FAILED is held exactly when the FAIL is
                     * final and the node holds originator authority; any
                     * other FAIL re-enters SCANNING at once, recorded as
                     * (FAILED, BEGIN_SCAN), and its walker step retunes
                     * unless the node holds authority. (Before (b) a final
                     * FAIL always held, and parked a node without
                     * authority in STANDBY.) */
                    const uint8_t final_fail = (uint8_t)(g_fail_seen != 0U &&
                        (g_last_fail_sub & (1U << SX1276_RX_SCAN_FAIL_SUB_BIT_FINAL)) != 0U);
                    const uint8_t hold = (uint8_t)(final_fail != 0U && auth != 0U);
                    if (g_tick_left_failed == hold) {
                        ++g_sc.viol_fail_rule;
                    }
                    expect = (auth != 0U) ? 0U : 1U;
                    if (final_fail != 0U) {
                        ++g_sc.fail_final;
                        if (g_tick_left_failed != 0U) { ++g_sc.final_restarts; }
                    }
                    break;
                }
                default:
                    expect = 0U;
                    break;
            }
            if (g_tick_extra_bad != 0U ||
                (g_tick_left_failed != 0U &&
                 g_tick_action != SX1276_RX_SCAN_ACTION_FAIL)) {
                ++g_sc.viol_fail_rule;
            }
            if (g_freq_calls != expect) {
                ++g_sc.viol_walker;
            }
            if ((g_tick_action == SX1276_RX_SCAN_ACTION_BEGIN_SCAN ||
                 g_tick_action == SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL)) {
                if (g_freq_calls != 0U) {
                    ++g_sc.walker_retunes;
                    if (g_decay_at_us != 0U) {
                        if (g_sc.walker_after_decay_us == 0U) {
                            g_sc.walker_after_decay_us = g_now_us - g_decay_at_us;
                        }
                        /* One dwell (+ the pass that notices it). */
                        if (g_now_us - g_decay_at_us >
                            (uint64_t)SX1276_RX_SCAN_DWELL_MS * 1000ULL + 5000ULL) {
                            ++g_sc.walker_after_decay_viol;
                        }
                        g_decay_at_us = 0U;
                    }
                } else {
                    ++g_sc.walker_suppressed;
                }
            }
        } else if (g_freq_calls != 0U) {
            ++g_sc.viol_walker;
        }
        /* RS-12.21 (b): FAILED survives a scan tick only under originator
         * authority, or while a TX is keyed or the host has the radio
         * parked (the tick is frozen then). */
        if (g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED && auth == 0U &&
            busy_before == 0U && parked_before == 0U) {
            ++g_sc.viol_failed_unauth;
        }
    }

    /* Slot follow: every retune must be allowed by the follow rule and
     * land on the clock's channel. */
    {
        const uint8_t adopted = sx1276_rx_grid_adopted();
        const uint8_t auth = auth_now_ms(platform_now_ms());
        const uint8_t locked = (uint8_t)(g_scan_mirror == SX1276_RX_SCAN_STATE_LOCKED);
        g_freq_calls = 0U;
        g_ctx = CTX_FOLLOW;
        sx1276_rx_slot_follow(now_ms);
        if (g_freq_calls != 0U) {
            if (!((locked != 0U && adopted != 0U) || auth != 0U)) {
                ++g_sc.viol_follow_gate;
                if (locked != 0U && adopted == 0U) {
                    ++g_sc.window_follow;
                }
            }
            if (auth != 0U && !(locked != 0U && adopted != 0U)) {
                ++g_sc.follow_own;
            } else {
                ++g_sc.follow_locked;
            }
            if (sx1276_fhss_clock_valid() == 0U ||
                g_last_freq != hop_hz(sx1276_fhss_clock_abs_slot(now_ms))) {
                ++g_sc.viol_follow_chan;
            }
        }
        if (auth != 0U) {
            ++g_sc.auth_passes;
        } else if (g_prev_auth != 0U) {
            /* The one-dwell bound is for a SCANNING walker -- and, since
             * RS-12.21 (b), a FAILED one, which scans again on its next
             * tick; LOCKED waits out LOCK_LOSS. */
            g_decay_at_us = (g_scan_mirror == SX1276_RX_SCAN_STATE_SCANNING ||
                             g_scan_mirror == SX1276_RX_SCAN_STATE_FAILED)
                                ? g_now_us : 0U;
            g_sc.walker_after_decay_us = 0U;
        }
        if (locked != 0U) {
            g_decay_at_us = 0U;   /* re-acquired: no walk to time */
        }
        g_prev_auth = auth;
        if (adopted != 0U && g_prev_adopted == 0U) {
            ++g_sc.adoptions;
            if (g_heard_tuner == CTX_FOLLOW) {
                ++g_sc.adopt_via_follow;
            } else if (g_heard_tuner == CTX_SCAN) {
                ++g_sc.adopt_via_walker;
            } else {
                ++g_sc.adopt_via_tx;   /* TX retune (+ re-arm / raw RXCONT) */
            }
        }
        g_prev_adopted = adopted;
    }

    g_ctx = CTX_GAMMA1;
    sx1276_rx_tick(now_ms);
    g_ctx = CTX_NONE;
    ++g_sc.passes;

    /* WFI: next SysTick, or the next radio IRQ if earlier. */
    {
        uint64_t wake = ((g_now_us / 1000ULL) + 1ULL) * 1000ULL;
        if (g_radio.dio0 != 0U) {
            wake = g_now_us + WAKE_US;
        } else {
            if (g_radio.mode == SX1276_STATE_TX && g_radio.tx_end_us + WAKE_US < wake) {
                wake = g_radio.tx_end_us + WAKE_US;
            }
            for (unsigned i = 0; i < AIR_MAX; ++i) {
                if (g_air[i].used != 0U && g_air[i].end_us + WAKE_US < wake &&
                    g_air[i].end_us >= g_now_us) {
                    wake = g_air[i].end_us + WAKE_US;
                }
            }
        }
        sim_advance_to(wake);
    }
}

static void run_for_ms(uint32_t ms) {
    const uint64_t until = g_now_us + (uint64_t)ms * 1000ULL;
    while (g_now_us < until) {
        fw_pass();
    }
}

/* Fresh node (profile-1 activation + boot arm), fresh peer, fresh stats.
 * Time stays monotonic so the airtime / dwell windows simply expire. */
static void scenario_begin(const char *name, role_t role, uint32_t seed) {
    /* Let the previous scenario's TX and air drain with the peer off. */
    g_role = (role_t)0;
    for (int i = 0; i < 5000 && (sx1276_tx_busy() || g_txq_count > 0U); ++i) {
        fw_pass();
    }
    /* Start every scenario on a fixed 1000 s grid (+3 s), so its run --
     * and the -v trace hash -- does not depend on where the previous
     * scenario happened to end (pre-fix vs fixed builds diff exactly). */
    g_now_us = ((g_now_us / 1000000000ULL) + 1ULL) * 1000000000ULL + 3000000ULL;
    memset(&g_sc, 0, sizeof(g_sc));
    g_sc.name = name;
    memset(g_cmd_recv, 0, sizeof(g_cmd_recv));
    memset(g_cmd_covered_clean, 0, sizeof(g_cmd_covered_clean));
    memset(&g_base, 0, sizeof(g_base));
    memset(&g_orig, 0, sizeof(g_orig));
    memset(&g_th, 0, sizeof(g_th));
    memset(&g_bh, 0, sizeof(g_bh));
    g_txq_head = 0U;
    g_txq_count = 0U;
    g_submit_fail = 0U;
    g_rng = seed;
    radio_reset();
    g_ctx = CTX_BOOT;
    memset(&g_profile, 0, sizeof(g_profile));
    g_profile.profile_id = (uint8_t)REG_PROFILE_FCC_15_247_FHSS_50CH_BW250;
    sx1276_legal_dwell_reset();
    (void)sx1276_fhss_init(0ULL, 0ULL, 0U);
    sx1276_rx_scan_reset();
    g_scan_mirror = SX1276_RX_SCAN_STATE_BOOT;
    g_walk_next = 0U;
    g_lock_anchor_ms = 0U;
    g_prev_adopted = 0U;
    g_prev_auth = 0U;
    g_decay_at_us = 0U;
    memset(g_fail_ms, 0, sizeof(g_fail_ms));
    memset(g_fail_sub, 0, sizeof(g_fail_sub));
    g_final_fail_us = 0U;
    g_last_crc_us = 0U;
    (void)sx1276_rx_arm();
    g_ctx = CTX_NONE;
    g_role = role;
}

static void print_scenario(void) {
    if (g_verbose == 0) {
        return;
    }
    printf("  [%s]\n", g_sc.name);
    if (g_sc.sent != 0U) {
        printf("    cmds sent=%u recv=%u (%.1f%%) | covered=%u clean=%u clean_recv=%u"
               " | miss coll=%u wrong_ch=%u not_rx=%u late_arm=%u\n",
               g_sc.sent, g_sc.recv, 100.0 * g_sc.recv / (double)g_sc.sent,
               g_sc.covered, g_sc.covered_clean, g_sc.covered_clean_recv,
               g_sc.miss_coll, g_sc.miss_wrong_ch, g_sc.miss_not_rx,
               g_sc.miss_late_arm);
        printf("    dec aligned=%u snapped=%u locked_out=%u epoch_drift=%u\n",
               g_sc.dec[SX1276_FHSS_SNAP_DEC_ALIGNED],
               g_sc.dec[SX1276_FHSS_SNAP_DEC_SNAPPED],
               g_sc.dec[SX1276_FHSS_SNAP_DEC_REJECTED_LOCKED_OUT],
               g_sc.dec[SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT]);
    }
    if (g_sc.frames_sent != 0U) {
        printf("    originator frames sent=%u recv=%u\n",
               g_sc.frames_sent, g_sc.frames_recv);
    }
    if (g_sc.crc_sent != 0U) {
        printf("    corrupt cmds sent=%u crc_irqs=%u | FAIL sub-bytes %02x %02x %02x %02x\n",
               g_sc.crc_sent, g_sc.crc_err, g_fail_sub[0], g_fail_sub[1],
               g_fail_sub[2], g_fail_sub[3]);
    }
    printf("    fw tx_ok=%u first_anchor=%u demotion kept=%u reset=%u adoptions=%u"
           " (via follow=%u walker=%u tx=%u) locks=%u demotions=%u"
           " scan_fail_urc=%u (final=%u) auth_passes=%u/%u\n",
           g_sc.fw_tx_ok, g_sc.first_anchor, g_sc.dem_kept, g_sc.dem_reset,
           g_sc.adoptions, g_sc.adopt_via_follow, g_sc.adopt_via_walker,
           g_sc.adopt_via_tx, g_sc.locks, g_sc.demotions, g_sc.scan_fail_urc,
           g_sc.fail_final, g_sc.auth_passes, g_sc.passes);
    printf("    walker retunes=%u suppressed=%u viol=%u scan_park_viol=%u | follow"
           " locked=%u own=%u gate_viol=%u (window=%u) chan_viol=%u | tx_chan_viol=%u"
           " mirror_mismatch=%u submit_fail=%u trace=%08x\n",
           g_sc.walker_retunes, g_sc.walker_suppressed, g_sc.viol_walker,
           g_sc.viol_scan_park,
           g_sc.follow_locked, g_sc.follow_own, g_sc.viol_follow_gate,
           g_sc.window_follow, g_sc.viol_follow_chan, g_sc.viol_tx_chan,
           g_sc.mirror_mismatch, g_submit_fail, g_sc.trace);
    {
        uint32_t seen = 0U;
        for (uint8_t i = 0U; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
            seen += (uint32_t)((g_sc.walker_seen >> i) & 1ULL);
        }
        printf("    RS-12.21: FAILED -> SCANNING at a final FAIL=%u on a tick=%u |"
               " walker channels %u/%u | viol fail_rule=%u failed_unauth=%u"
               " walk_order=%u early_demotion=%u\n",
               g_sc.final_restarts, g_sc.tick_restarts, seen,
               (unsigned)SX1276_FHSS_CHANNEL_COUNT, g_sc.viol_fail_rule,
               g_sc.viol_failed_unauth, g_sc.viol_walk_order,
               g_sc.viol_early_demotion);
    }
}

/* Structural checks every scenario must pass. */
static void check_structure(void) {
    CHECK(g_sc.viol_walker == 0U,
          "[%s] walker retunes must match the dispatched scan actions (%u violations)",
          g_sc.name, g_sc.viol_walker);
    CHECK(g_sc.viol_scan_park == 0U,
          "[%s] the scan machine took the receiver out of RX while the node held"
          " authority (%u times)", g_sc.name, g_sc.viol_scan_park);
    CHECK(g_sc.viol_follow_gate == 0U,
          "[%s] slot follow retuned outside (LOCKED on an adopted grid) or authority"
          " (%u times, %u in the decay window)",
          g_sc.name, g_sc.viol_follow_gate, g_sc.window_follow);
    CHECK(g_sc.viol_follow_chan == 0U,
          "[%s] slot follow retuned off the clock's channel (%u times)",
          g_sc.name, g_sc.viol_follow_chan);
    CHECK(g_sc.viol_tx_chan == 0U,
          "[%s] %u own transmissions keyed off their header slot's hop channel",
          g_sc.name, g_sc.viol_tx_chan);
    CHECK(g_sc.mirror_mismatch == 0U,
          "[%s] scan-state mirror lost track (%u)", g_sc.name, g_sc.mirror_mismatch);
    CHECK(g_submit_fail == 0U,
          "[%s] %u TX submissions refused", g_sc.name, g_submit_fail);
    CHECK(g_sc.recv_dup == 0U,
          "[%s] %u commands delivered twice", g_sc.name, g_sc.recv_dup);
    /* RS-12.21 (2026-10-24): (b), d2 and L. */
    CHECK(g_sc.viol_fail_rule == 0U,
          "[%s] RS-12.21 (b): %u scan FAILs held or left against the rule (held"
          " exactly when final under originator authority; every exit recorded"
          " as (FAILED, BEGIN_SCAN))", g_sc.name, g_sc.viol_fail_rule);
    CHECK(g_sc.viol_failed_unauth == 0U,
          "[%s] RS-12.21 (b): scan FAILED after %u scan ticks without originator"
          " authority or a TX keyed", g_sc.name, g_sc.viol_failed_unauth);
    CHECK(g_sc.viol_walk_order == 0U,
          "[%s] RS-12.21 d2: %u walker steps not to the next channel (channel 0"
          " only after a BEGIN_SCAN out of BOOT or LOCKED)", g_sc.name,
          g_sc.viol_walk_order);
    CHECK(g_sc.viol_early_demotion == 0U,
          "[%s] RS-12.21 L: %u LOCKED -> SCANNING demotions sooner than"
          " LOCK_LOSS_MS after the LOCK or the last accepted frame", g_sc.name,
          g_sc.viol_early_demotion);
}

#ifndef RX_REVERSE_DELIVERY_SIM_ONLY
/* ------------------------------------------------------------------ */
/* Scenarios                                                           */
/* ------------------------------------------------------------------ */

/* Tractor role: a camera-shaped stream with a command plane. */
static void tractor_setup(const char *name, uint32_t seed,
                          uint32_t period_ms, uint32_t jitter_ms,
                          const uint8_t train_w[4], cmd_mode_t mode,
                          uint32_t min_gap_ms, uint32_t extra_gap_ms,
                          uint32_t delay_min_ms, uint32_t delay_max_ms) {
    scenario_begin(name, ROLE_TRACTOR, seed);
    g_th.period_ms = period_ms;
    g_th.jitter_ms = jitter_ms;
    memcpy(g_th.train_w, train_w, 4U);
    g_th.next_frame_us = g_now_us + 50000ULL;
    g_th.next_rxcont_us = g_now_us;
    base_model_init();
    g_base.mode = mode;
    g_base.min_gap_ms = min_gap_ms;
    g_base.extra_gap_ms = extra_gap_ms;
    g_base.delay_min_ms = delay_min_ms;
    g_base.delay_max_ms = delay_max_ms;
    /* The first command waits until the stream has settled (authority
     * needs eight own transmissions). */
    g_base.next_allowed_us = g_now_us + 5000000ULL;
}

static void run_tractor(const char *name, uint32_t seed, uint32_t duration_ms,
                        uint32_t period_ms, uint32_t jitter_ms,
                        const uint8_t train_w[4], cmd_mode_t mode,
                        uint32_t min_gap_ms, uint32_t extra_gap_ms,
                        uint32_t delay_min_ms, uint32_t delay_max_ms) {
    tractor_setup(name, seed, period_ms, jitter_ms, train_w, mode,
                  min_gap_ms, extra_gap_ms, delay_min_ms, delay_max_ms);
    run_for_ms(duration_ms);
}

static void expect_reverse_delivery(uint32_t min_sent) {
    CHECK(g_sc.sent >= min_sent, "[%s] only %u commands sent", g_sc.name, g_sc.sent);
    CHECK(g_sc.covered_clean * 10U >= g_sc.sent * 7U,
          "[%s] the tractor held authority over only %u of %u commands",
          g_sc.name, g_sc.covered_clean, g_sc.sent);
    CHECK(g_sc.covered_clean_recv == g_sc.covered_clean,
          "[%s] reverse delivery: %u of %u clean commands sent while the tractor"
          " held authority were received (all %u sent: %u received;"
          " misses wrong_ch=%u not_rx=%u late_arm=%u coll=%u; first miss at"
          " t=%llu us on %u Hz, receiver on %u Hz, mode %u, scan state %u,"
          " last tuned by ctx %u)",
          g_sc.name, g_sc.covered_clean_recv, g_sc.covered_clean, g_sc.sent,
          g_sc.recv, g_sc.miss_wrong_ch, g_sc.miss_not_rx, g_sc.miss_late_arm,
          g_sc.miss_coll, (unsigned long long)g_sc.first_miss_us,
          g_sc.first_miss_pkt_hz, g_sc.first_miss_radio_hz, g_sc.first_miss_mode,
          g_sc.first_miss_scan, g_sc.first_miss_tuner);
    CHECK(g_sc.follow_own > 0U, "[%s] the tractor never followed its own clock",
          g_sc.name);
    /* The originator's own grid, kept and never renumbered. */
    CHECK(g_sc.first_anchor == 1U && g_sc.dem_reset == 0U && g_sc.adoptions == 0U &&
          g_sc.dec[SX1276_FHSS_SNAP_DEC_SNAPPED] == 0U,
          "[%s] grid moved: first_anchor=%u demotion_reset=%u adoptions=%u snapped=%u",
          g_sc.name, g_sc.first_anchor, g_sc.dem_reset, g_sc.adoptions,
          g_sc.dec[SX1276_FHSS_SNAP_DEC_SNAPPED]);
}

static const uint8_t k_short_trains[4]  = { 50U, 50U, 0U, 0U };
static const uint8_t k_camera_trains[4] = { 23U, 45U, 23U, 9U };  /* leg H mix */

static void test_between_trains_light(void) {
    /* (1) 600 ms frames of 1-2 fragments; one command every 2.5-6 s
     * (longer than LOCK_LOSS, so a lucky hit cannot carry the next one),
     * decided 5-60 ms after a train ends, i.e. in the quiet gap. */
    run_tractor("1 between-trains light", 0x1234567U, 300000U, 600U, 40U,
                k_short_trains, CMD_IN_GAP, 2500U, 3500U, 5U, 60U);
    print_scenario();
    check_structure();
    expect_reverse_delivery(40U);
}

static void test_between_trains_storm(void) {
    /* (2) same stream, the RS-12.14 1.0 s gate: a command in every gap. */
    run_tractor("2 between-trains storm", 0x2345678U, 120000U, 600U, 40U,
                k_short_trains, CMD_IN_GAP, 1000U, 0U, 5U, 60U);
    print_scenario();
    check_structure();
    expect_reverse_delivery(80U);
}

static void test_camera_pump_light(void) {
    /* (3) leg H shape: 2 fps, 1-4 fragment trains, aligned pump at train
     * end, a command every ~17.5 s. */
    run_tractor("3 camera+pump light (leg H)", 0x3456789U, 600000U, 500U, 40U,
                k_camera_trains, CMD_PUMP_AT_TRAIN_END, 10000U, 15000U, 0U, 0U);
    print_scenario();
    check_structure();
    expect_reverse_delivery(25U);
}

static void test_camera_pump_storm(void) {
    /* (4) leg I shape: keyframe requests always pending, 1.0 s gate. */
    run_tractor("4 camera+pump storm (leg I)", 0x456789AU, 300000U, 500U, 40U,
                k_camera_trains, CMD_PUMP_AT_TRAIN_END, 1000U, 0U, 0U, 0U);
    print_scenario();
    check_structure();
    expect_reverse_delivery(150U);
}

static void test_scan_failed(void) {
    /* (8) the base sends nothing for 150 s, so the streaming tractor's scan
     * machine runs out of retries (FAIL x4, 30 s each) and parks in the
     * absorbing FAILED state; then the light command plane of (1) starts.
     * FAILED never LOCKs again, so before the fix only an in-slot hit
     * could land; the originator's own clock must keep driving the
     * receiver in FAILED too. (RS-12.21 (b), 2026-10-24: FAILED is held
     * because the tractor holds originator authority -- it streams without
     * a gap of 1 s -- and nothing parks the modem any more.) */
    tractor_setup("8 scan FAILED, then commands", 0x89ABCDEU, 600U, 40U,
                  k_short_trains, CMD_IN_GAP, 2500U, 3500U, 5U, 60U);
    g_base.next_allowed_us = g_now_us + 150000000ULL;
    run_for_ms(300000U);
    print_scenario();
    check_structure();
    CHECK(g_sc.fail_final == 1U && g_sc.locks == 0U,
          "[%s] the scan machine did not park in FAILED before the commands"
          " (final FAILs=%u, locks=%u)", g_sc.name, g_sc.fail_final, g_sc.locks);
    expect_reverse_delivery(20U);
}

static void test_final_fail_mid_command(void) {
    /* (9) the stream of (8), no command plane. After the third FAIL the
     * final one is due at a known ms: the third's pass ms + the 30 s
     * budget, unless a TX defers it. The base keys one command straddling
     * that ms. A seed whose stream is busy around it, or whose grid puts
     * that key outside the base's slot rule, is skipped. */
    static const uint32_t seeds[] = {
        0x9ABCDEFU, 0xA1B2C3DU, 0xB2C3D4EU, 0xC3D4E5FU,
        0xD4E5F61U, 0xE5F6172U, 0xF617283U, 0x1728394U,
    };
    const uint32_t toa = toa_us((uint8_t)(LORA_PKT_HDR_LEN + CMD_PAYLOAD_LEN));
    uint64_t key_us = 0U;
    uint32_t seed = 0U;
    int id = -1;
    for (unsigned s = 0U; s < sizeof(seeds) / sizeof(seeds[0]) && id < 0; ++s) {
        seed = seeds[s];
        tractor_setup("9 final FAIL mid-command", seed, 600U, 40U,
                      k_short_trains, CMD_IN_GAP, 2500U, 3500U, 5U, 60U);
        g_base.next_allowed_us = 0xFFFFFFFFFFFFFFFFULL;
        const uint64_t t0 = g_now_us;
        while (g_sc.scan_fail_urc < 3U && g_now_us - t0 < 200000000ULL) {
            fw_pass();
        }
        if (g_sc.scan_fail_urc != 3U) {
            break;
        }
        const uint64_t due_us =
            ((uint64_t)g_fail_ms[2] + SX1276_RX_SCAN_REDESIGN_MS) * 1000ULL;
        key_us = due_us - toa / 2U;
        while (g_now_us < key_us - 50000ULL) {
            fw_pass();
        }
        /* Nothing on air, queued or due from the tractor until 50 ms past
         * the FAIL. */
        if (!sx1276_tx_busy() && g_txq_count == 0U && g_th.train_active == 0U &&
            g_th.frames_waiting == 0U && g_th.outstanding == 0U &&
            g_th.next_frame_us > due_us + 50000ULL) {
            id = base_cmd_at_key(key_us);
        }
    }
    if (id < 0) {
        CHECK(0, "[%s] no seed left the tractor quiet at its final FAIL"
              " (FAILs=%u)", g_sc.name, g_sc.scan_fail_urc);
        return;
    }
    run_for_ms(2000U);
    print_scenario();
    if (g_verbose != 0) {
        printf("    seed %08x: final FAIL %lld us after the command's first"
               " symbol (ToA %u us)\n", seed,
               (long long)(g_final_fail_us - key_us), toa);
    }
    check_structure();
    CHECK(g_sc.fail_final == 1U && g_sc.locks == 0U &&
          g_final_fail_us > key_us && g_final_fail_us < key_us + toa,
          "[%s] the final FAIL did not fire while the command was on air"
          " (final FAILs=%u, locks=%u, %lld us after its first symbol)",
          g_sc.name, g_sc.fail_final, g_sc.locks,
          (long long)(g_final_fail_us - key_us));
    CHECK(g_cmd_covered_clean[id] != 0U,
          "[%s] the command was not clean and covered by authority", g_sc.name);
    CHECK(g_cmd_recv[id] != 0U,
          "[%s] the command on air at the final FAIL was lost (misses not_rx=%u"
          " late_arm=%u wrong_ch=%u coll=%u)", g_sc.name, g_sc.miss_not_rx,
          g_sc.miss_late_arm, g_sc.miss_wrong_ch, g_sc.miss_coll);
}

static void test_crc_then_clean_retry(void) {
    /* (10) the stream of (8). Early in the first scan attempt the base's
     * commands arrive with payload-CRC errors, then the base goes quiet.
     * A FAIL's crc_seen bit covers its own window (the last dwell) only,
     * as on a walking node: neither the first FAIL, ~20 s after the
     * errors, nor the CRC-free retry's may report one. */
    const uint8_t crc_bit = (uint8_t)(1U << SX1276_RX_SCAN_FAIL_SUB_BIT_CRC_SEEN);
    tractor_setup("10 CRC errors, then a clean retry", 0xABCDEF1U, 600U, 40U,
                  k_short_trains, CMD_IN_GAP, 1000U, 0U, 5U, 60U);
    g_base.corrupt = 1U;
    const uint64_t t0 = g_now_us;
    while (g_sc.crc_err == 0U && g_now_us - t0 < 20000000ULL) {
        fw_pass();
    }
    g_base.next_allowed_us = 0xFFFFFFFFFFFFFFFFULL;   /* the base goes quiet */
    g_base.decide_at_us = 0U;
    const uint32_t passes0 = g_sc.passes, auth0 = g_sc.auth_passes;
    while (g_sc.scan_fail_urc < 2U && g_now_us - t0 < 100000000ULL) {
        fw_pass();
    }
    print_scenario();
    check_structure();
    /* The path under test: authority on every pass from the CRC error on,
     * so no walker step could have cleared the flag. */
    CHECK(g_sc.crc_err >= 1U && g_sc.scan_fail_urc >= 2U && g_sc.locks == 0U &&
          g_last_crc_us + (uint64_t)SX1276_RX_SCAN_DWELL_MS * 2000ULL <
              (uint64_t)g_fail_ms[0] * 1000ULL &&
          g_sc.auth_passes - auth0 == g_sc.passes - passes0,
          "[%s] setup: CRC IRQs=%u, FAILs=%u, locks=%u, last CRC error %llu us"
          " before the first FAIL, authority over %u of the %u passes after it",
          g_sc.name, g_sc.crc_err, g_sc.scan_fail_urc, g_sc.locks,
          (unsigned long long)((uint64_t)g_fail_ms[0] * 1000ULL - g_last_crc_us),
          g_sc.auth_passes - auth0, g_sc.passes - passes0);
    CHECK((g_fail_sub[0] & crc_bit) == 0U && (g_fail_sub[1] & crc_bit) == 0U,
          "[%s] a FAIL reported a CRC error from an earlier window"
          " (sub-bytes %02x, %02x)", g_sc.name, g_fail_sub[0], g_fail_sub[1]);
}

/* Base role: the firmware follows an ideal originator. */
static void orig_start(uint64_t first_boundary_us, uint32_t first_abs) {
    g_orig.on = 1U;
    g_orig.next_slot_us = first_boundary_us;
    g_orig.next_abs = first_abs;
}

static void test_follower_unchanged(void) {
    /* (5) acquire, follow with duplex commands every 1.2-2.5 s (never
     * chains to authority), lose the originator for 4 s (demotion resets
     * the ADOPTED clock), re-acquire it on a new grid. */
    scenario_begin("5 follower unchanged", ROLE_BASE, 0x56789ABU);
    orig_start(g_now_us + 37000ULL, 1234U);
    g_bh.gap_min_ms = 1200U;
    g_bh.gap_max_ms = 2500U;
    g_bh.next_cmd_us = g_now_us + 1000ULL;
    g_bh.cmds_from_us = 0U;
    g_bh.cmds_to_us = 0xFFFFFFFFFFFFFFFFULL;
    run_for_ms(40000U);
    const uint32_t locks_before = g_sc.locks;
    const uint32_t frames_before = g_sc.frames_recv;
    g_orig.silent_from_us = g_now_us;
    g_orig.silent_to_us = g_now_us + 4000000ULL;
    run_for_ms(4000U);
    const uint32_t resets_after_loss = g_sc.dem_reset;
    /* Resume on a new grid: +17 slots, +73 ms. */
    orig_start(g_now_us + 73000ULL, g_orig.next_abs + 17U);
    run_for_ms(90000U);
    print_scenario();
    check_structure();
    CHECK(g_sc.auth_passes == 0U, "[%s] a follower held originator authority (%u passes)",
          g_sc.name, g_sc.auth_passes);
    CHECK(g_sc.walker_suppressed == 0U, "[%s] walker suppressed %u times on a follower",
          g_sc.name, g_sc.walker_suppressed);
    CHECK(g_sc.follow_own == 0U, "[%s] a follower followed its own clock", g_sc.name);
    CHECK(locks_before >= 1U && frames_before > 0U,
          "[%s] never acquired the originator (locks=%u frames=%u)",
          g_sc.name, locks_before, frames_before);
    CHECK(resets_after_loss >= 1U,
          "[%s] losing the originator must reset the ADOPTED clock", g_sc.name);
    CHECK(g_sc.locks > locks_before && g_sc.frames_recv > frames_before + 100U,
          "[%s] did not re-acquire the re-gridded originator (locks %u -> %u)",
          g_sc.name, locks_before, g_sc.locks);
}

static void test_authority_bound(void) {
    /* (6) acquire, lose the originator (adopted clock reset), then the
     * idle drain bursts 10 commands 120 ms apart with no grid adopted:
     * the node earns authority and follows its own clock -- but only
     * while it holds it; the walker resumes within a dwell of the decay
     * and the node re-acquires when the originator returns. */
    scenario_begin("6 authority bound (base burst)", ROLE_BASE, 0x6789ABCU);
    orig_start(g_now_us + 37000ULL, 4321U);
    run_for_ms(30000U);
    const uint32_t locks_before = g_sc.locks;
    g_orig.silent_from_us = g_now_us;
    g_orig.silent_to_us = g_now_us + 9000000ULL;
    run_for_ms(3000U);                 /* demoted, adopted clock reset */
    g_bh.burst_left = 10U;
    g_bh.burst_gap_ms = 120U;
    g_bh.burst_at_us = g_now_us;
    run_for_ms(6000U);
    const uint64_t decay_seen = g_sc.walker_after_decay_us;
    orig_start(g_now_us + 91000ULL, g_orig.next_abs + 29U);
    run_for_ms(90000U);
    print_scenario();
    check_structure();
    CHECK(g_sc.auth_passes > 0U && g_sc.follow_own > 0U,
          "[%s] the burst never produced self-follow (auth_passes=%u own=%u)",
          g_sc.name, g_sc.auth_passes, g_sc.follow_own);
    CHECK(decay_seen != 0U && g_sc.walker_after_decay_viol == 0U,
          "[%s] walker not back within one dwell of the authority decay"
          " (first walker retune %llu us after the decay)",
          g_sc.name, (unsigned long long)decay_seen);
    CHECK(locks_before >= 1U && g_sc.locks > locks_before,
          "[%s] did not re-acquire after the burst (locks %u -> %u)",
          g_sc.name, locks_before, g_sc.locks);
}

static void test_decay_window(void) {
    /* (7) 2 fps stream with a 2.5 s camera stall every 9 s; storm command
     * plane plus the idle drain during the stall. Authority decays 1 s
     * into each stall, typically while a command has the tractor LOCKED.
     * The receiver must not follow its own clock in that window. */
    static const uint8_t trains[4] = { 23U, 45U, 23U, 9U };
    scenario_begin("7 decay window (stalls)", ROLE_TRACTOR, 0x789ABCDU);
    g_th.period_ms = 500U;
    g_th.jitter_ms = 40U;
    memcpy(g_th.train_w, trains, 4U);
    g_th.next_frame_us = g_now_us + 50000ULL;
    g_th.next_rxcont_us = g_now_us;
    g_th.stall_every_ms = 9000U;
    g_th.stall_len_ms = 2500U;
    g_th.stall_from_us = g_now_us;
    g_th.stall_to_us = g_now_us;
    base_model_init();
    g_base.mode = CMD_PUMP_AT_TRAIN_END;
    g_base.min_gap_ms = 1000U;
    g_base.idle_drain = 1U;
    g_base.next_allowed_us = g_now_us + 5000000ULL;
    run_for_ms(300000U);
    print_scenario();
    check_structure();
    CHECK(g_sc.window_follow == 0U,
          "[%s] followed its own clock %u times in the decay window", g_sc.name,
          g_sc.window_follow);
}

int main(int argc, char **argv) {
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "-v") == 0) { g_verbose = 1; }
    }
    test_between_trains_light();
    test_between_trains_storm();
    test_camera_pump_light();
    test_camera_pump_storm();
    test_follower_unchanged();
    test_authority_bound();
    test_decay_window();
    test_scan_failed();
    test_final_fail_mid_command();
    test_crc_then_clean_retry();
    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] rx_reverse_delivery: %d failure(s)\n", g_failures);
        return 1;
    }
    printf("[PASS] rx_reverse_delivery: 10 SIL sequences against the real"
           " sx1276_rx.c/sx1276_tx.c -- reverse delivery between trains,"
           " follower path unchanged, self-follow bounded by authority\n");
    return 0;
}
#endif /* !RX_REVERSE_DELIVERY_SIM_ONLY */
