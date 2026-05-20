/*
 * FCC-A3 golden-vector + property tests for the FHSS hop scheduler.
 *
 * Cross-checks:
 *   (1) FNV-1a 32-bit seed hashes for three fixed (farm, node, epoch)
 *       inputs against values computed off-tree (see test header below).
 *   (2) Fisher-Yates / xorshift32 permutation for those seeds — first 8
 *       and last 8 entries plus bijection sum.
 *   (3) Bijection property: every emitted permutation contains each of
 *       0..49 exactly once.
 *   (4) Determinism: same (farm, node, epoch) → identical seed and
 *       identical permutation across reset cycles.
 *   (5) Distinctness: incrementing the epoch changes the seed (and
 *       almost surely the permutation).
 *   (6) Full-epoch coverage via next_channel(): one call to next_channel
 *       per slot visits each of the 50 channels exactly once before the
 *       epoch auto-advances.
 *   (7) Cold-start warm-up: blacklist refused with BLACKLIST_WARMUP for
 *       the first 50 hops.
 *   (8) Legal floor: even after warm-up, blacklist refused with
 *       BLACKLIST_FLOOR (50-channel narrowband profile = no slack).
 *   (9) LBT block accounting is independent of blacklist and active set.
 *  (10) APIs called before init return NOT_INIT (fail-closed).
 *  (11) Epoch advance regenerates permutation and resets slot.
 *  (12) Auto-advance on slot wrap increments the epoch.
 *
 * Golden values computed externally (see commit message for the small
 * PowerShell script). If you regenerate the algorithm, recompute these.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sx1276_fhss.h"
#include "sx1276_fhss_chantab.h"

static int g_failures = 0;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            ++g_failures;                                             \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);      \
            fprintf(stderr, __VA_ARGS__);                             \
            fprintf(stderr, "\n");                                    \
        }                                                             \
    } while (0)

typedef struct {
    const char *name;
    uint64_t    farm_id;
    uint64_t    node_id;
    uint32_t    epoch;
    uint32_t    expected_seed;
    uint8_t     expected_first_8[8];
    uint8_t     expected_last_8[8]; /* perm[42..49] */
} golden_t;

static const golden_t k_golden[] = {
    {
        .name = "zero",
        .farm_id = 0ULL, .node_id = 0ULL, .epoch = 0U,
        .expected_seed = 0xB9FEE455UL,
        .expected_first_8 = { 3, 19, 31, 14, 22, 46, 10, 0 },
        .expected_last_8  = { 13, 33, 48, 2, 11, 15, 42, 39 },
    },
    {
        .name = "nominal",
        .farm_id = 0x1122334455667788ULL,
        .node_id = 0xAABBCCDDEEFF0011ULL,
        .epoch   = 42U,
        .expected_seed = 0xB871A5DFUL,
        .expected_first_8 = { 10, 23, 42, 14, 38, 35, 5, 16 },
        .expected_last_8  = { 19, 21, 48, 8, 45, 12, 18, 0 },
    },
    {
        .name = "max",
        .farm_id = 0xFFFFFFFFFFFFFFFFULL,
        .node_id = 0xFFFFFFFFFFFFFFFFULL,
        .epoch   = 0xFFFFFFFFUL,
        .expected_seed = 0x5236AAE1UL,
        .expected_first_8 = { 40, 8, 22, 34, 20, 15, 48, 47 },
        .expected_last_8  = { 29, 14, 30, 38, 28, 45, 36, 6 },
    },
};

static bool is_bijection_0_to_49(const uint8_t *perm) {
    uint8_t seen[SX1276_FHSS_CHANNEL_COUNT] = {0};
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (perm[i] >= SX1276_FHSS_CHANNEL_COUNT) return false;
        if (seen[perm[i]] != 0U) return false;
        seen[perm[i]] = 1U;
    }
    return true;
}

static void test_golden_vectors(void) {
    for (size_t v = 0; v < sizeof(k_golden) / sizeof(k_golden[0]); ++v) {
        const golden_t *g = &k_golden[v];
        const uint32_t seed = sx1276_fhss_compute_seed(g->farm_id, g->node_id, g->epoch);
        CHECK(seed == g->expected_seed,
              "seed mismatch [%s]: got 0x%08" PRIX32 " expected 0x%08" PRIX32,
              g->name, seed, g->expected_seed);

        uint8_t perm[SX1276_FHSS_CHANNEL_COUNT];
        sx1276_fhss_compute_permutation(seed, perm);

        CHECK(is_bijection_0_to_49(perm),
              "permutation [%s] is not a bijection of 0..49", g->name);

        for (uint8_t i = 0; i < 8U; ++i) {
            CHECK(perm[i] == g->expected_first_8[i],
                  "perm[%s][%u]: got %u expected %u",
                  g->name, (unsigned)i, (unsigned)perm[i],
                  (unsigned)g->expected_first_8[i]);
        }
        for (uint8_t i = 0; i < 8U; ++i) {
            const uint8_t pos = (uint8_t)(42U + i);
            CHECK(perm[pos] == g->expected_last_8[i],
                  "perm[%s][%u]: got %u expected %u",
                  g->name, (unsigned)pos, (unsigned)perm[pos],
                  (unsigned)g->expected_last_8[i]);
        }
    }
}

static void test_determinism_and_distinctness(void) {
    uint8_t a[SX1276_FHSS_CHANNEL_COUNT];
    uint8_t b[SX1276_FHSS_CHANNEL_COUNT];
    uint8_t c[SX1276_FHSS_CHANNEL_COUNT];

    const uint32_t s1 = sx1276_fhss_compute_seed(0xCAFEBABEULL, 0xDEADBEEFULL, 7U);
    const uint32_t s2 = sx1276_fhss_compute_seed(0xCAFEBABEULL, 0xDEADBEEFULL, 7U);
    const uint32_t s3 = sx1276_fhss_compute_seed(0xCAFEBABEULL, 0xDEADBEEFULL, 8U);

    CHECK(s1 == s2, "seed not deterministic: 0x%08" PRIX32 " vs 0x%08" PRIX32, s1, s2);
    CHECK(s1 != s3, "seed did not change when epoch advanced (0x%08" PRIX32 ")", s1);

    sx1276_fhss_compute_permutation(s1, a);
    sx1276_fhss_compute_permutation(s2, b);
    sx1276_fhss_compute_permutation(s3, c);

    CHECK(memcmp(a, b, sizeof(a)) == 0, "permutation not deterministic");
    CHECK(memcmp(a, c, sizeof(a)) != 0, "permutation identical across distinct epochs");
}

static void test_zero_seed_fallback(void) {
    /* The FNV-1a hash can in principle produce 0, in which case the
     * permutation helper substitutes XORSHIFT32_NONZERO_FALLBACK. We
     * can't easily produce a 0 hash from a real (farm, node, epoch)
     * tuple, but we CAN feed 0 directly to the helper and verify it
     * doesn't collapse to identity (which is what an un-substituted
     * xorshift32 with state==0 would yield). */
    uint8_t perm[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_compute_permutation(0U, perm);
    CHECK(is_bijection_0_to_49(perm), "zero-seed permutation not a bijection");
    /* Identity would have perm[i] == i for all i. Any deviation in the
     * first few positions is enough. */
    bool identity = true;
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        if (perm[i] != i) { identity = false; break; }
    }
    CHECK(!identity, "zero-seed permutation collapsed to identity (xorshift32 zero-state trap)");
}

static void test_not_init_fails_closed(void) {
    sx1276_fhss_reset();
    uint8_t idx = 0xFFU;
    uint32_t hz = 0xFFFFFFFFUL;
    CHECK(sx1276_fhss_next_channel(&idx, &hz) == SX1276_FHSS_ERR_NOT_INIT,
          "next_channel pre-init must return NOT_INIT");
    CHECK(idx == 0xFFU && hz == 0xFFFFFFFFUL,
          "next_channel must not mutate out-params on NOT_INIT");
    CHECK(sx1276_fhss_epoch_advance() == SX1276_FHSS_ERR_NOT_INIT,
          "epoch_advance pre-init must return NOT_INIT");
    CHECK(sx1276_fhss_blacklist(0) == SX1276_FHSS_ERR_NOT_INIT,
          "blacklist pre-init must return NOT_INIT");
    CHECK(sx1276_fhss_is_initialized() == 0U,
          "is_initialized must be 0 after reset");
}

static void test_null_arg_fails_closed(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 2ULL, 3U);
    uint8_t idx;
    uint32_t hz;
    CHECK(sx1276_fhss_next_channel(NULL, &hz) == SX1276_FHSS_ERR_NULL_ARG,
          "next_channel(NULL, &hz) must return NULL_ARG");
    CHECK(sx1276_fhss_next_channel(&idx, NULL) == SX1276_FHSS_ERR_NULL_ARG,
          "next_channel(&idx, NULL) must return NULL_ARG");
}

static void test_full_epoch_coverage(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(0xDEADBEEFULL, 0xCAFEBABEULL, 100U);

    uint8_t seen[SX1276_FHSS_CHANNEL_COUNT] = {0};
    const uint32_t start_epoch = sx1276_fhss_current_epoch();
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        uint8_t idx;
        uint32_t hz;
        const sx1276_fhss_status_t st = sx1276_fhss_next_channel(&idx, &hz);
        CHECK(st == SX1276_FHSS_OK, "next_channel slot %u failed: %d", (unsigned)i, (int)st);
        CHECK(idx < SX1276_FHSS_CHANNEL_COUNT, "next_channel returned bad idx %u", (unsigned)idx);
        CHECK(hz == sx1276_fhss_chantab_center_hz(idx),
              "next_channel hz mismatch at slot %u: hz=%" PRIu32 " expected %" PRIu32,
              (unsigned)i, hz, sx1276_fhss_chantab_center_hz(idx));
        CHECK(seen[idx] == 0U, "channel %u visited twice within one epoch", (unsigned)idx);
        seen[idx] = 1U;
    }
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        CHECK(seen[i] == 1U, "channel %u not visited within one epoch", (unsigned)i);
    }
    CHECK(sx1276_fhss_current_epoch() == start_epoch,
          "epoch must not advance until 51st call (got %" PRIu32 ", expected %" PRIu32 ")",
          sx1276_fhss_current_epoch(), start_epoch);
}

static void test_auto_advance_on_wrap(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(0x1ULL, 0x2ULL, 500U);
    uint8_t idx;
    uint32_t hz;
    for (uint8_t i = 0; i < SX1276_FHSS_CHANNEL_COUNT; ++i) {
        (void)sx1276_fhss_next_channel(&idx, &hz);
    }
    /* 51st call must auto-advance epoch from 500 -> 501. */
    const sx1276_fhss_status_t st = sx1276_fhss_next_channel(&idx, &hz);
    CHECK(st == SX1276_FHSS_OK, "51st next_channel failed: %d", (int)st);
    CHECK(sx1276_fhss_current_epoch() == 501U,
          "epoch must auto-advance to 501, got %" PRIu32, sx1276_fhss_current_epoch());
}

static void test_warmup_refuses_blacklist(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(7ULL, 9ULL, 0U);
    /* Hop 49 times, leaving exactly 1 warm-up slot remaining. */
    uint8_t idx;
    uint32_t hz;
    for (uint8_t i = 0; i < (SX1276_FHSS_WARMUP_HOPS - 1U); ++i) {
        (void)sx1276_fhss_next_channel(&idx, &hz);
    }
    CHECK(sx1276_fhss_blacklist(0U) == SX1276_FHSS_ERR_BLACKLIST_WARMUP,
          "blacklist during warm-up must return BLACKLIST_WARMUP");
    CHECK(sx1276_fhss_active_count() == SX1276_FHSS_CHANNEL_COUNT,
          "active count must not drop during refused blacklist");
    /* Complete warm-up. */
    (void)sx1276_fhss_next_channel(&idx, &hz);
    /* Now warmup is done; blacklist must still be refused by the FLOOR
     * rule because the table size IS the floor. */
    CHECK(sx1276_fhss_blacklist(0U) == SX1276_FHSS_ERR_BLACKLIST_FLOOR,
          "blacklist after warm-up must return BLACKLIST_FLOOR (50ch == floor 50)");
    CHECK(sx1276_fhss_active_count() == SX1276_FHSS_CHANNEL_COUNT,
          "active count must remain 50 (floor refusal)");
}

static void test_blacklist_bad_idx(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 1ULL, 0U);
    CHECK(sx1276_fhss_blacklist(SX1276_FHSS_CHANNEL_COUNT) == SX1276_FHSS_ERR_BAD_IDX,
          "blacklist(50) must return BAD_IDX");
    CHECK(sx1276_fhss_blacklist(0xFFU) == SX1276_FHSS_ERR_BAD_IDX,
          "blacklist(255) must return BAD_IDX");
}

static void test_lbt_block_independent(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 1ULL, 0U);
    /* Recording LBT blocks must not change the active set. */
    sx1276_fhss_record_lbt_block(7U);
    sx1276_fhss_record_lbt_block(7U);
    sx1276_fhss_record_lbt_block(7U);
    sx1276_fhss_record_lbt_block(42U);
    /* Out-of-range index must be silently dropped, not asserted. */
    sx1276_fhss_record_lbt_block(200U);

    CHECK(sx1276_fhss_lbt_block_count(7U) == 3U,
          "lbt_block_count(7)=%" PRIu32 " expected 3", sx1276_fhss_lbt_block_count(7U));
    CHECK(sx1276_fhss_lbt_block_count(42U) == 1U,
          "lbt_block_count(42)=%" PRIu32 " expected 1", sx1276_fhss_lbt_block_count(42U));
    CHECK(sx1276_fhss_lbt_block_count(0U) == 0U,
          "lbt_block_count(0) should be 0");
    CHECK(sx1276_fhss_lbt_block_count(200U) == 0U,
          "lbt_block_count out-of-range must return 0");
    CHECK(sx1276_fhss_active_count() == SX1276_FHSS_CHANNEL_COUNT,
          "active count must be 50 (LBT records do not blacklist)");
}

static void test_epoch_advance_regenerates(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(0xABCDULL, 0x1234ULL, 1000U);
    const uint32_t seed_a = sx1276_fhss_current_seed();
    uint8_t idx_a, idx_b;
    uint32_t hz_a, hz_b;
    (void)sx1276_fhss_next_channel(&idx_a, &hz_a);
    CHECK(sx1276_fhss_epoch_advance() == SX1276_FHSS_OK, "epoch_advance must succeed");
    CHECK(sx1276_fhss_current_epoch() == 1001U,
          "epoch must be 1001 after advance, got %" PRIu32, sx1276_fhss_current_epoch());
    const uint32_t seed_b = sx1276_fhss_current_seed();
    CHECK(seed_a != seed_b,
          "seed must change across epoch_advance (a=0x%08" PRIX32 " b=0x%08" PRIX32 ")",
          seed_a, seed_b);
    CHECK(sx1276_fhss_current_slot() == 0U,
          "slot must reset to 0 on epoch advance");
    (void)sx1276_fhss_next_channel(&idx_b, &hz_b);
    /* Almost surely a different channel — but at minimum the test must
     * succeed without internal error. */
    CHECK(idx_b < SX1276_FHSS_CHANNEL_COUNT, "post-advance idx out of range");
}

/* FCC-A6b-2-i: snap_to() fails closed before init and never mutates
 * state when it returns an error. */
static void test_snap_to_fails_closed(void) {
    sx1276_fhss_reset();
    CHECK(sx1276_fhss_snap_to(42U, 0U) == SX1276_FHSS_ERR_NOT_INIT,
          "snap_to before init must return NOT_INIT");

    (void)sx1276_fhss_init(11ULL, 22ULL, 100U);
    const uint32_t epoch_before = sx1276_fhss_current_epoch();
    const uint8_t  slot_before  = sx1276_fhss_current_slot();
    CHECK(sx1276_fhss_snap_to(999U, (uint8_t)(SX1276_FHSS_CHANNEL_COUNT + 1U))
              == SX1276_FHSS_ERR_BAD_IDX,
          "snap_to with slot > CHANNEL_COUNT must return BAD_IDX");
    CHECK(sx1276_fhss_current_epoch() == epoch_before,
          "BAD_IDX snap_to must not mutate epoch");
    CHECK(sx1276_fhss_current_slot() == slot_before,
          "BAD_IDX snap_to must not mutate slot");
}

/* FCC-A6b-2-i: snap_to(target_epoch, snap_to_slot) places the scheduler
 * such that the next next_channel() returns permutation[snap_to_slot] of
 * H(farm, node, target_epoch). Cross-checked against compute_seed +
 * compute_permutation (the host mirror's source of truth). */
static void test_snap_to_lands_on_expected_slot(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(0xDEADBEEFCAFEBABEULL, 0x1234ULL, 1U);

    const uint32_t target_epoch = 4242U;
    const uint8_t  target_slot  = 17U;

    CHECK(sx1276_fhss_snap_to(target_epoch, target_slot) == SX1276_FHSS_OK,
          "snap_to(4242, 17) must succeed");
    CHECK(sx1276_fhss_current_epoch() == target_epoch,
          "snap_to must set epoch to target");
    CHECK(sx1276_fhss_current_slot() == target_slot,
          "snap_to must set slot to target");
    CHECK(sx1276_fhss_current_seed() ==
              sx1276_fhss_compute_seed(0xDEADBEEFCAFEBABEULL, 0x1234ULL, target_epoch),
          "snap_to must regenerate seed from H(farm, node, target_epoch)");

    /* What next_channel() must return = permutation[target_slot] of the
     * new seed (cross-checked via the public pure helper). */
    uint8_t expected_perm[SX1276_FHSS_CHANNEL_COUNT];
    sx1276_fhss_compute_permutation(sx1276_fhss_current_seed(), expected_perm);
    const uint8_t expected_idx = expected_perm[target_slot];

    uint8_t actual_idx;
    uint32_t actual_hz;
    CHECK(sx1276_fhss_next_channel(&actual_idx, &actual_hz) == SX1276_FHSS_OK,
          "next_channel after snap_to must succeed");
    CHECK(actual_idx == expected_idx,
          "next_channel after snap_to landed on perm[%u]=%u, expected %u",
          target_slot, actual_idx, expected_idx);
    CHECK(sx1276_fhss_current_slot() == (uint8_t)(target_slot + 1U),
          "next_channel must advance slot by 1");
}

/* FCC-A6b-2-i: snap_to(epoch, CHANNEL_COUNT) is legal — the next
 * next_channel() trips the wrap path and auto-advances to
 * (target_epoch + 1, slot 0). */
static void test_snap_to_slot_equal_count_wraps(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(5ULL, 6ULL, 0U);

    const uint32_t target_epoch = 700U;
    CHECK(sx1276_fhss_snap_to(target_epoch, SX1276_FHSS_CHANNEL_COUNT)
              == SX1276_FHSS_OK,
          "snap_to with slot == CHANNEL_COUNT must succeed");
    CHECK(sx1276_fhss_current_slot() == SX1276_FHSS_CHANNEL_COUNT,
          "slot must be parked at CHANNEL_COUNT");

    uint8_t idx;
    uint32_t hz;
    CHECK(sx1276_fhss_next_channel(&idx, &hz) == SX1276_FHSS_OK,
          "next_channel after wrap-snap must succeed");
    CHECK(sx1276_fhss_current_epoch() == (target_epoch + 1U),
          "next_channel must have auto-advanced epoch");
    CHECK(sx1276_fhss_current_slot() == 1U,
          "next_channel after epoch wrap must leave slot=1");
}

/* FCC-A6b-2-i: snap_to clears warmup_hops_remaining (snapping to a
 * remote implies the remote is already past warm-up, so the local node
 * has joined an active hopset and the warm-up grace no longer applies).
 * Blacklist must still be refused by the legal-floor rule, but the
 * BLACKLIST_WARMUP path must NOT fire. */
static void test_snap_to_clears_warmup(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 2ULL, 0U);
    /* Confirm we are in warm-up: blacklist must refuse with WARMUP. */
    CHECK(sx1276_fhss_blacklist(0U) == SX1276_FHSS_ERR_BLACKLIST_WARMUP,
          "pre-snap blacklist should refuse with BLACKLIST_WARMUP");

    CHECK(sx1276_fhss_snap_to(50U, 0U) == SX1276_FHSS_OK, "snap_to");

    /* Post-snap: WARMUP must no longer fire; FLOOR still must
     * (50 channels == floor 50 leaves no headroom). */
    CHECK(sx1276_fhss_blacklist(0U) == SX1276_FHSS_ERR_BLACKLIST_FLOOR,
          "post-snap blacklist should now refuse with BLACKLIST_FLOOR, not WARMUP");
}

/* FCC-A6b-2-i: snap_to preserves blacklist_bits and lbt_block counters
 * (per-channel quality history survives a snap). The 50-channel legal
 * floor rule means we can't actually install a blacklist bit through
 * the public API, so we exercise this via the lbt_block path which IS
 * decoupled from blacklist. */
static void test_snap_to_preserves_quality_history(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(3ULL, 4ULL, 10U);
    sx1276_fhss_record_lbt_block(7U);
    sx1276_fhss_record_lbt_block(7U);
    sx1276_fhss_record_lbt_block(7U);
    CHECK(sx1276_fhss_lbt_block_count(7U) == 3U,
          "pre-snap lbt_block_count[7] should be 3");

    CHECK(sx1276_fhss_snap_to(500U, 5U) == SX1276_FHSS_OK, "snap_to");

    CHECK(sx1276_fhss_lbt_block_count(7U) == 3U,
          "post-snap lbt_block_count[7] must still be 3 (history preserved)");
    CHECK(sx1276_fhss_active_count() == SX1276_FHSS_CHANNEL_COUNT,
          "active_count must remain CHANNEL_COUNT");
}

/* FCC-A6b-2-ii-alpha: consider_remote fails closed before init and on
 * malformed remote_hop_idx; both paths must NOT mutate scheduler state. */
static void test_consider_remote_fails_closed(void) {
    sx1276_fhss_reset();
    CHECK(sx1276_fhss_consider_remote(0U, 0U)
              == SX1276_FHSS_SNAP_DEC_REJECTED_NOT_INIT,
          "consider_remote before init must return REJECTED_NOT_INIT");

    (void)sx1276_fhss_init(100ULL, 200ULL, 50U);
    const uint32_t epoch_before = sx1276_fhss_current_epoch();
    const uint8_t  slot_before  = sx1276_fhss_current_slot();
    CHECK(sx1276_fhss_consider_remote(50U, SX1276_FHSS_CHANNEL_COUNT)
              == SX1276_FHSS_SNAP_DEC_REJECTED_BAD_HOP,
          "remote_hop_idx == CHANNEL_COUNT must return REJECTED_BAD_HOP");
    CHECK(sx1276_fhss_consider_remote(50U, 0xFFU)
              == SX1276_FHSS_SNAP_DEC_REJECTED_BAD_HOP,
          "remote_hop_idx == 0xFF must return REJECTED_BAD_HOP");
    CHECK(sx1276_fhss_current_epoch() == epoch_before,
          "REJECTED_BAD_HOP must not mutate epoch");
    CHECK(sx1276_fhss_current_slot() == slot_before,
          "REJECTED_BAD_HOP must not mutate slot");
}

/* FCC-A6b-2-ii-alpha: ALIGNED fires iff remote (epoch, hop_idx) matches
 * the slot the local scheduler just consumed. No mutation on ALIGNED. */
static void test_consider_remote_aligned(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 2ULL, 10U);
    /* Burn one hop so local_hop_idx is well-defined as slot 0 of
     * current epoch. */
    uint8_t idx;
    uint32_t hz;
    (void)sx1276_fhss_next_channel(&idx, &hz);
    /* After one next_channel: current_slot==1, so local_hop_idx==0. */
    const uint32_t epoch_now = sx1276_fhss_current_epoch();
    const uint8_t  slot_now  = sx1276_fhss_current_slot();

    CHECK(sx1276_fhss_consider_remote(epoch_now, 0U)
              == SX1276_FHSS_SNAP_DEC_ALIGNED,
          "matching (epoch, hop_idx=0) after one hop must be ALIGNED");
    CHECK(sx1276_fhss_current_epoch() == epoch_now,
          "ALIGNED must not mutate epoch");
    CHECK(sx1276_fhss_current_slot() == slot_now,
          "ALIGNED must not mutate slot");
}

/* FCC-A6b-2-ii-alpha: SNAPPED fires for in-tolerance (|Δepoch| ≤ 1)
 * mismatched hints; post-snap state is consistent with snap_to spec
 * (next slot == remote_hop_idx + 1, epoch == remote_epoch). */
static void test_consider_remote_snapped_same_epoch(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(7ULL, 8ULL, 500U);
    /* Burn 3 hops: local_hop_idx will be 2 (slot 3 → -1). */
    uint8_t idx;
    uint32_t hz;
    for (int i = 0; i < 3; ++i) {
        (void)sx1276_fhss_next_channel(&idx, &hz);
    }
    const uint32_t epoch_now = sx1276_fhss_current_epoch();
    CHECK(sx1276_fhss_current_slot() == 3U, "sanity: slot should be 3");

    /* Remote claims it just emitted on hop_idx 20 (same epoch). */
    CHECK(sx1276_fhss_consider_remote(epoch_now, 20U)
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "same-epoch hop mismatch must be SNAPPED");
    CHECK(sx1276_fhss_current_epoch() == epoch_now,
          "post-snap epoch must equal remote epoch");
    CHECK(sx1276_fhss_current_slot() == 21U,
          "post-snap slot must equal remote_hop_idx + 1, got %u",
          sx1276_fhss_current_slot());
}

/* FCC-A6b-2-ii-alpha: SNAPPED accepts Δepoch=+1 and Δepoch=-1 (covers
 * a remote that rolled the epoch boundary just before us / just after us). */
static void test_consider_remote_snapped_epoch_drift_one(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(11ULL, 22ULL, 1000U);
    uint8_t idx;
    uint32_t hz;
    (void)sx1276_fhss_next_channel(&idx, &hz);

    /* +1 epoch drift accepted. */
    CHECK(sx1276_fhss_consider_remote(1001U, 5U)
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "Δepoch=+1 must be SNAPPED");
    CHECK(sx1276_fhss_current_epoch() == 1001U,
          "post-snap epoch must follow remote");

    /* -1 epoch drift accepted. */
    CHECK(sx1276_fhss_consider_remote(1000U, 5U)
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "Δepoch=-1 must be SNAPPED");
    CHECK(sx1276_fhss_current_epoch() == 1000U,
          "post-snap epoch must follow remote");
}

/* FCC-A6b-2-ii-alpha: REJECTED_EPOCH_DRIFT for |Δepoch| > 1, including
 * the uint32 wrap boundary (proves the (int32_t)(a-b) cast handles wrap). */
static void test_consider_remote_rejected_drift(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 1ULL, 500U);
    uint8_t idx;
    uint32_t hz;
    (void)sx1276_fhss_next_channel(&idx, &hz);
    const uint32_t epoch_before = sx1276_fhss_current_epoch();
    const uint8_t  slot_before  = sx1276_fhss_current_slot();

    /* Δepoch = +2 → REJECTED. */
    CHECK(sx1276_fhss_consider_remote(502U, 0U)
              == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT,
          "Δepoch=+2 must be REJECTED_EPOCH_DRIFT");
    /* Δepoch = -2 → REJECTED. */
    CHECK(sx1276_fhss_consider_remote(498U, 0U)
              == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT,
          "Δepoch=-2 must be REJECTED_EPOCH_DRIFT");
    /* Stale frame from far past. */
    CHECK(sx1276_fhss_consider_remote(0U, 0U)
              == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT,
          "Δepoch=-500 must be REJECTED_EPOCH_DRIFT");
    /* Far-future replay. */
    CHECK(sx1276_fhss_consider_remote(0xFFFFFFFFU, 0U)
              == SX1276_FHSS_SNAP_DEC_REJECTED_EPOCH_DRIFT,
          "Δepoch=UINT32_MAX must be REJECTED_EPOCH_DRIFT");

    CHECK(sx1276_fhss_current_epoch() == epoch_before,
          "REJECTED_EPOCH_DRIFT must not mutate epoch");
    CHECK(sx1276_fhss_current_slot() == slot_before,
          "REJECTED_EPOCH_DRIFT must not mutate slot");
}

/* FCC-A6b-2-ii-alpha: uint32 epoch wrap boundary — local=UINT32_MAX,
 * remote=0 must be treated as Δ=+1 (SNAPPED), not Δ=-UINT32_MAX
 * (REJECTED). Proves the wrap math is correct. */
static void test_consider_remote_epoch_wrap_boundary(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(1ULL, 1ULL, 0xFFFFFFFEU);
    uint8_t idx;
    uint32_t hz;
    /* Hop to push epoch to 0xFFFFFFFE+0 still, slot=1. */
    (void)sx1276_fhss_next_channel(&idx, &hz);

    /* Δepoch = (0xFFFFFFFF - 0xFFFFFFFE) = +1 → SNAPPED. */
    CHECK(sx1276_fhss_consider_remote(0xFFFFFFFFU, 10U)
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "Δepoch=+1 across UINT32_MAX-1 must be SNAPPED");
    CHECK(sx1276_fhss_current_epoch() == 0xFFFFFFFFU,
          "post-snap epoch must equal UINT32_MAX");

    /* Now wrap: Δepoch = (0 - 0xFFFFFFFF) = +1 (mod 2^32) → SNAPPED. */
    CHECK(sx1276_fhss_consider_remote(0U, 5U)
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "Δepoch=+1 across UINT32_MAX→0 wrap must be SNAPPED");
    CHECK(sx1276_fhss_current_epoch() == 0U,
          "post-wrap-snap epoch must equal 0");
}

/* FCC-A6b-2-ii-alpha: SNAPPED with remote_hop_idx == CHANNEL_COUNT-1
 * snaps to slot CHANNEL_COUNT (the wrap-park position); next
 * next_channel() must auto-advance the epoch. */
static void test_consider_remote_snapped_wrap_slot(void) {
    sx1276_fhss_reset();
    (void)sx1276_fhss_init(3ULL, 4ULL, 100U);
    uint8_t idx;
    uint32_t hz;
    (void)sx1276_fhss_next_channel(&idx, &hz);

    CHECK(sx1276_fhss_consider_remote(100U,
                                      (uint8_t)(SX1276_FHSS_CHANNEL_COUNT - 1U))
              == SX1276_FHSS_SNAP_DEC_SNAPPED,
          "remote on last hop must be SNAPPED");
    CHECK(sx1276_fhss_current_slot() == SX1276_FHSS_CHANNEL_COUNT,
          "post-snap slot must be parked at CHANNEL_COUNT");

    (void)sx1276_fhss_next_channel(&idx, &hz);
    CHECK(sx1276_fhss_current_epoch() == 101U,
          "next_channel after wrap-snap must auto-advance epoch");
}

int main(void) {
    test_golden_vectors();
    test_determinism_and_distinctness();
    test_zero_seed_fallback();
    test_not_init_fails_closed();
    test_null_arg_fails_closed();
    test_full_epoch_coverage();
    test_auto_advance_on_wrap();
    test_warmup_refuses_blacklist();
    test_blacklist_bad_idx();
    test_lbt_block_independent();
    test_epoch_advance_regenerates();
    test_snap_to_fails_closed();
    test_snap_to_lands_on_expected_slot();
    test_snap_to_slot_equal_count_wraps();
    test_snap_to_clears_warmup();
    test_snap_to_preserves_quality_history();
    test_consider_remote_fails_closed();
    test_consider_remote_aligned();
    test_consider_remote_snapped_same_epoch();
    test_consider_remote_snapped_epoch_drift_one();
    test_consider_remote_rejected_drift();
    test_consider_remote_epoch_wrap_boundary();
    test_consider_remote_snapped_wrap_slot();

    if (g_failures != 0) {
        fprintf(stderr, "[FAIL] fhss_scheduler_vectors: %d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("[PASS] fhss_scheduler_vectors: 23 cases\n");
    return EXIT_SUCCESS;
}
