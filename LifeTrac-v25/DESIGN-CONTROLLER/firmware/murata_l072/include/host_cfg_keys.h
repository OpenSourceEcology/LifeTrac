#ifndef LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H
#define LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H

#include <stdint.h>

#define CFG_KEY_TX_POWER_DBM                 0x01U
#define CFG_KEY_TX_POWER_ADAPT_ENABLE        0x02U
#define CFG_KEY_LBT_ENABLE                   0x03U
#define CFG_KEY_LBT_THRESHOLD_DBM            0x04U
#define CFG_KEY_FHSS_ENABLE                  0x05U
#define CFG_KEY_FHSS_QUALITY_AWARE           0x06U
#define CFG_KEY_FHSS_CHANNEL_MASK            0x07U
#define CFG_KEY_DEEP_SLEEP_ENABLE            0x08U
#define CFG_KEY_BEACON_ENABLE                0x09U
#define CFG_KEY_BEACON_CHANNEL_IDX           0x0AU
#define CFG_KEY_HOST_BAUD                    0x0BU
#define CFG_KEY_REPLAY_WINDOW                0x0CU
#define CFG_KEY_IWDG_WINDOW_MS               0x0DU
#define CFG_KEY_CRYPTO_IN_L072               0x0EU
/* 0x0F intentionally unused to preserve stable key numbering. */
#define CFG_KEY_LBT_MAX_BACKOFF_MS           0x10U
#define CFG_KEY_CAD_SYMBOLS                  0x11U
#define CFG_KEY_FHSS_DWELL_MS                0x13U
#define CFG_KEY_REG_PROFILE                  0x14U
/*
 * FCC-EVID-D6-2-a-ii: validator inputs for the regulatory-profile
 * two-phase commit synthesised at cfg_set(CFG_KEY_REG_PROFILE).
 * Both accept full type range at the wire (no validator clamp);
 * the host_cfg_profile validator is the authoritative gate and
 * is the only path that can produce CFG_STATUS_PROFILE_REJECT_*.
 */
#define CFG_KEY_ANTENNA_GAIN_DBI             0x15U
#define CFG_KEY_HW_CEILING_DBM               0x16U
/*
 * FHSS hop-sequence identity (2026-07-29). Both u64 little-endian,
 * synthesised into the profile request alongside 0x15/0x16.
 *
 * These feed sx1276_fhss_init(farm_id, node_id, epoch), whose FNV-1a
 * seed drives the Fisher-Yates channel permutation. Before they
 * existed that call site passed a hardcoded (0, 0, 0), so every radio
 * in the fleet derived the SAME 50-channel hop order — two machines in
 * the same area would collide on every dwell instead of statistically
 * avoiding each other, and the hop sequence was fully predictable.
 *
 * CFG_KEY_FHSS_NODE_ID IS LINK-SCOPED, NOT PER-RADIO. Both ends of one
 * link MUST be given the same node_id or their permutations diverge:
 * the follower retunes to the wrong channel every slot, loses lock and
 * demotes to SCANNING indefinitely. Provision one id per
 * tractor<->base pair, not one per board. The name is inherited from
 * sx1276_fhss_init(); read it as "link id".
 *
 * Default 0 on both keys reproduces the pre-2026-07-29 permutation
 * exactly, so an un-provisioned fleet is bit-for-bit unchanged.
 *
 * ORDERING CONTRACT: write these BEFORE CFG_KEY_REG_PROFILE. The seed
 * is consumed only inside host_cfg_profile_activate(), and there is no
 * re-seed path short of re-activating the profile.
 *
 * There is deliberately NO epoch key. epoch is a runtime time
 * coordinate — one per 10 s, continuously re-derived from the slot
 * clock and re-anchored from the peer's air header — not stable
 * identity. Its init-time value is a throwaway phase origin that the
 * first FHSS TX overwrites.
 */
#define CFG_KEY_FHSS_FARM_ID                 0x17U
#define CFG_KEY_FHSS_NODE_ID                 0x18U

#define CFG_KEY_PROTOCOL_VERSION             0x80U
#define CFG_KEY_WIRE_SCHEMA_VERSION          0x81U
#define CFG_KEY_CFG_DIRTY                    0x82U

#define CFG_KEY_MAX_VALUE_LEN                8U
#define CFG_KEY_COUNT                        25U

/*
 * Regulatory operating-mode enum for CFG_KEY_REG_PROFILE (u8).
 *
 * Cross-references:
 *   - AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md
 *     §14 (v3.0 consolidation), §14.2 step 2 (FCC-PROFILE-ENUM).
 *   - TODO.md Stage S1.5 -> FCC-PROFILE-ENUM, FCC-A4, FCC-A5.
 *
 * Default = REG_PROFILE_BENCH_ONLY_FIXED_915 (bench fixed-channel only).
 *
 * Activation invariant (enforced in host_cfg.c validator): the production
 * profile FCC_15_247_FHSS_50CH_BW250 is rejected with
 * CFG_STATUS_PROFILE_UNROUTED unless the FCC-A4 TX path is compiled in
 * (i.e. the build symbol LIFETRAC_FHSS_TX_ROUTED is defined). The DTS
 * profile FCC_15_247_DTS_BW500 is similarly gated for now (no DTS TX
 * path exists yet); it can be ungated once a DTS routing layer ships.
 *
 * This prevents an operator from selecting a regulatory profile that
 * the firmware cannot actually route TX onto. See plan §13.1 / §14
 * delta #1.
 */
#define REG_PROFILE_BENCH_ONLY_FIXED_915     0U
#define REG_PROFILE_FCC_15_247_FHSS_50CH_BW250 1U
#define REG_PROFILE_FCC_15_247_DTS_BW500     2U
#define REG_PROFILE_MAX                      REG_PROFILE_FCC_15_247_DTS_BW500

#endif /* LIFETRAC_MURATA_L072_HOST_CFG_KEYS_H */
