# LoRa Firmware — Tranche 2 Implementation Plan
### B2 CFG transport · C1 OpMode ownership extraction · Reserved STATS field wiring

**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Date:** 2026-05-05
**Predecessor:** [2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md)
**Scope:** Plan only. No code in this document. Targets the firmware after the B1 + DMA-HT/TC tranche has merged: see [host/host_cmd.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c), [host/host_stats.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c), [include/host_types.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h), [radio/sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c).

**Authoritative references:** [02 Firmware Architecture Plan](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md), [04 Hardware Interface and Recovery](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) §2.3 + §6, [05 Method G Review](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md).

---

## 0. Snapshot of what already landed

- B1 host protocol set: PING / VER / UID / RESET / STATS / REG_READ / REG_WRITE (allowlisted) with versioned ERR_PROTO sub-codes; type IDs centralized in `host_types.h` with a compile-time uniqueness check.
- F1↔C0 reconciliation done: `0xC0 RADIO_IRQ_URC` and `0xF1 FAULT_URC` (FAULT not yet emitted but the slot is reserved).
- DMA HT/TC + IDLE servicing live; counters `host_irq_ht / tc / te / idle` reach the wire.
- `STATS_URC` is **already wire-locked at 64 B** (see `host_stats_wire_t` in [host/host_stats.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c)). The last five `u32` slots `{radio_crc_err, radio_rx_ok, radio_tx_ok, radio_tx_abort_lbt, radio_state}` are **declared but written as zero** today. **Do not** re-pack the struct — fill the existing slots.

---

## 1. Sequencing

| Order | Item | Why |
|---|---|---|
| 1 | **B2-step-1** — CFG schema + RAM store + CFG_SET/GET/OK/DATA wire | Pure host-layer work; lands without touching the radio. Required by C1 for the `lbt_enable / lbt_threshold_dbm / fhss_enable` knobs C2/C5 will consume. |
| 2 | **B2-step-2** — apply-hook table; first hook = `tx_power_dbm` (already settable via existing `sx1276_set_tx_power_dbm`) | Validates the apply-hook pattern end-to-end against a real registered effect before the radio FSM exists. |
| 3 | **C1** — `sx1276_modes` extraction; sole owner of `RegOpMode`, `RegDioMapping1/2`; declares `sx1276_state_t` enum used by `radio_state` STATS slot | Blocks all later radio work; must precede C2/C3 so they can attribute counters to a known state. |
| 4 | **C2** — RX path lands → fills `radio_rx_ok`, `radio_crc_err`, `radio_state` | First real radio counters reach STATS. |
| 5 | **C3** — TX path lands → fills `radio_tx_ok`, `radio_tx_abort_lbt` (zero until C5 LBT) | Closes out the reserved-slot wiring. |
| 6 | **B2-step-3** — register radio apply-hooks (`tx_power_dbm`, `lbt_enable`, `lbt_threshold_dbm`, `fhss_enable`, `fhss_quality_aware`, `replay_window`, `iwdg_window_ms`, `host_baud`, `deep_sleep_enable`, `beacon_enable`, `beacon_channel_idx`, `crypto_in_l072`) | Some are no-ops at this phase (intentionally) — register them anyway with a documented "deferred" status so the wire surface is complete and stable. |

Each item below specifies file touches, design constraints, and acceptance gates.

---

## 2. B2 — CFG_SET / CFG_GET (RAM-backed first)

### 2.1 Wire format

Inner payload of `CFG_SET (0x20)` and `CFG_DATA_URC (0xA1)`:
```
+------+------+----------+
| key  |  len |  value   |
| u8   |  u8  |  N B     |
+------+------+----------+
```
Inner payload of `CFG_GET (0x21)`: single byte `key`.
Inner payload of `CFG_OK_URC (0xA0)`: `{u8 key, u8 status, u8 actual_len, u8 reserved}`.

`status` codes (new in `host_types.h`):
- `0` OK
- `1` UNKNOWN_KEY
- `2` BAD_LENGTH (caller's `len` ≠ key's declared size)
- `3` OUT_OF_RANGE (value clamped or rejected per key policy)
- `4` APPLY_FAILED (apply-hook returned error; previous value retained)
- `5` DEFERRED (value stored; takes effect on next reset — applies to `host_baud`)
- `6` READ_ONLY (key exists but is GET-only, e.g. `protocol_version`)

### 2.2 Key registry

Authoritative table is [04 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md). Implement these IDs in `include/host_cfg_keys.h`:

| `key` | Name | Wire type | Bytes | Apply on set? | Apply target |
|---|---|---|---|---|---|
| `0x01` | `tx_power_dbm` | `i8` | 1 | yes | `sx1276_set_tx_power_dbm` |
| `0x02` | `tx_power_adapt_enable` | `bool` | 1 | yes (RAM flag, radio FSM reads it) | radio config flag |
| `0x03` | `lbt_enable` | `bool` | 1 | yes | radio config flag (consumed in C5) |
| `0x04` | `lbt_threshold_dbm` | `i8` | 1 | yes | radio config flag |
| `0x05` | `fhss_enable` | `bool` | 1 | yes | radio config flag |
| `0x06` | `fhss_quality_aware` | `bool` | 1 | rejected (`APPLY_FAILED`) at this phase — N-06 deferred to Phase 4+ | none |
| `0x07` | `fhss_channel_mask` | `u64` | 8 | yes (validated against region default) | radio config |
| `0x08` | `deep_sleep_enable` | `bool` | 1 | rejected (Phase 5) | none |
| `0x09` | `beacon_enable` | `bool` | 1 | yes | radio config flag |
| `0x0A` | `beacon_channel_idx` | `u8` | 1 | yes | radio config flag |
| `0x0B` | `host_baud` | `u32` | 4 | DEFERRED — store + persist on reset | `host_uart_init` re-runs on next boot |
| `0x0C` | `replay_window` | `u8` | 1 | yes — bookkeeping field for H7 (L072 stores but does not use; thin-modem profile per [05 §A1](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md)) | none |
| `0x0D` | `iwdg_window_ms` | `u16` | 2 | yes | platform_iwdg_set_window |
| `0x0E` | `crypto_in_l072` | `bool` | 1 | rejected (Profile A locked at launch) | none |
| `0x80` | `protocol_version` | `u8` | 1 | READ_ONLY | returns `HOST_PROTOCOL_VER` |
| `0x81` | `wire_schema_version` | `u8` | 1 | READ_ONLY | returns `HOST_WIRE_SCHEMA_VER` |
| `0x82` | `cfg_dirty` | `bool` | 1 | READ_ONLY | true if any RAM CFG differs from boot defaults (informs H7 that reset will lose values) |

Compile-time check: a `_Static_assert(sizeof(cfg_key_table) / sizeof(cfg_key_t) == CFG_KEY_COUNT)` so adding a new key without registering the apply-hook is a build error.

### 2.3 Architecture

Two new files:

| Path | Responsibility |
|---|---|
| `host/host_cfg.c` + `include/host_cfg.h` | RAM-backed key/value store, range validation, apply-hook dispatch, serialization |
| `include/host_cfg_keys.h` | Key IDs and per-key descriptor table (size, range, apply fn pointer) |

`host_cfg.h` API (minimal):
```
typedef enum { CFG_OK, CFG_UNKNOWN_KEY, CFG_BAD_LENGTH,
               CFG_OUT_OF_RANGE, CFG_APPLY_FAILED,
               CFG_DEFERRED, CFG_READ_ONLY } cfg_status_t;

typedef cfg_status_t (*cfg_apply_fn)(const uint8_t *value, uint8_t len);

void cfg_init(void);                          // load defaults
cfg_status_t cfg_set(uint8_t key, const uint8_t *value, uint8_t len);
cfg_status_t cfg_get(uint8_t key, uint8_t *out, uint8_t out_cap, uint8_t *out_len);
bool cfg_is_dirty(void);
```

`host/host_cmd.c` gains two cases (`HOST_TYPE_CFG_SET_REQ`, `HOST_TYPE_CFG_GET_REQ`) that thin-wrap `cfg_set`/`cfg_get` and emit `CFG_OK_URC`/`CFG_DATA_URC`. **No** apply logic in `host_cmd.c`; the registry is the single source of truth.

### 2.4 Persistence (intentionally deferred)

CFG region in Flash exists per [memory_map.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/memory_map.h) (`MM_CFG_BASE`, 8 KB). **Do not** wire the writer in this PR — Phase 6 work item. For now:
- `cfg_init()` always loads compile-time defaults from `config.h`.
- `host_baud` writes return `CFG_DEFERRED` and store in RAM only — they will not survive a reset until Phase 6. Document this in [04 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md). H7 must not rely on it across resets.
- `cfg_is_dirty()` lets the H7 detect "values exist that won't survive reset" and re-apply post-reset until persistence ships.

### 2.5 Constraints

- **No allocation.** Store is a fixed-size array of `cfg_value_t { u8 key; u8 len; u8 bytes[8]; }` indexed by key descriptor.
- **No floats.** All values are integer types. `tx_power_dbm` etc. stay `i8`/`u8`/`u16`/`u32`/`u64`.
- **Endianness.** Multi-byte values are little-endian on the wire (matches existing `put_u32_le` convention).
- **`CFG_SET` is idempotent.** Setting the same value twice returns `CFG_OK` both times; apply-hook still runs (so H7 can use it as a "force-apply" path after a register-reset detected via `STATS_URC.radio_state == 0`).
- **Bounds before apply.** Range-check first, then call apply-hook. If apply-hook fails, restore the previous value before returning `APPLY_FAILED` — the URC's `actual_len` reflects what's now in the store.

### 2.6 Acceptance gates

1. Set/get round-trip for every key in the table: `CFG_SET k v` then `CFG_GET k` returns `v` (or the clamped value).
2. Out-of-range values return `OUT_OF_RANGE`, not silently clamped, **unless** the key descriptor explicitly opts into clamp semantics (`tx_power_dbm` clamps to 2…17 per existing `sx1276_set_tx_power_dbm`).
3. `tx_power_dbm` apply-hook is observable: after `CFG_SET 0x01 0x0F`, a `REG_READ 0x09` returns `0x80 | (0x0F - 2) = 0x8D`.
4. Unknown key returns `UNKNOWN_KEY`, never crashes.
5. `cfg_dirty` flips true after the first non-default `CFG_SET` and stays true until reset.
6. Wire-format contract test in `bench/host_proto/cfg_round_trip.c` covers every key descriptor.

### 2.7 Risks

- **Key-ID collisions with future additions.** Mitigate by reserving `0x40..0x4F` for radio-deep keys (Phase 4+) and `0x80+` for read-only meta-keys.
- **`host_baud` half-applied.** If a future hook tried to apply at runtime it would brick the H7 link mid-frame. Hard-coded `CFG_DEFERRED` is the safe path. Add `_Static_assert` that key `0x0B`'s apply fn pointer is `NULL`.

---

## 3. C1 — Strict OpMode ownership extraction (`sx1276_modes`)

### 3.1 Why this lands before any FSM logic

[radio/sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) currently writes `RegOpMode` from three places: `sx1276_init`, `apply_profile_full`, and (via the SLEEP→STDBY sequence) reset. Adding RX/TX state machines without first centralizing OpMode ownership reproduces the exact "two callers race over the radio mode" bug that bit MKRWAN history. Per [02 §2](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) and the recommendation in the prior plan, **`sx1276_modes` must be the only file that touches `RegOpMode` and `RegDioMapping1/2`.**

### 3.2 Extracted module

| Path | Responsibility |
|---|---|
| `radio/sx1276_modes.c` + `include/sx1276_modes.h` | OpMode transitions, DIO routing, RF switch coordination, public `sx1276_state_t` enum |

Public API (minimal):
```
typedef enum {
    SX1276_STATE_UNINIT  = 0,
    SX1276_STATE_SLEEP   = 1,
    SX1276_STATE_STANDBY = 2,
    SX1276_STATE_TX      = 3,
    SX1276_STATE_RX_CONT = 4,
    SX1276_STATE_RX_SINGLE = 5,
    SX1276_STATE_CAD     = 6,
    SX1276_STATE_FAULT   = 0xFFU
} sx1276_state_t;

bool sx1276_modes_init(void);                 // sets SLEEP→STDBY, programs DIO map for STDBY
sx1276_state_t sx1276_modes_get_state(void);  // pure RAM read; consumed by STATS.radio_state
bool sx1276_modes_to_sleep(void);
bool sx1276_modes_to_standby(void);
bool sx1276_modes_to_tx(void);                // sets RegDioMapping1/2 for TX, then OpMode=TX
bool sx1276_modes_to_rx_cont(void);
bool sx1276_modes_to_rx_single(uint16_t timeout_symbols);
bool sx1276_modes_to_cad(void);
```

DIO routing per OpMode is a `static const` table (per [§3.3 of the predecessor plan](2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md)) read inside the transition fn — never duplicated at call sites.

### 3.3 Refactor of existing `sx1276.c`

Surgical, not rewriting:

1. **Move** the `SX1276_OPMODE_*` macros to `sx1276_modes.c` (file-local).
2. **Replace** the two existing `sx1276_write_reg(SX1276_REG_OP_MODE, ...)` call sites in [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) (lines in `sx1276_init` and `apply_profile_full`) with `sx1276_modes_to_sleep()` / `sx1276_modes_to_standby()`.
3. **Move** `sx1276_set_rf_switch_tx()` into `sx1276_modes.c` since it must be coupled to the OpMode transition (TX switch closes before OpMode→TX, RX switch closes before OpMode→RX). Expose it only as a side effect of the mode call.
4. **Add** a `_Static_assert`-style runtime guard: a debug build (`HOST_DEBUG_OPMODE_GUARD=1`) reads `RegOpMode` after any `sx1276_write_reg` call and faults if the cached `s_state` and the chip disagree — catches stray writers in test runs.
5. **grep enforcement** in CI (Makefile target `check-opmode-owner`): `grep -n 'REG_OP_MODE' radio/*.c` must return only matches in `sx1276_modes.c`. Failing greps fail the build.

### 3.4 Effect on existing tests

`sx1276_init` still returns `bool` and still emits `BOOT_URC`. Existing `RADIO_IRQ_URC` debug path unchanged. No host-side compatibility break.

### 3.5 Acceptance gates

1. `grep -RIn 'REG_OP_MODE' radio/` finds the macro **only** inside `sx1276_modes.c`.
2. `STATS_URC.radio_state` reflects mode transitions: after boot it reads `SX1276_STATE_STANDBY = 2`; a forced `sx1276_modes_to_sleep()` in a unit test moves it to `1`.
3. `make` clean; existing `RADIO_IRQ_URC` smoke test still passes (DIO IRQs still captured).
4. Build with `HOST_DEBUG_OPMODE_GUARD=1`: no FAULT_URC during a 60-second idle run.

### 3.6 Risks

- **TX/RX RF switch timing.** SX1276 reference designs require the antenna switch to settle ~10 µs before OpMode→TX/RX. Encode the delay inside `sx1276_modes_to_tx/rx_*` — never the caller's job.
- **CAD entry from RX.** SX1276 datasheet allows CAD only from STANDBY. `sx1276_modes_to_cad()` must transit through STANDBY automatically; failing to do so silently no-ops on real hardware.

---

## 4. C2 + C3 — wire real radio counters into the reserved STATS slots

### 4.1 The reserved slots, and who fills each

Today the wire schema has these five slots zero-filled (see `host_stats_serialize` tail):

| Slot | Field | Filled by |
|---|---|---|
| 11 | `radio_crc_err` | C2 (RX path) on `RegHopChannel.RxPayloadCrcOn=1 && RegIrqFlags.PayloadCrcError=1` at `RxDone` |
| 12 | `radio_rx_ok` | C2 (RX path) on `RxDone && CRC ok && URC emitted` |
| 13 | `radio_tx_ok` | C3 (TX path) on `TxDone` |
| 14 | `radio_tx_abort_lbt` | C5 (LBT gate) — stays zero until then; **do not** decrement, just leave zero |
| 15 | `radio_state` | C1 (modes) — `sx1276_modes_get_state()` snapshot at serialize time |

**No struct change required.** Just replace the trailing zero writes in `host_stats_serialize` with the real getters.

### 4.2 Counter API extension to `host_stats`

Add to `include/host_stats.h`:
```
void host_stats_radio_rx_ok(void);
void host_stats_radio_crc_err(void);
void host_stats_radio_tx_ok(void);
void host_stats_radio_tx_abort_lbt(void);   // unused until C5 — present so call sites are stable
```
Storage: file-local `static uint32_t s_radio_*` in `host_stats.c`. `host_stats_reset()` zeros all five (already does the four DIO counters; just extends the same pattern).

`radio_state` is **not** a counter — `host_stats_serialize` calls `sx1276_modes_get_state()` directly and casts `(uint32_t)` for the wire. Rationale: state is a snapshot, not an accumulation; reading via the modes module avoids two sources of truth.

### 4.3 Call-site discipline

The radio FSM (C2/C3) must call the counter fns from **main-loop poll context only**, never from EXTI/IRQ handlers — per the "no SPI, no host-stats writes in IRQ" rule from [05 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md). Counter writes are otherwise unprotected (single-writer, single-reader — the serialize path is also main-loop).

If a counter ever needs to be written from IRQ, switch to `__atomic_fetch_add(_, _, __ATOMIC_RELAXED)`. Not needed today.

### 4.4 STATS_URC schema versioning is unchanged

`HOST_WIRE_SCHEMA_VER = 1` already accounts for these slots (they were declared, just zeroed). Do **not** bump the schema version when filling them — that would falsely signal a breaking change to the H7 driver. Document in the C2 PR that "schema v1 means slots 11/12/15 may be non-zero; slots 13/14 may be non-zero from C3/C5 onward."

### 4.5 Acceptance gates

After C1+C2:
1. With a known-good loopback transmitter on the bench, `STATS_URC.radio_rx_ok` increments by exactly 1 per delivered frame.
2. With a deliberately CRC-corrupted transmitter, `radio_crc_err` increments by exactly 1 per malformed frame; `radio_rx_ok` does not.
3. `radio_state` reports `SX1276_STATE_RX_CONT` while the FSM is in continuous RX, transitions to `STANDBY` between frames.
4. `STATS_RESET` zeros all four new counters (state is read-only — unaffected).

After C3:
5. Each successful `TX_FRAME` increments `radio_tx_ok` by 1.
6. `radio_state` shows `SX1276_STATE_TX` during a long-payload TX.

After C5 (later tranche, called out for completeness):
7. `radio_tx_abort_lbt` increments when LBT rejects a TX attempt.

### 4.6 Risks

- **Double-counting CRC errors.** SX1276 latches `PayloadCrcError` until cleared; on continuous RX the FSM must clear `RegIrqFlags` before re-arming or a single bad packet bumps the counter twice. Standard pattern; just note it.
- **`radio_state` racing with serializer.** The serialize fn runs in main loop; mode transitions also run in main loop (per C1's "polled FSM" rule). No race. If a future tranche moves transitions into IRQ, this breaks — leave a comment.

---

## 5. Combined sequencing & branch strategy

| Order | Branch | Contents |
|---|---|---|
| 1 | `feat/host-cfg-b2-skeleton` | `host_cfg.[ch]`, `host_cfg_keys.h`, CFG_SET/GET/OK/DATA wire, `tx_power_dbm` apply-hook only, contract tests |
| 2 | `feat/radio-modes-c1` | `sx1276_modes.[ch]`, refactor `sx1276.c` callers, CI grep guard, `radio_state` slot wired |
| 3 | `feat/host-cfg-b2-radio-hooks` | Register radio apply-hooks (`lbt_enable`, `lbt_threshold_dbm`, `fhss_enable`, beacon, deep-sleep stubs); intentionally-deferred keys return `APPLY_FAILED`/`DEFERRED` with documented status |
| 4 | `feat/radio-rx-c2` | RX FSM (consumes B2 config), fills `radio_rx_ok` + `radio_crc_err` |
| 5 | `feat/radio-tx-c3` | TX FSM, fills `radio_tx_ok`; `radio_tx_abort_lbt` still zero |

PR #3 (radio hooks) intentionally lands *between* C1 and C2 so the RX FSM can read finalized config flags from `host_cfg_get_*()` accessors instead of compile-time defaults — keeps C2's PR scope to "radio behavior" only.

---

## 6. Out of scope (do NOT land in this tranche)

- CFG persistence to Flash (Phase 6).
- LBT logic (C5; lands after C3).
- Per-frame FHSS hopping logic (Phase 3 W3-1).
- N-06 quality-aware FHSS (Phase 4+).
- Deep-sleep mode entry from FSM (Phase 5).
- `crypto_in_l072` (Profile A locked at launch per [05 §5.4](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md)).
- Bumping `HOST_WIRE_SCHEMA_VER` — slots being filled are already declared; filling them is an additive runtime change, not a wire change.

---

## 7. One-line summary

> **Land `host_cfg` first as the RAM-backed CFG_SET/GET store with a typed key registry and apply-hook table (only `tx_power_dbm` wired live, all others stubbed with documented status); then extract `sx1276_modes` so `RegOpMode` and `RegDioMapping1/2` have exactly one writer (CI-enforced via grep), exposing `sx1276_state_t` for the `radio_state` STATS slot; then fill the four reserved radio counters (`radio_rx_ok`, `radio_crc_err`, `radio_tx_ok`, `radio_tx_abort_lbt`) from the C2/C3 RX/TX paths without touching the wire schema or bumping `HOST_WIRE_SCHEMA_VER`.**

---

## 8. Companion Analysis (Gemini 3.1 Pro Preview, 2026-05-05)

**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)
**Purpose:** Technical validation of Tranche 2 implementation steps, registry architecture, and state constraints.

### 8.1 Config Registry & Apply Hooks (Work Item B2)
- **Deferred host_baud (0x0B):** The decision to mark baud rate changes as `CFG_DEFERRED` (only applied after reset) is a critical safety measure. Applying baud changes mid-frame would reliably sever the IPC link. **Recommendation:** Ensure the H7 host-side driver is explicitly aware that when it commands a baud rate change, it must send a `RESET` command, wait, and then re-initialize its own UART peripheral to match.
- **Bounds Checking Prior to Apply:** The contract requiring range validation *before* executing the apply-hook ensures that hardware drivers (`sx1276_set_tx_power_dbm`) never have to defend against invalid inputs.

### 8.2 SX1276 OpMode Centralization (Work Item C1)
- **Single-Writer Enforcement:** Restricting `RegOpMode` and `RegDioMapping` to `sx1276_modes.c` is the cleanest way to prevent state machine deadlocks. Using a CI-enforced `grep` step to block rogue register writes in future PRs is an excellent CI/CD practice.
- **RF Switch Timing:** The 10 µs settling delay for the RX/TX RF switches you've highlighted is an easily-missed detail that completely destroys signal integrity if ignored. Integrating this delay directly into `sx1276_modes_to_tx/rx_*` guarantees it is respected.

### 8.3 STATS_URC Reserved Slots
- **Schema Immutability:** Filling out existing `0x00` fields without bumping `HOST_WIRE_SCHEMA_VER` is the correct approach to maintain ABI stability with the H7.
- **Context Polling (`radio_state` vs Counters):** The distinction between counters (updated via `host_stats_*`) and state (polled dynamically at serialization via `sx1276_modes_get_state()`) cleanly separates accumulating metrics from transient system state, preventing race conditions during UART IDLE draining.
