# LoRa Firmware — Tranche 3 Implementation Plan
### B2 step-2 key policies + CFG contract tests · C2 RX-path counter wiring · CI enforcement of `check-opmode-owner`

**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Date:** 2026-05-05
**Predecessors:**
- [2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Next_Tranche_Implementation_Plan_Copilot_v1_0.md)
- [2026-05-05_LoRa_Firmware_Tranche_2_CFG_Modes_Stats_Plan_Copilot_v1_0.md](2026-05-05_LoRa_Firmware_Tranche_2_CFG_Modes_Stats_Plan_Copilot_v1_0.md)

**Scope:** Plan only. No code in this document. Targets the firmware after the Tranche 2 land:
- `host/host_cfg.c`, `include/host_cfg.h`, `include/host_cfg_keys.h` exist with the 17-key registry, RAM store, validators, and a single live apply-hook (`tx_power_dbm`).
- `radio/sx1276_modes.c` is the sole writer of `RegOpMode` / `RegDioMapping1/2` / RF switch GPIOs; `Makefile` defines `check-opmode-owner` but **CI does not yet invoke it**.
- `host_stats.c` already calls `sx1276_modes_get_state()` for `radio_state`; counter setters `host_stats_radio_rx_ok / crc_err / tx_ok / tx_abort_lbt` exist but **have no callers yet**.

**Authoritative references:**
- [02 Firmware Architecture Plan](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)
- [04 Hardware Interface and Recovery](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) §6 (CFG semantics) and §2.3 (RX flow)
- [05 Method G Review](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) §6 (no-SPI-in-IRQ rule)
- SX1276 datasheet §4.1.6 (LoRa RX flow), §6.4 (RegIrqFlags)

---

## 0. What's already merged (recap, do not re-do)

- Per-key bounds checking and clamp/reject semantics for every key in `cfg_validate_and_normalize` (see [host/host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)). `tx_power_dbm` clamps to 2..17; bools reject >1; baud whitelists {9600, 115200, 921600}; IWDG whitelists 50..5000 ms; FHSS mask rejects all-zero.
- `cfg_set` already restores prior value on `APPLY_FAILED`, returns `CFG_STATUS_DEFERRED` for the `CFG_FLAG_DEFERRED` keys, returns `CFG_STATUS_READ_ONLY` for the meta keys, marks `s_cfg_dirty` only on first real divergence from defaults.
- `host_cmd.c` has `handle_cfg_set` / `handle_cfg_get` wired with the proper URC framing.
- Reserved STATS slots (rx_ok, crc_err, tx_ok, tx_abort_lbt, radio_state) are emitted; only the *callers* are missing.
- `Makefile` `check-opmode-owner` target works locally but isn't run by GitHub Actions.

The work below fills the three remaining gaps that close out the Tranche 2 → Tranche 3 transition.

---

## 1. Sequencing

| Order | Branch | Contents | Why this order |
|---|---|---|---|
| 1 | `chore/ci-opmode-owner` | Add a new GHA job `firmware-l072-static-checks` running `make check` and `make check-opmode-owner`. **Lands first** because it has zero risk of breaking the build, becomes a guard for everything after it. |
| 2 | `feat/host-cfg-policy-tests` | Lock the per-key policy table in a Markdown spec; add a host-runnable contract-test binary `bench/host_proto/cfg_contract.c` that exercises every key + status-code path; wire it into the new CI job. |
| 3 | `feat/radio-rx-c2` | Implement the SX1276 RX poll FSM (`radio/sx1276_rx.c`) consumed from `main.c`, emitting `RX_FRAME_URC` and calling `host_stats_radio_rx_ok()` / `host_stats_radio_crc_err()` at the documented decision points. |

CI lands first so PRs #2 and #3 cannot regress the OpMode guard or the CFG contract.

---

## 2. CI integration of `check-opmode-owner` (Item 3, lands first)

### 2.1 New GHA job

Add to [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml) (or a new sibling workflow `firmware-l072-ci.yml` — see §2.4 trade-off below):

```yaml
firmware-l072-static-checks:
  runs-on: ubuntu-latest
  name: L072 firmware static checks (memory map, OpMode owner, CFG contract)
  steps:
    - uses: actions/checkout@v4
    - name: Install host toolchain
      run: sudo apt-get update && sudo apt-get install -y build-essential
    - name: make check (memory map invariants)
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
      run: make check
    - name: make check-opmode-owner
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
      run: make check-opmode-owner
    - name: CFG contract tests       # added in §3 below
      working-directory: LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
      run: make check-cfg-contract
```

The job runs on every push/PR that already triggers `arduino-ci.yml` (paths filter already covers `LifeTrac-v25/DESIGN-CONTROLLER/firmware/**`), so no new path filters are needed.

### 2.2 Branch-protection wiring

Mark `firmware-l072-static-checks` as a **required** check on `main`. This is the contract that makes the OpMode guard load-bearing. Document the new required check in [LifeTrac-v25/DESIGN-CONTROLLER/ARDUINO_CI.md](../DESIGN-CONTROLLER/ARDUINO_CI.md) under a new "L072 static gates" subsection.

### 2.3 Failure-mode rehearsal (one-time, do **not** commit)

Before merging the CI PR, run a throwaway local test:

1. In a scratch branch, add a stray `sx1276_write_reg(SX1276_REG_OP_MODE, 0x81U)` line into `radio/sx1276.c`.
2. `make check-opmode-owner` must exit non-zero with the file:line of the stray write printed.
3. Revert. PR description must record this rehearsal with the exact failing output pasted.

This proves the guard isn't silently passing (the existing target uses `|| true` semantics in the grep stage to avoid grep's "no match" exit 1; the explicit `if [ -n "$$hits" ]` check is what fails the build — verify it actually does).

### 2.4 Single workflow vs. sibling workflow

**Recommendation: extend `arduino-ci.yml`.** Reasons:
- Same paths filter; no duplication.
- Job grouping in the GitHub PR UI keeps "all firmware checks" under one workflow row.
- Branch protection only needs one rule update.

Sibling workflow would be justified only if the L072 job needed a different runner image or a private toolchain — neither applies for `make check` + `make check-opmode-owner` + a host-compiled C binary.

### 2.5 Acceptance gates

1. PR with intentional opmode-owner violation fails the new job with non-zero exit.
2. Clean PR (no opmode violation) shows job green in <30 s wall time (host-only, no cross compile).
3. `firmware-l072-static-checks` listed as required check in branch protection settings (manual repo admin step — call it out in the PR description so the maintainer flips it).
4. ARDUINO_CI.md mentions the new job and its three sub-steps.

### 2.6 Risks

- **Branch-protection step is manual** (cannot be done from PR). Mitigate by including a one-line "TODO for repo admin" in the PR body. If unflipped, the check still runs and fails noisily — just doesn't block merge until the toggle.
- **Grep target's `|| true` masking.** The current Makefile recipe is correct (the `|| true` only swallows grep's exit-1-on-no-match; the `if [ -n "$$hits" ]` then promotes a non-empty match list to exit 1). Verify in §2.3 rehearsal.

---

## 3. B2 step-2 — policy lock-down + host-side contract tests

### 3.1 What "policy lock-down" means

The **behavior** is implemented in [host/host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c). What is missing is:

1. A canonical **table of expected behaviors** that humans and tests both consult.
2. A host-runnable test binary that asserts every row of that table against the real `cfg_set` / `cfg_get` / `cfg_validate_and_normalize` functions linked from the firmware tree.

Without (1), drift is invisible. Without (2), drift is undetectable until a hardware run.

### 3.2 Policy table (canonical — to be embedded in [04 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) and mirrored as a comment block at the top of `host_cfg.c`)

Columns: `key`, `wire len`, **OOR policy** (clamp / reject / unconstrained), **boundary cases that MUST round-trip**, **boundary cases that MUST return OUT_OF_RANGE**, **apply-hook outcome**, **dirty-bit effect**.

| Key | Len | OOR policy | Round-trip OK | Returns OUT_OF_RANGE | Apply outcome | Sets `cfg_dirty`? |
|---|---|---|---|---|---|---|
| `0x01 tx_power_dbm` | 1 (u8) | **clamp** to 2..17 | input 2,14,17 | none (clamp never rejects) | calls `sx1276_set_tx_power_dbm`, returns OK | yes if differs from default 14 |
| `0x02 tx_power_adapt_enable` | 1 (bool) | reject >1 | 0, 1 | 2..255 | no apply (RAM flag) | yes if differs from default |
| `0x03 lbt_enable` | 1 (bool) | reject >1 | 0, 1 | 2..255 | no apply | yes if differs |
| `0x04 lbt_threshold_dbm` | 1 (i8) | reject < -120 or > 0 | -120, -90, 0 | -128, +1, +127 | no apply | yes if differs from -90 |
| `0x05 fhss_enable` | 1 (bool) | reject >1 | 0, 1 | 2..255 | no apply | yes if differs from 1 |
| `0x06 fhss_quality_aware` | 1 (bool) | reject >1 → OOR | 0, 1 (range OK)... | 2..255 | ...but apply returns `APPLY_FAILED` (Phase 4+ deferred). RAM value reverts. | **no** (revert leaves prior) |
| `0x07 fhss_channel_mask` | 8 (u64) | reject all-zero mask | any non-zero u64 | `0x0000000000000000` | no apply | yes if differs from `0x00000000000000FF` |
| `0x08 deep_sleep_enable` | 1 (bool) | reject >1 | 0, 1 | 2..255 | apply returns `APPLY_FAILED` (Phase 5). RAM reverts. | **no** |
| `0x09 beacon_enable` | 1 (bool) | reject >1 | 0, 1 | 2..255 | no apply | yes if differs |
| `0x0A beacon_channel_idx` | 1 (u8) | reject >7 | 0..7 | 8..255 | no apply | yes if differs from 0 |
| `0x0B host_baud` | 4 (u32 LE) | reject not in {9600, 115200, 921600} | those three | 19200, 460800, 1000000 | no apply (DEFERRED). Returns `CFG_STATUS_DEFERRED`. | yes if differs from `HOST_BAUD_DEFAULT` |
| `0x0C replay_window` | 1 (u8) | reject 0 or >64 | 1, 16, 64 | 0, 65, 255 | no apply (bookkeeping for H7) | yes if differs from 16 |
| `0x0D iwdg_window_ms` | 2 (u16 LE) | reject <50 or >5000 | 50, 1000, 5000 | 0, 49, 5001, 65535 | no apply at this phase | yes if differs from `IWDG_RUN_WINDOW_MS` |
| `0x0E crypto_in_l072` | 1 (bool) | reject >1 | 0, 1 | 2..255 | apply returns `APPLY_FAILED` (Profile A locked). RAM reverts. | **no** |
| `0x80 protocol_version` | 1 (u8) | n/a (READ_ONLY) | n/a | n/a | `cfg_set` returns `CFG_STATUS_READ_ONLY` for any input | no |
| `0x81 wire_schema_version` | 1 (u8) | n/a (READ_ONLY) | n/a | n/a | `cfg_set` returns `CFG_STATUS_READ_ONLY` | no |
| `0x82 cfg_dirty` | 1 (bool) | n/a (READ_ONLY) | n/a | n/a | `cfg_set` returns `CFG_STATUS_READ_ONLY`; `cfg_get` returns live `s_cfg_dirty` | no |

Notes on subtle behaviors that the test must enforce:

- **Clamp ≠ silent acceptance.** `tx_power_dbm` is the **only** key that clamps. All other numeric keys reject. The test must distinguish `cfg_get` returning the **clamped** value from the **input** value.
- **Apply-failure dirty semantics.** When apply returns `APPLY_FAILED` (`fhss_quality_aware`, `deep_sleep_enable`, `crypto_in_l072`), the RAM value is restored, so `cfg_dirty` must **not** flip from a failed write — the test case asserts `cfg_is_dirty() == false` after the failed `cfg_set`.
- **DEFERRED ≠ failure.** `host_baud` write returns `CFG_STATUS_DEFERRED` (status code 5) and *does* update RAM and *does* set `cfg_dirty`. Reads after the deferred set must return the new value.
- **Bad length precedes range.** A `cfg_set` with the wrong length returns `BAD_LENGTH` even if the bytes would be in range — order of checks is `READ_ONLY` → `BAD_LENGTH` → `OUT_OF_RANGE` → `APPLY_FAILED`.
- **`cfg_get` for `cfg_dirty`** is **live**, not snapshotted from the value array — verify by setting a non-default value and reading back `cfg_dirty == 1` even though the descriptor's default `bytes` is 0.

### 3.3 Contract-test binary

New file: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_contract.c`.

Strategy:

- Compile **the production `host_cfg.c` itself** with the host C compiler. It only depends on `<stdbool.h>`, `<stddef.h>`, `<string.h>`, `host_cfg_keys.h`, `host_types.h`, `config.h`, and one symbol from `sx1276.h` (`sx1276_set_tx_power_dbm`).
- Provide a **mock** `sx1276_set_tx_power_dbm` in a new `bench/host_proto/sx1276_stub.c` that records the last value passed for assertion.
- Test driver iterates the policy table (encoded as a `static const` C struct array — keep it in sync with §3.2 manually; CI catches drift via test failure).

Rough test driver shape:

```
typedef struct case_s {
    uint8_t        key;
    const uint8_t *value;
    uint8_t        len;
    cfg_status_t   expect_status;
    bool           expect_get_equals_input;   // false for clamp & revert cases
    const uint8_t *expect_get_value;          // NULL means "= input"
    bool           expect_dirty_after;
} case_t;

static const case_t k_cases[] = {
    /* tx_power_dbm clamp-low */
    { CFG_KEY_TX_POWER_DBM, (uint8_t[]){0},  1, CFG_STATUS_OK, false,
      (uint8_t[]){2}, true },
    /* tx_power_dbm clamp-high */
    { CFG_KEY_TX_POWER_DBM, (uint8_t[]){30}, 1, CFG_STATUS_OK, false,
      (uint8_t[]){17}, true },
    /* tx_power_dbm in-range */
    { CFG_KEY_TX_POWER_DBM, (uint8_t[]){10}, 1, CFG_STATUS_OK, true,
      NULL, true },
    /* lbt_threshold rejects below floor */
    { CFG_KEY_LBT_THRESHOLD_DBM, (uint8_t[]){(uint8_t)-128}, 1,
      CFG_STATUS_OUT_OF_RANGE, false, NULL, false },
    /* host_baud deferred-but-stored */
    { CFG_KEY_HOST_BAUD, (uint8_t[]){0x00,0xE1,0xF5,0x00}, 4,   /* 921600 */
      CFG_STATUS_DEFERRED, true, NULL, true },
    /* fhss_quality_aware in-range but apply fails -> revert, no dirty */
    { CFG_KEY_FHSS_QUALITY_AWARE, (uint8_t[]){1}, 1,
      CFG_STATUS_APPLY_FAILED, false, /* expect default */ NULL, false },
    /* protocol_version is READ_ONLY */
    { CFG_KEY_PROTOCOL_VERSION, (uint8_t[]){42}, 1, CFG_STATUS_READ_ONLY,
      false, NULL, false },
    /* unknown key */
    { 0xFE, (uint8_t[]){0}, 1, CFG_STATUS_UNKNOWN_KEY, false, NULL, false },
    /* bad length precedes range */
    { CFG_KEY_TX_POWER_DBM, (uint8_t[]){0,0}, 2, CFG_STATUS_BAD_LENGTH,
      false, NULL, false },
    ...one row per row in §3.2 table, both pass and fail boundary...
};

int main(void) {
    cfg_init();
    for (size_t i = 0; i < ARRLEN(k_cases); ++i) {
        run_case(i, &k_cases[i]);
        cfg_init();   // reset between cases
    }
    return g_failures ? 1 : 0;
}
```

Each `run_case`:
1. Calls `cfg_set(c->key, c->value, c->len)`.
2. Asserts return == `c->expect_status`.
3. Calls `cfg_get(c->key, buf, sizeof buf, &len)`.
4. Compares `buf` to `c->expect_get_value` (or `c->value` if `expect_get_equals_input`).
5. Asserts `cfg_is_dirty() == c->expect_dirty_after`.
6. Prints a `[FAIL] case N: <key> <reason>` line and bumps `g_failures`.

### 3.4 Wire-vector tests (separate from §3.3 unit tests)

Also under `bench/host_proto/`, add `cfg_wire_vectors.c` that asserts the **byte-for-byte** wire encoding of the expected `CFG_OK_URC` and `CFG_DATA_URC` payloads for representative requests. These vectors become the H7 driver's contract.

Each vector is a `(REQ bytes) -> (URC bytes)` pair. Encode them as inline arrays:

```
/* CFG_GET 0x01 -> CFG_DATA_URC {0x01, 0x01, 14} */
static const uint8_t k_get_tx_power_req[]  = { 0x01 };
static const uint8_t k_get_tx_power_urc[]  = { 0x01, 0x01, 0x0E };

/* CFG_SET 0x01 0x12 (out of range hi -> clamps to 17) -> CFG_OK {key, OK, len=1, 0} */
static const uint8_t k_set_clamp_req[]     = { 0x01, 0x01, 0x12 };
static const uint8_t k_set_clamp_urc[]     = { 0x01, 0x00, 0x01, 0x00 };

/* CFG_SET 0x80 ... -> CFG_OK {0x80, READ_ONLY=6, 0, 0} */
static const uint8_t k_set_ro_urc[]        = { 0x80, 0x06, 0x00, 0x00 };
```

The test calls a **direct serializer** (a small wrapper around `cfg_set`/`cfg_get` mirroring `handle_cfg_set` / `handle_cfg_get` in [host/host_cmd.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c)) and `memcmp`s. **Do not** copy/paste the host_cmd handlers — extract the URC payload-builder helpers into `host/host_cfg_wire.c` (single-source) and call from both production and test.

### 3.5 Makefile target

Add to `Makefile` (under the existing `check-*` targets):

```
HOST_CC ?= cc
CFG_CONTRACT_BIN := $(BUILD)/cfg_contract
CFG_WIRE_BIN     := $(BUILD)/cfg_wire_vectors

check-cfg-contract: check-cfg-contract-unit check-cfg-contract-wire

check-cfg-contract-unit: $(BUILD)
	@echo "[CFG] Building host contract test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. \
	    bench/host_proto/cfg_contract.c \
	    bench/host_proto/sx1276_stub.c \
	    host/host_cfg.c \
	    -o $(CFG_CONTRACT_BIN)
	@echo "[CFG] Running ..."
	@$(CFG_CONTRACT_BIN)

check-cfg-contract-wire: $(BUILD)
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. \
	    bench/host_proto/cfg_wire_vectors.c \
	    bench/host_proto/sx1276_stub.c \
	    host/host_cfg.c \
	    host/host_cfg_wire.c \
	    -o $(CFG_WIRE_BIN)
	@$(CFG_WIRE_BIN)
```

Both binaries return non-zero on any failed assertion. CI invokes `make check-cfg-contract` after `make check-opmode-owner`.

### 3.6 Acceptance gates

1. `make check-cfg-contract` passes locally.
2. Removing one validator branch (e.g. delete the `LBT_THRESHOLD_DBM` range check) makes a contract test fail with a clearly named case.
3. Bumping a default value in `k_cfg_desc` without updating the test table makes the dirty-bit assertions fail (catches drift).
4. `host/host_cfg_wire.c` is the **only** file that emits CFG URC byte layouts; `host/host_cmd.c` calls into it. CI grep guard analogous to OpMode: `grep -RIn 'HOST_TYPE_CFG_OK_URC\|HOST_TYPE_CFG_DATA_URC' host/*.c | grep -v 'host/host_cfg_wire.c\|host/host_cmd.c'` returns empty.
5. CFG contract test runs in <2 s on a GHA ubuntu-latest runner.

### 3.7 Risks

- **Endianness mismatch on the test host vs. target.** The validators use explicit `read_u32_le` / `read_u16_le` so endianness on host doesn't matter; the test uses literal LE byte arrays so the assertions stay correct on big-endian hosts (none in CI today, but the discipline costs nothing).
- **`config.h` pulling in MCU-only headers.** If `config.h` `#include`s anything platform-specific, the host build breaks. Inspect once; if needed, split the LoRa-firmware-only constants into `include/lora_fw_config.h` (consumed by both target and host) and keep MCU-only bits behind `#ifdef LIFETRAC_MCU_BUILD`.
- **`sx1276_stub.c` stale.** When new apply-hooks land in later tranches, the stub must mock those too. Add a header comment in the stub listing exactly which symbols it provides; reviewer checks the list when adding hooks.

---

## 4. C2 — RX-path wiring to real counter increments

### 4.1 Current state and what's missing

- `sx1276_modes_to_rx_cont()` exists and correctly programs DIO map + OpMode for RX.
- `sx1276_take_irq_events()` returns the OR of DIO0..3 events captured by EXTI handlers (no SPI in IRQ — events are just GPIO-edge counters).
- Main loop already calls `sx1276_take_irq_events()` and `host_cmd_on_radio_events()`.
- **Nothing reads `RegIrqFlags` after a DIO0 edge.** That is the C2 work: a polled FSM that, on `SX1276_EVT_DIO0`, reads `RegIrqFlags`, distinguishes `RxDone+CRC ok`, `RxDone+CRC err`, `ValidHeader`, and increments the appropriate STATS counter, then emits `RX_FRAME_URC`.

### 4.2 SX1276 RX decision tree (datasheet §4.1.6 + §6.4)

After radio is in `RXCONTINUOUS` and DIO0 fires (LoRa default DIO mapping `DIO0=RxDone`):

```
read RegIrqFlags (0x12)
├── PayloadCrcError (bit 5) == 1
│      → CRC error path:
│        - host_stats_radio_crc_err()
│        - clear PayloadCrcError + RxDone in RegIrqFlags (write 1 to clear)
│        - DO NOT emit RX_FRAME_URC (frame is garbage)
│        - radio stays in RXCONTINUOUS — no mode transition needed
│
├── PayloadCrcError == 0 AND RxDone (bit 6) == 1
│      → Good-frame path:
│        - read RegRxNbBytes (0x13)
│        - read RegFifoRxCurrentAddr (0x10), set RegFifoAddrPtr (0x0D)
│        - burst-read RegFifo (0x00) for nbBytes
│        - read RegPktSnrValue (0x19), RegPktRssiValue (0x1A)
│        - host_stats_radio_rx_ok()
│        - emit RX_FRAME_URC{ payload, snr, rssi, rx_timestamp_us }
│        - clear RxDone in RegIrqFlags
│
└── neither bit set (spurious edge / out-of-order DIO)
       → log nothing, clear all flags as a safety net
```

Two subtleties:

- **CRC-on-payload bit must be set in `RegHopChannel.RxPayloadCrcOn`** for `PayloadCrcError` to be meaningful. C2 must verify after `sx1276_init` that the modem config the firmware programs has CRC enabled (`RegModemConfig2.RxPayloadCrcOn = 1`). If CRC isn't on, every frame counts as `rx_ok` and `crc_err` stays zero — silently masking real corruption. **Acceptance gate** below covers this.
- **`ValidHeader` (bit 4) is informational only.** Don't increment any counter on it; it fires before `RxDone` for every frame whose header decodes. Counting it would double-count or pre-count.

### 4.3 New module

| Path | Responsibility |
|---|---|
| `radio/sx1276_rx.c` + `include/sx1276_rx.h` | Polled RX FSM: arm/disarm RX, on DIO0 event read `RegIrqFlags` + FIFO, dispatch CRC/OK paths, clear flags. |

API (minimal):

```
typedef struct sx1276_rx_frame_s {
    uint8_t  payload[256];
    uint8_t  length;
    int8_t   snr_db;       /* RegPktSnrValue / 4 */
    int16_t  rssi_dbm;     /* -157 + RegPktRssiValue (HF port) */
    uint32_t timestamp_us; /* from platform_now_us() at DIO0 service */
} sx1276_rx_frame_t;

bool sx1276_rx_arm(void);                                   /* enters RXCONTINUOUS */
void sx1276_rx_disarm(void);                                /* back to STANDBY */
bool sx1276_rx_service(uint32_t events,
                       sx1276_rx_frame_t *out_frame);       /* returns true if out_frame populated */
```

`sx1276_rx_service` is what increments the counters. It is called **only from the main loop**, with the events bitmask pulled from `sx1276_take_irq_events()`.

### 4.4 Main-loop integration

Modify [main.c](../DESIGN-CONTROLLER/firmware/murata_l072/main.c):

```c
sx1276_rx_arm();   // after sx1276_init() returns true

for (;;) {
    host_uart_poll_dma();
    while (host_uart_pop_frame(&frame)) {
        host_cmd_dispatch(&frame);
    }

    const uint32_t radio_events = sx1276_take_irq_events();
    host_cmd_on_radio_events(radio_events);          /* DIO counters */

    sx1276_rx_frame_t rx;
    if (sx1276_rx_service(radio_events, &rx)) {
        host_cmd_emit_rx_frame(&rx);                  /* new helper, builds RX_FRAME_URC */
    }

    cpu_wfi();
}
```

`host_cmd_emit_rx_frame` is a new thin wrapper in `host/host_cmd.c` that COBS-encodes a `RX_FRAME_URC (0x91)` with payload layout `{ u8 length, u8 snr_db, i16 rssi_dbm_le, u32 timestamp_us_le, payload[length] }`. Document the layout in `include/host_types.h` next to the existing URC type IDs and add the wire-vector test pattern.

### 4.5 Counter-increment placement (the "real RX decision points")

To make the increments unambiguous and reviewable:

- `host_stats_radio_rx_ok()` is called in **exactly one place**: inside `sx1276_rx_service`, on the success branch, *after* the FIFO read succeeds and *before* returning `true`. Not on `ValidHeader`. Not on emitting the URC (separation: stats reflect radio truth, URCs reflect host delivery).
- `host_stats_radio_crc_err()` is called in **exactly one place**: inside `sx1276_rx_service`, on the `PayloadCrcError == 1` branch, *before* clearing flags.
- Neither setter is ever called from any IRQ context (per [05 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md)).

Add a CI grep (analogous to OpMode owner):

```makefile
check-rx-counter-owner:
	@hits=$$(grep -RIn "host_stats_radio_rx_ok\|host_stats_radio_crc_err" \
	         radio/ host/ main.c | grep -v "radio/sx1276_rx.c" || true); \
	if [ -n "$$hits" ]; then \
	  echo "[FAIL] RX counters must only be incremented in radio/sx1276_rx.c"; \
	  echo "$$hits"; exit 1; fi; \
	echo "[OK] RX counter ownership check passed"
```

Wire into CI alongside `check-opmode-owner`.

### 4.6 Bench rehearsal (no hardware required)

Add `bench/radio/rx_fsm_dryrun.c` — a tiny host-compilable harness that mocks `sx1276_read_reg` / `sx1276_read_burst` to feed scripted `RegIrqFlags` sequences into a refactored `sx1276_rx_service_with_io(io_ops *)` variant. Tests:

1. CRC-error-then-good-frame sequence → counters: `crc_err=1`, `rx_ok=1`. `RX_FRAME_URC` emitted exactly once.
2. ValidHeader-only edge (RxDone never sets) → counters unchanged, no URC.
3. RxDone+CRC-good on zero-length payload → `rx_ok=1`, URC payload length = 0.
4. Two back-to-back RxDone events serviced in one main-loop tick → both counted.

To support this, factor `sx1276_rx_service` to take an `io_ops` struct in production too (real impl) so the test can swap. **This is the only refactor C2 introduces to existing files** — keeps blast radius small.

### 4.7 Out of scope for C2

- TX path (`tx_ok`) — that's C3.
- LBT (`tx_abort_lbt`) — that's C5.
- FHSS hopping on `FhssChangeChannel` IRQ (DIO1 in some mappings) — Phase 3.
- SNR/RSSI calibration tables for the LF port — bench works with HF only at 915 MHz.

### 4.8 Acceptance gates

1. `make check-cfg-contract && make check-opmode-owner && make check-rx-counter-owner` all pass.
2. `bench/radio/rx_fsm_dryrun` passes all 4 scripted sequences.
3. After flashing on bench with a known-good loopback transmitter, `STATS_DUMP` shows `radio_rx_ok` incrementing 1:1 with delivered frames over a 60-second soak.
4. Deliberately CRC-corrupting transmitter: `radio_crc_err` increments 1:1 with bad frames, `radio_rx_ok` does not move.
5. After verifying CRC actually arrives at the modem (`sx1276_read_reg(0x1E) & 0x04 != 0` → RxPayloadCrcOn), document the CRC-on assertion in `sx1276_rx_arm()` and **fault the boot** if the modem is configured CRC-off (because then `crc_err` is structurally meaningless).
6. `RX_FRAME_URC` is emitted exactly once per good frame; payload length matches `RegRxNbBytes`.

### 4.9 Risks

- **`PayloadCrcError` latch.** Per datasheet, the CRC-error flag is sticky until cleared. The dryrun test must include a "two CRC errors in a row without explicit clear between events" case to verify the FSM clears flags before re-arming poll.
- **FIFO pointer race.** Reading `RegFifoRxCurrentAddr` then setting `RegFifoAddrPtr` must happen with the modem in RXCONTINUOUS — no mode switch in between. If C2 ever needs to drop to STANDBY between RX events, the pointer must be re-set.
- **`platform_now_us()` precision.** RX timestamp is captured at *service* time, not at *DIO0 edge* time. For Phase 2 host RX-rate metering this is fine (resolution: main-loop period, ~tens of µs). When precision matters (Phase 4 FHSS quality), the EXTI handler should snapshot `platform_now_us()` into a per-line buffer; for now, document the sloppy timestamp in the URC field as "service time, not edge time."
- **256-byte stack frame.** `sx1276_rx_frame_t` carries 256 B of payload. Putting it on the main-loop stack is acceptable (main stack reserved at 2.5 KB per the memory map). Double-check by adding a `_Static_assert(sizeof(sx1276_rx_frame_t) <= 280, "rx frame fits");` so a future field addition forces a stack-budget conversation.

---

## 5. Combined branch sequencing & PR plan

| # | Branch | Diff size | Reviewer focus |
|---|---|---|---|
| 1 | `chore/ci-opmode-owner` | ~30 lines (1 GHA job, 1 docs paragraph) | Verify `check-opmode-owner` actually fails on intentional violation (paste rehearsal output in PR body). |
| 2 | `feat/host-cfg-policy-tests` | ~600 lines (4 new files: `host_cfg_wire.c/h`, `cfg_contract.c`, `cfg_wire_vectors.c`, `sx1276_stub.c`) + Makefile + GHA tweak | Walk every row of the §3.2 table against the test table to confirm no row is missing. Validate the `host_cfg_wire.c` extraction (no behavior change to `host_cmd.c` payloads). |
| 3 | `feat/radio-rx-c2` | ~400 lines (`sx1276_rx.c/h`, `rx_fsm_dryrun.c`, `host_cmd_emit_rx_frame` helper, main.c hook, Makefile target) | Counter call-site discipline (only in `sx1276_rx.c`); CRC-on-modem assertion; bench dryrun coverage. |

---

## 6. Companion Analysis (Gemini 3.1 Pro Preview, 2026-05-05)

**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)
**Purpose:** Technical validation of Tranche 3 test strategies, RX State Machine, and CI constraints. (Note: Assuming "Tranche 4" in the prompt was a typo for this Tranche 3 document).

### 6.1 CI OpMode Guard (Work Item 1)
- **Regex vs AST Constraints:** Using `grep` to enforce single ownership of the `RegOpMode` register is a brilliant, lightweight "poka-yoke" (mistake-proofing) technique. 
- **Risk:** A regex check can be intentionally circumvented using macros or pointer indirection. However, in a tightly-controlled bare-metal C environment, this grep target provides high ROI. **Recommendation:** Ensure all developers are aware that wrapping or obscuring `SX1276_REG_OP_MODE` is strictly prohibited by convention, backing up the automated check.

### 6.2 Host-Side Contract Testing (Work Item 2)
- **Binary Equivalence:** Compiling the *exact* target `host_cfg.c` on the host PC using a mocked radio (`sx1276_stub.c`) is an elite firmware testing strategy. It strictly isolates the configuration bounds checking, serialization, and URC formatting from hardware nuances.
- **Endianness Safety:** Providing explicit `read_u32_le` structures ensures the protocol testing remains deterministic regardless of the host PC's native architecture.

### 6.3 RX Polled FSM vs Interrupts (Work Item 3)
- **SPI Isolation:** Processing the SX1276 FIFO via a polled FSM inside the main loop—rather than immediately within an external interrupt (EXTI)—is precisely the right pattern. It prevents the 32MHz Cortex-M0+ from stalling in an SPI wait-state, totally protecting the high-speed 921600 baud UART DMA ingest path.
- **Timestamp Accuracy (Phase 4 Hook):** The plan explicitly notes that `rx_timestamp_us` is captured at *service* time rather than *DIO0 edge* time. While acceptable for Phase 2/3, this jitter (often 10–50 µs depending on UART polling delays) will disrupt Phase 4 slotted TDMA/Time-Sync. **Recommendation for Phase 4:** The EXTI ISR for DIO0 should latch `platform_now_us()` into an intermediate volatile buffer *immediately*, which the `sx1276_rx_service()` FSM can later pick up for precise time-of-air alignment.
- **Hardware CRC Checking:** Validating `RegHopChannel.RxPayloadCrcOn` at boot prevents silent failures. Enforcing that `crc_err` is meaningful guarantees the application layer is never fed corrupt framing.

PR #1 is mandatory before #2 and #3; #2 and #3 can land in either order but **#2 should precede #3** so the wire schema for `RX_FRAME_URC` (added by #3) lands with contract tests already gating CFG so reviewers aren't juggling two new test infrastructures at once.

---

## 6. Out of scope (do NOT land in this tranche)

- TX path implementation (C3) — separate tranche; counter slot already wired with no caller.
- LBT (C5).
- FHSS hopping logic.
- CFG persistence to Flash (Phase 6).
- Quality-aware FHSS (N-06, Phase 4+).
- Bumping `HOST_PROTOCOL_VER` or `HOST_WIRE_SCHEMA_VER` — additions in this tranche are payload-additive on already-declared URC types, no schema bump.
- Cross-compile gate (`make all` in CI). The `arm-none-eabi-gcc` toolchain isn't yet pinned in CI — keep this tranche host-only-CI to avoid coupling. Phase 1 close-out PR will add the cross gate.

---

## 7. One-line summary

> **Land the `firmware-l072-static-checks` GHA job first so `make check-opmode-owner` (and the new `make check-cfg-contract` from PR #2) become required gates; then ship a host-compilable CFG contract + wire-vector test suite (driven by an explicit `(key, value, expected_status, expected_dirty, expected_get)` table mirroring the canonical §3.2 policy, with `host/host_cfg_wire.c` extracted as the single CFG-URC byte-layout author); then implement the polled `radio/sx1276_rx.c` RX FSM whose `sx1276_rx_service` is the *only* place `host_stats_radio_rx_ok()` and `host_stats_radio_crc_err()` are ever called — branch is decided by reading `RegIrqFlags` after a DIO0 edge, with a CI grep guard to enforce sole ownership and a bench dryrun to validate the CRC-error / good-frame / spurious-edge / back-to-back paths without hardware.**
