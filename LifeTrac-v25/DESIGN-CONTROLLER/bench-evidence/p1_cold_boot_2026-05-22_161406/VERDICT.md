# P1 cold-boot — Phase 2.3 ROOT-CAUSE VERDICT (2026-05-22 T2)

## Verdict

**STALE FLASHED FIRMWARE.** The L072 image currently on the chip does
not contain `CFG_KEY_REG_PROFILE = 0x14U` in its cfg descriptor table.
`cfg_get(0x14)` returns `CFG_STATUS_UNKNOWN_KEY`, the handler emits
`CFG_OK_URC` (0xA0), the host correlator waits for `CFG_DATA_URC`
(0xA1) → silent `TimeoutError`. Definitive — 5/5 cycles
byte-identical forensic evidence below.

## Forensic evidence (5/5 cycles identical)

Phase 2.3 added a post-timeout dump in `emit_runtime_profile_enum`
that pulls remaining bytes off the wire and dumps `link.urc_queue`:

```
__PROFILE_TIMEOUT_QUEUE_COUNT__=0
__PROFILE_TIMEOUT_LATE_COUNT__=1
__PROFILE_TIMEOUT_RX_BUF_HEX__=
__PROFILE_TIMEOUT_FRAME__[0]=wire type=0xA0 seq=3 payload=14010000
```

Decoding the frame:
- `type=0xA0` = `HOST_TYPE_CFG_OK_URC` (NOT STATS_URC = 0xC1).
- `seq=3` matches the third send in the link's send-counter (one of
  the three Phase 2.2 backoff attempts).
- `payload=14 01 00 00` parsed under
  `host_cfg_wire_encode_ok(key, status, actual_len, pad)`:
  - `key = 0x14`  = `CFG_KEY_REG_PROFILE`
  - **`status = 0x01` = `CFG_STATUS_UNKNOWN_KEY`** ← the smoking gun
  - `actual_len = 0x00`
  - `pad = 0x00`

The host is waiting for `HOST_TYPE_CFG_DATA_URC = 0xA1U`, so
`link.request()` rejects the `0xA0` frame (it goes to `urc_queue`,
then is consumed by the next `drain_pending` between backoff
attempts — that's why `QUEUE_COUNT=0` and `LATE_COUNT=1` after the
final attempt's response arrived too late to be drained before the
timeout fired).

## Why the prior verdict updates were wrong

1. **Phase 2.0** "firmware-not-ready" — falsified by dwell-invariance
   (0.05 s → 5.0 s).
2. **Phase 2.1** "host-race (URCs poison correlator)" — plausible
   from the visible `drained=2` and AT+VER success, but didn't
   explain WHY only CFG_GET fails.
3. **Phase 2.2** drain+backoff fix — landed cleanly, 8/8 self-test
   PASS, instrumentation confirmed drains AND retries worked → STILL
   5/5 timeouts. Falsified the host-race hypothesis.
4. **2026-05-22 16:10** "stale firmware (via `fw=v0.0.0 build=0`
   metadata)" — directionally correct CONCLUSION reached from WRONG
   evidence. The version string is a fixed source artifact
   (`include/version.h` defines all majors/minors/patches to 0,
   `FW_GIT_SHA = "dev"`), so any build of current source reports
   that string. Cannot be used as a staleness discriminator.
5. **Phase 2.3** (this verdict) — direct forensic capture of the
   firmware's actual response provides unambiguous proof:
   `CFG_STATUS_UNKNOWN_KEY` for key `0x14`.

## Why same-boot AT+VER and W1-10 REG_READ/WRITE work

Those code paths existed in the firmware long before the
2026-05-20 FCC-B3-1 CFG_KEY_REG_PROFILE addition. They're in the
flashed image; CFG_KEY_REG_PROFILE descriptor is not.

## Source vs flashed image mismatch

- `include/host_cfg_keys.h:24` — defines `CFG_KEY_REG_PROFILE 0x14U` ✓
- `host/host_cfg.c:130` — descriptor entry for `0x14` ✓
- `host/host_cmd.c:606` — `handle_cfg_get` dispatches correctly ✓
- `host/host_cfg.c:414` — `cfg_get` returns `CFG_STATUS_OK` if key
  is found, else `CFG_STATUS_UNKNOWN_KEY` ✓

All source is correct. The **flashed binary is older than the source**.

## Required next step (HARDWARE)

Re-flash the L072 with the latest firmware build, then re-run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File `
  .\LifeTrac-v25\tools\p1_cold_boot_discriminator.ps1 `
  -RxSerial 2D0A1209DABC240B -Cycles 5 -PreProbeSleepS 0.05
```

Expected post-re-flash:
- `RUNTIME_PROFILE_ENUM=<0|1|2>` (e.g., `=1` for FCC_FHSS) on all
  cycles.
- `__PROFILE_DRAINED__ ∈ {0, 1, 2}`, `__PROFILE_ATTEMPTS__=1`.
- No `__PROFILE_TIMEOUT_FRAME__` lines emitted at all.

If post-re-flash STILL produces `__PROFILE_TIMEOUT_FRAME__[0]=wire
type=0xA0 seq=*` with `status=0x01`, escalate: descriptor entry was
removed/commented out, OR `cfg_init` ran before `host_cfg_keys`
were linked in, OR the build hash didn't actually change (verify
`fw=v0.0.0 build=<NON-ZERO>` after a build-id injection patch).

## Secondary anomaly (low-priority follow-up)

The boot drain logs show two unsolicited `type=0xA0 seq=1` and
`seq=2` frames arriving at cold boot, BEFORE the probe sends any
CFG request. CFG_OK_URC is supposed to be a response to
CFG_GET/CFG_SET, not unsolicited. Possibilities:
- Firmware emits CFG_OK_URC during `cfg_init()` (unlikely — should
  not echo without a request).
- Carry-over bytes from prior process flushed at NRST exit (would
  not have matching seq increments though).
- LPUART RX buffer holds stale bytes that survive NRST (host-side
  not L072 reset semantics).

Should be revisited after re-flash if the seq=1,2 frames persist.

## Files

- This verdict: `bench-evidence/p1_cold_boot_2026-05-22_161406/VERDICT.md`
- 5/5 cycle logs: `bench-evidence/p1_cold_boot_2026-05-22_161406/cycle_0[1-5]_stdout.txt`
- Patched probe (forensic dump added):
  `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py`
  function `emit_runtime_profile_enum`
- Firmware source (correct, just needs flashing):
  - `firmware/murata_l072/include/host_cfg_keys.h` (line 24)
  - `firmware/murata_l072/include/host_types.h` (line 51-52 — note
    OK=0xA0, DATA=0xA1 — STATS is 0xC1)
  - `firmware/murata_l072/host/host_cfg_wire.c` (encode_ok layout)
  - `firmware/murata_l072/host/host_cfg.c` (descriptor + cfg_get)
  - `firmware/murata_l072/host/host_cmd.c` (handle_cfg_get)
- Prior (now-superseded) verdicts:
  - `bench-evidence/p1_cold_boot_2026-05-22_160255/VERDICT.md` (Phase 2.1 host-race)
  - `bench-evidence/p1_cold_boot_2026-05-22_160757/VERDICT.md` (Phase 2.2 falsification)

*Signed:* **GitHub Copilot, Phase 2.3 root-cause verdict 2026-05-22 16:15**
