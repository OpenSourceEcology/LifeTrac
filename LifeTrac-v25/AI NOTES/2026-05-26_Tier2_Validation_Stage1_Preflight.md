# 2026-05-26 — Tier 2 Validated: Stage1 Preflight + Wrapper Hardening

## Summary
Tier 1 (pure-Python observability) and Tier 2 (Stage1 wrapper hardening) edits
from the 2026-05-26 blocker triage are applied, validated on hardware, and
have **moved the bench failure mode**.

| Layer | Before (v1.0 baseline) | After Tier 1+2 |
|---|---|---|
| Wifi air-gap | tractor on `MyWifi` | **disabled** (rfkill block + NM unmanaged + masked wpa_supplicant + oneshot disable-wifi.service) |
| ERR_PROTO `1001080000` | printed as raw hex | decoded → `offending_type=0x10 offending_ver=1 err_code=0x08(FORBIDDEN) detail=0` |
| 0xC3 RFCO_PERTX URC (21-byte) | dropped as "unrelated" | decoded → `tx_status=0x04(ABORT_QOS) …` |
| `run_flash_l072.sh` ttymxc3-held bail | comment-only no-op | exits 90 if device still held after evict |
| Stage1 per-cycle SWD DPIDR | `0xdeadbeef` after a contaminated cycle | **clean `0x6ba02477` every cycle** |
| Stage1 per-cycle observability | none | `PREFLIGHT_HOLDERS=` / `PREFLIGHT_OPENOCD=` / `PREFLIGHT_GPIO10=1` |
| FAIL_SYNC vs FAIL_ID | mixed (residue-dependent) | **isolated, repeatable FAIL_ID** every cycle |

## Validation evidence
- WiFi: `adb shell ping -c 2 8.8.8.8` → `Network is unreachable`; nmcli WIFI=disabled; ip route shows no default gateway.
- Tier 1 decoders: validated by `py` launcher against synthetic payloads (FORBIDDEN/detail=0, FORBIDDEN/detail=5, ABORT_QOS).
- Tier 2 wrapper: bench-evidence dir
  `T6_stage1_standard_quant_2026-05-26_171522_*` — three consecutive cycles,
  all preflight HOLDERS/OPENOCD empty, GPIO10=1, openocd `SWD DPIDR 0x6ba02477`,
  Phase A–D GPIO checks green.
- Mechanism behind shift: prior runs had cycle-N inherit ttymxc3 holders +
  stale openocd from cycle N-1, contending for the H7 SWD pins and giving
  `SWD DPIDR 0xdeadbeef`. Per-cycle preflight evicts holders, kills openocd,
  re-exports gpio8/10/15, and drives NRST high before the next attempt.

## Implementation notes / gotcha
- First attempt embedded the preflight body inline as
  `bash -c "...; echo PREFLIGHT_HOLDERS=\"$HOLD\"; ..."`. The embedded `"`
  characters truncated the bash double-quoted argument at the host shell
  layer, leaving only `pkill -9 -f ...` reaching bash with mangled args.
  Launcher.log showed either an empty preflight block or `pkill: no matching
  criteria specified`.
- Fix: extracted preflight body to
  `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/per_cycle_preflight.sh`,
  pushed with the helper toolkit, invoked as
  `echo fio | sudo -S -p '' bash /tmp/lifetrac_p0c/per_cycle_preflight.sh`.
  Zero shell-quoting surface area.

## Residual blocker (NEW, downstream of Tier 2)
SWD + preflight are clean; the next failure layer is the STM32L072 ROM
bootloader itself not ACKing the AN3155 0x7F autobaud byte. Evidence
(cycle 1 flash_run.log):
```
Opening /dev/ttymxc3 at 19200 8E1...
  (termios unavailable; relying on prior stty configuration)
Connecting to Bootloader...
Sending sync byte 0x7F...
Timeout waiting for ACK
Sync failed (got NACK or Timeout). The autobaud might already be locked.
Attempting to proceed anyway...
Sending GET command (0x00 0xFF)...
BlockingIOError: [Errno 11] Resource temporarily unavailable
```
- `SYNC_OK=1` in summary.txt is computed from the OpenOCD `READY:` banner,
  **not** from a real AN3155 ACK. Treat it as a "we got to the hold loop"
  signal only; cross-check `flash_run.log` for actual sync.
- `(termios unavailable; relying on prior stty configuration)` from the
  flasher is the first thing to chase: if the prior `stty` configured
  19200 8E1, but the actual UART format is something else, the L072 ROM
  bootloader will silently ignore the 0x7F byte.
- Plausible mechanisms for the AN3155 silence (NOT yet falsified — see
  user methodology note):
  1. UART is not actually 19200 8E1 on /dev/ttymxc3 at probe time (stty
     pre-config drifted or another writer reset it). Verify with
     `stty -F /dev/ttymxc3 -a` immediately before flasher run.
  2. L072 BOOT0 pin is not actually high at the L072 chip's BOOT0
     electrical input despite H7 PA11 being HIGH (wiring/level-shifter
     issue or wrong physical pin selected in `07_assert_pa11_pf4_long.cfg`).
     Falsification: probe BOOT0 with a meter during the 600s hold window.
  3. NRST timing too short — Phase C uses 250 ms low. STM32L072 ROM
     bootloader datasheet calls for ≥t_NRST + a few ms before BOOT0 is
     sampled. Should be fine but worth a 500 ms / 1 s comparative cycle.
  4. The L072 has a flashed firmware that re-disables the bootloader entry
     via option bytes (nBOOT_SEL / nBOOT0). Check option bytes with
     openocd `stm32l0x option read 0` on the H7 SWD chain — though here
     we're talking to the L072, not the H7, so option-byte read needs a
     direct L072 SWD path.
- Note: cycles 1 & 3 reported VERIFY_OK=1 / BOOT_OK=1 even with
  GETID_OK=0 / ERASE_OK=0 / WRITE_OK=0 / FAIL_ID. That's a bug in the
  contract script's verify/boot accounting and should not be treated as
  a green signal. Filed implicitly — flag for a follow-up patch in
  `run_stage1_standard_contract.sh`.

## Tier ranking re-evaluation
- Tier 1 (decoder fixes) — DONE, validated.
- Tier 2 (wrapper hardening + script-file preflight) — DONE, validated.
- Tier 3 (sx1276 refusal enum) — DEFERRED. Requires successful Stage1
  flash gate (the residual blocker above) to test end-to-end. Refusal-enum
  patch is still the right next firmware change once flash works, because
  it converts the four false-return paths in `sx1276_tx_begin()` into
  distinguishable ERR_PROTO `detail=` codes.
- New Tier 2b: investigate L072 ROM bootloader sync silence. Highest-value
  next experiment is a `stty -F /dev/ttymxc3 -a` capture in the helper
  toolkit right before the flasher runs, with the output mirrored into
  `flash_run.log`. Falsifies hypothesis (1) cheaply.

## Files touched in this session
- `disable_wifi.sh` (new, workspace root, archive of WiFi-disable op)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py` — ERR_PROTO 5/6-byte decoder.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py` — RFCO_PERTX 0xC3 21-byte decoder.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh` — hard bail if /dev/ttymxc3 still held after evict.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1` — per-cycle preflight invocation.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/per_cycle_preflight.sh` — NEW preflight body as standalone script.
