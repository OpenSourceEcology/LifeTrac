# 2026-05-25 — `rx_frames=0` Root Cause: TX and RX on Different LoRa Channels

## TL;DR

The L072 SX1276 driver firmware, when activated with `REG_PROFILE=1`
("FCC_15_247_FHSS_50CH_BW250") and a wide 50-channel FHSS channel mask
(`0x0003FFFFFFFFFFFF`), **independently picks a starting channel per
board** — even though `RegHopPeriod=0` (no per-packet hopping is
actually performed once a channel is chosen).

The two peers therefore sit on different fixed carrier frequencies and
cannot decode each other:

| Field                | TX peer (2E2C)   | RX peer (2D0A)   | Match    |
| -------------------- | ---------------- | ---------------- | -------- |
| **`freq_mhz`**       | **918.25**       | **915.00**       | **DIFF** |
| `sf`                 | 7                | 7                | OK       |
| `bw_hz`              | 250000           | 250000           | OK       |
| `cr`                 | 4/5              | 4/5              | OK       |
| `sync_word`          | 0x12             | 0x12             | OK       |
| `preamble_len`       | 8                | 8                | OK       |
| `implicit_header`    | False            | False            | OK       |
| `rx_payload_crc_on`  | True             | True             | OK       |
| `invert_iq_rx/tx`    | False / True     | False / True     | OK       |
| `hop_period_symbols` | 0                | 0                | OK       |
| `fhss_enabled`       | False            | False            | OK       |

(Decoded from `LifeTrac-v25/AI NOTES/2026-05-25_radio_state_dump_*.log`,
produced by `radio_state_dump.py` + `run_radio_state_dump.ps1`.)

A 3.25 MHz carrier gap with BW = 250 kHz is ~13 channels apart. The RX
receiver has zero chance of decoding the TX transmissions — exactly
matching the observed `rx_frames=0` while TX reports 200/200 fragments
`TX_DONE=OK`.

## Why the regulatory-profile call is not sufficient

`configure_regulatory_profile_if_needed(link)` in
`method_h_stage2_tx_probe_v2.py` (l. 324) does only four things:

1. `CFG_SET_REQ(FHSS_CHANNEL_MASK)` — 50-ch mask (`0xff ff ff ff ff ff 03 00`)
2. `CFG_SET_REQ(ANTENNA_GAIN_DBI=2)`
3. `CFG_SET_REQ(HW_CEILING_DBM=17)`
4. `CFG_SET_REQ(REG_PROFILE=1)` — activates FCC 15.247 FHSS 50-ch BW250

Nowhere is a specific operating channel or `RegFrf` value selected.
The L072 firmware, upon activating the profile, evidently picks one
channel from the mask to load into `RegFrfMsb/Mid/Lsb`. The
mechanism for that pick is not specified in any host-side code we
control; from the host's perspective it is effectively non-deterministic
per board.

This also explains why historic single-board tests
(`method_h_stage2_tx_probe_v2` loopback, `rx_liveness`, `tx_burst`) all
worked: those tests run TX and RX on **the same board sequentially**,
so the channel pick is consistent within a session.

## Fix options (cheapest → most invasive)

### Option A — Single-channel mask (recommended, 1-line fix)

Set the FHSS channel mask to **one bit only** on both peers. With one
allowed channel, the firmware has no choice but to pick it.

Concretely, replace step (1) above with:

```python
# Single channel mask — only channel 0 (915.0 MHz on the FCC 50-ch grid)
channel_mask_payload = bytes([
    CFG_KEY_FHSS_CHANNEL_MASK, 0x08,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
])
```

This is the same payload pattern, only the mask bits are narrowed.
**No firmware change needed.**

### Option B — Env-var gated, per-deployment

Wrap the single-channel mask behind `LIFETRAC_FORCE_SINGLE_CHANNEL=<N>`
in both `image_rx_daemon._open_link()` and `image_tx_daemon._open_link()`,
sent **after** `configure_regulatory_profile_if_needed()` so it
overrides the wide mask. Leaves the default path unchanged for
regulatory compliance later.

### Option C — Direct `write_reg` on `RegFrfMsb/Mid/Lsb`

Skip the profile pick entirely and program `0x06/0x07/0x08` to a known
value. Risky because (a) it bypasses the firmware's regulatory
duty-cycle / dwell-time tracking and (b) the L072 driver may rewrite
those registers on the next TX/RX state transition.

## Recommendation

Apply **Option A** in `configure_regulatory_profile_if_needed()` itself,
behind a default-True flag (`single_channel: bool = True`). Once
`rx_frames > 0` is confirmed end-to-end, revisit Option B to re-enable
real FHSS for over-the-air robustness and FCC compliance.

## Falsification harness used

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/radio_state_dump.py`
  — Python probe: opens HostLink, skips RESET_REQ (matches daemon's
  `LIFETRAC_SKIP_RESET_REQ=1`), runs the role-matching CFG sequence,
  dumps 21 SX1276 registers + decoded modem params as one
  `RADIO_DUMP <json>` line.
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_radio_state_dump.ps1`
  — Orchestrator: pulses gpio163 NRST + runs probe on TX peer and RX
  peer in their daemon containers, parses both JSON outputs, prints the
  side-by-side diff above, highlights air-critical mismatches.

This probe is a permanent diagnostic — re-run any time
`rx_frames=0` recurs to confirm modem config parity in seconds.

## Memory lesson

The instinct on a "configured identically, both report OK, no frames
decoded" failure was to suspect modem-param drift (SF/BW/CR/SyncWord)
or physical coupling (antennas, TX power). The actual cause was a
**lower-layer non-determinism**: a host-side CFG payload that grants
freedom (50 channels allowed) which the firmware exercises differently
on each board. Lesson: when a "wide" config is sent (channel masks,
frequency ranges, profile activations), always verify the *selected*
result, not the *permitted* set.

## Update 2026-05-25 07:21 � Mask narrowing alone was NOT sufficient

Two follow-up reg-dumps confirmed the L072 firmware does NOT re-tune
`RegFrf{Msb,Mid,Lsb}` on the SX1276 in response to `CFG_SET(REG_PROFILE)`
even though it returns `CFG_OK_URC`:

- **Attempt A** (mask narrowed to single channel 0, profile_id still 1):
  `2026-05-25_radio_state_dump_20260525_070928.log` � both peers retained
  their prior FRF (TX=918.25, RX=915.00). regprofile_ok=true, mask accepted,
  but FRF unchanged. Mechanism: re-activating the *same* profile id is treated
  as a no-op transition; the new mask is stored but FRF is not re-picked.

- **Attempt B** (profile_id=0 = BENCH_ONLY_FIXED_915):
  `2026-05-25_radio_state_dump_20260525_071536.log` � CFG_SET(REG_PROFILE=0)
  OK on both peers, but FRF still unchanged AND `bw_hz` still 250 kHz
  (profile 1's BW, not profile 0's documented 125 kHz). The L072 firmware
  acknowledges the request but does not actually apply the new profile to
  the radio after a non-cold-boot path.

## Final fix � direct FRF register-write workaround

Augmented `configure_regulatory_profile_if_needed` to, after the
`CFG_SET(REG_PROFILE)` call, force-write the SX1276 FRF registers
directly via `write_reg(link, 0x06/0x07/0x08, ...)`:

- `LIFETRAC_FORCE_FRF_HZ=<int>` env var � explicit override.
- When `profile_id==0`, auto-pin to 915_000_000 Hz (matching the
  documented BENCH profile carrier).
- Drops to STANDBY (0x81) before writing if currently in RX/TX, then
  restores the prior opmode so the new FRF commits on next mode entry.
- Verifies via readback and prints `FRF readback: ... (OK|MISMATCH)`.

Validation: `2026-05-25_radio_state_dump_20260525_072107.log` shows
both peers at **915.0 MHz** post-config; the orchestrator summary prints
*"All air-critical modem params match between TX and RX."*

Concurrent smoke (`run_concurrent_smoke.ps1`) updated to set
`-e LIFETRAC_REG_PROFILE=0` on both daemon docker invocations, which
makes the auto-FRF path activate.

## Open follow-ups

- The L072 firmware bug (CFG_SET(REG_PROFILE) ACKs OK but does not
  re-apply the profile after the first profile activation) deserves a
  fix in `host_cfg_profile.c` so that re-activation re-runs the radio
  setup path. The current Python force-FRF workaround is a bench-only
  band-aid; over-the-air-compliant FHSS still needs the firmware fix.
- The bench requires an MQTT broker on `192.168.1.79:1883` (this
  Windows host) for the smoke test to validate `rx_frames > 0`.
  Smoke run at 12:24:57 failed with `MQTT connect ... timed out` �
  not a radio issue. Start mosquitto locally before re-running smoke.
