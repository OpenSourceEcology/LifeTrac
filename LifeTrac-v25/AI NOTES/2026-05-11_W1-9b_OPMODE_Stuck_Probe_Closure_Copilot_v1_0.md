# W1-9b — OPMODE-Stuck Discriminator Probe — Closure (Copilot v1.0)

**Date:** 2026-05-11
**Author:** GitHub Copilot (assistant)
**Plan:** [`2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md`](2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md)
**Predecessor:** [`2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md`](2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md)
**Hardware:** Portenta X8 ABX00049 (ADB `2E2C1209DABC240B`) + Max Carrier ABX00043 + Murata CMWX1ZZABZ-078
**Firmware under test:** `firmware/murata_l072/build/firmware.bin` = 16 432 bytes (W1-9 lenient diagnostic state, unchanged for W1-9b)
**Probe build:** `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py` extended with `--probe {tx,regversion,fsk,opmode_walk}` modes; shell + ps1 wrappers extended with `LIFETRAC_PROBE_MODE` env var / `-Probe` parameter.

---

## §1 Purpose recap

W1-9 isolated `RegOpMode` (0x01) as stuck at `0x80` (LoRa SLEEP) while every configuration register (`RegFrf*`, `RegPaConfig`, `RegModemConfig*`, `RegTcxo`, FIFO/PAYLOAD_LEN) latched correctly via the same `REG_WRITE_REQ` host path. Two competing hypotheses remained:

1. **No digital clock at the SX1276 die** — TCXO output not reaching the SiP `XTA` pin, so the chip can shadow registers but the digital state machine cannot transition mode bits. Implication: hardware investigation (W1-10).
2. **LoRa-mode-specific firmware/init bug** — chip is alive but our LoRa configuration sequence wedges the state machine. Implication: firmware bug hunt (W1-9c) before scoping hardware work.

W1-9b cheapened the discrimination cost from "scope/swap module" to three software bench probes (T1/T2 optional T3) executed against the existing W1-9 firmware. Per plan §11.A–§11.G acceptance gates.

---

## §2 Acceptance evidence (per plan §11.1 / §10.10 KEY=VALUE block)

```
__T1_TRANSPORT__=PASS
__T1_NON_VERSION_READS__=0
__T1_REQUEST_ERRORS__=0
__T2_STEP5_READBACK__=0x00
__T2_STEP8_READBACK__=0x00
__T2_RESTORE_VERDICT__=OK
__T2_RESTORE_READBACK__=0x80
__T2_RADIO_VERDICT__=FSK_SLEEP_ONLY
__T3_HF_TRANSITIONS__=skipped
__T3_LF_TRANSITIONS__=skipped
__HOST_INVARIANTS__=PASS
__POST_PROBE_QUANT__=5/5
__W1_9B_DECISION__=ESCALATE_TO_W1-10_HARDWARE
```

Bench evidence directories:

- T1 `regversion`: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-9b_regversion_2026-05-11_064830/`
- T2 `fsk`: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-9b_fsk_2026-05-11_064905/`
- T3 `opmode_walk`: **skipped** (plan §5 step 5 — only run if T2 = `FSK_STDBY_WORKS`).
- W1-7 5-cycle regression: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-11_064930_637-17856/` (`FINAL_RESULT_PASS=5`, gate `PASS`).

---

## §3 T1 — RegVersion burst (1024 reads)

**Purpose:** Establish that the host-protocol/SPI-shadow path is rock-solid before drawing any conclusion from T2's mode-bit readbacks. Eliminates "intermittent SPI/transport noise" as a confounder.

**Result:** 1024/1024 reads of `0x42 RegVersion` returned `0x12`. `host_parse_err` delta = 0. Elapsed 10.94 s, 93.6 reads/s.

```
T1 burst: correct=1024 zero=0 other=0 errors=0 elapsed=10.94s rate=93.6/s
__T1_TRANSPORT__=PASS
__T1_NON_VERSION_READS__=0
__T1_REQUEST_ERRORS__=0
__W1_9B_VERDICT__=T1_PASS
```

**Conclusion:** SPI shadow + host-protocol stack is bullet-proof. Any T2/T3 readback can be trusted at face value.

---

## §4 T2 — FSK STDBY datapath (the decisive probe)

**Sequence (per plan §5 step 4 + §11.D pre-state assertion + §11.B 8× wait):**

| Step | Action | Result |
|---:|---|---|
| 2 | Read `RegOpMode` pre-state | `0x80` (LoRa SLEEP) ✓ as expected |
| 3 | `REG_WRITE_REQ OPMODE = 0x00` (FSK SLEEP) | `REG_WRITE_ACK = 01 00` |
| 5 | Wait ~24 ms; readback | **`0x00`** ✓ (LongRangeMode bit 7 cleared) |
| 6 | `REG_WRITE_REQ OPMODE = 0x01` (FSK STDBY) | `REG_WRITE_ACK = 01 01` |
| 8 | Wait ~24 ms; readback | **`0x00`** ✗ (mode bits 2:0 did NOT transition `SLEEP→STDBY`) |
| restore | `REG_WRITE_REQ OPMODE = 0x80` | readback `0x80` ✓ |

**Verdict matrix (plan §10.2):** `(T2_STEP5, T2_STEP8) = (0x00, 0x00)` → `FSK_SLEEP_ONLY`.

**Interpretation (per plan §10.2 review note):**

> The `0x00 / 0x00` matrix row should be written up as "LongRangeMode latch works, FSK STDBY transition still fails" rather than as proof that the clock is alive.

This is the diagnostic gold the probe was designed to produce:

- The chip **can clear the LongRangeMode bit** (`bit 7`) via the same SPI-shadow path that already accepted FRF/PA/MODEM_CFG/RegTcxo writes in W1-9. **No new news here.**
- The chip **cannot transition mode bits 2:0** (`SLEEP → STDBY`) in **either** the FSK family **or** the LoRa family (W1-9). Both modem families fail identically.

The "LoRa-mode-specific firmware bug" hypothesis is **falsified**: if the LoRa init sequence were wedging the state machine, an FSK STDBY request after a clean `LongRangeMode=0` write would succeed (FSK STDBY does not depend on any LoRa-specific configuration register). It does not.

The "no digital clock" hypothesis is **strongly reinforced**: the SX1276 datasheet §4.1.6 mode-transition state machine requires the digital clock to actually advance between `SLEEP` and `STDBY` regardless of `LongRangeMode`. Bit-7 latching does not.

---

## §5 T3 — HF/LF OPMODE walk

**Skipped** per plan §5 step 5: T3 is only meaningful if T2 = `FSK_STDBY_WORKS` (i.e., we have a known-working transition to compare HF↔LF against). Since T2 = `FSK_SLEEP_ONLY`, every `(LongRangeMode, AccessSharedReg, LowFrequencyModeOn, mode[2:0])` combination would readback with mode bits stuck at `0`. Running T3 would produce `0/4` and `0/4` transition counts and add no diagnostic information beyond what T2 already proves.

---

## §6 Regression check — W1-7 5-cycle quant

Per plan §6 (W1-9b.F) acceptance: standard Stage 1 quant must remain ≥ 5/5.

```
RUN_ID=T6_stage1_standard_quant_2026-05-11_064930_637-17856
CYCLES=5
FINAL_RESULT_PASS=5
LAUNCHER_FAIL_COUNT=0
TIMEOUT_COUNT=0
GATE_RESULT=PASS
```

The W1-9 lenient firmware + the new `--probe {regversion,fsk,opmode_walk}` modes have **not** regressed the W1-7 / W1-8 baseline. Probe modes leave the host-protocol path otherwise untouched (the new code paths are only reached when `LIFETRAC_PROBE_MODE != tx`).

---

## §7 Decision — escalate to W1-10 (hardware-side TCXO investigation)

`__W1_9B_DECISION__ = ESCALATE_TO_W1-10_HARDWARE`.

W1-9b cracked open the W1-9 ambiguity at zero hardware cost. The LoRa-mode-specific bug hypothesis is dead; the no-digital-clock hypothesis now has positive corroboration from a second modem family (FSK). Continued software/firmware investigation has diminishing returns until the TCXO trace is observed at the SiP `XTA` pin.

**W1-10 entry conditions (carried forward from the W1-9 closure §8.7):**

1. Identify a second `TCXO_VCC` enable pin (or rail) on Max Carrier ABX00043 schematic if one exists beyond `PA12`.
2. Oscilloscope probe of TCXO output and the SiP `XTA` pin during boot + during a `REG_WRITE_REQ OPMODE=0x01` (FSK STDBY) attempt — the W1-9b T2 sequence is the cheapest scope-trigger.
3. Fall-back: swap to a second Murata module to reject "this die is dead" before reworking the carrier.

W1-10 is **out of agent scope** (requires bench instruments / hardware swap). Documented here for the human bench session.

**Out-of-scope for both W1-9b and W1-10 (plan §11.G):** PaDac/PaConfig high-power scaffolding, FSK-mode TX support, LoRaWAN-stack work — all blocked behind the OPMODE transition working at all.

---

## §8 Firmware / probe state at end of W1-9b

**Firmware:** unchanged. `firmware.bin` = 16 432 B, lenient diagnostic init still in place. `reg_write_allowed()` allowlist still includes OPMODE 0x01 and DIO_MAPPING1 0x40 — leave open through the W1-10 hardware investigation, then re-tighten as a single edit.

**Probe + wrappers:**

- `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py` — new `--probe {tx,regversion,fsk,opmode_walk}` modes. Default `tx` behavior preserved (W1-9 path unchanged). Helpers: `fetch_stats`, `drain_boot`, `read_reg`, `write_reg`, `host_invariants_violated`, `_classify_fsk_verdict`. Probe drivers: `run_regversion_burst`, `run_fsk_stdby` (with finally-style restore + pre-state assertion of `0x80`), `run_opmode_walk`.
- `firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx.sh` — reads `LIFETRAC_PROBE_MODE` env var (default `tx`), passes via `--probe`. Per-mode log filenames `method_h_stage2_${PROBE_MODE}.log` + `_ocd.log`.
- `firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1` — `-Probe` param `[ValidateSet("tx","regversion","fsk","opmode_walk")]`. Evidence dir: `W1-9_stage2_tx_<stamp>` for `tx`, `W1-9b_<probe>_<stamp>` for others. Wraps remote call as `sudo -S -p '' env LIFETRAC_PROBE_MODE=$Probe bash …` (the `sudo env` form was required because env-var injection ahead of the `sudo` pipe was being stripped — fixed mid-session, see §3 evidence dir naming).

Acceptance line emitters (per plan §11.1 / §10.10): every probe mode prints a `__W1_9B_VERDICT__` and (where applicable) `__T1_*` / `__T2_*` / `__T3_*` and `__HOST_INVARIANTS__` KEY=VALUE lines for grep-based downstream consumption.

---

## §9 Lessons captured

1. **A "config register write succeeded" in shadow ≠ "the chip's state machine can transition."** The SX1276 SPI shadow path tolerates writes (and reads them back) without the digital clock. Always include a mode-transition test (T2-style) when bringing up an SX127x family chip on a new carrier.
2. **FSK is the cheapest cross-family discriminator for OPMODE-stuck symptoms.** No FRF / PA_CONFIG / MODEM_CFG dependency — the chip should reach FSK STDBY from FSK SLEEP regardless of any LoRa-side configuration. If it doesn't, the digital clock is the prime suspect.
3. **Wait at least 8× the datasheet typical mode-transition time** (250 µs typ. → 2 ms minimum; we used ~24 ms) before declaring a transition failed — eliminates any "we measured too soon" objection.
4. **Pre-state assertion** before the discriminator (here: `RegOpMode == 0x80` before issuing the FSK SLEEP write) prevents misclassification when boot timing or a stray FAULT_URC has perturbed the chip.

---

**End of W1-9b closure.** Reopen as **W1-10** when bench instruments are in front of the carrier.
