# W1-9b — OPMODE-stuck-in-SLEEP cheap-probe plan (Copilot v1.0)

**Author:** Copilot (Claude Opus 4.7)
**Date:** 2026-05-11 (immediately follows W1-9 closure)
**Hardware:** Portenta X8 ABX00049 (ADB serial `2E2C1209DABC240B`) + Max Carrier ABX00043 + Murata CMWX1ZZABZ-078 (STM32L072 + SX1276)
**Predecessor:** [`2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md`](./2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md)
**Goal:** Decide between *"chip has no digital clock"* and *"chip has clock but a LoRa-mode-specific bug"* using only software probes — no schematic work, no oscilloscope, no second hardware unit.

---

## 1. Why this is worth doing before Option C

The W1-9 closure pinned the failure to "the SX1276 die's digital state-machine clock is missing" with strong but not airtight evidence. The asymmetry that drove that conclusion was:

| Register class | Behavior | Path |
|---|---|---|
| Configuration registers (FRF, PA_CONFIG, MODEM_CFG*, RegTcxo) | latch | SPI shadow path — does NOT need digital clock |
| `RegOpMode` mode bits (2:0) | stuck at `000` (SLEEP) | digital state machine — needs digital clock |

But there are still two software-cheap experiments that can either:
1. **Crack W1-9 outright** (if FSK STDBY works), or
2. **Make Option C (hardware investigation) go faster** (a definitive yes/no on "is the clock alive at all?" before someone scopes the board).

Each experiment is a sub-30-line addition to the existing
`method_h_stage2_tx_probe.py` and re-uses the OPMODE allowlist
already in `host_cmd.c`. **No firmware rebuild is required.**

---

## 2. Probe T1 — `RegVersion` jitter scan

### Hypothesis
If the SX1276 digital block has a clock, then back-to-back reads of
read-only register `RegVersion` (0x42) should always return `0x12`
*and* there should be measurable inter-read timing jitter caused by
the radio die's own SPI state machine being clocked.

If the die has no clock at all, two things may happen:
- Reads still return `0x12` because the SPI shadow path is purely
  passive (latched value mirrored to MISO), giving us *no* useful
  signal — we already know SPI shadow works.
- OR reads occasionally glitch (return `0x00`, partial bytes, or
  garbage) when the host SPI clock outpaces some other internal
  timing.

### Method
Add a probe sub-command `probe_regversion_burst`:

```python
# 1024 back-to-back REG_READ_REQ(0x42) with no inter-request delay
# Record: (count_correct, count_zero, count_other, distinct_other_values)
```

Acceptance:
- **All 1024 reads return `0x12` exactly** → result is *inconclusive
  by itself* (means SPI shadow is fine, says nothing about clock).
- **Any read != `0x12`** → digital path is partially alive but
  unstable; pivot W1-10 to clock-tree investigation with this evidence.

### Cost
~20 lines in `method_h_stage2_tx_probe.py` + 1 minute bench time.
No firmware change.

---

## 3. Probe T2 — FSK STDBY datapath (the decisive one)

### Hypothesis
The SX1276 has two top-level mode families selected by `RegOpMode`
bit 7 (`LongRangeMode`):
- `0` = FSK/OOK (legacy)
- `1` = LoRa

Both families use the same digital state machine and the same TCXO/XTA
clock domain. So:

- If `OPMODE = 0x01` (FSK STDBY) **does** latch and read back as
  `0x01` → the digital clock is alive, and W1-9 is a **LoRa-mode-specific**
  bug (frequency-band selection? high-frequency mode bit 3? we never
  set that bit, but at 915 MHz we should).
- If `OPMODE = 0x01` **also** stays at `0x80` → the digital clock is
  definitively absent regardless of mode family; Option C is the
  only path forward and we have a clean datapoint to hand to whoever
  scopes the board.

### Method
Add probe sub-command `probe_fsk_stdby`:

```python
# 1. CFG_SET_REQ LBT_ENABLE = 0  (avoid abort)
# 2. REG_READ_REQ 0x01 → record current opmode (expect 0x80)
# 3. REG_WRITE_REQ 0x01 = 0x00   (FSK SLEEP — required intermediate)
# 4. sleep 2 ms
# 5. REG_READ_REQ 0x01 → record (expect 0x00 if clock alive)
# 6. REG_WRITE_REQ 0x01 = 0x01   (FSK STDBY — needs clock to transition)
# 7. sleep 2 ms
# 8. REG_READ_REQ 0x01 → record (expect 0x01 if clock alive; 0x00 or 0x80 otherwise)
# 9. REG_WRITE_REQ 0x01 = 0x80   (back to LoRa SLEEP — restore baseline)
# 10. STATS_DUMP_REQ — confirm host_parse_err = 0, no FAULT_URC
```

Verdict matrix:

| Step 5 readback | Step 8 readback | Verdict |
|---|---|---|
| `0x00` | `0x01` | **Clock alive — W1-9 is LoRa-mode-specific** → pivot to LoRa-mode init review (RegOpMode bit 3 `LowFrequencyModeOn`, RegPaDac, etc.) |
| `0x00` | `0x00` | Clock alive enough for FSK SLEEP latch but FSK STDBY transition fails → **clock-domain fault on STDBY-only**; very unusual; flag for hardware review with this exact evidence. **Amended interpretation:** see §10.2 / §11.2 — `LongRangeMode` cleared via SPI shadow path does **not** prove clock-alive; this row is recorded as `T2_FSK_SLEEP_ONLY` in the W1-9b closure and falsifies the LoRa-mode-specific bug hypothesis. |
| `0x80` | `0x80` | **No clock at all — Option C is the only path** |
| `0x80` | `0x01` | Inconsistent — re-run; if reproducible, an SPI race we should fix first |

### Cost
~30 lines in `method_h_stage2_tx_probe.py` + 1 minute bench time.
No firmware change. OPMODE allowlist already in place from W1-9.

---

## 4. Probe T3 (conditional) — RegOpMode bit-by-bit walk

**Run only if T2 verdict is `0x00 / 0x01` (clock alive, LoRa-mode bug).**

If T2 confirms clock-alive, the next 5-minute experiment is to walk
each bit of `RegOpMode` and observe which writes latch:

```python
for bits in [0x80, 0x88, 0x81, 0x89, 0x83, 0x8B, 0x85, 0x8D]:
    REG_WRITE_REQ 0x01 = bits
    sleep 2 ms
    rb = REG_READ_REQ 0x01
    print(f"wrote {bits:02x} → readback {rb:02x}")
```

The 4 high-bit-3-clear values are LoRa LF-mode (0x80/SLEEP, 0x81/STDBY,
0x83/TX, 0x85/RXC); the 4 high-bit-3-set values are LoRa LF-mode
explicitly (`LowFrequencyModeOn = 1`). At 915 MHz we want HF mode
(bit 3 = 0), but it's worth seeing if either family transitions.

### Cost
~15 additional lines in the same probe + 30 s bench time.

---

## 5. Implementation order

1. **Edit `method_h_stage2_tx_probe.py`** — add `probe_regversion_burst`
   and `probe_fsk_stdby` sub-commands behind a CLI flag
   (`--probe regversion|fsk|tx`). Keep the existing TX probe as the
   default so nothing regresses. **No firmware rebuild needed.**
2. **Edit `run_method_h_stage2_tx.sh`** — accept a `--probe` argument
   and pass it through.
3. **Edit `run_method_h_stage2_tx_end_to_end.ps1`** — add a `-Probe`
   parameter (default `tx`) and forward it.
4. **Run T1 first** (`-Probe regversion`). 1 min bench.
5. **Run T2 second** (`-Probe fsk`). 1 min bench.
6. **If T2 verdict is "clock alive" → run T3** in the same probe pass.
7. **Capture all results into a single closure note** —
   `2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md`.
8. **Update repo memory + TODO** based on the verdict:
   - Clock-alive + LoRa-mode bug → reopen as **W1-9c** (LoRa-init bug
     hunt, no hardware scope needed) and bench it immediately.
   - Clock-dead → confirm W1-10 hardware scope is the only path,
     update Option C list with the FSK datapoint.

---

## 6. Acceptance criteria for this probe pass

| ID | Criterion |
|---|---|
| W1-9b.A | Stage 1 still PASS (sanity gate before/after) |
| W1-9b.B | T1 burst run completes without `host_parse_err` increment |
| W1-9b.C | T2 four register-write/readback pairs all complete; ack received for each |
| W1-9b.D | No `FAULT_URC` during either probe |
| W1-9b.E | Verdict from T2 verdict matrix is unambiguous (one of the four cells) |
| W1-9b.F | W1-7 quant 5-cycle re-run still 5/5 PASS after probe pass |

If E lands on cell `0x00 / 0x01` (clock alive), W1-9 is **NOT
deferred** anymore — we re-open it as W1-9c with a much narrower
scope (LoRa-mode-specific init), and the W1-10 hardware-scope work
becomes optional/preventative rather than blocking.

---

## 7. Time budget

| Step | Estimate |
|---|---|
| Probe edits + dry-run | ~15 min |
| T1 bench | ~1 min |
| T2 bench | ~1 min |
| T3 bench (if applicable) | ~30 s |
| Closure note + TODO/memory updates | ~10 min |

Total: well under one autonomous session, with a high probability of
either closing W1-9 outright or producing a single-line answer for
hardware review.

---

## 8. What this plan deliberately does **not** do

- Does **not** re-flash firmware. The OPMODE/DIO_MAPPING1 allowlist
  added in W1-9 is sufficient for every write needed here.
- Does **not** add new fault codes or new URC types.
- Does **not** touch `sx1276_modes.c` or `sx1276.c`. (If T2 says
  "LoRa-mode bug", *those* files become the next session's edit
  target — but in a new W1-9c scope, not here.)
- Does **not** require schematics, oscilloscope, or a second hardware
  unit. Those remain Option C / W1-10 if T2 verdict is "no clock".

---

## 9. Decision after this plan executes

| T2 verdict | Next action |
|---|---|
| Clock alive, FSK STDBY works | Open **W1-9c** — LoRa-mode init bug hunt; revert OPMODE/DIO_MAPPING1 from REG_WRITE allowlist after closure |
| Clock alive, FSK STDBY fails | Document carefully; this is an unusual silicon symptom; recommend Option C with this specific evidence |
| Clock dead | Confirm W1-10 hardware scope as only path; revert OPMODE/DIO_MAPPING1 from REG_WRITE allowlist; lock firmware in current lenient state until hardware investigation completes |

---

## 10. Copilot Review Notes (2026-05-11)

Overall: this is worth running before W1-10. It is cheap, it directly
falsifies the strongest remaining uncertainty in the W1-9 closure, and
T2 is the right software-only discriminator. A few guardrails will keep
the result from being over-interpreted:

1. **Treat T1 as a transport/SPI-shadow stress test, not a clock
  discriminator.** A Python/COBS/UART host round-trip cannot measure
  SX1276 internal timing jitter; the timing will be dominated by Linux
  scheduling, UART framing, and firmware dispatch. If any `RegVersion`
  read is not `0x12`, first classify it as harness/transport/SPI
  integrity until `host_parse_err`, echoed register address, frame
  length, and a short repeat run rule that out.

2. **T2 is decisive only at the FSK STDBY step.** The `0x80 -> 0x00`
  write mostly proves that `LongRangeMode` can be cleared while the
  chip is already in Sleep; it may not require the radio digital state
  machine. The strong clock-alive signal is `0x01` readback after
  `FSK STDBY`. Therefore the `0x00 / 0x00` matrix row should be written
  up as "LongRangeMode latch works, FSK STDBY transition still fails"
  rather than as proof that the clock is alive.

3. **Define the `FAULT_URC` window carefully.** Current W1-9 firmware
  may emit `HOST_FAULT_CODE_RADIO_OPMODE_DRIFT` during boot/init
  because the lenient retry loop notices the stuck mode. Drain and log
  startup frames separately, then apply W1-9b.D only after
  `stats_before` and immediately around the probe writes.

4. **Use fresh boot cycles for probe modes.** Keep `--probe regversion`,
  `--probe fsk`, and default `--probe tx` as separate wrapper runs. The
  FSK probe intentionally steps outside the normal `sx1276_modes.c`
  ownership model; even with a best-effort restore to `0x80`, a clean
  `08_boot_user_app.cfg` reset before the next probe is safer and makes
  evidence easier to interpret.

5. **Make restore paths `finally`-style.** In `probe_fsk_stdby`, always
  attempt `RegOpMode = 0x80` and then read it back, even if an
  intermediate request times out. Log every `REG_WRITE_ACK_URC` payload
  plus the final readback. If restore fails, mark the probe as
  transport/probe-incomplete and reboot before any follow-on test.

6. **Exit codes should mean "probe completed", not "good hardware" for
  the new modes.** For `regversion` and `fsk`, return `0` when the run
  completes with an unambiguous verdict, including "clock dead"; reserve
  `1` for incomplete/invariant-failed probe data and `2` for fatal
  transport setup. The existing PowerShell wrapper throws on nonzero rc,
  so using `1` for a valid clock-dead verdict would make expected
  evidence look like an execution failure.

7. **Put the probe name in evidence paths and logs.** The host launcher
  currently names evidence `W1-9_stage2_tx_$stamp` and pulls
  `method_h_stage2_tx.log`. When adding `-Probe`, include the mode in
  the evidence directory and in the top banner, e.g.
  `W1-9b_regversion_$stamp` / `W1-9b_fsk_$stamp`, so T1/T2/T3 cannot be
  mixed up later.

8. **T3 labels are right, but the interpretation should stay narrow.**
  Bit 3 set (`0x88`, `0x89`, etc.) is `LowFrequencyModeOn = 1`; at
  915 MHz the desired LoRa values keep bit 3 clear (`0x80`, `0x81`,
  `0x83`, `0x85`). If any LF value transitions while HF values do not,
  that is a RegOpMode/LoRa-mode-family clue, not yet a full
  frequency-plan diagnosis.

9. **Clean up one source-comment contradiction after the probe.** The
  current [`radio/sx1276_modes.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c)
  comment says `RegTcxo = 0x10` makes mode transitions complete normally,
  while the W1-9 closure and this plan say the bench-confirmed value
  `0x4B = 0x10` still leaves OPMODE stuck. If W1-9b confirms
  clock-dead, update that comment in the closure cleanup so future
  readers do not inherit a false fix history.

10. **A closure note should distinguish "radio verdict" from "host
   invariant verdict."** Suggested final lines:
   `T1_TRANSPORT=PASS/FAIL`,
   `T2_RADIO_VERDICT=CLOCK_DEAD|FSK_STDBY_WORKS|FSK_SLEEP_ONLY|INCONSISTENT`,
   `HOST_INVARIANTS=PASS/FAIL`, and `POST_PROBE_QUANT=5/5`. That keeps
   the hardware conclusion separable from harness health.


---

## 11. Author response to §10 review (Copilot Claude Opus 4.7, 2026-05-11)

I read §10 in full and accept all 10 points. Folding them in below
with concrete plan deltas, plus a few additional observations the
review surfaced indirectly.

### 11.1 Accepted with plan deltas

**§10.1 (T1 reframe — transport stress, not clock discriminator).**
Accepted. T1 is demoted from "clock discriminator" to "SPI shadow /
COBS / host round-trip stress test." Acceptance criterion W1-9b.B is
unchanged but its **interpretation** is rewritten: any non-`0x12`
read triggers a transport classification cascade
(`host_parse_err` delta ? echoed reg addr ? COBS frame length ?
3-shot repeat) BEFORE any radio inference. T1 now exists primarily
to harden T2's confidence that the SPI bus itself is not the variable.

**§10.2 (T2 verdict matrix narrowing).** Accepted. The verdict matrix
in §3 is amended: `0x00 / 0x00` now reads "**LongRangeMode latch
cleared but FSK STDBY transition failed** — consistent with no
digital clock; `LongRangeMode` cleared via the same configuration
shadow path that latches FRF/PA_CONFIG." This makes the only
unambiguous "clock alive" cell `0x00 / 0x01`.

**§10.3 (FAULT_URC window).** Accepted. Probe will:
1. Drain all frames during a 1.0 s settle window after the X8-side
   shell launches the helper (already done for `BOOT_URC`).
2. Run `STATS_DUMP_REQ` and snapshot `host_fault_count` / latest
   `FAULT_URC` codes — call this `fault_baseline`.
3. Apply W1-9b.D only to faults emitted between `fault_baseline` and
   `stats_after`. `HOST_FAULT_CODE_RADIO_OPMODE_DRIFT` from the lenient
   retry loop is expected during init; we explicitly tolerate it in
   the baseline window.

**§10.4 (fresh boot per probe mode).** Accepted. The launcher runs
`08_boot_user_app.cfg` (the L072 reset path used by Stage 1) before
each probe mode invocation. T1, T2, T3 each get their own clean L072
reset. Trade-off: ~3 extra seconds per probe — well within budget.

**§10.5 (`finally`-style restore in T2).** Accepted. `probe_fsk_stdby`
wraps the probe body in `try/finally`; the `finally` block always
attempts `REG_WRITE_REQ 0x01 = 0x80` and reads back. Output adds three
lines: `RESTORE_WRITE_ACK=...`, `RESTORE_READBACK=0x..`,
`RESTORE_VERDICT=OK|FAIL`. If restore fails, exit code is W1-9b's "2"
(fatal — needs reset).

**§10.6 (exit-code semantics).** Accepted with the following mapping
for `--probe regversion` and `--probe fsk`:

| rc | Meaning |
|---|---|
| 0  | Probe completed, verdict captured (ANY of the matrix cells, including "clock dead") |
| 1  | Probe incomplete or host-invariant violation (`host_parse_err > 0`, FAULT_URC delta in window, restore failure) |
| 2  | Fatal transport setup (no `BOOT_URC`, UART unconfigurable, helper crash) |

The PowerShell wrapper is updated to forward `__METHOD_H_RC__` and to
treat rc=0 as success regardless of radio verdict. Verdict is encoded
in a separate `__W1_9B_VERDICT__` line.

**§10.7 (probe name in evidence path / banner).** Accepted. Evidence
directories become `W1-9b_<probe>_<stamp>` and the helper banner
prints `MODE: regversion|fsk|tx` on its first line. Logs renamed
`method_h_<probe>.log`.

**§10.8 (T3 LF/HF interpretation narrowing).** Accepted. T3 output
is grouped into two blocks (HF: `0x80/0x81/0x83/0x85`,
LF: `0x88/0x89/0x8B/0x8D`) and the closure note records the cross
product as evidence rather than calling any single transition
"the fix." T3 stays diagnostic only.

**§10.9 (clean up the stale `sx1276_modes.c` comment).** Accepted —
but **deferred to whichever closure note actually fires**:
- W1-9c (clock alive, LoRa-mode bug) closure: delete the stale
  comment and replace with whatever fix lands.
- W1-10 (clock dead) closure: rewrite the comment to read "RegTcxo
  bit 4 set is necessary but not sufficient; mode transitions still
  blocked at the chip level pending hardware investigation — see
  W1-9b closure."

This deliberately keeps W1-9b itself a **read-only investigation** —
it does not edit firmware comments or any code. That preserves the
"no firmware rebuild required" property of §1 and §8.

**§10.10 (closure note key-value summary).** Accepted. The closure
note will end with a fenced `KEY=VALUE` block:

```
T1_TRANSPORT=PASS|FAIL
T1_NON_VERSION_READS=<count>
T2_STEP5_READBACK=0x..
T2_STEP8_READBACK=0x..
T2_RADIO_VERDICT=CLOCK_DEAD|FSK_STDBY_WORKS|FSK_SLEEP_ONLY|INCONSISTENT
T2_RESTORE_VERDICT=OK|FAIL
T3_HF_TRANSITIONS=<count>
T3_LF_TRANSITIONS=<count>
HOST_INVARIANTS=PASS|FAIL
POST_PROBE_QUANT=<n>/5
W1_9B_DECISION=W1-9c|W1-10|UNUSUAL_SILICON
```

This makes the closure machine-greppable and matches the convention
the existing TODO entries already use (`FINAL_RESULT_PASS=…`,
`__METHOD_G_RC__=…`).

### 11.2 Additional notes from this review pass

**§11.A — Probe T0 (precheck) added.** Before T1 runs, the probe
should issue one `VER_REQ` and one `STATS_DUMP_REQ` and confirm
matching values to the immediately-preceding Stage 1 baseline. If
`VER_URC` does not arrive in 200 ms we abort with rc=2 — never
attribute a radio verdict to a session where the host transport is
already misbehaving. Cost: ~5 lines, ~50 ms bench time.

**§11.B — T2 timing margin.** Datasheet §7.2.2 mode-transition
timing for FSK STDBY is typ. 250 µs from SLEEP. Our 2 ms readback
window is 8× margin, so a borderline-positive result is unlikely to
be a timing artifact; still, the closure note should record exact
elapsed wall time between write-ack and readback for each step so a
reviewer can sanity-check.

**§11.C — `0x80 / 0x80` ambiguity.** Even after §10.2 narrowing, the
`0x80 / 0x80` cell is technically ambiguous between
"`LongRangeMode` cannot be cleared from a SLEEP state" and "no
digital clock at all." Both still point at hardware investigation,
so for **decision purposes** they collapse into the same W1-10
branch. The closure note should still call out the distinction so
the W1-10 hardware investigator knows to check both
`LongRangeMode`-clear behavior and TCXO presence.

**§11.D — T2 ordering robustness.** The §3 sequence assumes the
chip is in LoRa SLEEP at probe entry. Because §11.A confirms boot
freshness and the W1-9 closure documents that the chip *consistently*
boots into LoRa SLEEP, this holds — but the probe should also assert
it via `REG_READ_REQ 0x01` at step 2 and abort with rc=1 if the
pre-state is anything other than `0x80`. Avoids interpreting a probe
run that started from an unexpected mode.

**§11.E — Per-byte write-ack capture.** The current
`REG_WRITE_ACK_URC` payload includes the register address echo; T2
should print the **raw 4-byte ACK payload** rather than just the
parsed value, so a future reviewer comparing the closure to
`host_cmd.c` can verify framing. Same for `REG_DATA_URC` reads.

**§11.F — Calibrate against Stage 1 register dump baseline.** Stage 1
already pulls a 128-byte register dump that includes `RegOpMode`
(0x01). The closure note should pull the most-recent Stage 1 dump as
"reference baseline" and place the T2 step-5/step-8 readbacks in a
3-row diff so the reader sees that the chip's *only* OPMODE state
across boot, Stage 1, and W1-9b is `0x80` (or learns it changed).

**§11.G — One thing this plan should explicitly NOT do.** The
review prompted me to consider also walking *other* mode-bit-affected
registers (e.g. `RegPaConfig` bit 7 PaSelect, `RegPaDac` 0x4D
high-power mode). I'm holding the line: those are W1-9c/W1-10 scope
once T2 produces a verdict. Adding them here would dilute the
decision the plan exists to make.

### 11.3 Final disposition

Plan stays at **v1.0**. §10 review accepted in spirit; deltas above
are sufficient to encode them at execution time without re-issuing
the document. If T2 lands on `CLOCK_DEAD` or `FSK_SLEEP_ONLY` we
write a v1.1 closure note; if T2 lands on `FSK_STDBY_WORKS` we open
W1-9c with its own v1.0 plan and the W1-9b closure becomes the
hand-off document.

Ready to proceed with execution on user instruction.

---

## 12. Outcome (post-execution, 2026-05-12)

Plan executed. T2 verdict at the bench was `T2_FSK_SLEEP_ONLY`
(step-5 readback `0x00`, step-8 readback `0x00`). Per the §10.2 /
§11.2 amended interpretation, this row falsifies the LoRa-mode-specific
bug hypothesis and points at a missing/unhealthy SX1276 digital clock
rather than firmware ordering.

- W1-9b closure: [`2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md`](./2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md)
- W1-9c software follow-up closure (Semtech-canonical NRESET Hi-Z + RegTcxo RMW; verdict unchanged → escalate hardware): [`2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md`](./2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md)
- Decision: `W1_9B_DECISION=W1-10` (TCXO/clock-path hardware investigation). See the W1-9c closure for the corrected ABX00043 sheet-6 TCXO routing and the W1-10 scope checklist (`VDD_TCX0`, `TCX0_OUT`, `R19`, `PH0-OSC_IN`).
