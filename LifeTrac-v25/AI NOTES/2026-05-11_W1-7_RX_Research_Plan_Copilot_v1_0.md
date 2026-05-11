# 2026-05-11 — W1-7 RX investigation plan (v1.0)

**Author:** Copilot (Claude Opus 4.7)
**Status:** PROPOSED — awaiting user nod before Phase A starts
**Supersedes/extends:** the experiment queue in [`2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md`](2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md) §3.5.6
**Open blocker:** [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42) — RX side of W1-7 still emits `H:ERR_LPUART1` + `H:COBS_ERR` on every COBS frame received from `/dev/ttymxc3`.

---

## 0. Methodology guardrails (apply to every experiment in this plan)

These exist because of the failure mode documented in `/memories/methodology.md` —
the prior "clock-tolerance" diagnosis was a plausible mechanism that was never
falsification-tested, was contradicted by an existing prior result we hadn't
re-read, and produced two no-op fix attempts before being retracted.

For every experiment below:

1. **State hypothesis + falsifier in writing** before flashing. The falsifier
   is the observable that, if absent, kills the hypothesis. If we can't name
   one, we don't run the experiment.
2. **Grep TODO.md + repo memory + recent AI NOTES** for prior matching
   experiments before declaring something untried. The 2026-05-10 "115 200
   baud" result already lived in TODO.md line 250 a day before we re-ran it.
3. **Verify register / clock preconditions at runtime** with a one-shot diag
   trace before claiming a register patch is "applied". A patch on top of a
   false precondition is a silent no-op (cf. our CCIPR=01 switch when SYSCLK
   was already HSI16).
4. **Mark each experiment "PASS" or "FALSIFIED"**, not "fix" — until phase C
   has run end-to-end, no experiment graduates to "fix".
5. **Instrument before patching.** Phase A produces a measurement vector. We
   do not propose a fix in Phase B that isn't supported by a Phase A number.

---

## 1. Open question (single-blocker focus)

**Q1. Why does every COBS frame received on LPUART1 get rejected?**

Observable today (from [`bench-evidence/T6_W1-7_RX_session_2026-05-11/`](../DESIGN-CONTROLLER/bench-evidence/T6_W1-7_RX_session_2026-05-11/)):

```
H:RX_LPUART1     ← ISR fires on first byte
H:ERR_LPUART1    ← USART_ISR.(PE|FE|NE|ORE) set on a follower byte
H:PROC_FRAME     ← parser entry reached
H:COBS_ERR       ← decoder rejected
```

`H:ERR_LPUART1` is a single trace covering **four physically distinct error
classes**:

| Bit | Name | Usually means |
|---|---|---|
| PE | parity error | n/a — we run 8N1 |
| FE | framing error | start/stop bit mistimed → clock or glitch |
| NE | noise error | sampler disagreement within a bit → SI / glitch |
| ORE | overrun | RDR not read before next byte → ISR latency |

**The key insight for this plan:** which bit is set tells us which
hypothesis survives. Today we don't know. Phase A's whole job is to find out.

**Q2 (deferred to Phase D).** Why doesn't HSE/TCXO lock on the Max Carrier?
Lower priority — HSI16 is sufficient for every protocol bit if Q1 clears.

---

## 2. Hypothesis register (current snapshot)

| ID | Hypothesis | Predicted dominant flag | Status |
|---|---|---|---|
| H1 | ISR self-stretch (blocking `platform_diag_trace()` inside LPUART1 IRQ error path causes overrun on follower bytes) | ORE | **Top** |
| H2 | USART1 mirror noise on PA10 corrupts the SHARED COBS state machine | NE on USART1 side, none on LPUART1 | Plausible |
| H3 | LPUART1 RX line glitch / inadequate pull on PA3 | NE or FE, isolated to first byte after IDLE | Plausible |
| H4 | Real baud / oversampling mismatch (despite the 115 200 falsifier) | FE on every byte, both LPUART1 and USART1 | **FALSIFIED 2026-05-11** by 115 200 test |
| H5 | COBS decoder bug (state corrupt across frames even with clean RX) | No UART-error flags, just `H:COBS_ERR` | Untested but unlikely (decoder is shared with TX path that works) |

---

## 3. Phase A — Instrumentation only (no fix attempts)

**Goal:** turn `H:ERR_LPUART1` into a 4-element vector `(PE, FE, NE, ORE)`
plus per-frame visibility, so Phase B can branch on data instead of
intuition. Total code change is small and localized.

### A1. Per-flag error counters in both ISRs

In [`host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) `RNG_LPUART1_IRQHandler()` (line ~885) and `USART1_IRQHandler()` (line ~935):

- Add `static volatile uint16_t s_stats_err_pe / _fe / _ne / _ore` (per peripheral).
- In the error branch, test each bit *individually*, increment its counter,
  then clear all four flags + drain RDR as today.
- **Do NOT** remove the `platform_diag_trace("H:ERR_LPUART1\r\n")` yet — that
  change is a Phase B intervention, not Phase A measurement. Phase A must
  measure the system *as it is today*.

### A2. Per-frame trace latch reset

Where the COBS frame delimiter `0x00` is consumed (in `ingest_rx_byte` /
frame-end handler), reset `s_trace_first_err_lpuart` and `s_trace_first_rx_lpuart`
to `0`. This makes the existing `H:ERR_LPUART1` trace fire *once per frame*
instead of *once per power-on*, telling us whether errors are bursty or steady.

### A3. Extend STATS_URC payload

In the existing periodic `STATS_URC` builder, append the eight new counters
(`pe/fe/ne/ore` × {LPUART1, USART1}) plus existing `host_parse_err`. Bump
URC version byte if the payload format is versioned. Keeps it observable
without polluting the diag stream.

### A4. RX stimulus harness

Reuse [`bench-evidence/T6_W1-7_RX_session_2026-05-11/test_rx_isr.sh`](../DESIGN-CONTROLLER/bench-evidence/T6_W1-7_RX_session_2026-05-11/test_rx_isr.sh)
unchanged. Add a post-stimulus `VER_REQ` to force a `STATS_URC` (or wait for
the periodic one) and capture it.

**Phase A pass criterion:** one capture file from a fresh boot + 3-burst
stimulus that includes a STATS_URC with the 8-element error vector decoded
in the harness output (e.g. `pe_lpu=0 fe_lpu=2 ne_lpu=0 ore_lpu=18 ...`).
File goes to `bench-evidence/T6_W1-7_phaseA_<timestamp>/`.

**No fix has been attempted at this point.** That is the whole point.

---

## 4. Phase B — Discriminator-driven fix branch

Branch chosen by the dominant counter from Phase A:

### B1. ORE dominant (predicted by H1)

Hypothesis confirmed if `ore_lpu >> fe_lpu + ne_lpu` and `ore_lpu` grows
roughly linearly with bytes-per-burst.

**Fix:** in `RNG_LPUART1_IRQHandler` and `USART1_IRQHandler`, replace the
inline `platform_diag_trace("H:ERR_*")` call with a `s_diag_trace_pending`
bitflag set under the same `s_trace_first_err_*` latch. Emit the trace
text from the main loop (`host_uart_poll_dma()` already runs there) right
after the existing `host_uart_take_diag_marks()` call. ISR stays
non-blocking; trace still surfaces once per frame courtesy of A2.

### B2. FE dominant (predicted by H4 — already falsified, included for completeness)

Try `USART_CR1.OVER8 = 1` (8× oversampling, halves the BRR sample-stretch
penalty) and `USART_CR3.ONEBIT = 1` (sample on a single bit centre instead
of three-of-five). LPUART1 doesn't support OVER8 — only USART1 does — so
this also forces us to switch primary RX to USART1 if FE survives.

### B3. NE dominant (predicted by H3)

Increase pull strength on PA3 (already PUPDR=01, can't go higher in HW —
add an external 10 k to 3V3 if the issue persists), and reduce GPIO output
slew on the Max Carrier UART4_TXD side via `pinctrl` if that knob exists.

### B4. Mixed / inconclusive

Run B1 *and* A2's per-frame reset together and re-measure. If the vector
flattens, ISR-self-stretch was the dominant effect masking everything else.
If FE/NE still dominate, escalate to Phase B2/B3.

### B-disqualification (any branch)

After applying a B-branch fix, re-run the Phase A capture. The hypothesis
is *confirmed* only if the targeted counter drops to ≤ 1 % of its prior
value AND `H:COBS_ERR` is no longer present. Anything less = falsified,
roll back, pick next branch.

---

## 5. Phase C — Validation gates

Three sequential gates, each must pass before the next runs:

1. **Stimulus gate.** `test_rx_isr.sh` capture: zero `H:COBS_ERR`, all
   error counters ≤ 1 over a 3-burst (300+ byte) stimulus.
2. **Probe gate.** [`method_g_stage1_probe.py`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py)
   → `0x81` response to a `VER_REQ` (full COBS round-trip).
3. **Quant gate.** `run_stage1_standard_quant_end_to_end.ps1 -Cycles 20`:
   ≥ 18/20 cycles produce BOOT_URC + successful VER_REQ→VER_URC.

Only after all three: tick W1-7, mark Step 6 complete, advance gate state to GO.

---

## 6. Phase D — HSE/TCXO followup (deferred until W1-7 clears)

Out of scope for this plan beyond:

- File a separate Wn ticket "Murata HSE/TCXO does not lock on Max Carrier".
- Capture evidence: VDD_TCXO rail probe (DMM or scope on the carrier test
  point, if exposed) and `BRINGUP_MAX_CARRIER §R12` cross-reference.
- Decision deferred: live with HSI16 (BRR re-validated for every baud we
  use) vs. patch hardware vs. switch to internal MSI for higher resolution.

---

## 7. Phase E — Cleanup after W1-7 clears

- Remove or `#ifdef` the `C:CLK_HSI_16M` / `C:CLK_HSE_32M` diag trace.
- Remove the PA12 TCXO_EN init from `platform_clock_init_hsi16()` *only if*
  we've confirmed it's not the right pin (Phase D output). Until then,
  leave it — it's harmless.
- Decide whether the per-flag error counters stay (recommended: yes, behind
  a config flag, since they're a permanent observability win).
- Remove any A2 / A3 instrumentation that wasn't promoted to permanent.
- Update [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42) with final
  disposition + a one-line root-cause statement.
- Append a Phase A→C evidence summary to the §3 trail of
  [`2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md`](2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md).

---

## 8. Risk register

| Risk | Likelihood | Mitigation |
|---|---|---|
| Phase A instrumentation itself perturbs ISR timing enough to mask the bug | Low — counter increments are <10 cycles each | Keep adds before the existing diag trace; don't reorder anything else |
| ORE dominates but B1 fix doesn't help (because the trace never blocked long enough to matter) | Low–Med | B4 plus revisiting H2/H3 |
| Periodic STATS_URC TX itself causes ISR self-stretch on the *RX* peripheral | Low (TX is on a separate UART entity from the RX-error path's blocking call) | If suspected, add a one-shot dump-on-VER_REQ instead of periodic |
| Bench regression mid-plan (boards lock up) | Med — has happened before | Established recovery: gpio10 toggle + `systemctl restart m4-proxy.service` |
| User reads vector and changes priority mid-plan | Expected — that's the point of Phase A | Plan is explicit about phase boundaries; B-branch is data-driven |

---

## 9. Effort + sequencing

Single agent session covers Phase A1+A2+A3 (~30 min code, one flash, one
capture). Phase B is one branch + one flash + one capture. Phase C is one
script run. Phase D is a separate ticket. Phase E is bookkeeping.

**Recommended next action when work resumes:** start Phase A1 (per-flag
counters in `RNG_LPUART1_IRQHandler` and `USART1_IRQHandler`). All other
Phase A items chain off that file's diff anyway.
