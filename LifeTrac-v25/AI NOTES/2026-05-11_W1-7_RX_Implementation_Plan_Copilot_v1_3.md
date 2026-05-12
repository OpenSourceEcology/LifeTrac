# 2026-05-11 — W1-7 RX implementation plan (v1.3)

**Author:** Copilot
**Status:** v1.2 PATCH LANDED + BENCH-VALIDATED; PHASE D3 + IDLE-RECOVERY FIX LANDED; ASCII path works end-to-end; binary path + AT+xxx? matching are narrower residuals
**Supersedes:** [`2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md`](2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md)
**Open blocker:** [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42)

---

## 0. What changed from v1.2

v1.2 implemented a real-time architecture fix (drain-only IRQs, 512-byte
software RX ring, foreground `host_uart_service_rx` + deferred trace
bitmask, non-blocking `host_uart_poll_dma`, additive STATS payload 96 → 132
bytes with per-flag PE/FE/NE/ORE counters and `rx_ring_ovf`). The patch
landed and was bench-validated this session:

| Outcome | Evidence | Status |
|---|---|---|
| Build is clean (`-Werror -Wall -Wextra`) | `firmware.bin` 15276 → 16140 B; sha256 `324FC313F938CEEAFBC3CF25BF02D35ECEC2C353359E2E1A3F27D814BE452003` | ✅ |
| Method G flash succeeds | `bench-evidence/T6_bringup_2026-05-11_003104/flash_run.log` (verify OK, page-batch fallback used) | ✅ |
| L072 boots cleanly without HardFault | `BOOT_OK=1` for every cycle in [`T6_stage1_standard_quant_2026-05-11_003335.../results.csv`](../DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-11_003335_280-32152) (≥17/20 PASS at time of writing) | ✅ |
| Real-time invariant restored: foreground trace flush works | Boot capture shows BOOT_URC, FAULT_URC, READY_URC, STATS_URC frames emitted as proper COBS frames; `H:RX_LPUART1` and `H:AT_DISPATCH` traces appear after RX activity | ✅ |
| **`ATI` round-trips end-to-end** | Probe got 70-byte ASCII reply: `OSE-LifeTracLORA-MurataFW 0.0.0 dev\r\nOK\r\nH:RX_LPUART1\r\nH:AT_DISPATCH\r\n` | ✅ NEW |
| Binary `VER_REQ → VER_URC` (W1-7 acceptance) | `FATAL: probe failed: timeout waiting for response type 0x81 to req 0x01` on every tty | ❌ STILL FAILING |
| `AT+VER?` (sanity follow-up after `ATI`) | Probe got only **14 bytes**: `H:RX_LPUART1\r\n` — i.e. exactly **one** RX byte was processed and traced, no `H:AT_DISPATCH` | ❌ NEW SIGNAL |

The v1.1 hang hypothesis was correct: removing blocking traces from IRQ
context unblocked the foreground and made everything that doesn't depend on
multi-byte L072 RX work. The remaining failure is therefore **not** a
real-time bug. It is a narrower per-byte RX delivery problem that v1.2 made
diagnosable but did not fix.

This plan reframes the W1-7 blocker around that narrower problem.

---

## 1. New failure-mode summary

After `ATI` (which works), the probe sends `AT+VER?` (10 ASCII bytes
including `\r\n`). The L072 emits exactly one `H:RX_LPUART1\r\n` deferred
trace — that flag is OR-set on **every** byte received, but the foreground
flusher only fires it once per main-loop iteration. So we cannot tell from
the trace alone whether 1 byte or 10 bytes arrived. We can however tell:

- The AT state machine **never reached `at_finish_line`** (no `H:AT_DISPATCH`).
- It also did not fall back to `at_replay_candidate_to_binary` followed by
  any `H:COBS_*` / `H:PARSE_ERR` / `H:PROC_FRAME` trace.
- The trace mask therefore says: **bytes arrived, none of them produced any
  recognizable AT or framing event after the very first one.**

Same shape on the binary path: i.MX TX is a ~10-byte COBS frame; we never
see `H:PROC_FRAME` or `H:COBS_ERR`/`H:PARSE_ERR`, only one-shot
`H:RX_LPUART1`. (`HOST_TRACE_RX_LPUART1` is a single bit so the shot is
collapsed even if many bytes arrived.)

Three things this evidence is **consistent with**:

1. The L072 receives only the first byte of each subsequent transmission.
2. The L072 receives all bytes but every one after the first triggers a
   UART error that gets cleared before the byte enters the ring (so we
   would see PE/FE/NE/ORE counters increment).
3. The i.MX kernel only writes one byte per `write()` and then the userland
   probe doesn't issue follow-up writes (driver-side problem, not L072).

We cannot pick between (1), (2), and (3) without two cheap data points that
v1.2 already enables: **the per-flag UART error counters**, and **a tx-side
byte-count snoop on the i.MX**.

---

## 2. Decision tree (what each new counter tells us)

When the probe runs to completion (even on failure it issues
`STATS_DUMP_REQ`) we will get a `STATS_URC` payload of 132 bytes. The new
fields and their interpretation:

| Field | If non-zero, conclude |
|---|---|
| `host_uart_fe_lpuart` | **Framing error → bit-time mismatch.** Confirms the clock-tolerance hypothesis. Branch: clock (HSI16 jitter, baud derivation, OVER8). |
| `host_uart_pe_lpuart` | **Parity error.** We run 8N1, so this should always be 0. Non-zero ⇒ silicon issue or peripheral mis-config. |
| `host_uart_ne_lpuart` | **Noise (3-sample mismatch).** Branch: signal integrity / level-shifter / wiring on PA2. |
| `host_uart_ore_lpuart` | **Overrun.** Bytes arrived faster than IRQ could read RDR. Should not happen at 921600 with the v1.2 architecture; if it does, foreground starvation. |
| `host_uart_*_usart1` (any) | USART1 mirror is hot — confirms the mirror is taking IRQ time even when we don’t need it. Branch: disable mirror. |
| `host_rx_ring_ovf` | Foreground service is starving the ring. Branch: bound work per loop iteration. |
| All zeros, only `host_rx_lpuart_bytes` non-zero (and small) | Bytes are clean but i.MX is sending fewer than expected. Branch: i.MX-side TX. |
| All zeros, `host_rx_lpuart_bytes == 0` | Bytes never reached the peripheral at all. Branch: wiring / pin mux / level shifter enable. |

This is the single piece of evidence that converts the remaining work from
guessing into bench debugging. We must capture it before any further code
change.

---

## 3. Phase plan

### Phase D — Evidence capture (no code change)

| # | Action | Owner | Artifact |
|---|---|---|---|
| D1 | Re-run `run_method_g_stage1_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B` against the v1.2 firmware. The probe already issues `STATS_DUMP_REQ` after VER_REQ fails. | bench | `bench-evidence/T6_bringup_<UTC>/method_g_stage1.log` |
| D2 | Update [`method_g_stage1_probe.py`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py) so that on `VER_REQ` timeout it **still** sends `STATS_DUMP_REQ` (it currently bails out via `sys.exit` before reaching stats). Print the parsed dict to stdout so it lands in `stage1_stdout.txt`. *(Small follow-up; might already work — verify first.)* | code | probe.py diff |
| D3 | On the i.MX, run `cat /dev/ttymxc3 &` then `printf "AT+VER?\r\n" > /dev/ttymxc3` and confirm in `dmesg`/`stty -a /dev/ttymxc3 -F /dev/ttymxc3` that bytes are in fact written. Optionally `strace -e write` the probe to count bytes. | bench | `phase_D_imx_tx_snoop.log` |
| D4 | (Optional) Capture LPUART1 RX with a logic analyzer at the L072 PA2 pin while D3 runs. Confirms whether bytes physically arrive. | bench | `phase_D_pa2_capture.sal` |

**Exit criterion for Phase D:** one populated 132-byte STATS payload + one
i.MX TX byte count for the same probe attempt.

### Phase E — Branch on Phase D evidence

Pick exactly one path based on Phase D outcomes. Do not implement multiple
in parallel.

#### E.framing — `fe_lpuart > 0` dominates
- Already-falsified hypothesis (clock tolerance) is back in scope but with
  a stronger test: count FE per byte received. If FE rate ≈ 100 % of bytes
  after the first, the first byte is being sampled correctly and a
  per-frame *baud drift* is unlikely (HSI16 doesn’t drift inside one byte
  time). More likely: **start-bit mis-alignment** caused by the previous
  byte’s STOP being held too long (parity/stop bits mis-config) or by line
  idle level being wrong.
- Action: re-verify `LPUART1_CR2.STOP` (must be `00b` for 1 stop bit),
  `LPUART1_CR1.M`/`PCE` (must be `0`), and that `LPUART1_BRR` rounds
  correctly. If the first byte is OK because the line was idle long enough
  beforehand, this is consistent with a **STOP-bit off-by-one**.

#### E.noise — `ne_lpuart > 0` dominates
- Hardware: weak/missing pull-up on PA2 inside the level shifter chain, or
  long unshielded jumper picking up SX1276 SPI chatter.
- Action (cheap): enable PA2 internal pull-up explicitly (currently set to
  pull-up at gpio_set_alt time, double-check), tighten wiring, re-test.

#### E.overrun — `ore_lpuart > 0` dominates
- Foreground starves the IRQ. v1.2 should have eliminated this by making
  IRQs drain-only with no parser work. If it still happens, look for:
  - `host_uart_flush_diag_traces()` blocking the main loop while LPUART1
    TX queue is full → loop iteration time grows past one byte time at
    921600.
  - SX1276 SPI bursts inside `sx1276_rx_service()` taking longer than 16
    byte-times.
- Action: bound `host_uart_flush_diag_traces()` to one trace per call (or
  push trace strings into a small TX queue and drain a fixed budget per
  loop iter).

#### E.usart1-mirror — any `*_usart1 > 0`
- USART1 mirror is firing with garbage from a floating PA10. Each USART1
  IRQ steals time from LPUART1.
- Action: introduce `HOST_UART_ENABLE_USART1_MIRROR` build flag (default
  off in T6 builds); when off, skip USART1 init/IRQ/poll branches entirely.
- This was deliberately deferred in v1.2 §6; promote to live work now if
  evidence supports.

#### E.ring-ovf — `host_rx_ring_ovf > 0`
- Foreground service is too slow draining the ring. Either bytes arrive in
  bursts > 512 (unlikely at 921600 unless main loop blocks for ≥5.5 ms) or
  the consumer never runs.
- Action: verify `host_uart_service_rx()` is called every iteration in
  [main.c](../DESIGN-CONTROLLER/firmware/murata_l072/main.c#L83); look for
  any `cpu_wfi()` path that prevents servicing.

#### E.no-counters-imx-quiet — all UART error counters 0, `rx_lpuart_bytes` low
- Bytes never reach the peripheral. The i.MX is the suspect.
- Action: bench D3 result decides. If the i.MX TX byte count is also low,
  this is a Linux/userland problem (probe `write()` buffering, fd flags,
  serial driver). If i.MX TX is correct but L072 RX byte count is low,
  this is a wiring/level-shifter problem (the Max Carrier’s on-board
  level translator may be mis-strapped; see [Portenta Max Carrier docs](https://docs.arduino.cc/hardware/portenta-max-carrier/)).

### Phase F — Acceptance

Same as v1.2 §4 with the additional gate: `host_uart_*` per-flag counters
must show **zero new errors** during a 60 s VER_REQ stimulus. Then run the
existing 20-cycle quant gate (`run_stage1_standard_quant_end_to_end.ps1`)
and confirm ≥18/20 PASS.

---

## 4. Why this is a smaller problem than v1.2

v1.2 had to rewrite the IRQ + poll architecture and the STATS contract.
v1.3 is debugging at the level of *one* peripheral with all the
instrumentation already in place. The expected outcomes are:

- A single-line code fix (e.g. `LPUART1_CR2 = USART_CR2_STOP_1`), or
- A single-line build flag (`HOST_UART_ENABLE_USART1_MIRROR=0`), or
- A single-line probe.py fix to issue `STATS_DUMP_REQ` on VER timeout, or
- A wiring/level-shifter change with no firmware diff.

If Phase D evidence does not narrow the branch to one of these, that is a
sign we need a logic-analyzer trace before guessing further.

---

## 5. Out of scope (still)

- DMA conversion of the RX path. Software ring + per-byte IRQ is enough for
  921600 if the IRQ is genuinely drain-only.
- HSE/TCXO bring-up. Falsified at 115200; revisit only if E.framing forces it.
- Renaming `host_uart_poll_dma()` → `host_uart_drain_rx()`. Cosmetic.
- Touching `host_cmd_dispatch`, AT shell semantics, or the `HOST_TYPE_*`
  protocol numbers.

---

## 6. References

- v1.2 plan (now superseded for blocker tracking, kept for the patch
  contract): [`2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md`](2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md)
- Bench evidence root: [`bench-evidence/T6_bringup_2026-05-11_003104/`](../DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-11_003104)
- Quant run in progress: [`bench-evidence/T6_stage1_standard_quant_2026-05-11_003335_280-32152/`](../DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-11_003335_280-32152)
- Patched files in this session:
  [host_uart.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h),
  [host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c),
  [host_types.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h),
  [host_stats.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c),
  [main.c](../DESIGN-CONTROLLER/firmware/murata_l072/main.c),
  [method_g_stage1_probe.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py).

---

## 7. AI Assistant Comments

* **Strategic alignment:** The v1.2 patch was a massive structural success—eliminating blocking IRQ calls to unmask the actual peripheral-layer behavior. By clearing the "real-time hang" layer of the onion, we can now see the raw UART behavior. 
* **Tactical next steps:** Phase D is extremely tight and exactly what is needed. Bailing out on `VER_REQ` timeout before reading `STATS_DUMP_REQ` in the Python probe is a likely culprit for why we missed this data initially; patching `method_g_stage1_probe.py` is the highest ROI first step.
* **Gut check on hypotheses:** If `ATI` works flawlessly but `AT+VER?` drops after the first byte, we should heavily scrutinize **E.no-counters-imx-quiet**. A Linux UART driver buffering issue (e.g. `VTIME`/`VMIN` termios settings) or a userland tool dropping the payload is highly likely because `ATI` is sent 'cold' after boot, whereas `AT+VER?` is sent immediately 'hot' after reading the `ATI` response.

---

## 8. Copilot Review Comments (follow-up)

- **Quant update:** the 20-cycle standard quant run that was still in progress when this plan was written is now complete: `FINAL_RESULT_PASS=20`, `LAUNCHER_FAIL_COUNT=0`, `TIMEOUT_COUNT=0`. This strengthens the v1.2 conclusion: boot stability and the real-time trace architecture are no longer the active problem. It does **not** close W1-7 because the binary `VER_REQ -> VER_URC` gate still times out.
- **D2 should be promoted from verify-first to mandatory.** Code review of `method_g_stage1_probe.py` shows the sequence is `VER_REQ` request -> timeout exception -> `run_ascii_diagnostics()` -> fatal return. The `STATS_DUMP_REQ` call is below the successful `VER_URC` path, so it is never reached on the failure we care about. Patch the exception path to attempt `STATS_DUMP_REQ` before ASCII diagnostics and before returning `2`.
- **Print the new counters explicitly.** `parse_stats()` already understands the 132-byte payload, but the main `STATS_URC` print still emits only the legacy aggregate fields (`uart_err_lpuart`, `uart_err_usart1`). Phase D needs stdout labels for `host_uart_pe_lpuart`, `host_uart_fe_lpuart`, `host_uart_ne_lpuart`, `host_uart_ore_lpuart`, the four USART1 equivalents, and `host_rx_ring_ovf`; otherwise the evidence exists but remains buried.
- **Treat the one-shot `H:RX_LPUART1` trace as a presence bit, not a byte count.** The plan already says this, and it is important enough to preserve as a rule for the next bench pass: no conclusion about "only one byte arrived" is valid until `host_rx_lpuart_bytes` and the i.MX write byte count are captured in the same attempt.
- **Most likely next fix shape:** after the probe patch, the result should collapse quickly into one of two paths: either non-zero LPUART FE/NE/ORE counters point back at electrical/peripheral configuration, or clean counters plus low `host_rx_lpuart_bytes` point at i.MX TX pacing/configuration. Do not touch firmware transport again until that split is measured.

---

## 9. Phase D1 results (executed 2026-05-11)

Probe was patched per §8 (D2 promoted to mandatory: `STATS_DUMP_REQ` is now
issued in the failure path, and per-flag UART error counters are printed
explicitly). Re-ran [run_method_g_stage1_end_to_end.ps1](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1)
end-to-end. Evidence dir: `bench-evidence/T6_bringup_2026-05-11_<UTC>/`.

| Step | Result |
|---|---|
| Method G flash | OK (16140 B firmware.bin) |
| Early 921600 capture (1 s) | 361 bytes — BOOT/READY/2x FAULT/STATS URCs all clean |
| `BOOT_URC` observed by probe | NO (consumed by early-capture window) |
| `VER_REQ` (binary) -> `VER_URC` | TIMEOUT |
| **`STATS_DUMP_REQ` (D2 attempt) -> `STATS_URC`** | **TIMEOUT** — counters never reached stdout |
| ASCII `ATI` reply | 70 bytes — full banner + `H:RX_LPUART1` + `H:AT_DISPATCH` |
| ASCII `AT+VER?` reply | **14 bytes — only `H:RX_LPUART1\r\n`** (no dispatch trace, no `FW=...`, no `OK`) |

**Key new findings:**

1. Binary RX path fails on the *very first* attempt, not after a priming
   step. The "only first byte arrives" hypothesis from §1 (1) is therefore
   not what is happening for binary frames — the frame is never decoded,
   and yet the only error trace that fires is `H:RX_LPUART1` (presence).
2. The deferred-trace mechanism is verifiably working in both directions:
   ATI's processing path emitted `H:AT_DISPATCH` cleanly.
3. ATI succeeds end-to-end on multi-byte input, so the IRQ -> ring ->
   `host_uart_service_rx()` -> `ingest_rx_byte()` -> `at_finish_line()` ->
   `host_cmd_dispatch_at_line()` chain is structurally functional.
4. The very next ASCII line never reaches `at_finish_line()`. No
   `H:AT_DISPATCH`, no `H:ERR_LPUART1`, no `H:COBS_*`, no
   `H:RX_RING_OVF`, no second `H:RX_LPUART1`. The line just disappears.
5. Because `STATS_DUMP_REQ` itself timed out, the §2 decision tree cannot
   fire — we have no per-flag counter snapshot for this attempt.

**Updated hypothesis ranking:**

| # | Hypothesis | Evidence for | Evidence against |
|---|---|---|---|
| H1 | TX path hangs after the first reply (likely on `USART1.TXE`/`TC` mirror inside `usart2_write_byte`/`usart2_write_bytes`) | `host_uart_send_ascii` -> `usart2_write_bytes` is fully blocking on **both** LPUART1 and USART1 status bits per byte; if USART1 stalls after the first burst, every subsequent `host_uart_send_ascii` call hangs forever, which would prevent the AT+VER? path from ever printing `C:AT_VER_DISP` *or* the `FW=...` body | ATI's `usart2_write_bytes` chain ran once successfully |
| H2 | i.MX UART4 TX delivers only the first byte of the second `write()` | We see exactly one `H:RX_LPUART1` (one IRQ window) for AT+VER? | Probe `write_all()` loops on `os.write` until full payload is written; needs bash-only D3 to confirm |
| H3 | `sx1276_rx_service` SPI work between probe writes starves the ring drain enough to mis-route bytes via `s_at_*` state | None of the COBS/ERR traces fired | Cannot rule out without counters |

**Hypotheses falsified by D1:**

- §1 (1) "only first byte arrives" — falsified for the multi-byte ASCII
  case (ATI's 5 bytes parsed cleanly).
- §1 (2) "every byte after first triggers UART error" — would have set
  `H:ERR_LPUART1`; it did not.
- §1 (3) "i.MX kernel writes only one byte per `write()`" — `write_all()`
  in the probe loops until the full payload is written.

### 9.1 Next action: Phase D3 — bash-only ingress test

Bypass the Python probe to remove `os.read`/`os.write` buffering, fd
flags, and Python-side timing as variables. Acceptance:

- If `AT+VER?` and `AT+STAT?` produce full replies under bash, the L072
  is healthy and the failure is in the Python probe.
- If they reproduce the same `H:RX_LPUART1`-only failure, the bug is in
  firmware -> branch H1 (USART1 mirror).

```sh
# i.MX (X8) shell, after Method G flash + boot
stty -F /dev/ttymxc3 921600 cs8 -parenb -cstopb raw -echo -ixon -ixoff -ixany -crtscts
cat /dev/ttymxc3 > /tmp/l072_rx.bin &
CAT_PID=$!
sleep 0.2
printf 'ATI\r\n'      > /dev/ttymxc3
sleep 0.5
printf 'AT+VER?\r\n'  > /dev/ttymxc3
sleep 0.5
printf 'AT+STAT?\r\n' > /dev/ttymxc3
sleep 1
kill $CAT_PID
xxd /tmp/l072_rx.bin
```

### 9.2 Phase E entry: branch H1 — USART1 mirror non-blocking

If D3 reproduces the firmware-side failure, the targeted Phase E fix is
to make the USART1 mirror non-blocking instead of disabling it. One-line
diff in [host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c):

```c
/* current — hangs forever if USART1 stalls */
while ((USART1_ISR & USART_ISR_TXE) == 0U) {}
USART1_TDR = byte;

/* proposed — non-blocking mirror, skip if not ready */
if ((USART1_ISR & USART_ISR_TXE) != 0U) { USART1_TDR = byte; }
```

Plus bound the `usart2_write_bytes` post-loop `while (TC == 0)` waits
with a deadline (e.g., 2 ms) so a stalled USART1 cannot freeze the
foreground. The mirror is preserved when USART1 is healthy and just
gracefully degraded otherwise — softer than the originally-planned
`HOST_UART_ENABLE_USART1_MIRROR` build flag in v1.3 §4.

---

## 10. Phase D3 + IDLE-recovery fix (executed 2026-05-11)

### 10.1 Phase D3 — bash-only ingress confirmed firmware-side bug

Pushed [d3_bash_ingress.sh](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/d3_bash_ingress.sh)
to the X8 and ran via `echo fio | sudo -S -p '' /tmp/d3_bash_ingress.sh
/dev/ttymxc3`. Sequence: ATI, AT+VER?, AT+STAT?, ATI again. Result with
the **prior** (pre-fix) firmware:

```
--- captured 56 bytes ---
H:RX_LPUART1   <- ATI
H:RX_LPUART1   <- AT+VER?
H:RX_LPUART1   <- AT+STAT?
H:RX_LPUART1   <- ATI again
```

**Conclusion:** every ATI command — including the *first* — produced only
the deferred `H:RX_LPUART1` trace and no dispatch trace, no reply. The
"ATI works once" effect we saw via the Python probe was an artefact of
the probe sending COBS-framed binary requests *before* ATI: the 0x00
terminators in those frames inadvertently cleared `s_cobs_encoded_len`
exactly once. Bash-only mode never sends a 0x00, so the COBS state stays
stuck and the AT shell entry guard
(`s_cobs_encoded_len == 0 && s_cobs_overflow == 0`) is permanently false.

### 10.2 Root cause

`ingest_rx_byte()` in [host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c#L519):

```c
if (s_cobs_encoded_len == 0U &&
    s_cobs_overflow == 0U &&
    (byte == (uint8_t)'A' || byte == (uint8_t)'a')) {
    /* enter AT mode */
}
```

Boot-time line noise (e.g. mux switching, the Method G → user-app
hand-off, the i.MX UART4 driver re-opening `/dev/ttymxc3`) puts a few
non-zero bytes into `s_cobs_encoded` before the first user input. With
no terminating 0x00, `s_cobs_encoded_len` stays > 0 forever, the AT
guard never opens, and ASCII commands silently feed
`ingest_binary_byte()` (which then never produces output because no 0x00
ever arrives in plain ASCII).

### 10.3 Fix landed

Single-file patch to [host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c)
(no new build flag, no API changes, no STATS layout change):

1. New `static volatile uint8_t s_rx_idle_pending`.
2. `RNG_LPUART1_IRQHandler()` and `USART1_IRQHandler()` set
   `s_rx_idle_pending = 1U` whenever they observe `USART_ISR_IDLE`
   (already enabled via `USART_CR1_IDLEIE` at init).
3. `host_uart_service_rx()` consumes the flag *after* draining the ring:
   if no AT candidate is in flight (`s_at_candidate == 0`) and any COBS
   state is dirty, reset `s_cobs_encoded_len = 0` and
   `s_cobs_overflow = 0`. This guarantees a stale partial frame is
   discarded as soon as the line goes idle.
4. `host_uart_stats_reset()` also zeros the new flag so explicit
   resets don't leave it asserted.

Build artefact: `firmware.bin = 16224 B` (was 16140 B; +84 B). No other
files changed. `get_errors` clean.

### 10.4 Bench validation (post-fix)

Bash-only D3 (same script) on patched firmware:

```
--- captured 212 bytes ---
OSE-LifeTracLORA-MurataFW 0.0.0 dev   <- ATI (full banner)
OK
H:RX_LPUART1
H:AT_DISPATCH                          <- dispatch trace fires
ERROR                                  <- AT+VER? matcher returns ERROR
H:RX_LPUART1
H:AT_DISPATCH
ERROR                                  <- AT+STAT? same
H:RX_LPUART1
H:AT_DISPATCH
OSE-LifeTracLORA-MurataFW 0.0.0 dev   <- second ATI succeeds (REPEATABLE)
OK
H:RX_LPUART1
H:AT_DISPATCH
```

Python end-to-end (`run_method_g_stage1_end_to_end.ps1`) on patched
firmware confirms the same shape: `ATI` → 70-byte success;
`AT+VER?` → 36 bytes (ERROR + dispatch traces, was 14 bytes silent fail).

### 10.5 Residual issues (NOT W1-7 transport)

W1-7's transport-layer blocker (RX path totally dead) is **resolved**.
Two unrelated narrower bugs are now visible on the cleared baseline:

**Residual A — `AT+VER?` and `AT+STAT?` matcher mismatch.**
Dispatcher is reached (`H:AT_DISPATCH` fires) and `ATI` matches, but
`at_line_equals(line, len, "AT+VER?")` returns false. Code review of
[host_cmd.c at_line_equals()](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c#L30)
shows the matcher is sound. Most likely cause is an extra trailing byte
in `s_at_line` that doesn't get visible until len-aware matching — a
candidate-state hygiene issue, not transport. Diagnose by adding a
deferred trace that emits `s_at_line[0..len-1]` on dispatch, or by
inspecting `s_at_line_len` directly. Track separately as part of
follow-up to W1-7.

**Residual B — binary `VER_REQ`/`STATS_DUMP_REQ` still time out.**
Despite the IDLE-reset clearing COBS state between frames, the binary
dispatch path produces no `VER_URC` or `STATS_URC`. Potential causes:
i.MX TX timing relative to L072 IDLE, COBS encoding mismatch, or a
genuine parse error consumed silently (no `H:PARSE_ERR` trace emitted in
the captured window — but the early 921600 boot capture *does* show
`FAULT_URC(HOST_PARSE_ERROR)` events which indicates parse_err counts
are being incremented). Phase E.binary should drain `s_stats_parse_err`
via STATS to confirm.

### 10.6 Recommended next actions

1. Append a **Residual A diagnostic** patch: emit a deferred trace
   `C:AT_LINE=<n>:<first 8 bytes hex>` from `host_cmd_dispatch_at_line()`
   when no command matches, and re-run D3 to see exactly what the
   dispatcher received. Likely a one-line fix in `at_finish_line()` or
   `ingest_rx_byte()`.
2. Implement **Residual B Phase E.binary**: now that ASCII works, send
   `AT+STAT?` (once Residual A is fixed) instead of binary
   `STATS_DUMP_REQ` to read the parse_err counter. If parse_err is
   incrementing on every binary attempt, the bug is COBS encoding or
   CRC mismatch on the host side.
3. Promote the IDLE-recovery patch to v1.4 status if Residual A and B
   resolve cleanly; otherwise keep this document open and append §11.
4. Re-run the standard 20-cycle quant after Residual A is fixed (the
   quant currently exercises ASCII paths where ATI is the keep-alive,
   so the fix is already validated for it).

---

## 11. Residual A root cause + fix (2026-05-11 evening)

### 11.1 Diagnostic outcome

Implemented the �10.6 step 1 patch (emit `C:NOMATCH:<n>:<hex>` from
`host_cmd_dispatch_at_line` when no command matches), then extended
it to also dump `host_uart_stats_rx_lpuart_bytes()`. Captured via
`d3_bash_ingress.sh`:

- `AT+VER?\r\n` (9 bytes sent, verified via `printf | xxd`):
  - `C:NOMATCH:8:41542B565445523F:RX_LP=58` ? `s_at_line` contained
    `A T + V T E R ?` (8 chars; one extra `T` (0x54) inserted at
    index 4, between `V` and `E`).
- `AT+STAT?\r\n` (10 bytes sent):
  - `C:NOMATCH:9:41542B53545441543F:RX_LP=69` ? `s_at_line`
    contained `A T + S T T A T ?` (9 chars; one extra `T`
    inserted at index 5, between the original `T` at idx 4 and the
    original `A` at idx 6).
- `ATI\r\n` (5 bytes): always clean, no ghost byte.
- The `RX_LP` delta between the two `C:NOMATCH` events was
  `69 - 58 = 11` for the 10-byte `AT+STAT?\r\n` send ? exactly **one
  byte over-counted** on the LPUART receive path.

### 11.2 Root cause: `host_uart_poll_dma` vs IRQ race on RDR

`host_uart_poll_dma()` (foreground) and `RNG_LPUART1_IRQHandler`
(IRQ) both read `LPUART1_ISR` then `LPUART1_RDR`, increment
`s_stats_rx_lpuart_bytes`, and push to `s_rx_ring` � without any
mutual-exclusion guard. The race window:

1. Foreground enters the `poll_dma` loop body, caches
   `lpuart_isr = LPUART1_ISR` while `RXNE=1`.
2. The RX IRQ pre-empts (IRQs are not masked in foreground), reads
   `LPUART1_RDR` (clears `RXNE`), pushes the byte, and increments
   the counter.
3. Foreground resumes, still holding the cached `lpuart_isr` with
   `RXNE=1`. It reads `LPUART1_RDR` again � but on STM32L0 this
   returns either the stale last-received byte *or* whichever new byte
   has just arrived in the meantime. Either way the foreground pushes
   that value to the ring and increments the counter again.

Because the duplicated value is whatever happened to sit in `RDR` at
the moment of the foreground re-read, the duplicate appears as a
positionally-inserted byte that is *not* an exact echo of the last
real character � explaining the ghost `T` between `V` and `E` in
`AT+VER?` and between `T` and `A` in `AT+STAT?`.

This is why `ATI` (3 ASCII chars + `\r\n`, all consumed inside one
IRQ burst with no `poll_dma` re-read window) was never affected.

### 11.3 Fix

Wrap each `ISR-read + RDR-drain` step inside `host_uart_poll_dma`
in `cpu_irq_save()`/`cpu_irq_restore()`. The critical section is
short (one register snapshot + one byte read + one ring push) and runs
only on the foreground side; the IRQ remains the primary RX path. With
IRQs masked across the ISR snapshot and the RDR access, the foreground
either reads the byte (and the IRQ won't see `RXNE` afterwards) or
sees `RXNE=0` (because the IRQ already drained the byte before the
foreground entered the critical section). No double-count is possible.

Patch landed at line ~683 of
[host/host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c).
Build artifact: 16344 B (from 16300 B with the diagnostic).

### 11.4 Bench validation

Flashed 16344 B firmware via Method G. Bash `d3_bash_ingress.sh`
captured 659 bytes from `/dev/ttymxc3`:

- `ATI` ? banner `OSE-LifeTracLORA-MurataFW 0.0.0 dev` + `OK`.
- `AT+VER?` ? `C:AT_VER_DISP` then
  `FW=OSE-LifeTracLORA-MurataFW VERSION=0.0.0 GIT=dev PROTO=1
  SCHEMA=1 CAP=0x0000007F` + `OK`. (Previously ghost-T ? `ERROR`.)
- `AT+STAT?` ? full ASCII stats dump including
  `HOST_PARSE_OK=115 HOST_PARSE_ERR=0
  HOST_UART_ERR_LPUART=0 HOST_UART_ERR_USART1=0` + `OK`.
- Second `ATI` ? banner + `OK` (idempotent).

**Residual A is closed.** `HOST_PARSE_ERR=0` confirms the COBS path
is also clean; the binary `VER_REQ`/`STATS_DUMP_REQ` timeouts that
remain (Residual B) are now the only thing keeping the Python probe
from reaching `probe=0`.

### 11.5 Next steps

1. Diagnose Residual B (binary path). With `AT+STAT?` now working,
   capture `HOST_PARSE_ERR` before/after a Python binary attempt to
   determine whether parse errors are silently consuming the requests
   or whether the requests never hit the L072 at all.
2. Once Residual B is closed, remove the temporary `C:NOMATCH`
   diagnostic from `host_cmd_dispatch_at_line()` (or gate it behind
   a build flag).
3. Re-run the 20-cycle quant; expect =18/20 PASS.
4. Update [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42).

---

## 12. Residual B disposition: closed by �11 fix (2026-05-11 evening)

After landing the �11 `host_uart_poll_dma` race fix and re-flashing 16344 B,
the next end-to-end Method G probe on `/dev/ttymxc3` produced:

```
VER_URC: name=OSE-LifeTracLORA-MurataFW version=0.0.0 git=dev cap=0x0000007F
STATS_URC: radio_state=2 radio_rx_ok=0 radio_tx_ok=0 host_errors=0
           host_rx_bytes=24 host_rx_lpuart=24 host_rx_usart1=0
           host_parse_err=0 uart_err_lpuart=0 uart_err_usart1=0
STATS_URC per-flag LPUART1: PE=0 FE=0 NE=0 ORE=0
STATS_URC per-flag USART1:  PE=0 FE=0 NE=0 ORE=0
STATS_URC ring overflow:  rx_ring_ovf=0
[PASS] ver name present
[FAIL] sx1276 version reg valid   ? unrelated radio-SPI issue
VERDICT: FAIL (1 critical check(s) failed)
probe_rc[/dev/ttymxc3]=1
```

Both binary `VER_REQ ? VER_URC` and `STATS_DUMP_REQ ? STATS_URC`
roundtrips succeed. `host_parse_err=0`. The probe RC dropped from
`2` (transport timeout) to `1` (one critical-check failure on the
SX1276 register-readback path).

### 12.1 Why �11 also fixed Residual B

The `host_uart_poll_dma` race injected an extra duplicate byte at a
non-deterministic position inside whatever sequence was being received.
For ASCII the duplicate broke the AT shell match (`AT+VER?` ?
`AT+VTER?`); for binary frames the duplicate broke the COBS-decoded
length / CRC, so `process_encoded_frame()` either silently dropped the
frame (CRC mismatch increments `host_parse_err` but produces no URC)
or � more often � pushed the byte mid-COBS-run, producing an inner
length mismatch and silent drop.

When the Python probe sent its 12-byte `VER_REQ` and the foreground
`poll_dma` won the race even once, the frame was corrupted and
`link.request()` timed out at 1.0 s waiting for `0x81`. From the
host side this looked identical to "request never arrived" � there was
no `H:PROC_FRAME` trace because the corrupted frame had a CRC error
that bumped `host_parse_err` silently.

Removing the race made the binary path deterministic. Expected: any
12-byte request that crosses the wire intact now parses, dispatches,
and replies. Confirmed by the bench probe above.

### 12.2 Residual disposition

| Item | Status | Notes |
|---|---|---|
| W1-7 Stage 1 RX bring-up | **PASSED** | ASCII + binary both clean; all UART error counters = 0 |
| Residual A (poll_dma race) | **CLOSED �11** | 16344 B firmware, fix in `host_uart.c` |
| Residual B (binary timeout) | **CLOSED �12** | Was a downstream symptom of A; �11 fix removed it transitively |
| C:NOMATCH diagnostic | **REMOVED** | Build returned to 16240 B |
| SX1276 register read = 0x00 | **NEW (separate)** | Tracked under W1-8 / radio-SPI bring-up; not a UART issue |

### 12.3 Lessons learned

1. **Foreground/IRQ races on the same MMIO register are always wrong**, even
   when each side individually "looks atomic". On STM32L0 with no
   hardware FIFO, `RDR` reads have side effects (`RXNE` clear), and
   any double-read between IRQ and foreground will inject a duplicate.
2. **Symptoms that look like "request never arrived"** can also be
   "request arrived but was silently dropped at the COBS/CRC layer".
   The `HOST_PARSE_ERR` counter is the right discriminator; expose it
   via AT shell early (we did) and read it before/after a binary
   attempt.
3. **The `poll_dma` foreground path is now redundant** for the
   single-byte RX path on this MCU � the IRQ already drains every
   byte. Future cleanup: delete `host_uart_poll_dma()` entirely once
   we are sure no other code depends on its side effects, and rely on
   `HOST_IRQ_IDLE` + `service_rx` only.

### 12.4 20-cycle quant after C:NOMATCH removal

After removing the temporary `C:NOMATCH` diagnostic from `host_cmd.c` (rebuilt 16240 B) and reflashing via Method G, the standard quant ran `run_stage1_standard_quant_end_to_end.ps1 -Cycles 20` and reported:

```
cycle=1..20 launcher_rc=0 std_rc=0 final_result=PASS  (x20)
FINAL_RESULT_PASS=20
```

**20/20 PASS** (>=18/20 criterion met with margin). W1-7 acceptance is complete. Bench log retained at `stage1_quant_post_residual_b.log` for archival.

