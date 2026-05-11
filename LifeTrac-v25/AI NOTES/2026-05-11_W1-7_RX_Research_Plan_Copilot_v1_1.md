# 2026-05-11 — W1-7 RX investigation plan (v1.1)

**Author:** Copilot
**Status:** REVISED after source audit
**Supersedes:** [`2026-05-11_W1-7_RX_Research_Plan_Copilot_v1_0.md`](2026-05-11_W1-7_RX_Research_Plan_Copilot_v1_0.md)
**Open blocker:** [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42)

---

## 0. Bottom line

The v1.0 plan was directionally right about ISR self-stretch, but it was too
conservative and it had one unsafe instruction: "measure first while keeping
`platform_diag_trace()` inside the IRQ." The source audit shows that keeping
that trace in place is itself a large perturbation and probably the failure
mechanism. Measurement that preserves it is not neutral.

The revised plan is:

1. First prove/fix the **architecture invariant**: RX IRQ and IRQ-disabled poll
   paths may only drain bytes, count errors, and set flags. They must not block
   on UART TX, parse frames, CRC frames, dispatch commands, or emit text.
2. Then validate with the same bench stimulus and the normal Stage 1 probe.
3. Only if that does not clear W1-7 do we branch into PE/FE/NE/ORE-specific
   electrical or clock hypotheses.

This keeps the methodology lesson from the failed clock-tolerance diagnosis,
but recognizes a stronger form of evidence: the code path itself violates a
real-time invariant.

---

## 1. Research findings from source audit

### R1. `platform_diag_trace()` is blocking by design

[`hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c):

- It waits for `LPUART1_ISR.TXE` before every byte.
- It waits for `LPUART1_ISR.TC` after the string.
- At 921600 baud, a 14-byte ASCII trace is about 152 us of wall time before
  function overhead. A COBS frame byte time is about 10.85 us.

So any call from RX IRQ context can span many incoming byte times.

### R2. The IRQ does more than drain bytes

[`host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c):

- `RNG_LPUART1_IRQHandler()` prints `H:ERR_LPUART1` and `H:RX_LPUART1` directly.
- `USART1_IRQHandler()` prints `H:ERR_USART1` and `H:RX_USART1` directly.
- Both IRQ handlers call `ingest_rx_byte()`.
- `ingest_rx_byte()` can call `ingest_binary_byte()`.
- On `0x00`, `ingest_binary_byte()` calls `process_encoded_frame()` synchronously.
- `process_encoded_frame()` can print `H:COBS_ERR`, `H:PARSE_ERR`,
  `H:VER_REQ_RX`, `H:FRAME_OK`, and `H:Q_FULL`.

Therefore `H:PROC_FRAME` and `H:COBS_ERR` are not merely parser traces; if the
frame delimiter is consumed by the IRQ path, those traces are also IRQ-context
blocking calls.

### R3. `host_uart_poll_dma()` also creates a blocking critical section

Despite its name, `host_uart_poll_dma()` currently disables interrupts with
`cpu_irq_save()`, then loops over error/RXNE state, and may call
`platform_diag_trace()` and `ingest_rx_byte()` before restoring interrupts.
That means even the foreground fallback can hold `PRIMASK=1` while blocking on
LPUART1 TX completion or parsing/CRC work.

### R4. DMA support is stubbed, not active

The file has DMA buffer/service helpers, but the audit found no active setup of
`DMA1_CPAR`, `DMA1_CMAR`, `DMA1_CCR`, or `USART_CR3_DMAR` for LPUART1 RX.
`host_uart_poll_dma()` is really RXNE/error polling under a global IRQ mask.
This matters because the design comment says "DMA-on-IDLE," but the actual
bring-up path is byte-at-a-time interrupt/poll service.

### R5. STATS_URC is already additive-only

[`include/host_types.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
explicitly says the STATS payload is additive-only and parsers must tolerate
trailing bytes. That makes extra error counters a good permanent observability
addition, not a risky schema break.

---

## 2. Revised hypothesis ranking

| ID | Hypothesis | Status after audit | Immediate discriminator |
|---|---|---|---|
| H1 | RX service self-stretches because blocking diagnostics/parser work run in IRQ or IRQ-disabled context | **Promoted from hypothesis to source-level bug** | Remove/defer those operations; `H:COBS_ERR` disappears and VER_REQ succeeds |
| H2 | USART1 mirror noise corrupts the shared parser | Secondary | If H1 fix fails, disable USART1 mirror and compare counters |
| H3 | PA3 RX line glitch/noise | Secondary | NE/FE counters persist after H1 fix |
| H4 | Baud/clock tolerance | Falsified | 115200 test still failed |
| H5 | COBS/CRC decoder logic bug | Low | Clean RX bytes but parser still fails |

Important nuance: H1 is a **source-level bug**, but W1-7 is not cleared until
bench validation passes. Avoid declaring the board fixed before the stimulus,
probe, and quant gates succeed.

---

## 3. Phase A — minimal architectural fix + observability

This replaces v1.0's "instrument first while leaving traces in the IRQ" phase.

### A1. Create deferred diagnostic flags

Add a small bitmask, for example:

- `HOST_TRACE_RX_LPUART1`
- `HOST_TRACE_RX_USART1`
- `HOST_TRACE_ERR_LPUART1`
- `HOST_TRACE_ERR_USART1`
- `HOST_TRACE_PROC_FRAME`
- `HOST_TRACE_COBS_ERR`
- `HOST_TRACE_PARSE_ERR`
- `HOST_TRACE_VER_REQ_RX`
- `HOST_TRACE_FRAME_OK`
- `HOST_TRACE_Q_FULL`
- `HOST_TRACE_COBS_OVF`
- `HOST_TRACE_AT_DISPATCH`

In RX/parsing code, replace `platform_diag_trace()` with `host_uart_note_trace(flag)`.
The note function must only OR a volatile bitmask and return.

### A2. Add per-flag UART error counters

Keep the v1.0 counter idea, but add it alongside the fix rather than before it:

- LPUART1: `pe`, `fe`, `ne`, `ore`
- USART1: `pe`, `fe`, `ne`, `ore`

Count the raw `ISR` snapshot before clearing flags. Use `uint32_t`, not
`uint16_t`, to match existing stats and avoid wrap during long quant runs.

### A3. Drain-only RX IRQ handlers

`RNG_LPUART1_IRQHandler()` and `USART1_IRQHandler()` should:

1. Snapshot ISR.
2. Clear IDLE if present.
3. If any error flag is present, count each flag, clear all error flags, read
   RDR once to release RXNE, increment aggregate error stats, set deferred
   trace flag, and return.
4. While RXNE is set, read RDR, update counters/seen flags, push raw byte into
   a small RX software ring, set deferred `RX_*` trace flag if this is the
   first observed byte.
5. Do **not** call `platform_diag_trace()`.
6. Do **not** call `ingest_rx_byte()`.
7. Do **not** call COBS decode, CRC, `queue_push()`, or `host_cmd_dispatch()`.

A 256-byte or 512-byte byte ring is enough for the current traffic. On overflow,
count/drop and set `HOST_TRACE_COBS_OVF` or a dedicated `HOST_TRACE_RX_RING_OVF`.

### A4. Fix `host_uart_poll_dma()` naming/behavior enough for bring-up

For this blocker, do not implement full DMA unless needed. Instead, turn the
poll fallback into a non-blocking drain helper:

- Do not hold `PRIMASK=1` across the whole loop.
- Read RDR/error state in short critical sections if needed.
- Push bytes into the same RX software ring used by the IRQ path.
- Emit deferred traces only after interrupts are restored.
- Consider renaming later; not necessary during bring-up.

### A5. Parse in foreground only

At the top of the main loop, after `host_uart_poll_dma()` and before fault/mark
emission, add a service function such as `host_uart_service_rx()` that drains
the software RX ring and calls `ingest_rx_byte()` in foreground context.

Now `process_encoded_frame()` may still print via deferred flags, but it no
longer executes in IRQ context. Parsed protocol frames still land in the
existing `s_frame_queue`, and `host_uart_pop_frame()` remains unchanged.

### A6. Emit deferred traces from foreground only

Add `host_uart_flush_diag_traces()` or fold it into `host_uart_poll_dma()` /
`host_uart_service_rx()` once interrupts are enabled. It maps each pending bit
to the existing ASCII string and calls `platform_diag_trace()` there.

This keeps the bench-visible breadcrumbs while eliminating the timing damage.

### A7. Extend STATS_URC additively

Append the eight per-flag counters to the existing STATS payload and update
`method_g_stage1_probe.py::parse_stats()` labels. Because the payload contract
is additive-only, older parsers should tolerate the trailing bytes.

---

## 4. Phase B — immediate validation of H1

After Phase A builds and flashes:

1. Run the existing RX stimulus script at 921600.
2. Expected result if H1 is confirmed:
   - No `H:ERR_LPUART1` during the valid binary burst, or at most one isolated
     error not followed by `H:COBS_ERR`.
   - `H:VER_REQ_RX` / `H:FRAME_OK` appear for a valid VER_REQ frame.
   - `host_parse_ok` increments.
   - `host_parse_err` remains zero for valid binary frames.
3. Run `method_g_stage1_probe.py` and require a `VER_URC` (`0x81`).
4. If that passes, run the 20-cycle standard quant and require ≥18/20.

Only after those three pass do we call W1-7 cleared.

---

## 5. Phase C — if H1 fix does not clear W1-7

Branch on the new counters:

| Counter result | Next action |
|---|---|
| ORE still dominant | Increase RX ring size; audit remaining IRQ-disabled sections; verify no TX calls under `PRIMASK=1` |
| FE dominant | Re-open clock / frame timing only after 115200 sanity repeat with new deferred-trace firmware |
| NE dominant | Treat as signal integrity / RX idle noise; try external pull-up / pad config / capture on PA3 |
| USART1 counters nonzero while LPUART1 path is active | Disable USART1 mirror PA9/PA10 + IRQ; its shared parser path is not worth keeping during bring-up |
| No UART errors, parse errors persist | Debug COBS/CRC or stimulus framing; compare captured raw bytes to `method_g_stage1_probe.py::build_frame()` |

---

## 6. Why this is better than v1.0

- It does not preserve the suspected timing bug just to measure it.
- It expands H1 from "error-path trace blocks" to the real scope:
  **any trace or parser work reachable from RX IRQ or IRQ-disabled poll context**.
- It removes a hidden timing hazard in `host_uart_poll_dma()`.
- It gives us permanent counters without making counters the first flash.
- It still preserves the falsification discipline: if H1's architectural fix
  does not pass the bench gates, the next branch is selected by counters, not
  by vibe.

---

## 7. Recommended next action

Implement Phase A as one focused patch:

1. Deferred trace bitmask.
2. RX software ring.
3. Drain-only IRQ handlers.
4. Foreground `host_uart_service_rx()`.
5. Additive STATS counters + probe parser labels.

Then build, flash, and run the three validation gates from Phase B.
