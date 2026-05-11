# 2026-05-11 — W1-7 RX implementation plan (v1.2)

**Author:** Copilot
**Status:** READY TO IMPLEMENT
**Supersedes:** [`2026-05-11_W1-7_RX_Research_Plan_Copilot_v1_1.md`](2026-05-11_W1-7_RX_Research_Plan_Copilot_v1_1.md)
**Open blocker:** [TODO.md Step 6](../DESIGN-CONTROLLER/TODO.md#L42)

---

## 0. What changed from v1.1

v1.1 named the right architectural fix but stopped at concept level. v1.2 is a
single-patch implementation contract grounded in the current source. It also:

- Adds a **Phase A0 baseline capture** before any code change so we have a
  before/after comparison when v1.2 is applied (closes the methodology gap
  where TX-fix and RX-fix were conflated).
- Pins **exact files, function names, line ranges, and new symbols** so the
  patch can be reviewed without re-reading the whole module.
- Keeps the v1.1 architecture: drain-only IRQs, software RX ring, foreground
  COBS parser, deferred trace bitmask, additive STATS counters.
- Defers the `host_uart_poll_dma()` rename and any real DMA work to a
  follow-up; v1.2 only fixes its blocking critical-section behavior.
- Explicitly **does not** disable the USART1 mirror in Phase A; that stays a
  Phase C branch driven by per-flag counters.

---

## 1. Source ground truth (verified before writing this plan)

### 1.1 Blocking trace primitive
- [`hal/platform.c::platform_diag_trace()`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c#L185)
  spins on `LPUART1_ISR.TXE` per byte, then `LPUART1_ISR.TC` at end. Worst
  case for a 14-byte string at 921600 ≈ 152 µs ≫ one byte time (10.85 µs).

### 1.2 RX paths that currently call the blocking trace from real-time context
[`host/host_uart.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c):

| Site | Function | Trace strings | Real-time context? |
|---|---|---|---|
| L331 | `process_encoded_frame()` | `H:COBS_ERR` | Yes (called from IRQ via `ingest_binary_byte` on `0x00`) |
| L339 | `process_encoded_frame()` | `H:PARSE_ERR` | Yes |
| L347 | `process_encoded_frame()` | `H:VER_REQ_RX` | Yes |
| L351 | `process_encoded_frame()` | `H:FRAME_OK` | Yes |
| L355 | `process_encoded_frame()` | `H:Q_FULL` | Yes |
| L370 | `ingest_binary_byte()` | `H:PROC_FRAME` | Yes |
| L378 | `ingest_binary_byte()` | `H:COBS_OVF` | Yes |
| L412 | `at_finish_line()` | `H:AT_DISPATCH` | Yes (reachable from IRQ) |
| L885 | `RNG_LPUART1_IRQHandler()` | `H:ERR_LPUART1`, `H:RX_LPUART1` | Yes (IRQ) |
| L922 | `USART1_IRQHandler()` | `H:ERR_USART1`, `H:RX_USART1` | Yes (IRQ) |
| L608 | `host_uart_poll_dma()` | all of the above (via `ingest_rx_byte`) | Yes (PRIMASK=1 critical section) |

### 1.3 IRQ-disabled critical section in the foreground poll
`host_uart_poll_dma()` does `cpu_irq_save()` at L612, then loops over both
LPUART1 and USART1 ISR until both are quiet, calling `ingest_rx_byte()` and
`platform_diag_trace()` while interrupts are masked, and only restores
interrupts at the end (L685). This makes the "foreground" path an
IRQ-equivalent context for timing purposes.

### 1.4 STATS contract is additive-only
[`include/host_types.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
documents: *"STATS_URC payload layout is additive-only; parsers must tolerate
trailing bytes."* `HOST_STATS_PAYLOAD_LEN = 96U`. Wire struct in
[`host/host_stats.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c#L9)
ends with `host_uart_err_lpuart` / `host_uart_err_usart1`.

### 1.5 What is NOT in the source today
- No active LPUART1 RX DMA setup (`DMA1_CPAR/CMAR/CCR` for LPUART1 RX,
  `USART_CR3_DMAR`).
- No software RX byte ring; bytes are consumed in-IRQ straight into the COBS
  state machine.
- No deferred trace mechanism; `host_uart_note_diag_mark()` exists but only
  carries 8 bits of high-level marks consumed in `main.c`.

---

## 2. Phase A0 — baseline capture (no code change)

Before any code change, run on the bench so we have a before-image:

1. Build current firmware (already 15276 bytes per last bench-evidence run).
2. Method G stage 1 flash via the existing PowerShell launcher.
3. Capture `/dev/ttymxc3 @ 921600` for 5 s while the X8 sends a known VER_REQ.
4. Save the raw byte log to `bench-evidence/T6_W1-7_RX_baseline_<UTC>/` along
   with the existing `stage1_stdout.txt`, `method_g_stage1.log`, and
   `run_metadata.json`.

Acceptance: artifacts saved. No firmware claim is made from A0; it is the
"before" image only.

---

## 3. Phase A1 — single implementation patch (v1.2)

One coherent commit. Files and the exact deltas:

### 3.1 `include/host_uart.h` — public surface additions

Add after the existing diag-mark defines:

```c
/* Deferred trace flags. Set from any context (IRQ-safe); flushed only from
 * foreground via host_uart_flush_diag_traces(). Order is not significant. */
#define HOST_TRACE_RX_LPUART1     (1U << 0)
#define HOST_TRACE_RX_USART1      (1U << 1)
#define HOST_TRACE_ERR_LPUART1    (1U << 2)
#define HOST_TRACE_ERR_USART1     (1U << 3)
#define HOST_TRACE_PROC_FRAME     (1U << 4)
#define HOST_TRACE_COBS_ERR       (1U << 5)
#define HOST_TRACE_COBS_OVF       (1U << 6)
#define HOST_TRACE_PARSE_ERR      (1U << 7)
#define HOST_TRACE_VER_REQ_RX     (1U << 8)
#define HOST_TRACE_FRAME_OK       (1U << 9)
#define HOST_TRACE_Q_FULL         (1U << 10)
#define HOST_TRACE_AT_DISPATCH    (1U << 11)
#define HOST_TRACE_RX_RING_OVF    (1U << 12)
```

Add new APIs (group with the existing stats getters):

```c
void     host_uart_service_rx(void);          /* foreground COBS parser */
void     host_uart_flush_diag_traces(void);   /* foreground trace emitter */

uint32_t host_uart_stats_uart_pe_lpuart(void);
uint32_t host_uart_stats_uart_fe_lpuart(void);
uint32_t host_uart_stats_uart_ne_lpuart(void);
uint32_t host_uart_stats_uart_ore_lpuart(void);
uint32_t host_uart_stats_uart_pe_usart1(void);
uint32_t host_uart_stats_uart_fe_usart1(void);
uint32_t host_uart_stats_uart_ne_usart1(void);
uint32_t host_uart_stats_uart_ore_usart1(void);
uint32_t host_uart_stats_rx_ring_ovf(void);
```

### 3.2 `host/host_uart.c` — internal additions

Add file-static state near the existing stats counters:

```c
#define HOST_RX_RING_LEN  512U   /* power of 2 */

static volatile uint8_t  s_rx_ring[HOST_RX_RING_LEN];
static volatile uint16_t s_rx_ring_head;   /* producer (IRQ/poll) */
static volatile uint16_t s_rx_ring_tail;   /* consumer (foreground) */
static volatile uint32_t s_rx_ring_ovf;

static volatile uint32_t s_diag_trace_flags;  /* HOST_TRACE_* bitmask */

static volatile uint32_t s_stats_uart_pe_lpuart;
static volatile uint32_t s_stats_uart_fe_lpuart;
static volatile uint32_t s_stats_uart_ne_lpuart;
static volatile uint32_t s_stats_uart_ore_lpuart;
static volatile uint32_t s_stats_uart_pe_usart1;
static volatile uint32_t s_stats_uart_fe_usart1;
static volatile uint32_t s_stats_uart_ne_usart1;
static volatile uint32_t s_stats_uart_ore_usart1;
```

Helpers (file-static):

```c
static inline void host_uart_note_trace(uint32_t flag) {
    /* IRQ-safe: 32-bit aligned write of OR is atomic on Cortex-M0+ */
    s_diag_trace_flags |= flag;
}

static inline bool rx_ring_push(uint8_t b) {
    uint16_t head = s_rx_ring_head;
    uint16_t next = (uint16_t)((head + 1U) & (HOST_RX_RING_LEN - 1U));
    if (next == s_rx_ring_tail) {
        s_rx_ring_ovf++;
        host_uart_note_trace(HOST_TRACE_RX_RING_OVF);
        return false;
    }
    s_rx_ring[head] = b;
    s_rx_ring_head = next;
    return true;
}

static inline bool rx_ring_pop(uint8_t *out) {
    uint16_t tail = s_rx_ring_tail;
    if (tail == s_rx_ring_head) return false;
    *out = s_rx_ring[tail];
    s_rx_ring_tail = (uint16_t)((tail + 1U) & (HOST_RX_RING_LEN - 1U));
    return true;
}
```

### 3.3 Replace blocking traces in the parser with deferred flags

In `process_encoded_frame()` (host_uart.c L325–L357) and
`ingest_binary_byte()` (L359–L379), and `at_finish_line()` (L412), replace
each `platform_diag_trace("H:XYZ\r\n")` with
`host_uart_note_trace(HOST_TRACE_XYZ)`. Mapping:

| Old call | New call |
|---|---|
| `platform_diag_trace("H:COBS_ERR\r\n")` | `host_uart_note_trace(HOST_TRACE_COBS_ERR)` |
| `platform_diag_trace("H:PARSE_ERR\r\n")` | `host_uart_note_trace(HOST_TRACE_PARSE_ERR)` |
| `platform_diag_trace("H:VER_REQ_RX\r\n")` | `host_uart_note_trace(HOST_TRACE_VER_REQ_RX)` |
| `platform_diag_trace("H:FRAME_OK\r\n")` | `host_uart_note_trace(HOST_TRACE_FRAME_OK)` |
| `platform_diag_trace("H:Q_FULL\r\n")` | `host_uart_note_trace(HOST_TRACE_Q_FULL)` |
| `platform_diag_trace("H:PROC_FRAME\r\n")` | `host_uart_note_trace(HOST_TRACE_PROC_FRAME)` |
| `platform_diag_trace("H:COBS_OVF\r\n")` | `host_uart_note_trace(HOST_TRACE_COBS_OVF)` |
| `platform_diag_trace("H:AT_DISPATCH\r\n")` | `host_uart_note_trace(HOST_TRACE_AT_DISPATCH)` |

### 3.4 Drain-only IRQ handlers

Rewrite `RNG_LPUART1_IRQHandler()` (L884) as:

```c
void RNG_LPUART1_IRQHandler(void) {
    const uint32_t isr = LPUART1_ISR;

    if ((isr & USART_ISR_IDLE) != 0U) {
        LPUART1_ICR = USART_ICR_IDLECF;
        s_stats_irq_idle++;
    }

    if ((isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        if (isr & USART_ISR_PE)  s_stats_uart_pe_lpuart++;
        if (isr & USART_ISR_FE)  s_stats_uart_fe_lpuart++;
        if (isr & USART_ISR_NE)  s_stats_uart_ne_lpuart++;
        if (isr & USART_ISR_ORE) s_stats_uart_ore_lpuart++;
        LPUART1_ICR = USART_ICR_PECF | USART_ICR_FECF |
                      USART_ICR_NCF  | USART_ICR_ORECF;
        (void)LPUART1_RDR;
        s_stats_errors++;
        s_stats_uart_err_lpuart++;
        host_uart_note_trace(HOST_TRACE_ERR_LPUART1);
        return;
    }

    while ((LPUART1_ISR & USART_ISR_RXNE) != 0U) {
        const uint8_t rx = (uint8_t)LPUART1_RDR;
        s_stats_rx_bytes++;
        s_stats_rx_lpuart_bytes++;
        s_rx_seen_flags |= HOST_RX_SEEN_FLAG_LPUART;
        host_uart_note_trace(HOST_TRACE_RX_LPUART1);
        (void)rx_ring_push(rx);
    }
}
```

Apply the symmetric rewrite to `USART1_IRQHandler()` (L911).

Constraints (must hold after the patch):
- No `platform_diag_trace()` call.
- No `ingest_rx_byte()` / `ingest_binary_byte()` / `process_encoded_frame()` call.
- No `host_uart_send_*()` call.
- No call into `host_diag_echo_rx_byte()` if it can block (audit needed; if it
  can, replace with deferred-flag equivalent or move to foreground).

### 3.5 Fix `host_uart_poll_dma()` critical section

Rewrite L608–L686 to: *not* hold PRIMASK across the loop. Pattern:

```c
void host_uart_poll_dma(void) {
    /* Snapshot + drain in short critical sections; never block while masked. */
    for (;;) {
        uint32_t irq_state = cpu_irq_save();
        const uint32_t lpuart_isr = LPUART1_ISR;
        const uint32_t usart1_isr = USART1_ISR;
        bool did_work = false;

        if ((lpuart_isr & (USART_ISR_PE|USART_ISR_FE|USART_ISR_NE|USART_ISR_ORE)) != 0U) {
            /* same per-flag counting as IRQ handler */
            ...
            LPUART1_ICR = USART_ICR_PECF|USART_ICR_FECF|USART_ICR_NCF|USART_ICR_ORECF;
            (void)LPUART1_RDR;
            did_work = true;
        } else if ((lpuart_isr & USART_ISR_RXNE) != 0U) {
            const uint8_t rx = (uint8_t)LPUART1_RDR;
            s_stats_rx_bytes++;
            s_stats_rx_lpuart_bytes++;
            s_rx_seen_flags |= HOST_RX_SEEN_FLAG_LPUART;
            (void)rx_ring_push(rx);
            did_work = true;
        }
        /* same for USART1 */

        cpu_irq_restore(irq_state);
        if (!did_work) break;
    }
}
```

Notes:
- Trace flag setting happens via `host_uart_note_trace()`; emission deferred.
- `host_diag_echo_rx_byte()` removed from this path; it was reachable from a
  PRIMASK=1 section.

### 3.6 Foreground RX service + deferred trace flush

New functions in `host_uart.c`:

```c
void host_uart_service_rx(void) {
    uint8_t b;
    while (rx_ring_pop(&b)) {
        ingest_rx_byte(b);   /* unchanged parser, now in foreground */
    }
}

void host_uart_flush_diag_traces(void) {
    uint32_t irq_state = cpu_irq_save();
    uint32_t flags = s_diag_trace_flags;
    s_diag_trace_flags = 0U;
    cpu_irq_restore(irq_state);
    if (flags == 0U) return;

    if (flags & HOST_TRACE_RX_LPUART1)  platform_diag_trace("H:RX_LPUART1\r\n");
    if (flags & HOST_TRACE_RX_USART1)   platform_diag_trace("H:RX_USART1\r\n");
    if (flags & HOST_TRACE_ERR_LPUART1) platform_diag_trace("H:ERR_LPUART1\r\n");
    if (flags & HOST_TRACE_ERR_USART1)  platform_diag_trace("H:ERR_USART1\r\n");
    if (flags & HOST_TRACE_PROC_FRAME)  platform_diag_trace("H:PROC_FRAME\r\n");
    if (flags & HOST_TRACE_COBS_ERR)    platform_diag_trace("H:COBS_ERR\r\n");
    if (flags & HOST_TRACE_COBS_OVF)    platform_diag_trace("H:COBS_OVF\r\n");
    if (flags & HOST_TRACE_PARSE_ERR)   platform_diag_trace("H:PARSE_ERR\r\n");
    if (flags & HOST_TRACE_VER_REQ_RX)  platform_diag_trace("H:VER_REQ_RX\r\n");
    if (flags & HOST_TRACE_FRAME_OK)    platform_diag_trace("H:FRAME_OK\r\n");
    if (flags & HOST_TRACE_Q_FULL)      platform_diag_trace("H:Q_FULL\r\n");
    if (flags & HOST_TRACE_AT_DISPATCH) platform_diag_trace("H:AT_DISPATCH\r\n");
    if (flags & HOST_TRACE_RX_RING_OVF) platform_diag_trace("H:RX_RING_OVF\r\n");
}
```

### 3.7 `main.c` main-loop integration

Order inside the main loop:
1. `host_uart_poll_dma();`  (now non-blocking, fills ring)
2. `host_uart_service_rx();` (drains ring, runs COBS parser, sets more flags)
3. `host_uart_flush_diag_traces();` (only place that calls `platform_diag_trace`)
4. Existing diag-mark + frame-pop + dispatch + radio service (unchanged).

### 3.8 STATS additive extension

In `include/host_types.h`:

```c
#define HOST_STATS_OFFSET_HOST_UART_PE_LPUART   96U
#define HOST_STATS_OFFSET_HOST_UART_FE_LPUART  100U
#define HOST_STATS_OFFSET_HOST_UART_NE_LPUART  104U
#define HOST_STATS_OFFSET_HOST_UART_ORE_LPUART 108U
#define HOST_STATS_OFFSET_HOST_UART_PE_USART1  112U
#define HOST_STATS_OFFSET_HOST_UART_FE_USART1  116U
#define HOST_STATS_OFFSET_HOST_UART_NE_USART1  120U
#define HOST_STATS_OFFSET_HOST_UART_ORE_USART1 124U
#define HOST_STATS_OFFSET_HOST_RX_RING_OVF     128U
#define HOST_STATS_PAYLOAD_LEN                 132U
```

In `host/host_stats.c`:
- Add 9 `uint32_t` fields to `host_stats_wire_t` in the same order.
- `_Static_assert` already validates the size.
- Append 9 `put_u32_le` calls in `host_stats_serialize()` using the new
  getters.

### 3.9 Probe parser labels

In
[`firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py::parse_stats()`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py),
append 9 labels:

```
HOST_UART_PE_LPUART, HOST_UART_FE_LPUART, HOST_UART_NE_LPUART,
HOST_UART_ORE_LPUART, HOST_UART_PE_USART1, HOST_UART_FE_USART1,
HOST_UART_NE_USART1, HOST_UART_ORE_USART1, HOST_RX_RING_OVF
```

Keep tolerance for older payloads (96 bytes) so old firmware still parses.

### 3.10 Dead code / comment cleanup

- Remove the `s_trace_first_*` one-shot booleans now that traces are deferred
  and naturally rate-limited per main-loop tick.
- Update the misleading comment in `host_uart_init()` that attributes RX
  failure to HSI16 drift.

---

## 4. Phase B — validation gates (must all pass for W1-7 closure)

After A0 baseline + A1 patch is built and flashed:

1. **Stimulus gate**: existing
   `bench-evidence/T6_W1-7_RX_session_2026-05-11/test_rx_isr.sh` (or
   equivalent X8-side stimulus) over 60 s. Acceptance:
   - `host_parse_ok` ≥ 1 per VER_REQ sent.
   - `host_parse_err` == 0 for valid frames.
   - At most isolated `H:ERR_*` traces; no sustained `H:COBS_ERR` storm.
   - `HOST_RX_RING_OVF` == 0.
2. **Probe gate**: `method_g_stage1_probe.py` returns
   `VER_URC` (`0x81`) within timeout on `/dev/ttymxc3 @ 921600`.
3. **Quant gate**: `run_stage1_standard_quant_end_to_end.ps1 -Cycles 20`
   yields ≥18/20 cycles with both BOOT_URC and VER_REQ→VER_URC.

Save artifacts to `bench-evidence/T6_W1-7_RX_v1_2_<UTC>/` mirroring the
baseline layout for direct A/B comparison.

---

## 5. Phase C — branch only if Phase B fails

Use new per-flag counters as the discriminator (unchanged from v1.1 §5):

| Counter result | Next action |
|---|---|
| `host_uart_ore_lpuart` still rising | Increase `HOST_RX_RING_LEN`; audit any remaining IRQ-disabled section that calls `platform_diag_trace()` indirectly (e.g. `host_diag_echo_rx_byte`) |
| `host_uart_fe_lpuart` dominant | Re-open clock/timing investigation; repeat 115200 sanity with deferred-trace firmware |
| `host_uart_ne_lpuart` dominant | Signal integrity on PA3; pull-up/pad config / scope capture |
| USART1 counters non-zero with LPUART1 active | Disable `USART1_IRQHandler` and CR1_RXNEIE for USART1; keep PA9/PA10 as plain GPIO during bring-up |
| All UART error counters zero, parse errors persist | Compare ring contents to `build_frame()`; suspect COBS/CRC logic |

---

## 6. Out of scope for v1.2 (do not expand the patch)

- Real LPUART1 RX DMA (`USART_CR3_DMAR`, `DMA1_CCR`, IDLE-line trigger).
- HSE/TCXO bring-up on the Max Carrier (separate ticket).
- Renaming `host_uart_poll_dma()` (cosmetic; ride along with the DMA work).
- Reworking `host_diag_echo_rx_byte` unless the audit shows it is blocking.
- Touching the radio path or LoRa QoS work.

---

## 7. Recommended next action

Execute in this order, no parallel work:

1. **Phase A0** baseline capture (no code change). Save artifacts.
2. **Phase A1** patch (single commit) per §3.
3. Local build (`firmware/murata_l072` build target). Confirm size delta and
   no warnings.
4. Method G flash via the existing PowerShell launcher.
5. **Phase B** gates 1 → 2 → 3 in order. Stop and branch into Phase C on the
   first failure.
6. On full pass: mark TODO Step 6 complete, append evidence reference to
   `/memories/repo/lifetrac-l072-hsi16-bug.md`, and close W1-7.
