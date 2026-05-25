# 2026-05-25 — Root Cause: RX VER warm-up failure (and phantom TX_DONE TIMEOUTs) caused by daemon UART RESET_REQ stacked on external gpio163 NRST

## TL;DR

Both yesterday morning's daemon failures — RX `VER warm-up failed` AND TX
`TX_DONE TIMEOUT` on every fragment — collapse to one root cause:

> `image_rx_daemon._open_link()` and `image_tx_daemon._open_link()` both
> issued a UART `RESET_REQ` (`link.send(0x03)`) **after**
> `run_concurrent_smoke.ps1` had already pulsed gpio163 NRST. The second,
> UART-triggered reset arrives while the L072 is mid-boot from the first
> reset and races the firmware's BOOT_URC / FAULT_URC emission, leaving
> garbage in the UART rx_buf AND leaving the SX1276 in an indeterminate
> state.

Removing the redundant UART RESET_REQ (opt-in via `LIFETRAC_SKIP_RESET_REQ=1`
when an external NRST has already been pulsed) made both daemons healthy:

- RX: `L072 VER warm-up ok` (15 ms latency, down from prior 1.0 s timeout).
- TX: every fragment `TX_DONE ok` (was `TIMEOUT` on most fragments).

`rx_frames` still equals 0 in the smoke run, but TX is now demonstrably
on air (50 frames × 4 fragments = 200 fragments TX'd cleanly). The
remaining RX=0 is a radio-air-coupling problem (channel/SF/SyncWord
mismatch or antenna), which is the "radio tuning needs a separate sweep"
gate already flagged in the morning's directive.

## Falsification matrix (rx_ver_warmup_diag.py + run_rx_ver_warmup_sweep.ps1)

Six cells: settle ∈ {1.5, 3.0, 5.0} s × {skipReset, sendReset}.
Each cell: gpio163 NRST → host sleep 1.5 s → diag in container → 3 VER
attempts (each attempt: open HostLink, [optionally] send UART RESET_REQ,
drain boot chatter for `settle_s`, request VER, 1.0 s timeout).

| settle_s | skip_reset_req | attempts | ver_ok | ver_latency_s | first_error |
|---------:|:--------------:|:--------:|:------:|:-------------:|:-----------:|
| 1.5      | **True**       | 3        | **3**  | 0.0151        | —           |
| 1.5      | False          | 3        | 1      | 0.0255        | VER timeout |
| 3.0      | **True**       | 3        | **3**  | 0.0152        | —           |
| 3.0      | False          | 3        | 3      | 0.0275        | —           |
| 5.0      | **True**       | 3        | **3**  | 0.0152        | —           |
| 5.0      | False          | 3        | 2      | 0.0284        | VER timeout |

**Observations**

1. Skip-reset is reliable at **every** settle time, including 1.5 s.
2. Send-reset is **non-deterministically flaky** — even 5.0 s settle was
   not enough (2/3). The variance suggests a race between
   "UART byte arrival ≈ MCU re-boot phase" rather than a fixed delay.
3. When send-reset succeeded, drain captured the boot urcs:
   `boot_urc=1 fault_urc=2 (code=0x08 sub=0x01 ; code=0x0A sub=0x00
   reason=no_ingress_during_probe_window)`.
4. When send-reset failed, drain captured 0 frames — i.e. the second
   reset's chatter arrived *after* the drain window closed and was the
   bytes the VER reader interpreted as protocol noise.
5. The failure mode "attempt 1 sometimes OK, attempts 2-3 timeout"
   reproduces the exact morning smoke symptom byte-for-byte.

## The fix

`image_rx_daemon.py` and `image_tx_daemon.py` `_open_link()` now branch
on `LIFETRAC_SKIP_RESET_REQ`:

```python
skip_reset = os.environ.get(
    "LIFETRAC_SKIP_RESET_REQ", "0") not in ("0", "", "false", "False")
if not skip_reset:
    link.send(0x03)         # HOST_TYPE_RESET_REQ
    drain_boot(link, settle_s=1.5)
else:
    LOG.info("LIFETRAC_SKIP_RESET_REQ=1 — relying on external NRST; "
             "draining boot chatter only")
    drain_boot(link, settle_s=0.25)
```

Default behaviour is **unchanged** (legacy callers that depend on the
daemon to reset the MCU still work).

`run_concurrent_smoke.ps1` now sets `LIFETRAC_SKIP_RESET_REQ=1` in both
docker run envs since it already pulses gpio163 NRST.

## Validation — concurrent smoke re-run (2026-05-25 11:49 PDT)

```
===RX LOG===
opening L072 HostLink on /dev/ttymxc3 @ 921600
LIFETRAC_SKIP_RESET_REQ=1 — relying on external NRST; draining boot chatter only
L072 VER warm-up ok                          # ← was: "VER warm-up failed"
SX1276 RXCONT autowake: opmode 0x81 -> 0x85
RX worker ready; draining RX_FRAME_URC...
stats: rx_frames=0  …                        # ← still 0 (air-side, not daemon)
stats: rx_frames=0  …  (×4 windows over 30 s)

===TX LOG===
frame seq=47 done: 4/4 fragments ok (0 fail)  # ← was: 0-2/4 ok, rest TIMEOUT
frame seq=48 done: 4/4 fragments ok (0 fail)
frame seq=49 done: 4/4 fragments ok (0 fail)
```

Every fragment now TX_DONE-OK on the TX peer; 200 fragments transmitted
cleanly over 30 s. The previous TX_DONE TIMEOUT failure mode (status=1
on most fragments) is **also** eliminated by this fix — confirming that
the SX1276 state corruption was a downstream consequence of the
double-reset, not an independent radio bug.

## Methodology note (cross-check before generalising)

Per `/memories/methodology.md`: "Blocking I/O calls inside ISR error
paths can self-DOS… always check whether the IRQ handler can preempt
itself or block on the same peripheral it's draining." The same shape
applies here at the OS level: a software RESET_REQ issued while the MCU
is still emitting boot URCs causes the UART rx_buf to interleave two
boot epochs' worth of chatter. The drain window then either (a) closes
before epoch-2 chatter arrives, leaving it for the VER reader, or (b)
catches it cleanly. Behaviour was load- and timing-dependent, which is
why prior 1.5 s drain "felt" adequate in some runs and not others.

The 2026-05-25 morning post-mortem (`2026-05-25_Smoke_RX_VER_Fail_TX_All_TIMEOUT.md`)
hypothesised the TX TIMEOUT was a radio-side issue (`status=1`) and the
RX VER failure was a UART/protocol issue — two independent bugs. They
are in fact the same bug with two symptom shapes, and the fix is one
line per daemon.

## Files modified

- [LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_rx_daemon.py](../DESIGN-CONTROLLER/base_station/image_rx_daemon.py) — `_open_link()` `LIFETRAC_SKIP_RESET_REQ` branch.
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py](../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py) — same branch.
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1) — `-e LIFETRAC_SKIP_RESET_REQ=1` in both docker envs.

## Files added (new diagnostics)

- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/rx_ver_warmup_diag.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/rx_ver_warmup_diag.py) — standalone VER warm-up probe (no MQTT, no TX peer).
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rx_ver_warmup_sweep.ps1](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rx_ver_warmup_sweep.ps1) — orchestrator: gpio163 NRST + container run + matrix parse.
- Sweep log: `2026-05-25_rx_ver_sweep_2D0A1209DABC240B_20260525_064033.log`.

## Remaining work (gated per morning's directive — "radio tuning needs a separate sweep, not more code")

- **rx_frames=0 in smoke** — TX is on air, RX is in RXCONT, but RX
  decodes nothing. Candidates (all radio-air, none daemon-code):
  - Channel / frequency mismatch (verify both peers on same FHSS channel after CFG_SET).
  - SF / BW / SyncWord mismatch.
  - Antenna routing / coupling on bench (RX peer was being moved around morning of 2026-05-25; spectrum analyser hop verification recommended).
  - LBT vs RXCONT race on the TX peer between fragments.
- Recommended next falsification: `w2_02_radio_rxcont_capture.py`-style
  standalone RX probe driven from a known-good TX peer, no MQTT/daemon
  path. If that still sees nothing, problem is RF-physical not software.

Per the original 2026-05-25 morning directive: **stop here on the
software side**. Two of three documented failure modes are eliminated;
the third is in a different problem domain.
