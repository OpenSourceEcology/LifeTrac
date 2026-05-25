# 2026-05-25 — IQ Hypothesis Falsified; RxDone IS Firing (~11% PER)

## TL;DR

The prior session's "RegInvertIQ 0x33=0x27 asymmetry is the rx_frames=0
root cause" hypothesis was **wrong**. Writing `0x33=0x26` in the L072
firmware's `sx1276_init()` made the air-coupling sniff metric strictly
**worse**: pre-fix `irq_events=9, irq_flags_or=0x10` (ValidHeader only)
→ post-fix `irq_events=0, irq_flags_or=0x00` (nothing fires). Reverted.

After revert, a fresh sniff shows:

```
irq_events      = 9, irq_flags_or = 0x50
RESULT: RF coupling DETECTED (max RSSI -78 dBm >= -100).
       IRQ flags fired; demod is running. Investigate RX_FRAME_URC plumbing.
```

`0x50 = 0x10 (ValidHeader) | 0x40 (RxDone)` — **RxDone IS firing** at
roughly 9 events per 80 bursts (~11% packet error rate). The rx_frames=0
symptom is therefore in the host-URC emission path or the python URC
parser, **not** in the RF or demod layers.

## What went wrong with the IQ hypothesis

I read `RegInvertIQ` (0x33) reset default `0x27` as "TX bit set = TX
IQ-inverted, RX bit clear = RX expects non-inverted → asymmetric, broken
for P2P." That mental model is **wrong**.

Semtech's own LoRaMac-node driver
(`src/radio/sx1276/sx1276Regs-LoRa.h`) defines:

```c
#define RFLR_INVERTIQ_RX_MASK     0xBF
#define RFLR_INVERTIQ_RX_OFF      0x00   // bit 6 = 0 → RX non-inverted
#define RFLR_INVERTIQ_RX_ON       0x40

#define RFLR_INVERTIQ_TX_MASK     0xFE
#define RFLR_INVERTIQ_TX_OFF      0x01   // bit 0 = 1 → TX non-inverted   (!!)
#define RFLR_INVERTIQ_TX_ON       0x00
```

The TX-OFF state is **bit set**, not bit clear. Reserved bits 5, 2, 1
read back as 1 always (chip wiring), so the canonical
RX-non-inverted, TX-non-inverted value is:

```
0x26 (reserved bits) | 0x01 (TX_OFF) = 0x27   ← the chip's reset default
```

The reset default 0x27 is already correct for symmetric non-inverted
P2P operation. Writing `0x26` clears the TX_OFF bit and INVERTS the
TX path, which is why post-fix `irq_events=0`.

## Why the demod still showed ValidHeader-only pre-revert

The pre-fix baseline (`irq_flags_or=0x10`, no RxDone) was already
partial demod success. ValidHeader (0x10) is sticky in `RegIrqFlags`
until the host clears it via a write; nine sticky readings over
30 s do **not** mean nine separate header detections. The RX peer
runs `sx1276_rx_service()` only on `events & SX1276_EVT_DIO0`
(RxDone wired to DIO0 in the firmware), so ValidHeader latches
without ever being cleared until a real RxDone event triggers
service-routine entry.

After the revert + fresh boot, the sniff window happened to catch
nine actual RxDone events (`0x50` = 0x40 | 0x10), confirming the
demod is producing payloads.

## Firmware deltas this session

Files touched in
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/`:

1. `radio/sx1276.c` — IQ writes added (`0x33=0x26, 0x3B=0x1D`),
   tested, falsified, **REVERTED**. Comment retained explaining
   why the reset defaults must not be touched.
2. `host/host_cmd.c reg_write_allowed()` — added `0x33` and `0x3B`
   to the diagnostic allowlist. **KEPT** (harmless; enables future
   python-side experiments without firmware rebuild).

Host-side delta in
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`:

3. `method_h_stage2_tx_probe_v2.py` — the IQ-normalize block that
   wrote `0x33=0x26` from the host was REMOVED (it would now succeed
   with the new allowlist and re-introduce the same bug).

Net firmware diff retained: only the allowlist additions for
diagnostic flexibility. Firmware binary size unchanged from
the prior session's baseline (21332 bytes).

## Evidence

- Build log: `flash_unitA_revert.log` (`flash_rc=0`).
- Pre-fix sniff: `air_coupling_patched_v2.log` — `irq_events=9,
  irq_flags_or=0x10`.
- Post-(bad)-fix sniff: `air_postiqfix.log` — `irq_events=0,
  irq_flags_or=0x00, rssi_max=-54 dBm` (closer than baseline, but
  demod completely broken).
- Post-revert sniff: `air_postrevert2.log` — `irq_events=9,
  irq_flags_or=0x50, rssi_max=-78 dBm`. **RxDone fires.**
- TX firmware dump confirming IQ-write took effect during the bad
  fix: `dump_tx_iqfix3.log` (RegInvertIQ=0x26, invert_iq_tx=False).
- TX firmware dump after revert would show RegInvertIQ=0x27 again
  (the chip's reset default).

## Next investigation

The demod fires 0x40 in IRQ flags but the upstream `image_rx_daemon`
reports `rx_frames=0`. Likely failure modes, ordered by likelihood:

1. `sx1276_rx_service()` reads payload from FIFO but the
   `host_uart_send_urc(HOST_TYPE_RX_FRAME_URC, ...)` call is gated
   by a precondition that BENCH profile fails. Check
   `radio/sx1276_rx.c` lines 200-300 for early-returns inside the
   ServiceRxDone path (CRC check, header validation,
   `sx1276_fhss_consider_remote` rejecting frame, etc.).
2. URC is emitted but `image_rx_daemon.py` parser drops it because
   the frame type code or version mismatched.
3. Frame is shorter than the daemon's minimum-payload-length filter.

Concrete next step: run `image_rx_daemon.py` directly on 2D0A in
verbose mode while triggering tx_burst from 2E2C, then grep for
`RX_FRAME_URC` lines. If URCs arrive but daemon rejects → fix
parser. If no URCs arrive → instrument `sx1276_rx_service`.

## Lesson reinforced

When a vendor-published register reset default looks "obviously
wrong," cross-check the **vendor's own driver source** before
patching. Bit names like `InvertIQ_TX` where set = OFF are common
in radio chips and easy to misread. Symptom of a misdiagnosed
register-bit patch: the headline metric goes the **wrong direction**
(here, irq_events 9 → 0). Falsification within one A/B cycle saves
days of chasing a non-bug.

(Logged to `/memories/misdiagnosis.md` as a generalized rule.)
