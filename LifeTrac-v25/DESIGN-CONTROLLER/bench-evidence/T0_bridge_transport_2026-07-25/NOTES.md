# T0 — lora_bridge ↔ L072 transport compatibility verdict

**Date:** 2026-07-25 (bench session)
**Board:** base `2D0A1209DABC240B`, L072 on `/dev/ttymxc3` @ 921600
**Bridge under test:** `design-controller-lora_bridge-1`, image `lifetrac-v25:latest`
built 2026-05-28 (22 commits behind repo at time of test)
**Question:** can the deployed `lora_bridge` exchange frames with the current
Method G L072 firmware? (RS-9.3(c) in DESIGN-CONTROLLER/TODO.md)

## Verdict: NO — three stacked incompatibilities, each independently fatal

1. **Baud.** Deployed bridge opens `serial.Serial(port, 115200, timeout=0.1)`
   (deployed `/app/base_station/lora_bridge.py:186`). The L072 host UART runs
   **921600** (`HOST_BAUD_DEFAULT`). Every byte arrives as framing garbage,
   with ~8× byte-aliasing at the fast receiver.
2. **Framing.** Bridge speaks **KISS** (`kiss_encode`, repo `:649`); firmware
   speaks **COBS**-framed Method G HostLink.
3. **Protocol.** Bridge assumes a transparent USB-CDC radio dongle (writes
   raw on-air bytes); firmware expects the `TX_FRAME_REQ` / `RX_FRAME_URC` /
   CFG command set.

## Method

Three pushed scripts (per ADB_TIPS §1.4), AT shell reads at 921600 between
controlled bridge start/stops. Captures in this directory are raw port reads
(ASCII stats + interleaved binary URC frames; `strings -n 4` to extract).

| Capture | Condition |
|---|---|
| `stat1.bin` | baseline — bridge stopped after 57 min running |
| `stat2.bin` | after bridge restart + ~11 s running (startup TX window) |
| `stat3.bin` | bridge stopped again |
| `stat4.bin` | after 20 s quiet window, port untouched |

## Measured

| Counter | stat1 | stat2 | Δ (bridge running ~15 s) | stat3 | stat4 | Δ (quiet 20 s) |
|---|---|---|---|---|---|---|
| `HOST_RX_BYTES` | 2,036,249 | 2,042,780 | **+6,531** | 2,094,952 | 2,094,962 | **+10** (own probe bytes) |
| `HOST_UART_ERR_LPUART` | 1,723,774 | 1,729,032 | **+5,258** | 1,772,977 | 1,772,977 | **+0** |
| `HOST_PARSE_OK` | 366 | 366 | 0 | 366 | 366 | 0 |
| `HOST_PARSE_ERR` | 0 | 0 | 0 | 0 | 0 | 0 |
| `RADIO_RX_OK` / `TX_OK` | 5797 / 50 | unchanged | — | unchanged | unchanged | — |

- Writer attribution: byte/error flow present **only** while the bridge runs.
  No getty (`serial-getty@ttymxc3` inactive), no other holder (`fuser` empty).
- Bridge side: **zero** frames decoded in 68 min of runtime; log contains no
  RX event of any kind. Its only logged TX is the startup `CMD_ENCODE_MODE`.
- The 366 `HOST_PARSE_OK` are prior harness HostLink probes — proof the L072
  parses fine when spoken to correctly.

## Implications

- **"Bridge running = hydraulic control maintained" is FALSE.** As deployed
  the bridge provides no control path; it only pollutes the L072's UART at
  ~500 B/s (≈2.09 M garbage bytes / 1.77 M line errors accumulated since the
  L072's last reset). There is currently **no** base→tractor hydraulic
  control path on this bench by any process.
- Bridge container **stopped 2026-07-25** after this test
  (`sudo -n docker start design-controller-lora_bridge-1` restores it).
- Fix path: port the bridge onto HostLink @ 921600 — RS-9.3(c) in
  [../../TODO.md](../../TODO.md).

## Positive side-finding (firmware robustness)

The L072 absorbed ~2 M garbage bytes with `HOST_PARSE_OK` uncorrupted, zero
`HOST_QUEUE_FULL`, zero `HOST_DROPPED`, and a responsive AT shell throughout —
the W1-7 IDLE-recovery design holding up under sustained line abuse.
