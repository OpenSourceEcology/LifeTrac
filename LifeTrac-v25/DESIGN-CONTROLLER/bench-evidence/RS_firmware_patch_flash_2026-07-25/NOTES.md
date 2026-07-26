# Firmware patch set — flash + verification on both boards

**Date:** 2026-07-25 (bench session)
**Artifact:** `firmware/murata_l072/build/firmware.bin`, **22,504 B**,
md5 `1f69131235ca2d865e815304c7e7e578` (was 16,440 B baseline pre-v25.0.7 era)
**Contents:** RS-4.9 (CAD abort), RS-4.11 (NVIC priority), RS-3.6 (SPI DIV64→DIV8),
RS-3.7 (USART1 TX mirror off + RADIO_IRQ_URC off), RS-2.3 (TX ring depth 4),
plus a `startup.c` `-Werror=missing-attributes` build fix (GCC 12.2).

## Results — both boards PASS

| Board | Role | Stage 1 quant | Method G probe | REG 0x42 |
|---|---|---|---|---|
| `2D0A1209DABC240B` | base | **3/3 PASS**, GATE_RESULT=PASS, all six flags=1 | **VERDICT: PASS** (rc=0) | **0x12** |
| `2E2C1209DABC240B` | tractor | **3/3 PASS**, GATE_RESULT=PASS, all six flags=1 | **VERDICT: PASS** | **0x12** |

Evidence dirs: `T6_stage1_standard_quant_2026-07-25_155702_427-14168` (base),
`T6_stage1_standard_quant_2026-07-25_160134_108-8684` (tractor).

## Gates cleared

- **RS-3.6 (SPI 250 kHz → 2 MHz):** `REG 0x42 VERSION = 0x12` on both boards,
  and the full 0x00–0x70 register dump reads coherent silicon contents (the
  W1-8 failure signature was all-`0x00`). SPI integrity holds at DIV8 with 5×
  margin to the SX1276's 10 MHz rating.
- **RS-3.7 (USART1 TX mirror off):** `host_rx_usart1=0`, `uart_err_usart1=0`,
  `host_parse_err=0` — host protocol unaffected; RX on both lanes retained.
- **RS-4.11 / RS-4.9 / RS-2.3:** no regression — flash/boot/protocol/register
  paths all clean; `rx_ring_ovf=0`.
- Full SIL suite green pre-flash (fhss_scheduler 23, fhss_clock 24,
  cfg_clamp_fuzz 12096, rx_scan_policy 20, rx_scan_walker golden vectors, …).

Baseline note: `host_errors=44 / FE=44` on both boards is probe-connection
baud settling (stty), not protocol error — contrast the 1.77 M FE the
mis-configured `lora_bridge` produced in the T0 session.

## Harness bug found (NEW — feeds RS-5.x)

`run_stage1_standard_quant_end_to_end.ps1` ends with **"Restoring
lifetrac-camera.service (post-quant)"**, which on the tractor starts a
*second, duplicate* camera stack (`tractor-camera` + `tractor-mosquitto`)
alongside the pre-existing `-v2` pair. Two problems:

1. **`tractor-camera` maps `/dev/ttymxc3`** (verified via `docker inspect`
   `HostConfig.Devices`); `tractor-camera-v2` does **not**. The duplicate
   therefore contends for the L072 host UART.
2. It **crash-loops** (traceback in `camera_service.py:1096 _build_frame`),
   so it opens/closes the port repeatedly.

Symptom this produced: the post-flash Method G probe on the tractor returned
`FATAL: timeout waiting for response type 0x81`, with `fuser` showing the port
*free* between crashes and a raw poke returning 6 garbled bytes — i.e. it
looked like a firmware failure and was not. After
`systemctl stop lifetrac-camera.service`, the identical probe returned
**VERDICT: PASS**.

`lifetrac-camera.service` is systemd-`disabled` (won't survive reboot) but is
started unconditionally by the quant launcher. **Any post-flash verification
on the tractor must stop it first**, or the launcher should stop restoring it.
This is exactly the RS-9.5 mutual-exclusion class, hit for real.

## State at end of session

- Both L072s running the new firmware; both idle (no daemons holding UARTs).
- Tractor container state restored to pre-flash (`-v2` stack only).
- Base `lora_bridge` remains stopped (T0 session).
- **Not yet done:** host-side `PIPELINE_DEPTH` 2→4 and the RS-2.1/RS-9.8
  throughput re-baseline. See NEXT below.

## NEXT — blocker for the throughput run

`run_live_radio_monitor.ps1:150` starts `tx_smoke` with
`-e LIFETRAC_MQTT_HOST=$HostIp` (192.168.1.79) — a **LAN** path from the
tractor to the bench PC's broker. Tractor WiFi was disabled this session per
MASTER_PLAN §8.13 (and the tractor has no ethernet), so that path no longer
exists. The harness must be re-pointed at the tractor-local broker before any
throughput run — which is also the architecturally correct wiring (the
LAN-MQTT hop was itself a workaround for the broken `adb reverse`, per
ADB_TIPS §3.1). Tracked under RS-9/RS-5.
