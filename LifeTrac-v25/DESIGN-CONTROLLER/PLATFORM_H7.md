# LifeTrac v25: Portenta H7-only platform profile

**Status:** analysis, 2026-10-13. Not adopted. Tracks [issue #141](https://github.com/OpenSourceEcology/LifeTrac/issues/141).
**Work items:** [TODO.md § Portenta H7-only platform profile (H7)](TODO.md#portenta-h7-only-platform-profile-h7).
**Related:** [MASTER_PLAN.md §8.9 / §8.10](MASTER_PLAN.md#89-compute-platform--portenta-x8-on-both-tractor-and-base), [X8_OS_IMPROVEMENTS.md](X8_OS_IMPROVEMENTS.md), [firmware/murata_l072/BRINGUP_MAX_CARRIER.md](firmware/murata_l072/BRINGUP_MAX_CARRIER.md).

The canonical v25 build puts a Portenta X8 on the tractor and on the base.
Issue #141 asks for a build with no X8 at all, because the X8 appears to carry
export restrictions. This page works out what an H7-only build looks like,
what carries over, what has to be written, and how the repository should hold
both builds.

## 1. Summary

| Area | Verdict | Why |
|---|---|---|
| Base station | **Easy** | The base H7 becomes a USB radio gateway. All of `base_station/` runs on an ordinary PC. The compose file already expects the radio on `/dev/ttyACM0`. |
| Tractor control | **Moderate, and safer than today** | `tractor_h7` already compiles for a standalone Portenta H7 in CI, and on an H7 the realtime MCU owns the LoRa UART directly. The control application still has to be joined to the Method G radio transport (§4.1), which the X8 build needs too. |
| Tractor video | **The hard part** | An H7 cannot host the UVC camera or the Python/OpenCV pipeline. The recommended answer is a second H7 with a camera (§5). |
| Message scheduling | **Straightforward** | The P0–P3 policy is already specified, the image stream is tiny (a VS1 frame is one fragment: a `TileDeltaFrame` of at most 203 B at FHSS or 243 B at DTS), and a pull model keeps the camera from ever pushing the control MCU. One caveat: the L072's TX ring is FIFO, so until it gets a priority lane (RS-10.12) a P0 frame's wait is bounded, not zero (§6). |
| Repository | **One tree, two profiles** | Protocol, crypto and base code stay shared. H7 is a build profile plus two new sketches (§9). |

Export classification has **not** been verified here. Confirm the ECCN of the
exact H7 SKU (ABX00042 Portenta H7, ABX00045 H7 Lite) and of any camera board
with Arduino before buying.

## 2. How the X8 is used today

| Node | X8 Linux side | X8's onboard STM32H747 |
|---|---|---|
| Tractor | `camera_service.py` and `x8_image_pipeline/` (capture, tile diff, VS1 vector encode, NanoDet), `image_tx_daemon.py` pushing fragments to the L072 on `/dev/ttymxc3`, Mosquitto, GPS/IMU through a USB FT232H/MCP2221A, logger, params, time service, SSH/adb, Docker | Intended: `tractor_h7.ino` (M7) and `tractor_m4.cpp` (M4). In practice Linux reflashes stock x8h7 into bank 1 at every boot ([X8_OS_IMPROVEMENTS.md](X8_OS_IMPROVEMENTS.md), X8OS-3), so no LifeTrac realtime image survives a reboot today. |
| Base | `lora_bridge.py`, `web_ui.py` (FastAPI + WebSocket), `image_rx_daemon.py` and `image_pipeline/`, Mosquitto, Docker | Nothing (MASTER_PLAN §8.2) |

## 3. Max Carrier facts that decide the design

From the ABX00043 pinout and schematics in
[`DESIGN-LORAFIRMWARE/`](DESIGN-LORAFIRMWARE/) (schematic rev V3.12):

| HD connector UART | Max Carrier destination | Free with an H7? |
|---|---|---|
| SERIAL0 | SP335 RS-232/422/485 transceiver, fieldbus connector | No: Modbus RTU to the Opta |
| SERIAL1 (J1-33/35) | SARA-R412M cellular modem | No: wired to the (unused) modem, no breakout |
| SERIAL2 (J2-26/28) | Onboard STM32F405 debugger, micro-USB | Console only |
| SERIAL3 (J2-25/27) | **Murata CMWX1ZZABZ LoRa** UART, also broken out on the J5 stacker pins 24/26/28/30 | No: this is the radio |

Other lines:

- Murata BOOT0 is on HD J2-67 (PWM4) and the Murata IRQ on J2-69 (PWM5). `portenta_m7_l072_passthrough_ping.ino` uses `LORA_BOOT0 PJ_11` and `LORA_RESET PD_5`.
- **The Murata's SPI is not routed to the HD connector.** The only path to the radio is the L072 over SERIAL3, which is why Method G exists.
- **J5 stacker (40-pin, 1.27 mm)** breaks out SPI1, I²C1, I²C2, SAI, PWM6–9, CAN0 logic lines and analog inputs. SPI1 appears nowhere else on the carrier.
- **J4 camera connector (20-pin 0.5 mm FFC, TE 2-1734248-0)** carries the HD camera lines: four CSI pairs plus clock (on an H7 these same HD pins are the DCMI parallel bus D0–D7, PCLK and VSYNC), HSYNC, PDM, I²C and 3V3. There is no camera master-clock or reset line on it.
- Ethernet RJ45 (J17) works with either module. The X8 runs it at gigabit; the H7's PHY is 10/100.

**Correction to BRINGUP_MAX_CARRIER.md §1.1.** That section excludes `Serial3`
as "reserved by the cellular modem". The pinout shows the modem on HD
SERIAL1 and the Murata on HD SERIAL3. The bench notes and Arduino's own Max
Carrier usage also name `Serial3` for the LoRa module on an H7. The remaining
check is that the mbed core's `Serial3` object on `PORTENTA_H7_M7` drives HD
SERIAL3 (variant `PeripheralPins.c`), confirmed on hardware (TODO H7-1).

## 4. Tractor on a Portenta H7

### 4.1 Control path

- Build `firmware/tractor_h7` for `arduino:mbed_portenta:envie_m7` (already a CI target) with `LIFETRAC_USE_METHOD_G_HOST=1` and `LIFETRAC_MH_SERIAL=Serial3` once H7-1 confirms it.
- **The gap:** in the Method G build today the sketch runs only the host runtime (`murata_host/mh_runtime.c`: ping, version, stats). Arbitration, Modbus master, AES-GCM and the IP-107 TX queue are compiled only in the legacy RadioLib build, which assumes SPI access to the SX1276 that the Max Carrier does not provide. Joining the control application to Method G (`TX_FRAME_REQ` out, `RX_FRAME_URC` in) is the first real firmware task, and it is the same task the X8's H747 faces.
- X8-only pieces drop out: `sysclock_x8.c`, `linker_script_x8_m7.ld` and the tick overrides (already gated by `LIFETRAC_X8_ENABLE_LEGACY_OVERRIDES`), the X8 `Serial2` shim, and the x8h7 flash-ownership problem of X8OS-3. A standalone H7 boots the image you flashed.
- `Serial1` in `tractor_h7.ino` carries the X8↔H747 link today (camera select and keyframe requests, around lines 509, 1195 and 1635–1650). On an H7 it becomes the camera link (§5), but see §3: on a Max Carrier the camera link cannot be a spare HD UART.

### 4.2 Safety gain

MASTER_PLAN §8.10 says the LoRa TX/RX hot path must stay off Linux. Because
the Murata UART is wired to the i.MX on an X8 (`/dev/ttymxc3`, `imx-uart`
driver), the X8 bench path in fact runs radio traffic through Linux daemons.
On an H7 the M7 owns the radio, which is the §8.10 intent. This is a reason to
want the H7 profile even without export control.

### 4.3 Linux sidecar services that need a new home

| X8 service | H7-only home | Notes |
|---|---|---|
| `gps_service.py`, `time_service.py` | u-blox on an H7 I²C bus (J5 I²C1 or I²C2) or UART; PPS on a J5 PWM pin used as GPIO | Revisits §8.10's "sensors off the RT MCU" rule. Poll on the M7 with bounded timeouts and I²C bus recovery, never on the M4 loop. |
| `imu_service.py` (BNO085/086) | Same bus | 50 Hz read is light. |
| `logger_service.py`, event recorder | Ring buffer in the H7's 8 MB SDRAM, flushed to microSD | Confirm the J11 microSD (HD SDC lines) works with the H7's SDMMC. |
| `params_service.py` | mbed KVStore on QSPI flash | |
| SSH/adb field debug | USB console plus the AT service shell ([DESIGN-LORAFIRMWARE/06](DESIGN-LORAFIRMWARE/06_AT_Service_Shell_Debug_Control_Plane.md)) | |
| `fusion_service.py`, `health_service.py` (reserved) | M4 idle time or deferred | |

### 4.4 Security

AES-GCM is already MbedTLS on the H7 ([DECISIONS.md](DECISIONS.md) D-C1). The
STM32H747 has a hardware RNG but no CRYP engine, so GCM stays in software,
which is fine at LoRa rates. The X8's SE050 has to be replaced by whatever
secure element the purchased H7 revision carries; check the board before
designing key storage.

## 5. Tractor video: the options

The image work is small by design. VS1 works on a 192×128 copy of each frame
([VECTOR_SCENE.md §2.1](VECTOR_SCENE.md)), sends one fragment per frame, and
runs at 2 fps by default. That fragment carries a `TileDeltaFrame` of at most
203 B at FHSS or 243 B at DTS (a 6 B header plus a VS1 body of ≤197 or ≤237 B);
on air the 4 B fragment header and 8 B hop header come on top
([VECTOR_SCENE.md §3.1](VECTOR_SCENE.md)). The tile modes send bigger keyframes
(about 2.4 KB on the RS-3.3 bench) but are still bounded by LoRa airtime. So
whatever produces the frames needs very little bandwidth to the radio MCU, a
few kB/s at most.

### 5.1 Option A: a second H7 with the Portenta Vision Shield (recommended H7-only answer)

A second Portenta H7 carries the Vision Shield and does capture and encoding.
The tractor H7 stays the only owner of the radio and the valves.

**Can the camera H7 connect to the Max Carrier?** Not by plugging in: the Max
Carrier has one Portenta socket and the tractor H7 is in it, and the Vision
Shield itself occupies the camera H7's HD connectors. It connects by cable,
and §3 rules out a spare UART, so the choices are:

| Link | Tractor side | Camera side | Use when | Notes |
|---|---|---|---|---|
| **SPI** (recommended inside the enclosure) | J5 SPI1, tractor H7 as controller, one J5 PWM pin as a data-ready input | H7 SPI on its MKR-style edge pins, as target with DMA | Camera H7 sits in the main enclosure behind a window, cable ≤30 cm | The controller owns the clock, so a dead or hung camera can only return bytes that fail CRC. It can never stall the M7, unlike I²C, which can hang with SDA held low. |
| **Ethernet** (recommended for a remote camera) | J17 RJ45 | Vision Shield **Ethernet** variant's RJ45 | Camera in its own housing at the mount point | UDP point to point, magnetics give isolation, rugged cabling is easy. Costs the shop-debug port (or add a small switch) and puts LwIP on the M7, which is fine at these rates with a rate limit. |
| I²C (fallback) | J5 I²C2 | Edge-pin I²C as target | Short runs only | Needs timeouts and bus recovery on the M7. |

What you get and give up with the Vision Shield:

- **Monochrome sensor** (Himax: 320×320 HM01B0 on early boards, VGA HM0360 on current ones; check the board you buy). `mono_g4` works unchanged in concept. VS1 loses its colour: fills become grey, and the L2 plants layer, which relies on ExG−ExR vegetation colour, does not work.
- **Compute:** the M7 at 480 MHz with 1 MB SRAM and 8 MB SDRAM holds a VGA frame easily. VS1's A53 estimate is 32–83 ms per frame with NumPy/OpenCV ([VECTOR_SCENE.md §2](VECTOR_SCENE.md)). A C port on the M7 is plausibly 5–15× slower, which puts the encode alone at roughly 0.16–1.25 s per frame (about 0.8–6 fps). Capture and the §2.1 pre-stage come on top, so the 2 fps default holds at the fast end of that range and drops below 1 fps at the slow end. That is an estimate to measure, not a result.
- **Person detection:** NanoDet does not fit. A small 96×96 detector (Edge Impulse FOMO-class) does, with lower recall. The safety story must not depend on it, which is already the rule (§8.19).
- **Port path:** the Portenta H7 + Vision Shield is an OpenMV-supported target, so the encoder can be ported to MicroPython with `ulab` first (closer to today's NumPy code) and moved to C where profiling says so.

**Colour alternative: Arduino Nicla Vision.** Same STM32H747, a 2 MP colour
sensor, OpenMV support, small enough to mount at the camera position. It keeps
VS1's colour and plant layers. It has no Ethernet, so its link to the tractor
H7 would be SPI or UART over a short cable (or RS-485 with transceivers for a
longer run). Its WiFi/BT radio must be disabled. Check its export
classification too.

**Second-radio variant (v26 idea).** The Vision Shield **LoRa** variant carries
its own Murata module. That is the "Radio B" of the two-radio split in
[MASTER_PLAN §8.17.1](MASTER_PLAN.md#8171-two-radio-split--documented-not-adopted-for-v25):
images on their own radio and channel, with no shared queue at all. Not for
v25 (coexistence, FCC and base-side cost are exactly the §8.17.1 list), but the
hardware would already be on the tractor.

### 5.2 Option B: a Linux camera companion

Any small Linux board that is not export-restricted runs `camera_service.py`,
`x8_image_pipeline/` and the VS1 encoder unchanged, and hands finished frames
to the tractor H7 using the `ipc_to_h747.py` framing. Highest reuse and full
image quality, but it is a Linux box again, so it only helps if that board's
classification is clean. Same link choices as §5.1.

### 5.3 Option C: a camera on the tractor H7 itself

With an H7 fitted, the Max Carrier's J4 FFC connector carries the H7's DCMI
parallel camera bus, so a parallel sensor could in principle feed the tractor
H7 directly. Not recommended: J4 has no master clock or reset line, no
off-the-shelf module matches its pinout (an adapter board is needed), and
image work would share the control MCU. There is also no practical UVC host in
the mbed core, so USB cameras are out.

### 5.4 Option D: no video in the first H7 build

Control and telemetry only. A sensible first milestone while Option A is
built.

### 5.5 What carries over regardless

The LoRa image wire format (`TileDeltaFrame`, VS1 codec 6, `EncodeMode`), the
P3 class and its loss policy, `CMD_REQ_KEYFRAME`, `CMD_CAMERA_SELECT`,
`CMD_ENCODE_MODE`, `CMD_ROI_HINT`, `CMD_PERSON_APPEARED`, and the whole base
side: reassembly, canvas, VS1 decoder, renderer and web UI.

## 6. Scheduling messages between the two H7s

Short answer: yes, it is straightforward, and simpler than on the X8, because
one MCU owns the radio and the policy is already written down. The one limit
is in the L072 firmware: its TX ring is FIFO, so the M7 can bound how long a
P0 frame waits behind image frames but cannot make that wait zero until the
firmware gets a priority-aware control lane (RS-10.12). Rule 2 below states
the bound.

**What already exists**

- The normative P0–P3 policy ([LORA_PROTOCOL.md § Priority class policy](LORA_PROTOCOL.md#priority-class-policy-canonical)): strict priority with FIFO tiebreak, P0 depth-1 latest-wins, P3 refresh-cancelable, a new keyframe cancels stale fragments.
- The L072 firmware keeps a 4-deep **FIFO** TX ring that answers `QUEUE_FULL` once full, and its config notes that at FHSS the host must keep at most 3 frames in flight to stay inside a 700 ms P0 budget (RS-9.7 adds that holding §8.15's 500 ms gate needs at most 2) ([firmware/murata_l072/config.h](firmware/murata_l072/config.h), `HOST_TXQ_DEPTH`). **It does not reserve a slot for P0:** `HOST_TXQ_P0_RESERVED` is defined but referenced by no C code ([SETTINGS_REFERENCE.md](SETTINGS_REFERENCE.md) marks it dead; tracked as RS-10.12 in [TODO.md](TODO.md) and F5 in [CONTROL_PLANE_DESIGN.md](CONTROL_PLANE_DESIGN.md)). P0 priority is therefore entirely the host's job, which the design below has to enforce.
- The M7's IP-107 async TX queue in `tractor_h7.ino` (two classes today) and its SIL model `base_station/tests/test_m7_tx_queue_sil.py`.
- The camera back-channel over `Serial1` in `tractor_h7.ino`.

**Recommended design: the tractor pulls, the camera never pushes**

1. The tractor M7 sends `REQ_FRAME(budget_bytes, now_ms)` only when the P3 mailbox has room and the airtime ledger allows another frame. The camera answers with at most one frame, or "none".
2. **Bounded, not strict, P0 priority at the radio.** The M7's own queue drains strictly by priority, as LORA_PROTOCOL.md specifies. But once a frame is handed to the L072 it joins a FIFO ring and goes out in submit order (`host_cmd.c`: append at the tail, drain from the head), so a P0 can wait behind non-P0 frames already parked there. Until RS-10.12 / CONTROL_PLANE_DESIGN F5 adds a firmware control lane, the M7 enforces a bounded policy instead:
   - At most `K` non-P0 frames in flight at the L072 (parked or on air).
   - At most one P0 parked in the ring. A newer P0 replaces the older one in the M7's depth-1 latest-wins mailbox and is submitted when `TX_DONE_URC` frees the parked one. The parked P0 cannot be recalled (the L072 has no cancel).
   - No new non-P0 frame is submitted while a P0 is waiting in the M7 mailbox, so nothing can join the ring ahead of it.
   - Together these keep the ring at most `K + 1` deep (≤ `HOST_TXQ_DEPTH`), so a P0 submit is never refused with `QUEUE_FULL`. **The worst case** is a P0 that arrives just after an earlier P0 was parked behind `K` image frames: it waits for those `K` frames and the earlier P0, then its own transmission. That is `K + 1` frames in flight ahead of it, which is what the "host in-flight" figures in RS-9.7 and `config.h` count: about 400 ms at FHSS with 2 in flight, about 560 ms at FHSS with 3, about 390 ms at DTS with 4.
   - **Defaults, held to the §8.15 field gate** (E-stop <500 ms p99, [MASTER_PLAN.md §8.15](MASTER_PLAN.md#815-field-test-gate--500-m-line-of-sight-1-frame-loss-10-min)): `K = 1` at FHSS (2 in flight, ~400 ms) and `K = 3` at DTS (4 in flight, ~390 ms). `K = 2` at FHSS (~560 ms) fits only the 700 ms operator budget, and RS-9.7 records that tension with §8.15 as open: use it only if §8.15 is formally revised. The SIL model must assert this arrival sequence, not just the image frames.
   - Strict priority (P0 starts before any parked image frame) needs RS-10.12. When it lands, keep this bound as a second layer.
3. Each frame carries `capture_ms` on the tractor clock. The camera does not use the request's `now_ms` as the capture time (the frame may have been captured before the request). Instead it records its own tick when each request arrives, keeps `offset = now_ms − local_rx_tick`, and stamps `capture_ms = local_capture_tick + offset`. Link latency over SPI or point-to-point Ethernet is well under a millisecond, so this keeps VS1's last-writer-wins capture times consistent without a separate time-sync protocol. A frame captured before the camera has seen any request is discarded.
4. The M7 fragments (a C port of `pack_telemetry_fragments`; fragmenting exists only in Python today), queues the fragments as P3, and lets a new keyframe cancel stale ones.
5. Downlink commands (`CMD_CAMERA_SELECT`, `CMD_REQ_KEYFRAME`, `CMD_ENCODE_MODE`, `CMD_ROI_HINT`) ride the same link to the camera.
6. A camera-side person alert becomes `CMD_PERSON_APPEARED` (P0) only through the M7, which rate-limits it, so a faulty camera cannot flood P0.
7. Every camera-link read is bounded per loop iteration. A missing, slow or corrupt camera means "no frame this cycle" and a stale flag in telemetry, never a wait.
8. Replace `ipc_to_h747.py`'s CRC-8 with CRC-16 or CRC-32 for the H7 link: CRC-8 is weak for multi-kilobyte keyframes.

**What actually needs writing:** extend the IP-107 queue from two classes to
P0–P3 with the P3 mailbox, the bounded ring admission of rule 2, the C
fragmenter, an airtime ledger, and the camera-link driver. VS1 frames are single-fragment, which makes them the easy
case. Multi-fragment tile keyframes are where the in-flight limit `K` of
rule 2 matters.

## 7. Base station on a Portenta H7 plus a PC

1. New sketch `firmware/base_h7_gateway/`: USB-CDC ↔ L072 HostLink passthrough on `Serial3`, plus control of the L072 RESET and BOOT0 lines so the L072 can still be flashed through the H7. Most of this exists in the `LIFETRAC_MH_SOFT_BRIDGE` mode of `tractor_h7.ino` and in `portenta_m7_l072_passthrough_ping.ino`.
2. Point the PC daemons at `/dev/ttyACM0` (or `COMx`) instead of `/dev/ttymxc3`. `lora_bridge.py`, `image_rx_daemon.py` and the bench tools take the port as an argument or environment variable.
3. Add a PC compose file or native-run docs. Docker Desktop on Windows cannot pass a USB serial port into a container; run natively there, or use the existing TCP transport (`murata_host/mh_uart_tcp.c`) behind a serial-to-TCP shim. A Linux PC avoids the problem.
4. Give `bench_tools/flash_l072.sh` an H7-gateway path (the gateway enters the L072 ROM bootloader and passes bytes through).

Everything else in `base_station/` stays as is. Optionally the gateway can
speak UDP over the H7's Ethernet instead of USB so the PC can sit far from the
mast; USB first.

## 8. Other gaps

1. Power management gets simpler (no Linux clean shutdown), so [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) needs an H7 variant.
2. Field service: `X8_HEALTH_AND_RECOVERY/`, adb provisioning and OSTree tooling do not apply. The H7 equivalent is a DFU or SWD reflash procedure.
3. Bench kit: the RS-12/RS-13 leg scripts assume two X8s over adb and need a PC-hosted variant.
4. Docs that hard-code the X8: MASTER_PLAN §8.9/§8.10, ARCHITECTURE, TRACTOR_NODE, BASE_STATION, HARDWARE_BOM, FIRMWARE_UPDATES, OTA_STRATEGY.
5. The handheld (MKR WAN 1310) and the Opta are unaffected.

## 9. Repository layout

One tree, with H7 as a platform profile. Protocol, crypto, base code and the
VS1 format are never forked.

```
DESIGN-CONTROLLER/
  PLATFORM_H7.md                 this page
  firmware/
    common/lifetrac_build_config.h   add LIFETRAC_PLATFORM_X8 / LIFETRAC_PLATFORM_H7
    tractor_h7/                  name kept; already builds for both. Move the
                                 X8-only units into tractor_h7/platform_x8/
    base_h7_gateway/             new: USB-CDC <-> L072 passthrough + BOOT0/RESET
    camera_h7/                   new: Vision Shield / Nicla Vision capture + encoder
    tractor_x8/                  kept; it is also the Option B companion code
  base_station/                  unchanged; add a PC compose file and docs
```

Plus a MASTER_PLAN §8.9 amendment listing the X8 and H7 profiles side by side,
a profile column in HARDWARE_BOM, and CI jobs for the H7 profile.

## 10. Suggested order of work

1. Confirm `Serial3` reaches the Murata on Max Carrier + H7 (H7-1).
2. Build `base_h7_gateway` and run the existing base stack on a PC (H7-2, H7-3).
3. Join the control application to Method G and bring up a standalone tractor H7 with control and telemetry only (H7-4, H7-5).
4. Move GPS, IMU, params and logging onto the H7 (H7-11).
5. Add the camera H7, the P0–P3 scheduler and the pull-model camera protocol (H7-6 to H7-10).
