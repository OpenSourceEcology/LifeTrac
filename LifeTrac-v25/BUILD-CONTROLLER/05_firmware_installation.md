# 5. Firmware & Software Installation

> **Heads up!** Don't flash any firmware to a tractor with the hydraulic system live. Always do first-time flashes on a bench, with the valve coil rail disconnected. Once the firmware passes [§ 6 Bring-Up & Testing](06_bringup_and_testing.md), it's safe to install on the live machine.

This section gets the four firmware/software components onto your hardware:

| # | Target | What runs there | Tooling |
|---|---|---|---|
| 1 | **Tractor X8 — H747 co-MCU** | `firmware/tractor_h7/` (M7 + M4) — radio, arbitration, Modbus master | Arduino IDE 2 or PlatformIO |
| 2 | **Tractor Opta WiFi** | `firmware/tractor_opta/` — Modbus slave, valve I/O, watchdog | Arduino IDE 2 |
| 3 | **Tractor X8 — Linux side** | `firmware/tractor_x8/` — GPS + IMU MQTT publishers (Python) | SSH + Docker |
| 4 | **Base X8 — Linux side** | `base_station/` — `lora_bridge.py` + `web_ui.py` + nginx + Mosquitto | SSH + Docker Compose |
| 5 | **(Optional) Handheld MKR WAN 1310** | `firmware/handheld_mkr/` — joystick read + LoRa TX | Arduino IDE 2 |

> **Pro tip:** Per [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.2, the **base station has no Arduino firmware target** — Linux on the base X8 drives the SX1276 directly over SPI. There's no `firmware/base_*/` to build or flash.

## Prerequisites

Install on your dev laptop:

- **Arduino IDE 2.x** ([download](https://www.arduino.cc/en/software))
  - Add **Arduino Mbed OS Portenta Boards** in *Boards Manager*.
  - Add **Arduino Mbed OS Opta Boards** in *Boards Manager*.
  - Add **Arduino SAMD Boards** in *Boards Manager* (for the MKR WAN 1310).
- **Libraries** (install from *Library Manager*; full list in [`../DESIGN-CONTROLLER/arduino_libraries.txt`](../DESIGN-CONTROLLER/arduino_libraries.txt)):
  - `RadioLib` ≥ 6.4
  - `ArduinoModbus`
  - `ArduinoRS485`
  - `Arduino_OptaController`
  - `Adafruit SSD1306` *(handheld only)*
- **Git** to clone the repo.
- **SSH client** (built into macOS / Linux / Windows 10+).
- **Docker Desktop** or any Docker installation, only if you want to test the base-station containers locally before deploying.

Clone the repo if you haven't already:

```bash
git clone https://github.com/OpenSourceEcology/LifeTrac.git
cd LifeTrac/LifeTrac-v25/DESIGN-CONTROLLER
```

## Step 1 — Provision the Pre-Shared Key

The LoRa link uses **AES-128-GCM** with a 16-byte pre-shared key. Generate it once and copy it to all three nodes' source trees **before** flashing.

```bash
# from the repo root
python3 -c "import secrets; print(secrets.token_hex(16))"
```

This prints something like `9f3a8c4e2b1d7506a8e3f4c5b9d2e1f0`. Copy it into a file called `firmware/common/key.h`:

```c
// key.h — DO NOT COMMIT
#define LIFETRAC_PSK_HEX "9f3a8c4e2b1d7506a8e3f4c5b9d2e1f0"
```

`firmware/common/key.h` is in `.gitignore`. Keep a backup somewhere safe (a password manager works). Losing this key means re-flashing every node.

## Step 2 — Flash the Tractor H747 Co-MCU

This is the radio + arbitration + Modbus master firmware that runs on the X8's onboard STM32H747.

1. Open `firmware/tractor_h7/tractor_m7.ino` in Arduino IDE 2.
2. Select **Tools → Board → Arduino Mbed OS Portenta Boards → Portenta H7 (M7 core)**. (The X8's onboard H747 enumerates the same way as a standalone H7 for the toolchain — same M7 binary.)
3. Select **Tools → Port** → the X8's USB-C serial port.
4. Click **Verify** (✓). Confirm it compiles cleanly.
5. Click **Upload** (→). The X8 reboots and enters DFU; a successful flash takes ~30 s.
6. Repeat for the M4 sketch: open `firmware/tractor_h7/tractor_m4.cpp`, switch board to **Portenta H7 (M4 core)**, upload.

Open the Serial Monitor at 115200. You should see:

```
[boot] LifeTrac v25 tractor M7 / firmware xxxxxxx
[lora] SX1276 init OK, freq 915.0 MHz, SF7 BW125 CR4/5
[mb]   modbus master @ 115200 8N1, slave addr 0x01
[arb]  no active source yet, all coils held neutral
```

If the Modbus master can't reach the Opta yet — that's expected. We haven't flashed the Opta. Move on.

## Step 3 — Flash the Tractor Opta

1. **Disconnect the valve coil rail** at the PSR safety relay. Even though the firmware boots into safe-state, do not give it the chance to drive a coil during a partial flash.
2. Connect a USB-C cable from your laptop to the Opta WiFi.
3. Open `firmware/tractor_opta/opta_modbus_slave.ino`.
4. Select **Tools → Board → Arduino Mbed OS Opta Boards → Opta WiFi**.
5. Select the Opta's serial port.
6. Click **Upload**.

Open the Serial Monitor at 115200:

```
[opta] boot, slave addr 0x01
[d1608s] enumerated 8 SSR + 16 input channels
[a0602]  enumerated 6 AI + 2 AO
[wd]    watchdog armed @ 200 ms, awaiting first master write
[selftest] cycling each SSR off->on->off ... PASS
```

Reconnect the M7 sketch's serial monitor; you should now see:

```
[mb]   slave 0x01 alive, watchdog handshake OK
[arb]  awaiting heartbeat from any source...
```

## Step 4 — Set Up the Base Station Linux Stack

SSH into the base X8:

```bash
ssh root@<base-station-ip>
```

> **Heads up!** The first thing you should do is `passwd` to change the default password, and `ssh-copy-id` your laptop's public key. If the base is on a network you don't fully control, also disable password SSH login.

Install Docker (Yocto images for the X8 ship with Docker available — check `docker --version`; if missing, follow the [Arduino Portenta X8 Docker guide](https://docs.arduino.cc/tutorials/portenta-x8/x8-and-docker)).

Clone the repo onto the base:

```bash
cd /opt
git clone https://github.com/OpenSourceEcology/LifeTrac.git lifetrac
cd lifetrac/LifeTrac-v25/DESIGN-CONTROLLER/base_station
```

Copy your `key.h` (rename to `key.env` — same content, different format for Docker):

```bash
echo "LIFETRAC_PSK_HEX=9f3a8c4e2b1d7506a8e3f4c5b9d2e1f0" > .env
```

Set the operator PIN (per [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.5):

```bash
echo "WEBUI_PIN=123456" >> .env   # change to your own 4–6 digit PIN
```

Bring up the stack:

```bash
docker compose up -d
docker compose ps
```

You should see `nginx`, `web_ui`, `mosquitto`, `lora_bridge`, and `timeseries` all running.

Browse to `http://<base-station-ip>/` from another machine on your LAN. You should get the operator login page.

> **Heads up!** Per [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.5, the v25 web UI is **plain HTTP, LAN-only, single shared PIN**. Do not expose it to the public internet. If you must, put it behind a VPN or a reverse proxy with TLS — don't roll your own.

## Step 5 — Set Up the Tractor X8 Linux Services

These two Python services publish IMU and GPS data to the bridge:

```bash
ssh root@<tractor-ip>     # over WiFi during bench setup, or wired Ethernet
cd /opt/lifetrac/LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8
pip3 install -r requirements.txt
```

Verify the MCP2221A enumerated and both Qwiic devices answer:

```bash
i2cdetect -y 0
# you should see 0x42 (NEO-M9N) and 0x4A or 0x4B (BNO086)
```

Start the services (systemd units in `/etc/systemd/system/lifetrac-*.service` — install per the README in `firmware/tractor_x8/`):

```bash
systemctl enable --now lifetrac-imu lifetrac-gps
journalctl -u lifetrac-imu -f
```

Telemetry should start streaming over the X8↔H747 UART; the M7 picks it up, frames it, and sends it as a TelemetryFrame on the next radio slot.

## Step 6 — Flash the Handheld *(Optional)*

1. Connect the MKR WAN 1310 over USB-C.
2. Open `firmware/handheld_mkr/handheld.ino`.
3. Select **Tools → Board → Arduino SAMD Boards → MKR WAN 1310**.
4. Select the serial port. Upload.

Open the Serial Monitor at 115200:

```
[boot] LifeTrac v25 handheld
[lora] SX1276 init OK, freq 915.0 MHz, SF7 BW125 CR4/5
[ui]   joysticks calibrated, OLED ready
[tx]   sending HeartbeatFrame @ 20 Hz
```

The OLED should show source ID, sequence number, and link state.

## Acceptance Test

- [ ] Tractor M7 logs show `lora init OK` and `modbus alive`.
- [ ] Tractor Opta logs show selftest PASS and watchdog alive.
- [ ] Base station web UI reachable at `http://<base-ip>/`, login screen renders.
- [ ] Logging in with the configured PIN shows the operator console (joystick widget, telemetry sidebar, source banner).
- [ ] Tractor X8 `i2cdetect` shows both 0x42 and 0x4A/0x4B.
- [ ] *(Optional)* Handheld MKR boots, OLED shows status.

## Updating Firmware Later

When the repo gets a new release (`git tag controller-vX.Y.Z`):

1. `git pull` on each Linux side; `docker compose pull && docker compose up -d` on the base.
2. Re-flash any sketches whose source changed (Arduino IDE will show a fresh build is needed).
3. **Always re-run the [§ 6 Bring-Up](06_bringup_and_testing.md) failsafe tests** before letting the tractor near a load.

## Next Step

[**6. Bring-Up & Testing →**](06_bringup_and_testing.md).
