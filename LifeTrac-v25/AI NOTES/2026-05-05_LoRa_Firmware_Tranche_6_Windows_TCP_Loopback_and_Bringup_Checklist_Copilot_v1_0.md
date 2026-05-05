# LoRa Firmware Tranche 6: Windows TCP Loopback + Bench Bring-up Checklist (Copilot v1.0)

Date: 2026-05-05
Scope: follow-up to Tranche 6 implementation with (1) non-WSL Windows loopback validation path and (2) concise bench commands aligned with current Max Carrier wiring assumptions.

## 1) Windows-compatible local loopback (non-WSL)

### What changed

- Added a TCP transport adapter for host-CC driver tests:
  - `firmware/tractor_h7/murata_host/mh_uart_tcp.c`
  - `firmware/tractor_h7/murata_host/mh_uart.h` (`mh_uart_tcp_state_t`, `mh_uart_tcp_init`)
- Extended loopback driver endpoint handling:
  - `bench/h7_host_proto/loopback_driver.c`
  - Accepts POSIX endpoint path (`/dev/pts/N`) and TCP endpoint (`tcp://host:port`)
  - Windows build now requires TCP endpoint mode
- Extended Python harness transport selection:
  - `tools/murata_host_loopback.py`
  - `--transport auto|pty|tcp` (auto picks PTY when available, else TCP)
- Updated mock writer interface so one chunking model works for PTY and TCP:
  - `tools/murata_host_l072_mock.py`

### Windows local command sequence

From repository root:

```powershell
Set-Location LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7
New-Item -ItemType Directory -Force -Path build | Out-Null

gcc -std=gnu11 -Wall -Wextra -Werror -Wpedantic -Imurata_host `
  bench/h7_host_proto/loopback_driver.c `
  murata_host/mh_cobs.c `
  murata_host/mh_crc16.c `
  murata_host/murata_host.c `
  murata_host/mh_stats.c `
  murata_host/mh_uart.c `
  murata_host/mh_uart_tcp.c `
  murata_host/mh_stream.c `
  -lws2_32 `
  -o build/loopback_driver_tcp.exe

python ../../tools/murata_host_loopback.py `
  --transport tcp `
  --driver build/loopback_driver_tcp.exe `
  --iterations 150
```

Expected result:

- `loopback verdict pass: ...`
- Non-zero reject counters for injected malformed frames (`rej_crc`, `rej_len`, `rej_ver`).

## 2) Concise bring-up command checklist (tailored to current bench wiring)

Assumed wiring profile:

- Target board: Portenta X8 + Max Carrier with Murata CMWX1ZZABZ-078
- Host transport path: L072 USART2 mapped to H7/X8 `Serial3` / `/dev/ttymxc3`
- Reset control path: `LORA_NRST` via `/dev/gpiochip5` line 3 (`gpio163`)
- SWD flash path: ST-Link on SWDIO/SWCLK/GND/VREF (+NRST when available)

### Step A: Build + flash Murata L072 firmware

```powershell
Set-Location LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
make all
openocd -f openocd/stlink.cfg -f openocd/stm32l0_swd.cfg -c "program build/firmware.elf verify reset exit"
```

### Step B: Boot-stage sanity checks on host UART

Required observations from early URCs:

- `clock_source_id == 0`
- `radio_ok == 1`
- No recurring `HOST_FAULT_CODE_CLOCK_HSE_FAILED (0x08)`

### Step C: RX validation (single-board)

- Keep RX armed (default firmware behavior)
- Inject a known-good LoRa packet from peer
- Confirm `HOST_TYPE_RX_FRAME_URC` and rising `radio_rx_ok`

### Step D: Two-board round-trip validation

- Board A issues `HOST_TYPE_TX_FRAME_REQ`
- Board A receives `HOST_TYPE_TX_DONE_URC`
- Board B receives matching `HOST_TYPE_RX_FRAME_URC`
- 5-minute attenuated pass target:
  - TX_DONE success >= 99%
  - payload-matching RX_FRAME >= 99%
  - no persistent `radio_tx_abort_airtime` growth

### Step E: Save evidence bundle

Store under:

- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/`

Recommended files:

- UART capture with BOOT/FAULT/STATS URCs
- TX_DONE/RX_FRAME timestamp CSV
- optional RF/mode screenshots when relevant

## Notes

- This checklist is derived from `firmware/murata_l072/BRINGUP_MAX_CARRIER.md` and the method-G wiring contract in `DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md`.
- PTY loopback remains the Linux CI path; TCP loopback is the local Windows parity path for the same stream-layer validation intent.
