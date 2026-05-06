# Method G Hardware Test Status (Copilot v1.1)

Date: 2026-05-05
Status: Toolchain and route diagnostics extended. Method G firmware is restored on both boards, but physical Murata host route remains unproven on this workstation/core setup.
Scope: Delta update after v1.0 while continuing bench bring-up work.

## 1. Delta from v1.0

This note adds results from additional next-step diagnostics run after the v1.0 status note.

## 2. Core/toolchain diagnostics performed

1. Arduino CLI core/config state
- Installed cores now include:
  - `arduino:mbed_portenta 4.5.0`
  - `arduino:renesas_portenta 1.5.3` (added during this run)
- `arduino-cli config dump` reports:
  - `board_manager.additional_urls: []`
- Local package indexes present only default `package_index.json` source.

2. Board availability after adding alternate core
- `renesas_portenta` adds Portenta C33 target only.
- Portenta X8 target remains from `arduino:mbed_portenta` core path.

3. Installed X8 `libmbed` symbol inspection
- `PORTENTA_X8/libs/libmbed.a` exports `SetSysClock` (weak), `lp_ticker_init`, `us_ticker_init`.
- `SetSysClock_HSE_disabled` symbol not found.
- Interpretation: currently installed X8 core appears to be stock-style symbol layout, not the previously described patched variant.

## 3. Method G route compile diagnostics (current core)

All Method G route candidates beyond Serial1 fail on this machine/core:

1. `LIFETRAC_MH_SERIAL=Serial2`
- Link failure: undefined reference to `_UART2_`.

2. `LIFETRAC_MH_SERIAL=Serial4`
- Link failure: undefined reference to `_UART4_`.

3. `LIFETRAC_MH_SERIAL=Serial5`
- Compile failure: `Serial5` not declared.

Implication:
- Current X8 core path does not expose a viable non-Serial1 Method G UART instance via this build flow.

## 4. DFU tooling delta

- `dfu-util` is now present from Arduino tool package:
  - `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/tools/dfu-util/0.11.0-arduino5/dfu-util.exe`
- `dfu-util -l` was run and returned no enumerated DFU targets in current board state.

## 5. Serial1 route probe attempt

A temporary probe sketch (`x8_uart_route_probe`) was created to emit marker lines on `Serial1` at 115200 and flashed to board A (COM11) for route discovery.

Linux-side scan on board A:
- `/dev/ttymxc1`, `/dev/ttymxc2`, `/dev/ttymxc3` checked as root.
- No `LT_ROUTE_PROBE` marker observed on any tested endpoint.

Interpretation:
- `Serial1` is not currently observable on those Linux endpoints in this setup, or the active route is elsewhere/not bridged in a readable way.

Cleanup:
- Board A (COM11) was restored to the Method G firmware image after probe testing.

## 6. Current board firmware state

- Board A (COM11 / 2D0A1209DABC240B): Method G image restored (upload exit 0).
- Board B (COM12 / 2E2C1209DABC240B): Method G image remains from prior flash (upload exit 0 earlier in session).

## 7. Current blocker statement

With the currently installed X8 core stack on this workstation:

1. Method G code changes and flashing are working.
2. Live runtime observability and physical Murata route proof remain blocked by missing/unsupported UART route exposure for `Serial2/4/5` and non-observable `Serial1` route mapping.

## 8. Recommended next actions

1. Obtain the exact patched X8 core package source (URL/package name/version) expected by this project and install it in Arduino CLI.
2. Re-run route compile checks for `Serial2`/`Serial4`/`Serial5` immediately after patched core install.
3. If patched core is unavailable, capture physical UART route with external sniff (UART_SNIFF/logic analyzer) and align Method G host serial selection to that proven route.
4. Repeat BOOT/VER evidence capture once a proven host route is active.
