# T6 Flash Path Proof

Date: 2026-05-05
Scope: Gate 3 flash-path readiness/proof evidence

## Local tool probe

Initial workstation probe:
- `adb`: missing
- `dfu-util`: missing
- `openocd`: missing

Action taken:
- Installed OpenOCD via winget package `xpack-dev-tools.openocd-xpack`.
- Resolved binary path:
  - `C:/Users/dorkm/AppData/Local/Microsoft/WinGet/Packages/xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe/xpack-openocd-0.12.0-7/bin/openocd.exe`

Current status:
- `openocd`: available by absolute path
- `dfu-util`: still missing on this workstation

## Proven command template (SWD path)

From `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072`:

```powershell
& 'C:/Users/dorkm/AppData/Local/Microsoft/WinGet/Packages/xpack-dev-tools.openocd-xpack_Microsoft.Winget.Source_8wekyb3d8bbwe/xpack-openocd-0.12.0-7/bin/openocd.exe' `
  -f openocd/stlink.cfg -f openocd/stm32l0_swd.cfg `
  -c "program build/firmware.elf verify reset exit"
```

## Physical-board proof status

Live USB presence captured (2026-05-05):
- `Ports | USB Serial Device (COM11) | OK | USB\VID_2341&PID_0061&MI_00\6&27FE2CFF&0&0000`
- `Ports | USB Serial Device (COM12) | OK | USB\VID_2341&PID_0061&MI_00\6&25D14DA8&0&0000`
- `USBDevice | ADB Interface | OK | USB\VID_2341&PID_0061&MI_02\6&27FE2CFF&0&0002`
- `USBDevice | ADB Interface | OK | USB\VID_2341&PID_0061&MI_02\6&25D14DA8&0&0002`
- `USB | USB Composite Device | OK | USB\VID_2341&PID_0061\2D0A1209DABC240B`
- `USB | USB Composite Device | OK | USB\VID_2341&PID_0061\2E2C1209DABC240B`

Still pending in this session:
- ST-Link/J-Link SWD verify transcript on the target board
- `dfu-util -l` transcript (tool not yet installed on this workstation)
- CN2-solder photo

## Latest continuation probe (2026-05-05)

Windows USB enumeration still shows the two connected Portenta X8 / Max Carrier USB devices:
- `USB Serial Device (COM11)` / `USB\VID_2341&PID_0061...`
- `USB Serial Device (COM12)` / `USB\VID_2341&PID_0061...`
- two matching `ADB Interface` entries under `VID_2341&PID_0061`

No present USB device matched DFU, ST-Link/STLINK, J-Link/SEGGER, `VID_0483`, or `VID_1366` during this probe.

Tool state:
- OpenOCD available: `xPack Open On-Chip Debugger 0.12.0+dev-02228-ge5888bda3-dirty`
- `dfu-util`: still missing

Interpretation:
- Gate 3 flash proof is blocked on physical flash-path setup, not H7 Method G source code.
- To close this file, connect a soldered-CN2 SWD debugger or put the target L072 into a proven DFU-visible mode and archive the resulting verify/flash transcript.

## Gate interpretation

- SWD tooling prerequisite is now partly closed (OpenOCD installed).
- Board-specific flash proof remains open until one of the following is archived:
  1. successful OpenOCD verify transcript on target board, or
  2. successful DFU enumeration + flash transcript on target board.
