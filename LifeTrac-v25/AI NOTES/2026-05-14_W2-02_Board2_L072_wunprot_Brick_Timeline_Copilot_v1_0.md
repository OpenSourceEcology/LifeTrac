# 2026-05-14 — W2-02 Board 2 L072 `wunprot` Bricking Timeline (Copilot v1.0)

## Overview
This document serves as a historical record pinpointing the exact moment the STM32L072 on Board 2 became permanently locked (bricked) via software. The MCU entered an Option Byte Comparison Error lockout state after executing the WRITE_UNPROTECT (`wunprot`) routine natively on the Portenta X8.

## Execution Timeline

**Date and Time of Event:**
- **UTC:** `2026-05-14 03:06:01Z`
- **Local Time:** May 13th, 2026 at 10:06:01 PM

**Command Invocation:**
The event was triggered when the following command string was issued to the Portenta X8 via ADB:
```bash
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S bash /tmp/recover_l072_opt.sh 2>&1"
```

**Underlying Execution:**
Inside the `recover_l072_opt.sh` wrapper, it executed the Python flashing utility over the internal UART bus (`/dev/ttymxc3`):
```bash
python3 flash_l072_via_uart.py wunprot
```

## Result and State Change

1. **Successful Receipt of Command (03:06:01Z):**
   The host script successfully issued the `0x73` (WRITE_UNPROTECT) command, and the L072 Bootloader acknowledged it.
   *Log Output:*
   > `WRITE_UNPROTECT ACK — chip is auto-resetting; OPT WRP should now be 0xFFFF`

2. **Hardware Lockout (03:06:02Z - onwards):**
   The L072 completed its required self-reset. However, because the unprotect routine corrupted the Option Bytes (causing the primary bytes to no longer match their complements - specifically `nRDP` and `nUSER`), the MCU core permanently halted itself in hardware per RM0376 Section 3.4.2 (Option Byte Mismatch Lockout).
   
   When the host script immediately polled for the subsequent verification state, it received no response:
   *Log Output:*
   > `OSError: cmd 0x00: no response ← chip silent`

## Conclusion
At that precise second, Board 2's L072 transitioned from acting responsively (able to read OPT bytes) to permanently silent. All subsequent software-based attempts, including isolated UART initialization and aggressive reset timings, proved futile because the core is physically halted prior to executing the ROM Bootloader. 

Recovery requires a physical SWD connect to directly write corrected Option Bytes to the Flash interface.