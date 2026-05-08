# Method G Phase 1 Test Execution Findings (Copilot v1.0)

Date: 2026-05-06  
Author: GitHub Copilot (GPT-5.3-Codex)  
Scope: Execution of the agreed "next tests" plan for Murata L072 Method G firmware bring-up gates.

---

## 1. Executive Summary

The software-side Phase 1 gates that can be executed from this Windows workspace are green.

The full bench-dependent gates (flash/recovery over H7 path, live AT command interaction, and on-target forced HardFault runtime proof) remain blocked in this session due missing confirmed hardware path/port response.

Result: **Phase 2 is not yet unlocked**. Build and static regression quality is now strong; hardware validation is still required.

---

## 2. Test Plan Coverage and Status

| Plan Item | Status | Notes |
|---|---|---|
| 1) Host-side smoke (artifact sanity, check target) | PASS | clean/check/all completed; ELF/BIN/HEX generated; map and section checks passed |
| 2) Flash + brick-resistance (Phase 1 W1-5/W1-6) | BLOCKED | Could not execute H7-driven flash/reset/recovery sequence from this host session |
| 3) AT service shell validation | BLOCKED | COM11/COM12 opened but returned no response at tested baud rates |
| 4) Boundary/regression tests | PARTIAL PASS | Windows clean/all pass; LD flag path behavior verified; WSL test blocked |
| 5) Gate to Phase 2 | NOT CLEARED | Hardware-dependent exit criteria still red/unknown |

---

## 3. Commands Executed and Findings

### 3.1 Clean build and check flow

Executed from:
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072`

Commands:

```powershell
$cross="C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-"
mingw32-make clean
mingw32-make check
mingw32-make CROSS="$cross" all
```

Observed:
- `check` passed with `[OK] memory map invariants hold`.
- Full compile/link succeeded.
- Outputs produced: `build/firmware.elf`, `build/firmware.bin`, `build/firmware.hex`.

### 3.2 Artifact sanity checks

Commands:

```powershell
arm-none-eabi-size build/firmware.elf
arm-none-eabi-objdump -h build/firmware.elf
arm-none-eabi-nm --print-size --size-sort build/firmware.elf | Select-Object -Last 20
```

Key outputs:
- `size`: text=12856, data=8, bss=10888, dec=23752, hex=5cc8.
- Section placement confirmed:
  - `.isr_vector` at `0x08000000`
  - `.boot_text` at `0x080000c0`
  - `.boot_rodata` at `0x0800037c`
  - `.text` at `0x08001000` (matches MM_APP_BASE)
  - `.cfg_reserved` at `0x0802e000` (matches MM_CFG_BASE)
- Boot region occupancy from build summary:
  - `.boot_text` 700 bytes
  - `.boot_rodata` 20 bytes
  - total 720 bytes (within 4 KB BOOT budget)

### 3.3 Symbol placement for safe-mode constants

Command:

```powershell
arm-none-eabi-objdump -t build/firmware.elf | Select-String "kSafeMode"
```

Observed:
- `kSafeModeBauds` at `0x0800037c` in `.boot_rodata`
- `kSafeModeMagic` at `0x08000388` in `.boot_rodata`

This verifies the section-mapping fix is active and linked as intended.

### 3.4 HardFault handler assembly validation (static)

Command:

```powershell
arm-none-eabi-objdump -d build/firmware.elf | Select-String "<HardFault_Handler>" -Context 0,16
```

Observed disassembly confirms:
- `mov r2, lr`
- `tst r2, r1`
- conditional `mrs r0, MSP` / `mrs r0, PSP`
- branch to `HardFault_Handler_C`

This confirms Thumb-1 compatible codegen for Cortex-M0+ and expected fault-context handoff behavior in static form.

### 3.5 Windows clean/remove regression

Command:

```powershell
mingw32-make clean
if (Test-Path build) { "build_dir_exists" } else { "build_dir_removed" }
```

Observed:
- `build_dir_removed`

Then a full rebuild was rerun and succeeded, validating `clean` + fresh build behavior.

### 3.6 Optional linker warning flag path behavior

Command:

```powershell
mingw32-make clean
mingw32-make CROSS="$cross" LD_NO_WARN_RWX=1 all
```

Observed:
- Linker failed with `unrecognized option '--no-warn-rwx-segments'` on Arduino 7-2017q4 `ld`.

Interpretation:
- Expected behavior on this older toolchain.
- Confirms the Makefile toggle path is wired correctly and defaulting flag OFF is required here.

### 3.7 WSL/Linux-side validation attempt

Command:

```powershell
wsl.exe -e bash -lc "uname -a"
```

Observed:
- WSL not installed.
- Install requires elevation.

Result:
- Linux-side counterpart test is blocked on host environment setup.

### 3.8 Serial port and AT probing

Detected ports:
- COM3 (Intel AMT SOL)
- COM11 (USB Serial Device)
- COM12 (USB Serial Device)

AT probes executed on COM11 and COM12 at 921600, 115200, 9600, and 19200 using CRLF commands (`AT`, `ATI`, `AT+VER?`, `AT+RADIO?`, `AT+STAT?`, `AT+HELP`, `AT+BIN`, `ATBOGUS`).

Observed:
- No response on all attempted combinations.

Result:
- Live AT shell verification remains blocked in this session.

---

## 4. Hardware-Path Context Discovered

Existing scripts in repository indicate Linux/X8-side UART workflows are already established:
- `LifeTrac-v25/tools/at_probe.sh`
- `LifeTrac-v25/tools/at_probe2.sh`
- `LifeTrac-v25/tools/at_probe.py`

These target `/dev/ttymxc3` and GPIO reset lines, which suggests bench validation is expected from the controller-side Linux environment rather than this Windows host terminal alone.

---

## 5. Gate Decision

Per `DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md` Phase 1 exit criteria:
- Build pipeline quality and static checks: **satisfied**.
- Boot/recovery/AT live hardware checks: **not yet satisfied**.

Decision: **Do not start Phase 2 yet**.

---

## 6. Recommended Immediate Next Actions

1. Identify which physical connection corresponds to COM11/COM12 and verify that it is the active H7-L072 host UART path.
2. Run flash and recovery tests through the intended H7-driven bootloader flow (W1-4/W1-5/W1-6), then capture logs.
3. Execute AT command matrix on the confirmed live endpoint and record full transcript.
4. Run one deliberate on-target HardFault injection and confirm `HardFault_Handler_C` context behavior at runtime.
5. Re-evaluate Phase 1 gate immediately after these bench results are captured.

---

## 7. Confidence Statement

Confidence is high for toolchain/build-system stability on Windows and for correctness of the recent linker/startup/section fixes at static-analysis level.

Confidence is incomplete for end-to-end bring-up until hardware-path validation (flash, safe-mode jump, reset path, AT interaction) is observed live on target.
