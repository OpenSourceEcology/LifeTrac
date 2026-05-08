# Portenta X8 & Max Carrier: External Segger J-Link Bus Contention Bypass

**Date**: May 7, 2026
**Objective**: Successfully attach the external Max Carrier Segger J-Link/RTT debugger to the Portenta X8 STM32H747 target.
**Status**: Comprehensive fix combining hardware isolation toggles and Linux bus contention resolution.

---

## 1. The Root Cause of `InitTarget() Failed`

Attempts to use the Max Carrier's micro-USB debug port on the Portenta X8 fail due to two overlapping blockers:

1. **Hardware Isolation (The Gate)**: The Max Carrier routes the JTAG/SWD lines through an isolation circuit controlled by the target STM32 itself via pin `PC_0`. If `PC_0` is not asserted by the sketch, the physical connection to the micro-USB is severed.
2. **Electrical Bus Contention (The Clamp)**: The Portenta X8 runs an internal `openocd` service on its i.MX8M Linux host (`monitor-m4-elf-file.service`). This service bit-bangs the debugging signals directly using Linux GPIOs 8, 10, and 15. Because Linux is driving these pins, connecting an external J-Link causes an electrical fight (bus contention). The external J-Link reads back corrupted SW-DP IDs (like `0x5ba02477`) and fails.

To unlock external debugging, we must open the hardware gate *and* force Linux to let go of the SWD lines.

---

## 2. Hardware Setup

**Do NOT unplug any cables.** Both data pathways must be active simultaneously.
1. **12V Barrel Power**: Plugged into the Max Carrier to prevent brownouts.
2. **USB-C Cable**: Plugged into the Portenta X8 directly. This provides the ADB access needed to kill the Linux services.
3. **Micro-USB Cable**: Plugged into the Max Carrier debug port. This routes into the Segger J-Link chip.

---

## 3. Step-by-Step Jailbreak Procedure

### Step 1: Open the Hardware Gate (`PC_0`)
Your Arduino sketch must intentionally un-isolate the JTAG lines on boot.
Add this to the absolute top of your `setup()` function in the target sketch:

```cpp
void setup() {
  // PC_0 controls the JTAG SWD isolation on the Max Carrier
  pinMode(PC_0, OUTPUT);
  // Assert LOW to route SWD to the external Segger (change to HIGH if inverted)
  digitalWrite(PC_0, LOW); 

  // Continue with normal setup...
  SEGGER_RTT_Init();
}
```
*Compile and upload this sketch via your standard script (using the USB-C ADB route).*

### Step 2: Release the Linux Bus Contention
Immediately after the sketch uploads and the board reboots, the Linux OS will grab the SWD lines again. You must kill its internal openocd service before launching Segger J-Link.

Open your local terminal and execute:
```bash
# Drop into the ADB shell
adb shell

# Stop the services holding the SWD pins (default fio password)
echo fio | sudo -S systemctl stop monitor-m4-elf-file.service
echo fio | sudo -S systemctl stop m4-proxy.service
```

### Step 3: Attach External J-Link
With the Linux OS effectively blinded and the STM32 holding the Max Carrier gate open, the external J-Link is now the sole master of the SWD bus.

1. Open **J-Link Commander** (`JLink.exe`).
2. Type `connect`.
3. Select Device: `STM32H747XI_M7` (or Cortex-M7).
4. Select Interface: `SWD`.

The target should successfully initialize, returning the correct `0x6BA02477` SW-DP ID, finally enabling full hardware step-through and RTT logging.