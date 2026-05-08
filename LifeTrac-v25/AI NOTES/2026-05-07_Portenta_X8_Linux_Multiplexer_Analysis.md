# Portenta X8 Linux Multiplexer Analysis (The "Rabbit Hole" Approach)

**Date**: May 7, 2026
**Target**: Arduino Portenta X8 + Max Carrier
**Objective**: Force the hardware multiplexer to route UART/SWD from the STM32 target to the Segger J-Link micro-USB by manipulating the i.MX8M Linux OS.

## 1. Overview
The Max Carrier routes its `UART_SNIFF` and `SWD/JTAG` lines through physical hardware multiplexer ICs (U16/U19). By default, the i.MX8M Linux processor on the Portenta X8 acts as the master and holds the GPIO control pins that dictate which way these multiplexers route signals. To unlock the micro-USB debugger port without physical wire hacking, we must log into the Linux root shell and manually flip the internal state of those GPIO pins.

## Phase 1: Schematic & Pin Identification
Before we can write a command, we have to identify the exact Linux GPIO pin number.
1. **Analyze Portenta X8 Schematics**: We must trace the lines `UART_CH1-3_S` and `CORTEX_JTAG_EN` (or equivalent net names) from the high-density connectors on the X8 module back to the i.MX8Mini processor.
2. **Translate Pin name to i.MX8M GPIO Bank**: The i.MX processor groups pins into banks (e.g., `GPIO1_IO05`).
3. **Calculate Linux Sysfs Number**: The Linux kernel maps these banks to flat integer numbers. 
   - Formula: `(Bank - 1) * 32 + IO_Number`
   - *Example: `GPIO1_IO05` = (1-1)*32 + 5 = Linux GPIO 5.*

## Phase 2: Gaining Root Access
We must utilize the ADB connection to gain root-level access to the Yocto/Foundries.io Linux environment running on the Portenta.
1. Connect via USB-C and ensure ADB is stable.
2. Authorize adb root access:
   ```bash
   adb root
   adb shell
   ```

## Phase 3: Inspecting & Modifying the Pin State
We will use the modern `libgpiod` tools (or legacy sysfs) strictly within the Linux shell to seize control of the multiplexer pin.

### Approach A: Using `libgpiod` (Preferred for modern kernels)
1. Find the GPIO chip and line name:
   ```bash
   gpioinfo | grep -i "mux"  # Or search for the known pin
   ```
2. Force the pin HIGH (or LOW, depending on the schematic logic) to switch the multiplexer channel:
   ```bash
   gpioset gpiochipX Y=1
   ```

### Approach B: Using Legacy `/sys/class/gpio`
If `gpioset` is unavailable, we use raw file operations:
1. Export the calculated integer pin (e.g., 5):
   ```bash
   echo 5 > /sys/class/gpio/export
   ```
2. Set the pin as an output:
   ```bash
   echo out > /sys/class/gpio/gpio5/direction
   ```
3. Set the pin HIGH (1) or LOW (0) to switch the gate:
   ```bash
   echo 1 > /sys/class/gpio/gpio5/value
   ```

## Phase 4: Overcoming the Device Tree Blockade (The True Rabbit Hole)
If Linux responds to the above commands with `Resource busy` or `Device or resource busy`, it means the Linux Kernel's **Device Tree (DT)** has permanently claimed the pin at boot time for another driver.
1. **Extract the Device Tree**:
   We must extract the active configuration from the live system:
   ```bash
   dtc -I fs -O dts -o /tmp/extracted_dt.dts /sys/firmware/devicetree/base
   ```
2. **Decompile & Edit**:
   Open `/tmp/extracted_dt.dts`, search for the offending pin assignment (e.g., a node claiming our multiplexer control pin), and delete or disable (`status = "disabled";`) the node.
3. **Recompile the Device Tree Blob (DTB)**:
   ```bash
   dtc -I dts -O dtb -o /tmp/custom_dt.dtb /tmp/extracted_dt.dts
   ```
4. **Flash / Replace the DTB**:
   Mount the boot partition as read-write, swap the `.dtb` file, and completely reboot the Portenta X8 Linux environment to free the pin.

## Phase 5: Validation
Once the GPIO is successfully flipped (either dynamically or via a hacked Device Tree), the hardware multiplexer on the Max Carrier will physically click over.
1. Re-connect Segger RTT Viewer or open COM8/COM13 in a Serial Monitor.
2. Execute the Arduino sketch on the M7 core.
3. The logs should instantly route through the micro-USB to the PC.

*Note: This configuration will not survive a hard reboot unless scripted into a boot service (like a custom systemd unit script).*