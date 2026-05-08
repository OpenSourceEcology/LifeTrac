# Portenta X8 Max Carrier JTAG/SWD Multiplexer Bypass via STM32

**Date**: May 7, 2026
**Objective**: Enable Segger J-Link / RTT debugging on the Portenta X8 through the Max Carrier micro-USB port without modifying the i.MX8M Linux Device Tree.

## The Problem
When attempting to debug the Portenta X8's STM32H7 core using a Segger J-Link connected to the Max Carrier's micro-USB debug port, the J-Link Commander fails with an `InitTarget() failed` error. 

This occurs because the `SWDIO`, `SWCLK`, and `SWO` lines on the Max Carrier are routed through a set of hardware multiplexers (U16/U19). Official documentation generally implies that the Portenta's i.MX8M Linux coprocessor acts as the master and gatekeeps these debug lines, leading developers down a complex path of trying to gain ADB root access, parse `/sys/kernel/debug/gpio`, and hack the Linux Device Tree with `libgpiod` to release the pins.

## The Breakthrough Discovery
By meticulously cross-referencing the `MaxCarrierBoard` and `X8Board` schematics, we made a critical discovery that circumvents the Linux OS entirely:

1. The data lines for SWO/JTAG map to the High-Density connectors (e.g., `JTAG_TDO/SWO` connects to J2 pin `E26`).
2. The multiplexer toggle control line on the Max Carrier is named the `UART CH1-2` net.
3. Tracing the `UART CH1-2` net across the High-Density connectors reveals that it is wired directly into the **U15A STM32 PIN 20 (PA4)**.

Because the switch is physically wired to the STM32 target processor, **the STM32 itself has full control over the multiplexer.** We can toggle the routing hardware using standard Arduino C++ commands, completely bypassing the Linux Linux OS.

## The Implementation Procedure

To unlock the debug port, you simply need to command the STM32 to flip `PA_4` during its boot sequence.

### Step 1: Modify the Arduino Sketch
Open your target sketch (e.g., `x8_uart_route_probe.ino`) and add the following lines at the absolute beginning of your `setup()` function:

```cpp
void setup() {
  // Take control of Max Carrier multiplexer (U16/U19) via PA_4
  pinMode(PA_4, OUTPUT);
  // Pull LOW (or HIGH) to route SWD/JTAG to the micro-USB port
  digitalWrite(PA_4, LOW); 

  // ... rest of your setup (e.g., SEGGER_RTT_Init)
}
```

### Step 2: Compile and Flash
Compile and upload the modified sketch to the Portenta X8 (e.g., targeting `COM11` via `arduino-cli` or the Arduino IDE). 
*Note: Make sure your 12V barrel jack is providing stable power so the board doesn't brown out during the upload.*

### Step 3: Test J-Link Connection
1. Once the Portenta X8 finishes rebooting and the sketch is running, open Segger J-Link Commander (`JLink.exe`).
2. Type `connect`.
3. Select Device: `Cortex-M7`.
4. Select Interface: `SWD`.

### Step 4: Logic Verification (LOW vs HIGH)
Multiplexers act as A/B switches. If J-Link Commander successfully attaches to the CPU without the `InitTarget()` error, `LOW` is the correct logic state. 

If it still fails, the multiplexer simply expects an inverted signal. Change `LOW` to `HIGH` in your sketch (`digitalWrite(PA_4, HIGH);`), upload it one more time, and J-Link will connect perfectly.