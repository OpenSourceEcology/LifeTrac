# Portenta X8 Max Carrier JTAG/SWD Bypass via PC0 (Updated)

**Date**: May 7, 2026
**Objective**: Enable Segger J-Link / RTT debugging on the Portenta X8 through the Max Carrier micro-USB port by toggling the correct debug isolation pin (`PC_0`).

## Background & Previous Execution Findings
Initial analysis of the `MaxCarrierBoard` schematics led us to believe `PA_4` controlled the JTAG lines. However, execution findings showed that toggling `PA_4` (both LOW and HIGH) failed to unblock the `InitTarget()` failure in J-Link Commander. 

Upon closer inspection of the schematics:
1. `PA_4` is wired to the `SEL` pin of a multiplexer that only handles **UART SNIFF** routing.
2. The actual physical debug lines (`PA13`/`JTMS-SWDIO` and `PA14`/`SWCLK`) bypass the UART multiplexer scheme.
3. The true gatekeeper for the JTAG lines is a specific isolation net labeled `JTAG SWD`, which is wired directly to the STM32's **`PC_0`** pin.

By commanding `PC_0`, the STM32 can physically enable the connection to the Max Carrier's debugging USB interface.

## The Implementation Procedure

To unlock the debug port, command the STM32 to flip `PC_0` during its boot sequence.

### Step 1: Modify the Arduino Sketch
Open your target sketch (e.g., `x8_uart_route_probe.ino`) and insert the `PC_0` toggle at the start of your `setup()` function:

```cpp
void setup() {
  // PA_4 was just UART Sniffing. PC_0 is the actual JTAG_SWD isolation control!
  pinMode(PC_0, OUTPUT);
  
  // Test LOW first. Change to HIGH if J-Link attach still fails.
  digitalWrite(PC_0, LOW);

  // ... rest of your setup (e.g., SEGGER_RTT_Init)
}
```

### Step 2: Compile and Flash
Compile and upload the modified sketch to the Portenta X8 via your standard script, targeting `COM11` (the Portenta X8 target).

*Note: Maintain stable power (12V barrel) during upload to prevent ADB drops.*

### Step 3: Test J-Link Connection
1. Once the Portenta X8 finishes rebooting and the sketch is running, open Segger J-Link Commander (`JLink.exe`).
2. Type `connect`.
3. Select Device: `Cortex-M7`.
4. Select Interface: `SWD`.

### Step 4: Logic Verification (LOW vs HIGH)
If J-Link Commander successfully attaches to the CPU without the `InitTarget() failed` error, `LOW` is the correct logic state and you now have full SWD/RTT access.

If `InitTarget()` still fails, the isolation circuitry expects an inverted signal. Change `LOW` to `HIGH` in your sketch (`digitalWrite(PC_0, HIGH);`), re-flash, and test again.