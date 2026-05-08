/*
 * x8_lora_bootloader_helper.ino
 * ---------------------------------------------------------------------------
 * Portenta X8 + Max Carrier — M4 helper that holds the Murata L072 LoRa modem
 * in STM32 ROM bootloader so a host process on Linux can reflash it via
 * /dev/ttymxc3 + a tool like stm32flash.
 *
 * Why this exists:
 *   The X8's M7 `x8h7` bridge firmware does NOT export PG_7 (LORA_BOOT0) to
 *   Linux through the x8h7_gpio chip (verified by exhaustive sweep on
 *   2026-05-07; only LORA_RESET = PC_7 = Linux gpio163 is exposed). Without
 *   BOOT0 control we cannot enter the L072 ROM bootloader from Linux alone.
 *
 *   This M4 sketch fills the gap: it drives PG_7 HIGH and pulses PC_7 LOW/HIGH
 *   exactly once at boot, then sits idle. Linux can then run, e.g.:
 *     stm32flash -b 19200 -m 8e1 -w mlm32l07x01.bin -v -g 0 /dev/ttymxc3
 *   while the M4 keeps BOOT0 asserted.
 *
 * After a successful flash, the user should:
 *   - Push a no-op (empty setup/loop) M4 sketch to release PG_7 and PC_7, OR
 *   - Power-cycle the X8 so the (newly-flashed) L072 firmware boots normally.
 *
 * Build + deploy (on Windows host, sketch on X8 M4):
 *   arduino-cli compile --fqbn arduino:mbed_portenta:portenta_x8 \
 *     --build-path %TEMP%\lifetrac_x8_boot_helper_build \
 *     LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper
 *   adb -s <serial> push %TEMP%\lifetrac_x8_boot_helper_build\x8_lora_bootloader_helper.ino.elf \
 *     /tmp/arduino/m4-user-sketch.elf
 *   adb -s <serial> exec-out "echo fio | sudo -S -p '' sh -c 'touch /tmp/arduino/m4-user-sketch.elf'"
 *   # monitor-m4-elf-file.path triggers reflash via internal openocd.
 *
 * Pin map (per PORTENTA_H7_M7 variant header — same physical H747 die on X8):
 *   LORA_BOOT0 = PG_7   (Murata pin 43)
 *   LORA_RESET = PC_7   (also reachable from Linux as gpio163)
 *
 * Liveness: the on-board LED toggles every 500 ms. If you don't see it blink,
 * the M4 sketch did not boot (M7 firmware may have failed to assert
 * RCC_GCR.BOOT_C2 — see 2026-05-07_Portenta_X8_Internal_OpenOCD_RTT note).
 */

#include <Arduino.h>

#ifndef LORA_BOOT0
#define LORA_BOOT0 (PG_7)
#endif

#ifndef LORA_RESET
#define LORA_RESET (PC_7)
#endif

void setup() {
  // Assert BOOT0 high BEFORE releasing reset — required by STM32 boot sequencer.
  pinMode(LORA_BOOT0, OUTPUT);
  digitalWrite(LORA_BOOT0, HIGH);

  pinMode(LORA_RESET, OUTPUT);
  digitalWrite(LORA_RESET, LOW);
  delay(50);
  digitalWrite(LORA_RESET, HIGH);

  // Allow ROM bootloader to finish UART autobaud detection.
  delay(200);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Keep BOOT0 latched HIGH for as long as this sketch runs.
  // (Re-assert each loop in case anything else touched the pin.)
  digitalWrite(LORA_BOOT0, HIGH);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
