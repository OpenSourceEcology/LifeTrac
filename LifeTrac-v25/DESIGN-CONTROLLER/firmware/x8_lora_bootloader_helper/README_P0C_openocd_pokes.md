# Portenta X8 — OpenOCD Register-Poke (Option P0-C) toolkit

This directory contains TCL scripts for the **OpenOCD register-poke** strategy
documented in
`LifeTrac-v25/AI NOTES/2026-05-07_Portenta_X8_Murata_OpenOCD_Register_Poke_Plan_Copilot_v1.0.md`
plus the corrections in the assistant's review (RMW writes, IDR readback,
halted-mode pin sweep).

The scripts must run **on the X8 Linux host**, not on the dev PC. They use the
on-board OpenOCD + `imx_gpio` adapter that already programs the M4 via
`monitor-m4-elf-file.service`.

## Files

| File | Purpose |
|---|---|
| `01_halted_dump.cfg` | Halt M7, dump every GPIO bank's MODER + IDR to stdout. Zero side effects. |
| `02_assert_pg7_pc7.cfg` | Halt M7, RMW-enable GPIOC/G clocks, set PG_7 high (BOOT0), pulse PC_7 (NRST). Reads IDR back to confirm. Then sleeps so a second SSH session can run `stm32flash`. |
| `03_boot0_hunt.cfg` | Halt M7. For every output-capable pin in GPIOA..K (skipping PC_7 which is NRST), set HIGH, prompt the operator to send 0x7F via ttymxc3 in a second shell, log result. Slow but definitive identifier of the real BOOT0 net. |
| `99_release_and_reset.cfg` | Halt M7, release all writes, `reset run`. Use this after a successful flash (or as cleanup) to bring the H7 back online. The cleanest recovery is still a power cycle. |
| `verify_l072_rom.sh` | (Linux-side) Probe `/dev/ttymxc3` — first sends AT (should now be silent if BOOT0 took effect), then sends 0x7F at 8E1 (should ACK with 0x79). |

## Usage on the X8

Push this whole directory to the X8 once:

```powershell
adb -s 2E2C1209DABC240B push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper /tmp/lifetrac_p0c
```

Then on the X8 (SSH or `adb shell`):

```bash
sudo openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
             -f /tmp/lifetrac_p0c/01_halted_dump.cfg
```

`/usr/arduino/extra/openocd_script-imx_gpio.cfg` ends with `init; targets; reset
halt`, so by the time our overlay runs the M7 is already halted at the System
Bootloader (PC=0x1ff09abc per the v1.0 RTT note). Our scripts assume that and
do not call `init` themselves.

## Important caveats (carried over from the assistant's review)

1. **The H7 is halted in System Bootloader, not mid-x8h7-firmware.** This is
   actually better — bootloader doesn't touch GPIOC/G. But it also means that
   if the L072 still answers AT after we drive PG_7 high, it's because PG_7 is
   *not* the L072 BOOT0 net on the X8 + Max Carrier SKU pair, not because we
   lost a race with x8h7.
2. **Always RMW.** Bare `mww` of MODER would clobber Ethernet RMII pins on
   GPIOG (PG_11/13/14) and SDMMC1 pins on GPIOC (PC_8..12). The scripts use
   read-modify-write throughout.
3. **Always read IDR after BSRR.** This is the single best diagnostic — it
   tells us whether the bit went high electrically (vs. MODER lock, external
   pull, or wrong pin).
4. **Keep openocd alive for the entire flash.** `02_assert_pg7_pc7.cfg` ends
   with a `gets stdin` that holds the session open until the operator hits
   Enter. Don't Ctrl-C until stm32flash has finished and verified.
5. **x8h7 bridge will stall** once openocd halts the H7. `gpio167`, `gpio163`,
   etc. will return `Connection timed out`. This is expected. Plan a power
   cycle as the final cleanup step.
