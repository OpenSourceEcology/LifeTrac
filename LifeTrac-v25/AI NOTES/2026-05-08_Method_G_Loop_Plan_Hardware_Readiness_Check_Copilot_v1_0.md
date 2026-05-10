# Method G Loop-Plan Hardware Readiness Check

**Date:** 2026-05-08
**Author:** GitHub Copilot (GPT-5.3-Codex)
**Version:** v1.0
**Status:** PROCEED with custom LoRa firmware bench testing (conditional go)

## 1. Decision

Proceed to the next Method G implementation/testing tranche now.

Reason: the critical hardware gates for custom L072 firmware ownership have already been demonstrated on real hardware (bootloader entry, erase/write/verify, and execution of a custom binary over the X8->L072 path).

## 2. Hardware Readiness Snapshot

### 2.1 Proven (green)

1. L072 bootloader-entry control from X8 stack is proven:
- BOOT0 asserted through PA_11 and NRST pulsed through PF_4 using the helper flow.
- 0x79 STM32 ROM ACK captured.

2. End-to-end flash path is proven on real Murata CMWX1ZZABZ-078:
- AN3155 sync/Get/GetID/ReadMemory/erase/write/verify all working over /dev/ttymxc3 at 19200 8E1.
- MKRWAN-sized image (83032 B) flashed and verified.

3. Arbitrary custom firmware execution is proven:
- Custom 677-byte hello-world binary flashed and observed transmitting runtime banner/tick output.

4. Repeatable no-manual-power-cycle workflow is proven:
- prep_bridge + wdt_pet + flash pipeline works.
- Board auto-reboots after flash sessions; adb reconnect is sufficient (no physical unplug required).

### 2.2 Open risks (amber)

1. X8 bridge still wedges after openocd halt sessions and recovers by auto-reboot.
- This is operationally manageable for bench work, but still a reliability risk for high-throughput iteration.

2. HIL Wave-4 gate status remains open for full system validation:
- W4-pre and W4-00 are not signed off as fully complete in the runbook evidence model.
- This does not block L072 firmware implementation work, but does block later hydraulic safety claims.

3. Long-term maintainability risk:
- Current flow depends on helper scripts and timing discipline; upstream x8h7 opcode support for LoRa BOOT0/NRST control is still the clean architectural fix.

## 3. Loop-Plan Hardware Checklist (what to physically confirm before each test day)

1. Two Portenta + Max Carrier assemblies available and identifiable.
2. Known-good USB-C data cables and stable host port assignment.
3. LoRa antenna/load discipline for any RF stage:
- either SMA antenna attached and spaced >= 30 cm
- or 50 ohm load/attenuated conducted setup.
4. X8 Linux access stable (adb + sudo for fio user).
5. Watchdog helper available and executable before long flashes.
6. Bench logging enabled for evidence capture (pipeline logs + serial captures + gate notes).

## 4. Recommended Next Test Sequence (to proceed now)

1. Method G firmware trunk entry check
- Run full_flash_pipeline with a known image and confirm clean verify.

2. Host-wire protocol burn-in
- Run H7 host loopback harness and long-duration malformed-frame injection.
- Record reject counters and no-crash evidence.

3. Two-board RF bring-up for Method G wire protocol
- Move from UART-only proof to controlled RF frame exchange between two Murata nodes.
- Capture TX_DONE / RX_FRAME ratios and latency histograms.

4. Gate re-baseline against runbook
- Re-run W4-pre evidence package where still missing.
- Start W4-00 sub-gates using Method G path as primary.

## 5. Proceed/Block Rule

Proceed if all of the following are true at session start:
- bootloader entry ACK reproducible,
- flash verify succeeds,
- post-flash runtime output observed,
- watchdog pet is active for long writes.

Block and debug first if any one fails.

## 6. Source References

- DESIGN-CONTROLLER/HIL_RUNBOOK.md
- MASTER_TEST_PROGRAM.md
- AI NOTES/2026-05-08_Method_G_Phase1_End_to_End_Flash_Success_Copilot_v1_0.md
- AI NOTES/2026-05-08_Method_G_HelloWorld_Validation_Copilot_v1_0.md
- AI NOTES/2026-05-08_Method_G_Avoiding_Power_Cycles_Copilot_v2_0.md
- DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/full_flash_pipeline.sh
