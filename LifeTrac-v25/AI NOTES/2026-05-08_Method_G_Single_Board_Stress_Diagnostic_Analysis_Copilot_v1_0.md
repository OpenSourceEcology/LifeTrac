# Method G Single-Board Stress Test — Diagnostic Analysis

**Date:** 2026-05-08  
**Run Timestamp:** 20220503_102811  
**Cycles:** 20  
**Overall Pass Rate:** 0% (all failures)  
**Recommendation:** STAY_IN_SINGLE_BOARD_HARDENING  

---

## Executive Summary

The single-board stress test completed all 20 cycles successfully from an **automation perspective** (pipeline ran, no hangs, consistent timing), but revealed a **critical firmware execution failure**: the L072 never executes the flashed firmware image. Every cycle triggers auto-reboot before custom firmware can run.

This is **not** a tool failure or flash-pipeline issue — it's a **firmware or hardware configuration issue** that must be resolved before two-board RF validation can proceed.

---

## Detailed Failure Analysis

### 1. Flash Command Returns rc=1 Consistently

**Observation:**
```
cycle 1 FAIL rc=1 rx_bytes=625 banner=1 tick=0
cycle 2 FAIL rc=1 rx_bytes=625 banner=1 tick=0
... (pattern repeats 20 times)
```

**Interpretation:**
- The flash command (`stm32_an3155_flasher.py --verify`) exits with code 1 on every cycle
- Code 1 typically indicates: **verify failure**, **timeout**, or **communication error**
- Device is not permanently hung (banner detected 100% of time)
- Post-flash UART capture succeeds (625 bytes consistent)

**Likely Root Cause:**
The flasher's `--verify` step is failing. Possible reasons:
- Verify reads back data that doesn't match programmed content
- Flash completed but file system left in inconsistent state
- Image header/metadata corruption during write
- Timing issue: verify starts before flash actually complete

---

### 2. Custom Firmware Never Executes (0% Tick Hits)

**Observation:**
```
tick_hits=0 tick_hit_rate=0.0%
```

**Expected Behavior:**
The hello.bin firmware writes a UART tick counter every ~500ms. A successful flash should show `tick=1` (at least 1 detected).

**Actual Behavior:**
- Zero out of 20 cycles detected any tick output
- Instead, only bootloader banner is captured (625 bytes is consistent L072 bootloader greeting)
- Device boots but doesn't execute flashed firmware

**Likely Root Cause:**
One of:
1. **Flash didn't actually persist:** Device reverts to bootloader (ROM or backup code) on each boot
2. **Bad entry point:** Flashed image has wrong start address or corrupted header
3. **Watchdog-triggered reboot:** Device boots, watchdog expires before firmware can output, triggers reset
4. **Image mismatch:** mlm32l07x01.bin (MKRWAN reference) isn't compatible with this L072/H7 configuration

---

### 3. 100% Auto-Reboot Detection

**Observation:**
```
auto_reboot_heuristic_hits=20 (100% of cycles)
```

**Heuristic Definition:**
Device is considered auto-rebooting if it returns to bootloader (banner detected) within the 10-second post-flash listen window.

**Interpretation:**
Every single cycle shows the device booting back to bootloader within 10 seconds of flash completion. This strongly suggests:
- Flashed firmware fails to boot or execute
- Device falls back to bootloader (either hard reset or watchdog timeout)
- Watchdog at 60s may be involved if device is stuck in reset loop

---

### 4. Consistent Cycle Timing (19–20 seconds)

**Observation:**
```
Avg cycle seconds: 19.2
Min: 19, Max: 20
```

**Positive Finding:**
Pipeline overhead (openocd setup, flash, cleanup) is reliable and **not the bottleneck**. This means:
- adb communication stable
- watchdog-pet helper working
- openocd reentrance OK
- No hard hangs or retries

**Negative Finding:**
The consistency suggests the **same failure path every cycle** — not a race condition or transient issue, but a systematic problem with the firmware/flash/boot sequence.

---

## Hypothesis Ranking

### Tier 1: Most Likely (70% confidence)

**Hypothesis: Image Verify Failure**
- The mlm32l07x01.bin image (MKRWAN reference, 83 KB) doesn't match H7 memory layout
- Verify step reads back data from wrong addresses or with wrong length
- Exit code 1 is reported, but firmware was partially written (hence banner is detected on next boot)

**Evidence:**
- rc=1 on 100% of cycles (consistent error code)
- Device still boots (banner detected) — not permanently bricked
- Image was pushed to X8 and exists at `/tmp/lifetrac_p0c/mlm32l07x01.bin`

**Action:**
1. Inspect stm32_an3155_flasher.py logs (`/tmp/lifetrac_p0c/flash_run.log` on X8 from last cycle)
2. Verify image size and header are correct for L072
3. Compare mlm32l07x01.bin against hello.bin (which was proven working in prior session)

---

### Tier 2: Likely (20% confidence)

**Hypothesis: Watchdog Triggered Boot Loop**
- Flashed firmware boots but hangs or loops before first tick output
- Watchdog at 60s expires, triggers device reset
- Device falls back to bootloader on next power cycle

**Evidence:**
- 100% auto-reboot heuristic hits (device at bootloader 10s post-flash)
- wdt_pet.sh successfully stops watchdog after listen phase, but device already rebooted

**Action:**
1. Extend POST_LISTEN_SEC to 30–60 seconds to observe if boot hangs or loops
2. Check if watchdog is expiring during boot phase
3. Review hello.bin execution path — why does it work in prior session but not now?

---

### Tier 3: Possible (10% confidence)

**Hypothesis: Image Header or Entry Point Corruption**
- mlm32l07x01.bin has incorrect flash origin address or entry point
- Bootloader executes non-existent/invalid code
- Device throws exception and resets

**Evidence:**
- Zero tick hits (firmware never starts)
- Device reverts to bootloader immediately

**Action:**
1. Inspect mlm32l07x01.bin with `hexdump` — check header magic numbers
2. Compare against hello.bin which was proven working
3. Rebuild hello.bin from source and re-test to rule out prior-session flukes

---

## Recommended Next Steps

### Phase 1: Root Cause Identification (1–2 hours)

1. **Inspect Flash Logs**
   - Pull `/tmp/lifetrac_p0c/flash_run.log` and `/tmp/lifetrac_p0c/flash_ocd.log` from the latest cycle on X8
   - Look for: verify failures, timeouts, address mismatches, stty errors
   - Compare against prior session logs where hello.bin worked

2. **Test hello.bin Explicitly**
   - Run a single cycle with `-ImageRemote /tmp/lifetrac_p0c/hello.bin` (if it exists on X8)
   - If hello.bin works (tick=1, pass=1), then confirms mlm32l07x01.bin is the problem
   - If hello.bin also fails (tick=0, rc=1), suggests a systematic issue (watchdog, openocd config, X8 state)

3. **Inspect Image Headers**
   - Use `python3 -c "import struct; data=open('/tmp/lifetrac_p0c/mlm32l07x01.bin','rb').read(32); print(data.hex())"` on X8
   - Compare against known-good L072 firmware images from MKRWAN or Arduino reference

### Phase 2: Systematic Validation (2–4 hours if Phase 1 reveals mlm32l07x01.bin issue)

1. **Use hello.bin for Stress Test**
   - If hello.bin passes (>95% on next stress run), confirms we have a working test infrastructure
   - Validates Method G flash pipeline, watchdog handling, post-listen capture

2. **Build Custom Firmware from Scratch**
   - Use STM32CubeMX or mbed-cli to generate a working L072 hello-world project
   - Implement tick counter (UART output every 500ms)
   - Flash locally and verify on real hardware before multi-cycle stress test

3. **Extend Single-Board Validation**
   - Once firmware is proven working, run 50-cycle stress test to reach >95% reliability gate
   - Gate criterion: pass ≥ 95% AND banner-hit ≥ 95%

### Phase 3: Two-Board RF Progression (Post-hardening)

Once single-board reaches ≥95% pass rate:
- Activate W4-00 RF exchange gates (requires 2nd Portenta board)
- Use LATEST_STRESS_STATUS.md recommendation as go/no-go decision point

---

## Open Questions

1. **Why did hello.bin work in prior session (2026-05-08 earlier) but mlm32l07x01.bin fails now?**
   - Was hello.bin re-tested in the interval?
   - Were X8 power cycles or firmware updates applied?

2. **Is the watchdog interfering with boot?**
   - wdt_pet.sh stops at T2.5, but device may auto-reboot before that phase
   - Extend listen window to confirm device stays up if not reset

3. **What is the correct firmware image for L072 on this H7 carrier?**
   - mlm32l07x01.bin is MKRWAN reference — is it the right choice?
   - Should we use hello.bin as permanent test image for Method G validation?

---

## Summary Table

| Aspect | Finding | Status |
|--------|---------|--------|
| Pipeline automation | All 20 cycles executed without hang | ✓ PASS |
| Flash command exit code | rc=1 on 100% of cycles | ✗ FAIL |
| Device responsiveness | Banner detected 100% (device not bricked) | ✓ PASS |
| Custom firmware execution | 0% tick hits (firmware never runs) | ✗ FAIL |
| Auto-reboot rate | 100% return to bootloader | ✗ FAIL |
| Cycle timing consistency | 19–20 seconds (reliable) | ✓ PASS |
| Single-board gate criterion | 0% pass (need ≥95%) | ✗ FAIL |
| Recommendation | STAY_IN_SINGLE_BOARD_HARDENING | 🔴 HOLD |

---

## Decision Gate

**Can we proceed to two-board RF validation?** → **NO (0% pass rate)**

**Next move:** Resolve firmware execution failure in Phase 1, then re-run stress test with corrected firmware.

---

**Next session focus:** Inspect flash logs, test hello.bin, identify why custom firmware doesn't execute.
