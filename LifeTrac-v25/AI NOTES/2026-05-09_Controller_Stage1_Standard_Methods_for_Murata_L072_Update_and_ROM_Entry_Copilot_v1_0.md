# Standard Methods For Murata L072 Update And ROM Entry

**Date:** 2026-05-09  
**Author:** Copilot  
**Version:** v1.0  
**Scope:** Standard, documented methods for reliably entering and using the STM32L072 system bootloader inside the Murata `CMWX1ZZABZ-078` on the Portenta Max Carrier / X8 path.

---

## 1. Bottom line

The standard way teams handle this class of problem is **not** to treat ROM entry as a probabilistic OpenOCD timing game.

The normal hierarchy is:

1. **Vendor-supported in-system updater** if the board vendor already ships one.
2. **STM32 ROM bootloader over a documented transport** using a standard host tool (`STM32CubeProgrammer`, `stm32flash`, or an AN3155-equivalent flasher).
3. **SWD/JTAG programmer** for first bring-up, option-byte recovery, and hard brick escape.

For the Max Carrier Murata path, the closest standard methods are:

- **Arduino's `MKRWANFWUpdate_standalone` workflow** for supported modem firmware refresh.
- **STM32 UART system bootloader workflow** with BOOT0 + NRST control and an AN3155 host tool.
- **ST-LINK / J-Link SWD** as the non-negotiable recovery fallback.

Our current X8 method is partly standard and partly custom:

- **Standard:** BOOT0-high + reset, talk AN3155 over UART, erase/write/verify, reset into user flash.
- **Custom:** using X8-internal OpenOCD only to poke H7-controlled lines because BOOT0/NRST are not exposed to Linux in a normal API.
- **Non-standard and not a good long-term default:** repeated ROM-entry burst matrices to discover a "good" timing window as if delay were the main lever.

---

## 2. What the official ecosystem says is normal

### 2.1 Arduino / Murata-supported method

Arduino's published support guidance for LoRa modem maintenance is straightforward:

- Install the `MKRWAN` library.
- Open `MKRWANFWUpdate_standalone` from the examples.
- Upload the sketch.
- Let the host MCU update the modem firmware.

That is the vendor-standard answer for boards in the Arduino ecosystem. It means Arduino expects the host MCU to:

- drive the modem into update mode,
- transfer firmware over the modem's update path,
- and restore a known-good modem image **without external SWD**.

For LifeTrac, this matters because it confirms the Murata update path is intended to be **host-driven and in-system**, not dependent on manual probe wiring for normal maintenance.

### 2.2 ST's standard STM32 method

ST's mainstream programming tool, `STM32CubeProgrammer`, explicitly supports both:

- **debug interfaces:** SWD / JTAG,
- **bootloader interfaces:** UART, USB DFU, I2C, SPI, CAN.

That is the core standard model for STM32 devices:

- use **system bootloader transports** for normal scripted programming when those pins are available,
- use **SWD/JTAG** when you need stronger recovery, option-byte work, or a deterministic bench connection independent of the running application.

In other words, the industry-standard data plane is usually **CubeProgrammer or stm32flash-class tooling over the ROM bootloader**, not OpenOCD talking indirectly to a second MCU that happens to own BOOT0.

### 2.3 OpenOCD's standard role

OpenOCD's own reset documentation frames reset handling as board-specific and timing-sensitive, with explicit support for:

- SRST pulse width,
- SRST delay after release,
- custom reset sequences,
- board-specific reset configuration.

That makes OpenOCD standard for:

- **debug access**,
- **board-specific reset choreography**,
- **special bring-up sequences**.

It does **not** make OpenOCD the standard production flasher for the Murata L072 UART bootloader path. On this project it is best understood as a **control-plane helper** for asserting BOOT0 / NRST through the H7 when Linux cannot do so directly.

---

## 3. The standard STM32 UART bootloader sequence

For STM32 parts using the ROM bootloader over USART, the standard flow is:

1. Hold **BOOT0 high**.
2. Pulse **NRST**.
3. Open UART in the bootloader framing expected by the target.
4. Send **`0x7F` sync once** to trigger autobaud / initial handshake.
5. Issue discovery commands (`Get`, `Get ID`) to confirm the session.
6. Erase, write, and **verify**.
7. Drop BOOT0 low and reset into user flash, or use `Go` if the board/tool flow supports it cleanly.

For this Murata path, our repo already converged on the correct standard shape:

- BOOT0 and NRST are controlled externally.
- AN3155 is spoken over `/dev/ttymxc3`.
- verify is performed by readback.
- final boot is done by hardware reset with BOOT0 released.

That last point is conservative and standard enough for embedded production work: many teams prefer a real reset into user mode rather than relying only on `Go`, because reset gives a cleaner board-level state.

---

## 4. What standard practice would call out as wrong or weak in our current experiments

### 4.1 Multiple sync attempts inside one bootloader session

This is **not** a standard way to estimate ROM-entry reliability.

Once the STM32 ROM bootloader has accepted the initial sync and moved into command mode, later `0x7F` bytes are no longer independent entry attempts. They are just bytes in an already-open protocol session. Our own harness findings already exposed this.

Standard implication:

- **One reset cycle = one independent ROM-entry attempt.**
- Reliability should be measured across independent reset cycles, not repeated sync bytes inside a single session.

### 4.2 Using OpenOCD lifetime as a primary quality lever

This is also not standard.

Standard practice would tune:

- reset pulse width,
- reset-release delay,
- UART framing,
- BOOT0 hold timing,
- session timeout bounds,
- and post-reset verification.

But it would not usually treat "keep OpenOCD alive for 20 s vs 75 s" as a first-order programming parameter. That is a symptom of a board-control workaround leaking into the measurement loop.

### 4.3 Heavy ADB polling in the critical control loop

This is operationally fragile and not how a normal embedded flashing tool is structured.

Standard structure is:

- one host-side command launches the full bootloader operation on the target,
- the flash tool runs locally near the UART,
- logs are written to files,
- the outer transport only checks coarse status.

For us, ADB should be treated as a **job launcher and artifact transport**, not as the timing-critical programming loop itself.

---

## 5. What is standard for field updates vs. bench bring-up

### 5.1 Field / service standard

The standard field-updatable design pattern is:

- keep a **known-good golden image**,
- keep a **deterministic bootloader entry path**,
- always perform **write + verify**,
- log version and result,
- and verify application liveness after reboot.

On Arduino-class products, the closest standard user-facing implementation is exactly the `MKRWANFWUpdate_standalone` style: host MCU performs a controlled modem refresh from a packaged known-good payload.

### 5.2 Engineering bench standard

For engineering and CI-like automation, the standard pattern is:

- script **CubeProgrammer CLI** or **stm32flash-class tooling**,
- version-pin the flashing tool,
- record erase / write / verify logs,
- keep SWD available as an escape hatch.

That maps well to our current custom Python AN3155 flasher. The main thing missing from a standard bench setup is not protocol support; it is **direct, documented control of BOOT0 / NRST without the OpenOCD workaround**.

### 5.3 Hard recovery standard

When the board-level update path misbehaves, standard practice is to fall back to:

- **ST-LINK or J-Link over SWD**,
- inspect option bytes / RDP / WRP state,
- and recover from a known-good image outside the normal application-controlled path.

That should remain part of the LifeTrac design doctrine even if normal operation never needs it.

---

## 6. Where LifeTrac is aligned with standard practice

LifeTrac is already aligned on the important parts:

- **Using the ROM bootloader instead of inventing a raw flash writer.**
- **Using readback verify.**
- **Keeping a golden known-good image.**
- **Separating normal update from SWD recovery.**
- **Recognizing that one reset is the unit of independence.**
- **Treating the vendor firmware update path as a qualification gate before custom images.**

These are all standard embedded-software decisions.

---

## 7. Where LifeTrac is off the standard path

LifeTrac is off the standard path in one narrow but important way:

- the X8 Linux side does **not** have a normal board API for the Murata BOOT0 line,
- so the project currently uses **OpenOCD register pokes through the H7** to synthesize what is normally a direct hardware control path.

That is a valid engineering workaround, but it should be understood as a **carrier-specific exception**, not the design we should aspire to keep forever.

The standardized end-state would be one of these:

1. **Documented board service utility** that asserts BOOT0 + NRST and runs the UART updater.
2. **Versioned host-side updater** bundled into the X8 image, with stable logs and a one-shot command.
3. **Dedicated recovery/debug connector or board-level method** for direct SWD use when needed.

---

## 8. Recommended standard-method interpretation for the next move

If the question is "what should we do that looks like the normal professional way to handle this?", the answer is:

### Recommendation A — Treat the current Python AN3155 path as the bench-standard updater

That means:

- stop treating it as a temporary hack,
- version-pin it,
- stabilize logging and exit codes,
- and make it the canonical engineering flasher for this board.

### Recommendation B — Reduce OpenOCD to pin-control only

OpenOCD should only do:

- assert BOOT0,
- pulse NRST,
- release BOOT0.

It should not be the data-plane tool and it should not be the primary variable under study unless a specific reset-timing defect is being debugged.

### Recommendation C — Measure reliability by independent resets only

That means:

- `ATTEMPTS_PER_BURST=1` stays mandatory,
- one bootloader session equals one observation,
- and any success-rate estimate should be built from independent reset cycles.

### Recommendation D — Move from ADB-polled control to one-shot local execution

The standard operational model is:

- ADB launches a single remote script,
- the script performs the entire bootloader transaction locally on the X8,
- local files contain status, summary, and logs,
- ADB only pulls artifacts.

### Recommendation E — Keep SWD as the explicit escape hatch

Do not let the project drift into thinking the UART path replaces debug recovery. The standard method stack keeps both.

---

## 9. Concrete standard-method checklist for LifeTrac

The practical checklist that best matches standard practice is:

1. **Canonical updater:** one version-pinned AN3155 flasher on X8.
2. **Canonical control path:** one stable BOOT0/NRST orchestration wrapper.
3. **Canonical golden image:** one known-good Murata restore payload.
4. **Canonical verify:** always read back or byte-verify after write.
5. **Canonical post-flash health check:** app banner / heartbeat / protocol response.
6. **Canonical failure logs:** sync, Get ID, erase result, write result, verify result, boot result.
7. **Canonical recovery:** SWD path documented separately and kept available.

If those seven items exist, the project is following a standard embedded update discipline even though the board-specific BOOT0 control path is unusual.

---

## 10. Final conclusion

The standard method for this problem is **host-controlled entry into the STM32 ROM bootloader, programming with a standard AN3155-class tool, verify, then reset into user flash**, with **SWD as fallback**.

For the Murata on Max Carrier:

- Arduino's update sketch is the vendor-standard proof that in-system host-driven update is intended.
- ST's CubeProgrammer model confirms UART bootloader flashing is the mainstream STM32 path.
- OpenOCD belongs here only as a board-specific control helper because the X8/Max Carrier hides BOOT0 behind the H7.

So the right strategic interpretation is:

- **our AN3155 flasher path is directionally standard,**
- **our OpenOCD pin-control workaround is the non-standard board exception,**
- and **the burst-matrix delay hunt should now be treated as secondary to hardening a deterministic, one-shot updater flow.**

---

## Sources

- Arduino Support: `How to update the LoRa modem firmware`  
  https://support.arduino.cc/hc/en-us/articles/4405107258130-How-to-update-the-LoRa-modem-firmware
- ST: `STM32CubeProgrammer` product overview  
  https://www.st.com/en/development-tools/stm32cubeprog.html
- OpenOCD Manual: `Reset Configuration`  
  https://openocd.org/doc/html/Reset-Configuration.html
- Repo context: `2026-05-08_Method_G_Phase1_End-to-End_Flash_Success_Copilot_v1_0.md`
- Repo context: `2026-05-07_Portenta_X8_Internal_OpenOCD_RTT_Architectural_Block_Copilot_v1_0.md`

**Note on ST PDFs:** the canonical STM32 ROM-bootloader references for this topic are AN3155 and AN2606. The fetch tool did not extract usable text from those PDFs during this pass, so the analysis above relies on the accessible ST tool overview, Arduino's published workflow, OpenOCD's reset documentation, and the repo's already-validated AN3155 work.