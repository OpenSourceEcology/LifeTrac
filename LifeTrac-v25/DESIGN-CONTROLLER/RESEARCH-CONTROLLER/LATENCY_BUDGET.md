# Latency Budget — Operator Stick → Hydraulic Actuation

**Status:** Research / optimization roadmap. Not a build commitment.
**Created:** 2026-04-26
**Related canonical sections:** [`MASTER_PLAN.md` §8.12](../MASTER_PLAN.md) (wire-contract rates), [§8.15](../MASTER_PLAN.md) (field-test gate), [§8.17](../MASTER_PLAN.md) (LoRa control PHY + adaptive SF + no retries), [§8.18](../MASTER_PLAN.md) (D1608S SSR routing), [`LORA_PROTOCOL.md` § Adaptive control-link SF](../LORA_PROTOCOL.md#adaptive-control-link-sf).

This document analyzes the end-to-end latency from an operator deflecting a joystick on the handheld or web UI to the hydraulic cylinder actually moving. It identifies the dominant terms and ranks candidate optimizations by latency-per-dollar.

The §8.17 / §8.18 changes (SF7 default + no retries on `ControlFrame` + directional coils on D1608S SSR) are already pinned in the master plan based on this analysis. The remaining items here are research / future-version candidates.

---

## 1. End-to-end latency budget

Per-stage latency for a single operator stick deflection at the **default** rung (SF7 / BW 125 kHz / CR 4-5, D1608S SSR coils, mechanical valve + cylinder per current hydraulic spec).

| # | Stage | Typical | Worst case | Notes |
|---:|---|---:|---:|---|
| 1 | Joystick ADC + frame encode (handheld or web UI) | <1 ms | 2 ms | Static-priority encode loop, no allocation |
| 2 | LoRa TX air time (44 B on-air, SF7/BW125) | ~30 ms | ~30 ms | Pure PHY; not retry-able under §8.17 no-retry rule |
| 3 | LoRa RX demodulate + KISS deframe + CRC | 1–3 ms | 5 ms | RadioLib on H747 M7 |
| 4 | AES-GCM auth-tag verify | <1 ms | 2 ms | Hardware-accelerated on STM32H747 |
| 5 | M7 → M4 IPC (shared memory + flag) | <1 ms | 2 ms | Lock-free; non-blocking by §8.10 hard rule |
| 6 | M4 100 Hz control-loop quantization | 0–10 ms | 10 ms | Half-cycle average 5 ms |
| 7 | Modbus-RTU 50 Hz schedule slot | 0–20 ms | 20 ms | Half-cycle average 10 ms |
| 8 | RS-485 frame TX @ 115200 8N1 (~12 B) | 1 ms | 2 ms | Standard PROFIBUS-class wire |
| 9 | Opta sketch processing + register write | 1–3 ms | 5 ms | Modbus slave loop tick |
| 10 | **D1608S SSR pickup** (zero-cross, <1 ms) | <1 ms | 1 ms | **Was 8–20 ms with mechanical relays — fixed by §8.18** |
| 11 | Solenoid coil current ramp (24 V DC, ~50 mH) | 15–40 ms | 80 ms | L/R time constant; first 70% of pickup happens by ~20 ms |
| 12 | Spool shift + flow ramp (mechanical directional valve) | 30–80 ms | 150 ms | Spring-centered spool; depends on flow rate and dither |
| 13 | Cylinder pressure rise + load break-away | 20–100 ms | 300 ms | Function of mass, friction, accumulator state |
| | **End-to-end (sum)** | **~100–190 ms** | **~610 ms** | Typical median ~120–150 ms with §8.17/§8.18 in place |

> **Bench-measured caveat for rows #1–#5 (W1-11 L-3, 2026-05-12).** The
> values above are the **production M7-direct path** budget. The Method G
> bench harness uses an X8 → LPUART → L072 HostLink instead of the
> production M7-direct path; on that bench tier `elapsed_ms` (host
> `TX_FRAME_REQ` → `TX_DONE_URC`) measures **p50 ≈ 52 ms / p99 ≈ 58 ms /
> p999 ≤ 60.6 ms** across 2000 cycles at SF7/BW125 (`bench-evidence/W1-10b_rx_pair_2026-05-11_231533/summary.json`).
> Subtracting the firmware-reported ToA = 30.85 ms gives a one-way
> bench HostLink overhead of **~21 ms p50 / ~30 ms p999** — much higher
> than the ~5 ms originally pinned for rows #1+#3+#4 because every TX
> cycle pays an X8 ADB pipe + LPUART encode/decode + L072 dispatch
> round-trip that does not exist in the production M7 path. The
> production-path budget for rows #1+#3+#4+#5 (~5 ms total) is
> **unchanged**; the bench number is captured here only so latency
> analyzer output (`rtt_report.json` `verdicts.w4_00b_within_5pct`)
> can be interpreted correctly when the bench surrogate prints
> `OUTSIDE_BUDGET`. A true production RTT measurement requires a
> firmware echo handler (planned as L-1 / `--probe rx_echo`) running
> on the M7-direct path.
>
> **Update 2026-05-12 (W1-11 L-1 ping-pong).** The host-driven echo
> form of L-1 is now wired (`--probe rx_echo` on RX,
> `--probe ping_pong` on TX, accessible via
> `run_w1_10b_rx_pair_end_to_end.ps1 -Probe ping_pong`). First-light
> 100-cycle run (`bench-evidence/W1-11_pingpong_2026-05-12_115122/`)
> measured a true round-trip `pingpong_rtt_ms` p50=**107.0 ms**
> (p90=115, p99=118, p999=121.8, max=121.8) — close to the analytical
> prediction `2×ToA + 2×host_overhead = 2×33.4 + 2×21 = 108.8 ms`,
> independently confirming both the `+2.8%`/`+11.4%` ToA measurement
> and the new `PREDICTED_HOST_OVERHEAD_MS = 21.0` constant. This
> bench RTT still includes **two** host overheads (one per board) and
> therefore continues to print `OUTSIDE_BUDGET` against W4-00(b)'s
> `2*ToA + 30 ms = 96.8 ms ± 4.8` window. The
> production-equivalent value (RX-side echo done on-MCU, no host
> round-trip) is predicted to drop the RTT by ~21 ms to **~86 ms
> p50**, which lands inside the W4-00(b) window. Confirming that
> requires firmware-side echo on the W2-01 H747 transport bring-up.

### Observations

- **The radio is no longer the dominant single term** at SF7 (~30 ms) once §8.17 caps retries at 1.
- **Mechanical-side terms (#11, #12, #13) dominate** the typical case. The control electronics and radio together contribute ~50 ms; the hydraulic chain contributes 70–220 ms.
- **The §8.18 SSR fix removed a 10–20 ms mechanical-relay term (#10)** for free — we just had to spec the right Opta expansion variant (D1608S, not D1608E).
- **Worst-case (~600 ms) is dominated by hydraulic transients (#13)**, not radio. Even halving the radio latency would not change worst-case meaningfully without hydraulic changes.
- The §8.15 field-test gate (E-stop p99 <500 ms) is comfortable at SF7. At the SF9 fallback rung, radio adds ~70 ms more (item #2: 30→100 ms), pushing typical end-to-end to ~200 ms — still within the gate, but with less margin.

---

## 2. Ranked optimization candidates

Ordered by latency-per-dollar, dominant savings first. **Items marked ✅ DONE are already pinned in MASTER_PLAN §8.**

### ✅ DONE — Pin SF7 / BW 125 kHz / CR 4/5 for control link

- **Latency saved:** ~80–100 ms vs an SF9 default. SF7 was already the documented default but adaptive-down to SF8/SF9 on degraded link is now explicit (§8.17 + `LORA_PROTOCOL.md § Adaptive control-link SF`).
- **Cost:** zero — firmware-only.
- **Risk:** SF7 has shorter range than SF9 at the same TX power. Mitigated by the adaptive ladder (SF7 → SF8 → SF9) and the 8 dBi mast antenna at +20 dBm TX.

### ✅ DONE — Drop retries on P0 `ControlFrame` and `HeartbeatFrame`

- **Latency saved:** removes a 5–25 ms backoff and a re-TX from the failure case. Prevents stale-stick re-application (a correctness win, not just latency).
- **Cost:** zero — firmware-only. MAC layer now takes a `max_attempts` parameter; control = 1, telemetry = 3 (`LORA_PROTOCOL.md § MAC layer`).
- **Risk:** marginal-link single-frame loss is no longer recovered by MAC. The 20 Hz cadence is the recovery mechanism; failsafe at 500 ms is the safety net.

### ✅ DONE — Route directional valve coils to D1608S SSR channels

- **Latency saved:** ~10–20 ms (item #10 above). SSR pickup is sub-millisecond; mechanical relays add 8–20 ms of pickup time plus 1–3 ms of contact bounce.
- **Cost:** zero — we are already buying the D1608S (AFX00006); the EMR variant D1608E (AFX00005) was a doc error in the BOM.
- **Risk:** SSRs need a flyback diode across each inductive load (already standard practice). 24 VDC / 2 A per SSR comfortably covers typical hydraulic solenoid coils.
- **Action:** update the wiring harness diagram in `TRACTOR_NODE.md` so all 8 directional coils land on D1608S channels; the four onboard Opta EMRs become engine-kill / horn / brake-release / spare.

### Candidate A — PWM coil drive with current pre-bias (firmware + minor hardware)

- **Latency saved:** ~10–25 ms on item #11 (coil current ramp). By holding each coil at ~30% rated current ("pre-bias") and PWM-boosting to 100% on activation, the L/R time constant for the *transition* shrinks by roughly the pre-bias fraction.
- **Cost:** medium. Requires either MOSFET driver boards in series with the SSRs (so PWM doesn't chatter the SSR), or moving to dedicated coil-driver ICs (e.g. ZXBM5210, BTS50055-1TMA). Plus firmware for PWM duty management on the M4.
- **Risk:** thermal — pre-bias dissipates ~1 W per coil continuously. Acceptable at 8 coils on a heatsinked manifold; requires verification.
- **Recommendation:** defer to v26. Worth a bench prototype on one coil to confirm the savings before committing.

### Candidate B — Hydraulic accumulator on the implement supply

- **Latency saved:** ~30–80 ms on item #13 (cylinder pressure rise) for short, high-flow demands. An accumulator pre-charged to system pressure provides instantaneous flow when a valve opens, instead of waiting for the pump to ramp.
- **Cost:** $200–400 in hydraulic hardware (1–2 L bladder accumulator + tee + safety valves + piping). Hydraulic-team work, not controller-team work.
- **Risk:** adds stored energy to the system → safety review required (charge/discharge procedures, isolation valve, gauge). Adds weight.
- **Recommendation:** defer to v26 hydraulic redesign. Coordinate with `DESIGN-HYDRAULIC/` if pursued.

### Candidate C — Proportional directional valves with onboard amplifier

- **Latency saved:** ~50–120 ms on item #12 (spool shift). Proportional valves with built-in amplifiers (e.g. Eaton KBS, Sun Hydraulics XMD, Bürkert 8605 + cartridge proportional) shift the spool with a closed-loop position controller in milliseconds and eliminate spring-centered ramp time.
- **Cost:** **$1,800–4,800** depending on flow rating and which axes get upgraded. Major hydraulic redesign — replaces the current bang-bang directional valves entirely.
- **Risk:** big jump in cost and complexity. Operator ergonomics also change (continuous proportional response instead of on/off), which is generally an improvement but requires re-tuning of stick deadbands and ramps.
- **Recommendation:** evaluate for v26+ as a holistic hydraulic upgrade; not justified as a latency-only fix.

### Candidate D — Bench-measure E-stop p99 *before* committing to further optimizations

- **Latency saved:** unknown until measured.
- **Cost:** ~1 day of bench time. Oscilloscope on a dummy coil, repeated E-stop presses on the base, log min/median/p99 from H747.
- **Recommendation:** **do this during \[`MASTER_PLAN.md` §5 step 7\](../MASTER_PLAN.md) hydraulic bench bring-up.** Without measured numbers, all ranking above is informed estimation. The §8.15 gate (<500 ms p99) needs an actual measurement to clear.

---

## 3. Recommended actions

In order:

1. **Apply the three free firmware/spec pins** (✅ already done in §8.17, §8.18, MAC layer in `LORA_PROTOCOL.md`).
2. **Update the tractor wiring harness diagram** in `TRACTOR_NODE.md` to reflect §8.18 SSR routing for directional coils.
3. **Bench-measure E-stop p99** during §5 step 7 hydraulic bench bring-up. Use an oscilloscope on a dummy coil; press base-station E-stop button repeatedly; log min/median/p99 from H747 timestamps. This is the empirical foundation for everything below.
4. **If §8.15 field test passes:** stop here. The latency budget is comfortable; further optimization is not worth the cost or risk for v25.
5. **If §8.15 field test fails for radio reasons:** treat as a §7 reversal — open a new MASTER_PLAN decision, revisit [`WIRELESS_OPTIONS.md`](WIRELESS_OPTIONS.md), and pick a path then. No alternative is currently pre-selected.
6. **If §8.15 passes but operators report sluggish hydraulic response:** revisit Candidates A / B / C in §2 for v26 hydraulic redesign.

---

## 4. References

- [`MASTER_PLAN.md`](../MASTER_PLAN.md) — canonical scope and pinned decisions.
- [`LORA_PROTOCOL.md`](../LORA_PROTOCOL.md) — wire format, MAC layer, adaptive SF logic.
- [`TRACTOR_NODE.md`](../TRACTOR_NODE.md) — hardware wiring (needs §8.18 update).
- [`HARDWARE_BOM.md`](../HARDWARE_BOM.md) — D1608S SSR vs D1608E EMR distinction (now corrected).
- [`WIRELESS_OPTIONS.md`](WIRELESS_OPTIONS.md) — historical wireless comparison.
- [`AI NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md`](../../AI%20NOTES/2026-04-26_LoRa_QoS_Bandwidth_Management.md) — QoS / priority queueing on the LoRa link.
- [Semtech SX1276 datasheet](https://www.semtech.com/products/wireless-rf/lora-connect/sx1276) — PHY parameters.
- [Arduino Pro Opta Ext D1608S](https://store-usa.arduino.cc/products/opta-ext-d1608s) — SSR variant (AFX00006).
- [Arduino Pro Opta Ext D1608E](https://store-usa.arduino.cc/products/opta-ext-d1608e) — EMR variant (AFX00005), **not** what we are buying.
