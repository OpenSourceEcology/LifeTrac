# 6. Bring-Up & Testing

> **Heads up!** Every test in this section happens **on the bench, with the valve coil rail disconnected at the PSR safety relay** unless explicitly noted otherwise. The whole point is to catch firmware/wiring problems before they can move hydraulics.

You've got hardware assembled, firmware flashed, software running. This section is the verification gate — by the end, you'll know the three nodes can talk, the failsafes fire when they should, and the tractor is safe to wire to live hydraulics.

## Test Layout

Set up all three nodes (or two if you skipped the handheld) on a single bench:

```
   [Handheld]         [Tractor enclosure]            [Base Station]
        │                      │                            │
        └────── 1–3 m bench distance, all antennas connected ─────┘

   Optional: 8× LED stand-ins (with 1 kΩ resistors) wired to D1608S
             SSR1–SSR8 outputs in place of the actual valve coils.
             This lets you watch coil drive without hydraulic hardware.
```

> **Pro tip:** Two cheap 50 Ω SMA dummy loads can replace the antennas during the very first packet-exchange tests. Keeps the RF off-air while you're finding bugs.

## Phase 1 — Bench Pairing

Goal: the three nodes can hear each other and decode encrypted frames.

1. **Power-up** all three nodes (handheld first, then base, then tractor).
2. **Open the M7 serial monitor** on the tractor (Arduino IDE 2, port = X8 USB-C).
3. **Open the lora_bridge log** on the base: `docker compose logs -f lora_bridge`.
4. **Watch for these messages:**
   - Tractor: `[arb] active source = HANDHELD, rssi=-32 dBm, snr=8.5`
   - Base bridge: `RX TelemetryFrame topic=0x05 (link_health) src=tractor`
5. **Press a handheld button.** The corresponding bit should flip in the tractor M7's `[arb] CTRL` log line and in the base's web UI source banner.

If the tractor never sees a source, recheck:

- Same SF / BW / CR / sync-word / freq on all three nodes.
- Same `LIFETRAC_PSK_HEX` in `key.h` / `.env` on all three nodes.
- Antennas physically connected (this catches more bugs than you'd expect).

## Phase 2 — Single-Source Arbitration

Per [`../DESIGN-CONTROLLER/MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md) §8.3, priority is **HANDHELD > BASE > AUTONOMY-reserved**.

- [ ] **Handheld only on:** tractor follows handheld; base UI joystick widget is grayed out with banner "HANDHELD HAS CONTROL".
- [ ] **Base only on (handheld powered down):** tractor follows base UI; banner reads "BASE HAS CONTROL".
- [ ] **Both on simultaneously:** tractor follows handheld; base widget is grayed out.
- [ ] **Handheld releases control** (no input for 30 s + button release): banner switches to "BASE HAS CONTROL", base widget activates.
- [ ] **Base hits TAKE CONTROL while handheld active:** base button shows confirmation modal; on confirm, banner reads "BASE HAS CONTROL".

## Phase 3 — Failsafes

This is the most important phase. **Do not skip any item.**

- [ ] **Handheld E-stop pressed:** tractor goes to neutral within **100 ms** (measure with logic analyzer on a D1608S SSR output, or by eye on the LED stand-ins).
- [ ] **Power-off handheld** while it's the active source: tractor goes to neutral within **500 ms**, base UI banner switches to "NO ACTIVE SOURCE — HOLD".
- [ ] **Power-off base** while it's the active source: tractor goes to neutral within **500 ms**.
- [ ] **Pull the RS-485 cordset** between Max Carrier and Opta during operation: Opta's watchdog trips within **200 ms**, all SSR outputs drop, fault LED lights, buzzer beeps.
- [ ] **Re-plug RS-485:** within ~1 s, watchdog clears, system returns to "no source" hold state.
- [ ] **Replay attack:** capture a frame with an SDR (or copy a frame buffer in `lora_bridge.py` debug mode), retransmit later → tractor logs `seq rollback rejected`.
- [ ] **Tamper:** flip a bit in a captured frame, retransmit → tractor logs `auth tag fail`.

If any failsafe takes longer than its target latency, **stop and debug** before moving on.

## Phase 4 — Latency Measurement

Per [`../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/LATENCY_BUDGET.md`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/LATENCY_BUDGET.md), the targets are:

- Handheld joystick → SSR pickup: **≤ 150 ms median**
- Base UI joystick → SSR pickup: **≤ 250 ms median**

To measure:

1. Wire a logic-analyzer probe to one D1608S SSR output and another to a GPIO on the handheld (or a TTL signal from the base laptop).
2. Configure the analyzer to trigger on the input edge.
3. Push the joystick fully one direction; record the SSR output rising edge.
4. Repeat ~50 times; compute median, p99.

Log the results in `BUILD-CONTROLLER/test_logs/<date>_latency.csv` for posterity.

## Phase 5 — Soak

Leave the system running on the bench for **24 hours** with a script generating periodic stick inputs at the base:

- All three nodes powered.
- Inputs every ~5 s (random direction).
- Telemetry logged the whole time.

End-of-soak acceptance:

- [ ] Zero unexpected M7 / Opta resets in the logs.
- [ ] Zero unauthenticated frame errors.
- [ ] Link RSSI stable within ±3 dB of mean.
- [ ] No memory leak in any Docker container (`docker stats` should show flat memory).

## Phase 6 — Wire to Live Hydraulics

Only after Phases 1–5 pass:

1. **Tractor key OFF.** Master cutoff OFF.
2. Reconnect the valve coil rail at the PSR safety relay.
3. Land each D1608S SSR output to its mapped valve coil (per [02_tractor_node_assembly.md § Step 6](02_tractor_node_assembly.md#step-6--wire-the-valve-coils-to-the-d1608s)). Verify polarity and flyback diodes one more time.
4. **Master cutoff ON, key ON.** Engine OFF.
5. With the joystick **untouched**, confirm no valve is energized (listen for solenoid clicks, watch the pressure gauges).
6. Brief **single-axis test:** push the boom-up joystick gently. Boom should rise. Release; boom should hold.
7. **E-stop test on live hydraulics:** with the boom raised slightly, press the handheld E-stop. Boom should hold (NC valve). Release E-stop; system should return to neutral, not resume motion.
8. Repeat for each axis.

If anything moves unexpectedly at any point, **hit the manual cutoff and re-debug on the bench**.

## Phase 7 — Field Range Test

Once the tractor is running on its own hydraulics:

1. **100 m line-of-sight:** drive away from the base, return. Log RSSI and any link drops.
2. **500 m, 1 km, 5 km, 10 km, 15 km** (if the site allows). The 8 dBi mast antenna at 3 m height should clear at least 5 km comfortably; 15 km is the design target with clean LoS.
3. **Through light foliage** at 1 km and 3 km — expect 6–10 dB more loss.
4. **Handheld walk-around:** test at 100 m, 500 m, 1 km, 2 km from the tractor (handheld TX is +14 dBm, lower than the +20 dBm base, so range is shorter).

Document RSSI vs distance in `BUILD-CONTROLLER/test_logs/<date>_range.csv`.

## Phase 8 — Sign-Off

When all the above pass:

- [ ] Tag the build in the repo: `git tag controller-build-<your-name>-<date>`.
- [ ] Update `BUILD-CONTROLLER/built_systems.md` (TBD) with serial numbers of each board, install location, build date.
- [ ] Take photos of the final mounted system; add to `BUILD-CONTROLLER/photos/<your-build>/`.

You're done.

## Resources & Going Further

- **Hydraulic build:** [`../BUILD-HYDRAULIC/`](../BUILD-HYDRAULIC/)
- **Structural build:** [`../BUILD-STRUCTURE/`](../BUILD-STRUCTURE/)
- **Operator manual:** TBD — once a few systems are out in the field, the operator-facing material gets pulled out of this guide into its own document.
- **Issue tracker:** [github.com/OpenSourceEcology/LifeTrac/issues](https://github.com/OpenSourceEcology/LifeTrac/issues)

## Troubleshooting Quick Reference

| Symptom | Likely cause | Fix |
|---|---|---|
| Tractor never sees a source | Wrong SF/BW/CR/freq mismatch, or PSK mismatch | Verify all three nodes have the same `key.h` and same radio params |
| `auth tag fail` on every frame | PSK mismatch | Re-distribute `key.h` to the failing node |
| Coils energize on power-up | Firmware boot order — Opta wrote before reading mode | Confirm Opta sketch boots into safe-state with all SSRs OFF |
| Latency > 150 ms median (handheld) | Adaptive SF dropped to SF8 or SF9 | Check link health on base UI; reposition antennas, then retest |
| Watchdog trips intermittently | RS-485 termination missing or noisy | Confirm 120 Ω at both ends; check cable shield grounding |
| GPS never shows a fix | Patch antenna without sky view, or U.FL pigtail unseated | Reseat connectors, move antenna to clear sky |
| Web UI loads but joystick doesn't move tractor | Source banner says HANDHELD HAS CONTROL | Take control via base button, or power-down handheld |

For anything not on this table, open an issue with the relevant log excerpts.
