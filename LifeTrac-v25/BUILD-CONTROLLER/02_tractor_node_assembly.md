# 2. Tractor Node Assembly

> **Heads up!** The tractor's 12 V system can deliver hundreds of amps into a short. Before you touch any wiring, confirm the **master battery cutoff switch is OFF** and the keys are in your pocket. Re-check with a multimeter at the fuse panel before stripping wires.

This section walks you through building the tractor-side controller: a Portenta Max Carrier + X8 stack plus an Arduino Opta with two expansions, all on a single DIN rail in a Hammond 1554 IP66 enclosure.

## What You're Building

```
                      IP66 enclosure
   ┌──────────────────────────────────────────────────────────────┐
   │   ┌──────────┐  ┌──────────┐  ┌────────┐ ┌───────┐ ┌───────┐ │
   │   │   Max    │  │  Opta    │  │ D1608S │ │ A0602 │ │ Spare │ │
   │   │ Carrier  │  │  WiFi    │  │  SSR   │ │  AI/AO│ │  DIN  │ │
   │   │  + X8    │  │ (Modbus  │  │  ×8    │ │       │ │       │ │
   │   │          │  │  slave)  │  │        │ │       │ │       │ │
   │   └────┬─────┘  └────┬─────┘  └───┬────┘ └───┬───┘ └───────┘ │
   │        │  RS-485     │              │           │             │
   │        └─────────────┘              │           │             │
   │   12 V in ──► fuse ──► PSR ──► coil rail        │             │
   │                                                                │
   │   USB ──► MCP2221A ──► BNO086 IMU ──► NEO-M9N GPS              │
   └────────────────────────────────────────────────────────────────┘
        │                  │                │
   LoRa SMA            GPS SMA         Cab USB camera
```

## Required Materials

Pull these from your Tier 1 BOM (see [01_bill_of_materials.md](01_bill_of_materials.md)):

- Portenta Max Carrier + Portenta X8
- Arduino Opta WiFi + D1608S + A0602
- Hammond 1554 enclosure + DIN rail + end stops
- PSR safety relay + engine-kill relay
- 12 V power feed parts (cutoff, fuse, EMI ferrite, TVS)
- 18650 + TP4056 backup-power module
- 2× M12-A 5-pin RS-485 cordsets + 120 Ω termination
- Status LEDs, buzzer, cable glands

You will *not* mount the radios, GPS, or camera in this section — those go on after the box is closed and tested ([§ Sensor mounting](#sensor-mounting) at the end).

## Step 1 — Prep the Enclosure

1. **Mark the cable-gland positions.** Use a paper template (or just measure). Plan for:
   - 1× M20 — main 12 V power feed
   - 2× M16 — RS-485 to remote sensors / valve harness
   - 2× M16 — pressure-sensor cordsets
   - 1× M16 — USB to dash camera (or camera enclosure)
   - 1× M12 Gore-Tex breather vent (**don't skip this** — solar greenhouse effect can hit 70 °C inside a sealed box)
2. **Drill** with a step bit at the marked positions. Deburr.
3. **Install glands** finger-tight; you'll torque them down after cabling.
4. **Cut the DIN rail** to fit lengthwise across the bottom of the enclosure (~280 mm for the 1554). Mount with M4 hardware on rubber bobbin vibration mounts.

> **Pro tip:** If you can, drill the enclosure on a workbench *before* the boards are anywhere near it. Aluminum and polycarbonate chips inside a Max Carrier connector will ruin your week.

## Step 2 — Mount Boards on the DIN Rail

Snap onto the DIN rail in this order, left to right:

1. **Phoenix Contact PSR-MC38** safety relay (slimmest module, goes near the power-input gland).
2. **Arduino Opta WiFi** with the **D1608S** and **A0602** clipped to its right side via the expansion bus connector — they daisy-chain mechanically.
3. **DIN-rail clips for the Max Carrier + X8.** The Max Carrier doesn't ship with a DIN-rail mount; use a generic [DIN-rail PCB carrier](https://www.digikey.com/en/products/filter/pcb-mount-din-rail-carriers/2225) sized for the Max Carrier's footprint, or 3D-print one (`/cad/maxcarrier_din_mount.stl` in repo).
4. **Spare DIN clip** at the right end for the engine-kill relay socket and the TVS-protected fuse holder.

Verify each module is fully clicked into the rail and won't slide along its length. Add end stops at both ends.

## Step 3 — Mate the Portenta X8 to the Max Carrier

> **Heads up!** The Portenta high-density connectors are delicate. Inspect both halves for bent pins before mating.

1. Orient the **X8** so the SOM connectors line up with the Max Carrier's two 80-pin sockets.
2. Press straight down with even pressure on both ends — don't rock it. You should feel a single firm seat.
3. Torque the four M2.5 screws **finger-tight + 1/8 turn**. Don't crush the SOM.

## Step 4 — Wire the Power Train

This is the most safety-critical wiring you'll do. Take your time.

```
  Tractor 12 V  ──┬── master cutoff (175 A) ──┬── 20 A blade fuse ──┬── EMI ferrite
                  │                            │                     │
                  │                            └── TVS (SMBJ33A)     │
                  │                                                  ▼
                  │                                    ┌─────────────────────┐
                  │                                    │  Max Carrier  Vin   │
                  │                                    │  (6–36 V tolerant)  │
                  │                                    └─────────────────────┘
                  │
                  └── 5 A blade fuse ──► PSR safety relay coil rail ──► valve coils
```

1. **Run 12 AWG** from the battery + terminal through the master cutoff to the in-enclosure fuse block. Crimp a ring lug; do **not** rely on a screw terminal alone for the battery side.
2. **Snap the EMI ferrite** around the 12 V wire just inside the enclosure gland. Two turns of the wire through the ferrite if it fits.
3. **Wire the TVS (SMBJ33A)** anode-to-ground, cathode-to-12V across the input. This eats load-dump transients (up to ~80 V on a tractor) before they reach the Max Carrier.
4. **Connect Max Carrier `Vin`** (6–36 V tolerant — see [Portenta Max Carrier docs](https://docs.arduino.cc/hardware/portenta-max-carrier/)) and **GND**. Verify polarity with a multimeter *before* powering.
5. **Run a separate 5 A fused branch** to the PSR safety relay coil. The PSR's normally-open contact will gate the valve coil rail — wire that contact in series with the +24 V supply to the D1608S SSR commons.
6. **Wire the 18650 + TP4056** as a UPS on the Max Carrier's `Vbatt` input. This carries the X8 through engine-crank brown-outs (sub-7 V dips for ~200 ms).

Power up briefly with a bench supply at 12 V and confirm:

- Max Carrier power LED solid.
- X8 boot LED activity within ~20 s.
- No burning smell. (If you smell anything, kill power immediately and re-check polarity.)

## Step 5 — Wire the Modbus RS-485 Link

The Max Carrier's J6 connector is the RS-485 master; the Opta's RS-485 port is the slave. They need a daisy-chained pair with **120 Ω termination at each end**.

1. **From Max Carrier J6:** A → green wire of cordset, B → white wire, GND → bare drain.
2. **To Opta RS-485 port:** same A/B/GND mapping (refer to [Opta pin-out](https://docs.arduino.cc/hardware/opta/)).
3. **120 Ω resistor** across A↔B at the Max Carrier end.
4. **120 Ω resistor** across A↔B at the Opta end.
5. **Cordset length** ≤ 30 m for 115200 baud. Inside one enclosure, ~30 cm is fine.

> **Pro tip:** RS-485 wants a **twisted pair** for A/B. The TURCK / Phoenix preassembled cordsets are already twisted; if you're hand-terminating bulk cable, use Belden 3105A or equivalent.

## Step 6 — Wire the Valve Coils to the D1608S

This is the per-§8.18 SSR routing. **All eight directional valve coils land on the D1608S SSR channels.** The Opta's onboard EMRs are reserved for engine-kill / horn / brake / spare.

| D1608S SSR channel | Valve coil | Hydraulic function |
|---|---|---|
| SSR1 | BOOM_UP | Loader boom raise |
| SSR2 | BOOM_DOWN | Loader boom lower |
| SSR3 | BUCKET_CURL | Bucket curl in |
| SSR4 | BUCKET_DUMP | Bucket dump out |
| SSR5 | DRIVE_LH_FWD | Left track forward |
| SSR6 | DRIVE_LH_REV | Left track reverse |
| SSR7 | DRIVE_RH_FWD | Right track forward |
| SSR8 | DRIVE_RH_REV | Right track reverse |

For each coil:

1. **+24 V** from the PSR-gated coil rail → SSR common.
2. **SSR output** → coil + terminal.
3. **Coil – terminal** → ground bus.
4. **1N4007 flyback diode** across each coil, cathode to +24 V, anode to ground. **Do not skip this** — inductive kickback will kill the SSR and the diode is $0.10.

| Opta onboard EMR | Function |
|---|---|
| Relay 1 | Engine-kill (drives Bosch relay coil) |
| Relay 2 | Beacon / horn |
| Relay 3 | Parking-brake release |
| Relay 4 | Spare |

## Step 7 — Wire the Analog I/O (A0602)

The A0602 carries 6 analog inputs (0–10 V or 4–20 mA, software-selectable per channel) and 2× 0–10 V outputs.

| A0602 channel | Signal | Source |
|---|---|---|
| AI1 | `battery_v` | 12 V rail through 1:5 divider |
| AI2 | `oil_pressure` | (future engine sensor) |
| AI3 | `coolant_temp` | (future engine sensor) |
| AI4 | `hyd_supply_psi` | Pressure sensor 1, 4–20 mA |
| AI5 | `hyd_return_psi` | Pressure sensor 2, 4–20 mA |
| AI6 | spare | — |
| AO1 | `flow_setpoint_1` | → Burkert 8605 flow valve A |
| AO2 | `flow_setpoint_2` | → Burkert 8605 flow valve B |

For the **4–20 mA pressure sensors**:

1. Wire each sensor on its M12-A 4-pin cordset: brown = +24 V, blue = signal return, black = current signal, white = unused.
2. **Configure A0602 channels AI4/AI5 as 4–20 mA** (it's a software setting on the Opta firmware — see [`05_firmware_installation.md`](05_firmware_installation.md)).
3. Verify with a multimeter in series with the loop: 4 mA at zero pressure, 20 mA at full scale.

## Step 8 — Wire the IMU and GPS (Linux Side)

These hang off the X8's Linux side via a USB → I²C bridge. They are *not* on the M7 control loop.

1. **Plug the Adafruit MCP2221A** into one of the X8's USB host ports via a short USB-A → USB-C cable.
2. **Daisy-chain Qwiic cables** from the MCP2221A → BNO086 IMU → NEO-M9N GPS. The IMU mounts to a vibration-isolated standoff inside the enclosure (use the rubber bobbins again or a dab of silicone). The GPS sits flat — its SMA jack faces the bulkhead.
3. **GPS antenna feedline** from the SMA bulkhead → cab-roof magnetic patch antenna.

## Step 9 — Wire the Status LEDs and Buzzer

Land these on spare D1608S SSR channels (5–8 are unused if your hydraulic config doesn't need all 8 directional coils — check). Otherwise drive them from Opta digital outputs through an inline resistor.

- **POWER LED (green)** — wired to the +12 V switched rail. Always on when keys are on.
- **LINK LED (amber)** — driven by the M7 firmware's `link_ok` flag (heartbeat received in the last 500 ms).
- **FAULT LED (red)** — driven by Opta's `safety_state != OK`.
- **Buzzer** — driven from a spare Opta EMR, beeps on E-stop / watchdog.

## Step 10 — Close It Up

1. **Conformal-coat** the Max Carrier + X8 + Opta + expansions with one even pass of MG Chemicals 419 acrylic spray. Mask any connectors and the SOM separation line on the X8. Let dry 30 min.
2. **Torque cable glands** to the manufacturer's spec (typically 3–5 N·m).
3. **Install the Gore-Tex breather** in its M12 hole.
4. **Close the enclosure lid** with the captive Phillips screws. Don't over-torque — these are plastic.
5. **Mount the enclosure** in the cab on rubber bobbin vibration mounts. Do **not** hard-bolt to the chassis — the high-frequency vibration will kill solder joints over a season.

## Sensor Mounting

After the box is sealed:

- **LoRa whip antenna** — mount externally on a ground plane (cab roof or fender). 3 dBi whip with a magnetic base is fine for first builds. Keep ≥ 30 cm from the GPS antenna to avoid intermod.
- **GPS patch antenna** — magnetic mount on cab roof, ≥ 5 cm clear sky in all directions, ≥ 30 cm from LoRa whip.
- **USB camera** — see [`../DESIGN-CONTROLLER/HARDWARE_BOM.md` § "Camera enclosure path"](../DESIGN-CONTROLLER/HARDWARE_BOM.md#notes-on-substitutions). The cable runs through the USB-A panel-mount bulkhead with a gland.

## Acceptance Test

You're done with mechanical assembly when:

- [ ] Box is sealed, glands torqued, lid screws snug.
- [ ] Power-up at 12 V draws < 1.5 A steady-state.
- [ ] Max Carrier power LED + X8 boot LED both come on.
- [ ] Multimeter shows ~24 V on the PSR-gated coil rail when the PSR is energized.
- [ ] Multimeter shows < 0.5 V on every D1608S SSR output (none accidentally on).
- [ ] No smoke, no smell, no warm spots. Re-check after 10 min of idle power.

## Next Step

Head to [**3. Base Station Assembly →**](03_base_station_assembly.md).
