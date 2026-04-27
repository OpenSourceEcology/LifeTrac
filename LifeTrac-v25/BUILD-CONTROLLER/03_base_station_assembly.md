# 3. Base Station Assembly

> **Heads up!** The mast antenna step involves working at height and near electrical bonding. If you're not comfortable on a ladder or with grounding hardware, get help — and **always install the lightning arrestor before connecting the radio.**

The base station is a much simpler build than the tractor: it's an indoor box with a Max Carrier + X8, an Ethernet jack, a 12 V wall PSU, and a feedline running out to a mast antenna. No vibration, no waterproofing, no engine-room hostility.

## What You're Building

```
   ┌──────────── Indoor enclosure ────────────┐
   │                                           │
   │   ┌──────────────────────┐                │
   │   │  Portenta Max        │ ── SMA ── 8 dBi mast antenna
   │   │  Carrier + X8        │            (LMR-400 + arrestor)
   │   │                      │                │
   │   │  Ethernet ───────────┼── RJ45 ── LAN switch ── operator's browser
   │   │  USB-C (debug)       │                │
   │   └──────────────────────┘                │
   │           │                                │
   │   12 V / 5 A PSU ──── mini UPS ──── wall  │
   └───────────────────────────────────────────┘
```

## Required Materials

From your Tier 2 BOM ([01_bill_of_materials.md](01_bill_of_materials.md)):

- Portenta Max Carrier + Portenta X8
- 8 dBi 915 MHz omni mast antenna + LMR-400 coax + N-female lightning arrestor
- Galvanized mast + ground rod
- 12 V / 5 A indoor PSU + mini UPS
- Indoor ventilated enclosure (rack box or wall-mount)
- Cat6 Ethernet cable

## Step 1 — Mate the X8 to the Max Carrier

Same procedure as the tractor side (see [02_tractor_node_assembly.md § Step 3](02_tractor_node_assembly.md#step-3--mate-the-portenta-x8-to-the-max-carrier)). Inspect connectors, press straight down, four M2.5 screws finger-tight + 1/8 turn.

## Step 2 — Mount in the Indoor Enclosure

Lay the Max Carrier flat on a vented shelf or 3D-printed standoff bracket inside your indoor enclosure. Indoor environment means you can skip the DIN rail, the conformal coat, and the rubber bobbins.

Leave clearance for:

- The SMA pigtail to reach the bulkhead.
- The RJ45 to reach the LAN-side bulkhead.
- A USB-C cable to reach the X8 for first-time setup (you'll switch to SSH after).
- Airflow over the X8 — the i.MX 8M Mini gets warm under load.

## Step 3 — Power and Network

1. **12 V / 5 A wall PSU** → mini UPS (12 V input, 12 V output) → Max Carrier `Vin`. The UPS holds the base online for ~30 min through an outage.
2. **Cat6 Ethernet** from Max Carrier RJ45 → your LAN switch. Get the X8 a static DHCP lease (or set a static IP later in Yocto).
3. **(Optional) USB-C** to a debug laptop for first-boot SSH key setup.

Power up. Verify:

- Max Carrier power LED solid.
- X8 boot LED activity, then steady within ~30 s.
- Your DHCP server hands out a lease.
- `ssh root@<base-ip>` works (default credentials in the [Portenta X8 docs](https://docs.arduino.cc/tutorials/portenta-x8/getting-started)).

> **Heads up!** Change the default SSH password before connecting the base to any network you don't control.

## Step 4 — Mast and Antenna

> **Heads up!** Do not connect the antenna feedline to the radio until the lightning arrestor is installed and grounded.

1. **Pick a location** with clear line-of-sight to the typical work area. Higher is better; mast height roughly equals operating range, all else equal.
2. **Drive a ground rod** at least 2.5 m into the soil at the mast base. Use copper-clad steel.
3. **Erect the mast.** Concrete base for permanent installs; guy-wired mast for portable. ~3 m above the roofline is a good first target.
4. **Mount the 8 dBi omni** at the very top, vertical orientation.
5. **Run LMR-400** from the antenna down the mast, into the building, to where the base station enclosure lives.
6. **Install the N-female lightning arrestor** at the mast base, **before** the cable enters the building. Bond its ground lug to the ground rod with #6 AWG copper.
7. **SMA pigtail** from the arrestor's indoor side to the Max Carrier's LoRa SMA jack.

## Step 5 — SWR Check

Before you transmit, verify the antenna chain.

1. Disconnect the SMA pigtail from the Max Carrier.
2. Connect a NanoVNA-H4 (calibrated for 1-port SWR at 902–928 MHz).
3. Sweep the band. **SWR should be < 2:1** across 902–928 MHz — ideally < 1.5:1 at 915 MHz.
4. If SWR is bad, check (in order): connector torque, water in the connector, antenna ground plane, coax dielectric integrity (kinks, crushed sections).

If you don't have a NanoVNA, you can defer this to first-power-on and let the modem report the reflected-power flag — but you'll be transmitting blind.

## Step 6 — Reconnect and Power-On

1. Reconnect the SMA pigtail to the Max Carrier.
2. Power-cycle the base.
3. Confirm boot, network, and SSH.

The base is now hardware-ready. Software install happens in [`05_firmware_installation.md`](05_firmware_installation.md).

## Acceptance Test

- [ ] Base boots, gets DHCP, SSH-able.
- [ ] LoRa pigtail connected to a working antenna chain (SWR < 2:1).
- [ ] Lightning arrestor grounded to a driven rod with #6 AWG.
- [ ] UPS holds the base online when you unplug the wall PSU.

## Next Step

If you're building the optional handheld remote, head to [**4. Handheld Assembly →**](04_handheld_assembly.md). Otherwise skip to [**5. Firmware & Software Installation →**](05_firmware_installation.md).
