# LifeTrac v25 Hydraulic Build Guide *(Stub)*

This folder will hold the step-by-step assembly guide for the v25 hydraulic system, written in the same SparkFun-tutorial style as [`../BUILD-CONTROLLER/`](../BUILD-CONTROLLER/).

The canonical hydraulic design lives in [`../DESIGN-HYDRAULIC/`](../DESIGN-HYDRAULIC/). When this build guide is filled in, it will cover (in order):

1. **Bill of Materials** — pumps, valves, cylinders, hoses, fittings, with DigiKey/Mouser/McMaster-Carr links where applicable.
2. **Manifold Assembly** — directional valve stack, Burkert 8605 flow valves, pressure transducers.
3. **Cylinder Installation** — boom, bucket, drive cylinders.
4. **Hose Routing** — supply, return, drain, and pressure-transducer pickoffs.
5. **First Fill and Bleed** — purge air, set relief pressures, calibrate flow setpoints.
6. **Bring-Up & Testing** — bench-driven valve cycling (using the controller's bench mode), pressure-vs-stick calibration, leak check.

> **Status:** Design is mature; build guide drafting is **not yet started**. If you want to help, open an issue.

## Related

- [`../BUILD-CONTROLLER/`](../BUILD-CONTROLLER/) — the controller build guide. The controller and hydraulic builds can run in parallel until [§ 6 Bring-Up](../BUILD-CONTROLLER/06_bringup_and_testing.md) Phase 6, where the two come together.
- [`../DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md`](../DESIGN-HYDRAULIC/HYDRAULIC_DIAGRAM.md) — canonical hydraulic schematic.
- [`../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md`](../DESIGN-HYDRAULIC/FLOW_VALVE_CONFIGURATION.md) — flow valve setup.
