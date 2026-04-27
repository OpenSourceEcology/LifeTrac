# LifeTrac v25 Structural Build Guide *(Stub)*

This folder will hold the step-by-step fabrication and assembly guide for the v25 chassis, frame, and mechanical hardware, written in the same SparkFun-tutorial style as [`../BUILD-CONTROLLER/`](../BUILD-CONTROLLER/).

The canonical structural design lives in [`../DESIGN-STRUCTURAL/`](../DESIGN-STRUCTURAL/). When this build guide is filled in, it will cover (in order):

1. **Bill of Materials** — steel tubing, plate, fasteners, bearings, track-chain components, with McMaster-Carr / OnlineMetals links.
2. **CNC Cutting** — exporting parts from [`../DESIGN-STRUCTURAL/openscad/`](../DESIGN-STRUCTURAL/openscad/), nesting, plasma/waterjet cut order.
3. **Frame Welding** — main rails, cross members, ROPS, pivot mounts. Per [`../AI NOTES/2025-01-25_Pivot_Mount_Assembly.md`](../AI%20NOTES/2025-01-25_Pivot_Mount_Assembly.md).
4. **Track Drive Assembly** — UTU integration per [`../AI NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md`](../AI%20NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md).
5. **Wheel Hub Assembly** — UWU integration per [`../AI NOTES/2026-02-02_UWU_Integration_Summary.md`](../AI%20NOTES/2026-02-02_UWU_Integration_Summary.md).
6. **Boom and Bucket Installation** — pivot pins, cylinders, bushings.
7. **Cab and Operator Station** — mounting points for the controller enclosure (see [`../BUILD-CONTROLLER/02_tractor_node_assembly.md` § Step 10](../BUILD-CONTROLLER/02_tractor_node_assembly.md#step-10--close-it-up)).
8. **Final Assembly** — drop-in of hydraulic + controller systems, paint, sign-off.

> **Status:** Design is in active iteration. Build guide drafting is **not yet started**. If you want to help, open an issue.

## Related

- [`../BUILD-CONTROLLER/`](../BUILD-CONTROLLER/) — controller build guide.
- [`../BUILD-HYDRAULIC/`](../BUILD-HYDRAULIC/) — hydraulic build guide.
- [`../DESIGN-STRUCTURAL/README.md`](../DESIGN-STRUCTURAL/README.md) — canonical structural design overview.
- [`../DESIGN-STRUCTURAL/CNCLAYOUT_PARTS_LIST.md`](../DESIGN-STRUCTURAL/CNCLAYOUT_PARTS_LIST.md) — full parts list for CNC cutting.
