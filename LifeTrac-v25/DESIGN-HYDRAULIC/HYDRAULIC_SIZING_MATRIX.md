# Hydraulic Sizing Matrix

Trade-study spreadsheet for pump + track-motor + cylinder combinations under consideration for LifeTrac v25.

## File

[HYDRAULIC_SIZING_MATRIX.csv](HYDRAULIC_SIZING_MATRIX.csv) — opens in Excel, LibreOffice, Google Sheets, or any text editor. Plain CSV so git diffs are readable.

## Columns

| Column | Units | Formula / source |
|---|---|---|
| `scenario_id` | — | Short tag for cross-reference. |
| `topology` | — | single-pump / tandem / triple. |
| `pump_sku` | — | Manufacturer + model family. |
| `pump_native_flange` | — | What the pump ships with (SAE-AA / SAE-A / SAE-B). |
| `pump_native_shaft` | — | Spline tooth count + DP, or parallel + key spec. |
| `bell_housing_strategy` | — | "off-the-shelf X-to-Y kit" preferred; "custom" only when no commercial kit covers the engine + pump combination. |
| `bell_housing_sku` | — | Vendor + product family (Bailey Hydraulics, Magnaloy, Surplus Center, Northern Tool). |
| `bell_housing_cost_usd` | USD | Approximate kit price including the housing and (where bundled) the L-jaw coupling. |
| `coupling_sku` | — | Coupling part if sold separately from the housing kit; "Included in kit" if bundled. |
| `coupling_cost_usd` | USD | Extra coupling cost when not bundled. |
| `adapter_required` | yes/no | Whether anything beyond the standard kit is needed (custom plate, sleeve adapter, re-machined hub). |
| `adapter_notes` | — | What the adapter does and why. |
| `stack_length_added_in` | inches | Total distance the bell housing + coupling adds between engine PTO face and pump shaft (chassis envelope check). |
| `pump_section_a_cc`, `pump_section_b_cc` | cm³/rev | Per datasheet. `n/a` if single-pump. |
| `engine_rpm` | RPM | Vanguard 18 HP governs at 3,600. |
| `vol_eff`, `mech_eff` | — | Pump efficiency (datasheet typical: 0.96 vol, 0.88 mech for HY series; 0.85/0.88 conservative for surplus pumps). |
| `section_a_gpm`, `section_b_gpm` | GPM | `cc × rpm × vol_eff / 3785`. |
| `total_gpm` | GPM | Sum of sections. |
| `track_motor_cipr` | in³/rev | Track motor displacement. **Currently TODO_BENCH** — depends on sprocket pitch diameter from DESIGN-STRUCTURAL. |
| `track_motor_cc` | cm³/rev | `cipr × 16.387`. |
| `track_max_rpm` | RPM | Target sprocket RPM at full forward (sets ground speed via sprocket dia). |
| `track_gpm_per_side` | GPM | `cc × rpm / 3785`. |
| `track_gpm_total` | GPM | `2 × per side` — flow needed when **both** track valves open at full speed. |
| `relief_psi` | PSI | Pump-outlet relief setting. |
| `section_*_hp_at_relief` | HP | `gpm × psi / 1714`. |
| `total_hp_at_relief` | HP | Sum. **If > engine_hp_net**, relief opens before all sections reach setpoint (normal — but watch heat). |
| `engine_hp_net` | HP | Vanguard 18 HP gross ≈ 16 HP net. |
| `headroom_hp` | HP | `engine_hp_net − total_hp_at_relief`. Negative = relief mediates load (acceptable for transient peaks; problematic for sustained cycles). |
| `notes` | — | Free text. |

## How to extend

1. Add a row per new combination you want to evaluate.
2. Recompute derived columns by hand or by opening in Excel and pasting these formulas (zero-indexed example for row 2):
   - `section_a_gpm` = `pump_section_a_cc * engine_rpm * vol_eff / 3785`
   - `track_gpm_per_side` = `track_motor_cc * track_max_rpm / 3785`
   - `total_hp_at_relief` = `total_gpm * relief_psi / 1714`
3. For comparing to a target, the `target_skidsteer_perf` row at the bottom holds the requirements rather than a candidate solution.

## Decision flags to watch

- **`track_gpm_total` > `total_gpm`** → tracks alone outrun the pump even before arms move. Either over-displaced motors or under-sized pump.
- **`headroom_hp` < −5** → engine is stalling under sustained dual-section demand; relief will be hot and inefficient. Drop `relief_psi` or downsize a section.
- **Sections wildly mismatched (a vs b)** → if both feed the same circuit (e.g., track L vs track R), inherent flow imbalance manifests as a tractor that pulls to one side.

## Open data the matrix needs

- [ ] Final track sprocket pitch diameter (DESIGN-STRUCTURAL).
- [ ] Final track motor SKU (drives `track_motor_cipr`).
- [ ] Cylinder bore × stroke for arms and bucket (drives extend/retract GPM demand and cycle time — separate sub-table to be added once known).
- [ ] Confirmed Vanguard 18 HP PTO rotation direction (drives pump rotation code C/A in part number).
- [ ] Lock specific Bailey/Magnaloy bell-housing kit SKU for the chosen pump (S2/S3/S4/S9 all use the same kit — pick once).

## Bell-housing & adapter notes

**Default to off-the-shelf** engine-to-SAE pump mount kits. The Vanguard 18-23 HP family bolts directly to widely-available kits from Bailey Hydraulics, Magnaloy, Surplus Center, and Northern Tool. A typical kit includes:

- Cast or machined bell housing — engine bolt circle on one end, SAE-A or SAE-B pump face on the other
- L-jaw coupling spider + two hubs (one bored 1-1/8" keyed for the engine, one matching the pump shaft spline)
- Mounting hardware

Typical pricing: **$120–$280** for SAE-A kits, **$200–$320** for SAE-B kits.

**Go custom only when:**
- The pump uses an uncommon flange at this engine size (e.g., HY1 SAE-AA on a Vanguard V-twin → S6 in the matrix)
- The build needs cross-group tandem stacking that no factory or vendor supports (S10 anti-pattern)
- Chassis integration demands a non-standard envelope

**Coupling caveat for HY3-LN:** the datasheet calls for a **5/8" 13T spline** on SAE-B, which is non-standard — most off-the-shelf SAE-B coupling hubs are bored for **7/8" 13T**. Verify hub bore before ordering, or specify the **22P (7/8" parallel keyed)** shaft option from the HY3-LN order code, which matches the common 7/8" keyed hub.

**Vendor search terms** (so the BOM trail stays reproducible):
- Bailey Hydraulics: search "Vanguard pump mount kit" or "engine to SAE-A pump"
- Magnaloy: M200 series adapter housings + M500 series couplings
- Surplus Center: catalog §9, search "pump mount kit Vanguard"
- Northern Tool: "NorTrac Pump Mounting Kit"

## Related docs

- [HYDRAULIC_BOM.md](HYDRAULIC_BOM.md) — canonical BOM with TODO_BENCH markers this matrix is meant to retire.
- [HYDRAULIC_DIAGRAM.md](HYDRAULIC_DIAGRAM.md) — circuit topology.
- [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) — why tandem-centre spool drives the per-section flow split decisions.
