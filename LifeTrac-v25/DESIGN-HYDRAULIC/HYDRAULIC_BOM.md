# LifeTrac v25 — Hydraulic Bill of Materials

**Status:** First-pass canonical BOM. Engine, flow control, directional valves, and the soft-stop control strategy are operator-confirmed. Pump SKU, cylinder bores, motor displacement, reservoir, cooler, and plumbing line items carry `TODO_BENCH` markers — they depend on track/arm geometry decisions and on bench validation of pump availability.

**Related docs:**

- [HYDRAULIC_DIAGRAM.md](HYDRAULIC_DIAGRAM.md) — ASCII schematic of the full system.
- [FLOW_VALVE_CONFIGURATION.md](FLOW_VALVE_CONFIGURATION.md) — Single vs. dual EFC mode (D11 jumper on Opta D1608S, 4-20 mA via Burkert).
- [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) — **Canonical actuator soft-stop / soft-start sequencing.** Drives the directional-valve spool selection (tandem centre, D1VW SKU `008`) and removes the need for external load-holding hardware. Read this before §3 and §4 below.
- [hopsan/README.md](hopsan/README.md) — multi-domain simulation models that consume this BOM as their parameter source.
- [../DESIGN-CONTROLLER/HARDWARE_BOM.md](../DESIGN-CONTROLLER/HARDWARE_BOM.md) — controller hardware (Opta, Portenta, sensors).
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — IP-303 axis-ramp infrastructure that drives the EFC current ramp.
- [../DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h](../DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h) — `LIFETRAC_HYDRAULIC_*_RAMP_SECONDS` ramp tuning macros.

**Scope:** Hydraulic subsystem only — engine through actuators, plus reservoir, cooler, plumbing, and the controller-side parts (Burkert 8605) that physically belong to the hydraulic stack rather than the electronics enclosure.

---

## 1. Power group (engine + drive coupling + pump)

| Item | Canonical part | Key spec | Alternates | Hopsan mapping |
|---|---|---|---|---|
| **Engine** | **Briggs & Stratton Vanguard 18 HP** (V-twin, horizontal-shaft, electric start) — candidate SKU: Model 305447 or 356447 | 570 cc V-twin OHV; **18 HP gross / ~16 HP net** at 3,600 RPM governed; **1-1/8″ × 2.96″ keyed horizontal shaft**; electric start with ring gear; **16 A regulated alternator**; ~85 lb dry; gasoline | Honda GX630 V-twin (~$500 premium, gold-standard reliability), Kohler Command Pro CH640/730, Vanguard 23 HP (Model 386447) for higher-flow upgrade path | Constant `omega_engine` = 376.99 rad/s (3,600 RPM) feeding `HydraulicFixedDisplacementPump` with externally specified displacement. No combustion model — engine is treated as ideal mechanical source. |
| **Bell housing + coupling** | **SAE-A 2-bolt bell housing + L-jaw spider coupling** (rubber spider) | 1-1/8″ × 9 T spline or 1-1/8″ keyed bore engine half; SAE-A pump half (5/8″ × 9 T spline standard); torsionally compliant rubber spider damps engine impulses | Direct keyed coupling (cheaper, harsher), Lovejoy L-095 series, magnetic coupling (overload protection but $$$) | Out of scope for fluid model; modelled as ideal rigid coupling in v0 .hmf templates. |
| **Hydraulic pump** | **~11 GPM SAE-A gear pump** — TODO_BENCH for exact SKU | Fixed displacement, **1.4 cipr (~22.9 cm³/rev)** → 11 GPM at 3,600 RPM × 0.85 vol eff; **3,000 PSI continuous / 3,500 PSI peak**; SAE-A 2-bolt mount; 5/8″ × 9 T spline input; SAE 12 inlet / SAE 10 outlet typical | Haldex Concentric 1001601 family (1.39 cipr), Parker PGP315/PGP330, Casappa Polaris PLP10 series, Eaton 26500 series. **TODO_BENCH:** confirm in-stock SKU at Surplus Center or Northern Tool before locking. | `HydraulicFixedDisplacementPump` from default library; parameter `D = 22.9e-6` m³/rev. |
| **Inline pressure relief** | **Cartridge relief valve, 3,000 PSI cracking** (mounted on pump outlet block or first manifold port) | Direct-acting or pilot-operated; 15 GPM rated minimum (must exceed pump max flow); **3,000 PSI factory set** | Sun Hydraulics RPGC-LAN, Parker R4V series, surplus equivalents | `HydraulicPressureReliefValve`; `p_ref = 20.7e6` Pa (3,000 PSI). |

**Power budget sanity check:**

$$\text{HP}_{\text{hyd, max}} = \frac{11 \text{ GPM} \times 2{,}500 \text{ PSI}}{1714} \approx 16 \text{ HP}$$

That sits exactly at the engine's net rating, leaving no headroom for sustained 3,000 PSI relief operation. Working pressure target is **2,000–2,500 PSI**; 3,000 PSI is the safety relief, not a steady-state operating point. The Brand EFC's pressure compensator handles transient spikes.

---

## 2. Flow control (proportional, electronically commanded)

| Item | Canonical part | Key spec | Alternates | Hopsan mapping |
|---|---|---|---|---|
| **Proportional flow control valve** | **Brand Hydraulics EFC, 10 GPM body** | Pressure-compensated; CF (controlled flow) + EX (excess flow bypass) + IN ports; solenoid-driven proportional orifice; **3,000 PSI max**; **10 GPM body matches pump output → minimal bypass at full demand → minimal heat generation**; dashpot tunable for response speed; NBR seals (mineral oil) | Sun FCEAL-LAN proportional, Parker FA series, manual fallback: Brand FC51-10 (no electronic control). For dual-EFC mode (per [FLOW_VALVE_CONFIGURATION.md](FLOW_VALVE_CONFIGURATION.md)): two units in 6 GPM body each. | `HydraulicPressureCompensatingValve`, or pattern: `HydraulicVariableOrifice` + pressure compensator + pilot signal driven from BuildConfig `flow.target_gpm`. |
| **EFC controller / driver** | **Burkert 8605 Type 316532** | 4-20 mA input (PWM-conditioned); current-driven solenoid output sized for Brand EFC coil; configurable ramp / dither; powered from 12 V tractor electrical | Brand EC-AT01 (purpose-built for EFC), generic PWM driver from Opta D1608S **directly** (skips Burkert entirely — possible if EFC coil current is within Opta output rating; **TODO_BENCH** to verify) | Out of scope for fluid model; modelled as ideal current-to-area transfer function. |

---

## 3. Manifold + directional control

| Item | Canonical part | Key spec | Alternates | Hopsan mapping |
|---|---|---|---|---|
| **Manifold** | **NFPA D03 / CETOP 3 parallel manifold, 4 stations** | 4 D03 sub-base ports in parallel; common P, T, A1-A4, B1-B4 galleries; SAE 8 P&T external ports, SAE 6 work ports; **3,000 PSI min** rated; ductile iron or steel body | Daman manifolds, Hydraforce HMD series, custom fabrication. **TODO_BENCH:** confirm port size and check for built-in relief cavity. | Each station modelled as a parallel branch off the pump pressure node. |
| **Directional control valves × 4** | **Parker D1VW00*8*CNKW** (Parker D1VW family, tandem-centre spool variant) | NG06 / D03 / CETOP 3 mounting; 4-way 3-position; **spool 8 = tandem centre** (P→T open through, A & B blocked when centred); **12 VDC wet-armature solenoid**, two coils per valve (one per direction); 18.5 GPM max flow (Parker D1VW family rating, Grainger-derated for the SKU 2NMU8 = `004` variant — confirm matches for `008`); **5,000 PSI max system / 3,000 PSI max tank**; NBR seals; DIN 43650 Form A coil connector; -4 °F to 140 °F; ~25 lb each; ~$528 each | Eaton DG4V-3 (spool C), Vickers DG4V, Yuken DSG-01 (spool 3C2), Bosch Rexroth 4WE6 — all D03/NG06 drop-in compatible with equivalent tandem-centre spools. **Float-centre fallback variant** D1VW004CNKW (Grainger SKU 2NMU8) is documented in [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) as the **interim hardware bill** for any tractor built before BC-18 ships; that variant requires the load-holding hardware in (deleted) §4. | Each valve: `Hydraulic43Valve` from default library. **Tandem-centre modelling:** P→A and P→B orifice areas zero in centre; **P→T orifice present (through-pass)**; A→T and B→T orifice areas zero (cylinder lines blocked → load held). Solenoid response: `omega_h ≈ 100–150` rad/s, `delta_h ≈ 0.4–0.7` (≈30–50 ms shift). |
| **Coil connectors × 8** | **DIN 43650 Form A connector with cable gland**, 12 V LED-equipped (~$5–10 each) | Mates D1VW solenoid coils (sold separately); LED indicates coil energised; integrated PG9 cable gland | Hirschmann GDM, generic A8E series. **EASILY OVERLOOKED** — D1VW ships with bare coil pins, no plug. | N/A |

### Spool selection rationale (D1VW00*8*CNKW = tandem centre)

| Function | Tandem-centre verdict |
|---|---|
| **Track motors (left, right)** | ✅ With EFC ramp + valve settling delay (per [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md)) the motor sees zero flow at the moment the valve centres — no inertial spike, no cavitation. **Hydraulic hill-hold** is a positive bonus when fully stopped. The trade-off is loss of free coast on release; deceleration follows the IP-303 ramp ladder (250–2000 ms depending on magnitude). |
| **Arms cylinder** | ✅ A & B blocked when centred → cylinder holds load mechanically. **No external PO check valve required** (the deleted §4 of an earlier revision). |
| **Bucket cylinder** | ✅ Same — natural load-hold from spool. |
| **Pump idle** | ✅ When all four valves centre, the pump sees an open P→T path through the stack at near-zero pressure. **Drastically less idle heat** than the float-centre alternative, which forced the EFC to bypass full pump flow through its EX port. The cooler size in §6 sizes for *worst-case operating* bypass, not idle. |
| **Failure mode** | E-stop drops EFC and valves simultaneously. Tracks bottle their inertial flow → the **pump-side relief valve** in §1 cracks at 3,000 PSI within ~5 ms and dumps the spike to tank. **The relief valve cannot be removed** even though normal operation never reaches its setpoint. |

**Pre-BC-18 interim:** the EFC-ramp + valve-delay sequence requires firmware work tagged **BC-18** in [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md). Until that ships, populate the directional-valve slot with the **float-centre fallback** D1VW00*4*CNKW (Grainger SKU 2NMU8) and add 2 × pilot-operated check valves on the arm and bucket rod-side lines as the interim load-holding solution. Migrate to spool 8 at the next service interval after BC-18 lands.

**Other-builder flexibility.** This BOM documents the *canonical v25 build*. Other LifeTrac builders may pick a different spool centre (float, open, closed) and a different load-holding strategy (PO check, counterbalance, none). The firmware is built to support all four spool types via the proposed `hydraulic.spool_type` / `hydraulic.load_holding` / `hydraulic.valve_settling_ms` BuildConfig leaves (BC-19 schema work; see [SOFT_STOP_STRATEGY.md → Hydraulic variants & per-build configuration](SOFT_STOP_STRATEGY.md)). The three reference builds (OSE-legacy, v25-canonical, high-performance) are documented there with their hydraulic-leaf settings; copy `build.default.toml` → `build.<unit>.toml`, edit those leaves, and re-run the standard `lifetrac-config validate / bundle / verify` flow. Update §3 and (re-introduce or remove) §4 of *your* copy of this BOM to match your spool choice — the BOM is a per-build artifact even when the firmware is shared.

---

## 5. Actuators

| Item | Canonical part | Key spec | Alternates | Hopsan mapping |
|---|---|---|---|---|
| **Track drive motors × 2** | **TODO_BENCH** — sized for 11 GPM ÷ 2 = ~5.5 GPM per side at full forward | Bidirectional gerotor or geroler motor; ~10–15 cubic inch displacement (target ~50–100 RPM output at full flow into a track sprocket reduction); SAE-A 2-bolt mount; SAE 10 work ports; 3,000 PSI continuous | White Drive Roller Stator RS series, Eaton Char-Lynn H series, Parker TG/TF series. Final SKU depends on **track sprocket diameter** (TODO_BENCH from DESIGN-STRUCTURAL). | `HydraulicMotorQ` (low-speed high-torque) from default library; `D_motor` parameter from final SKU. |
| **Arm lift cylinders × 2** | **TODO_BENCH** — sized per arm geometry & target lift force | Double-acting; **TODO_BENCH bore** (3.0 in or 3.5 in typical for ~5,000–8,000 lbf lift at 2,500 PSI); **TODO_BENCH stroke** (depends on UWU dump-height target — see [AI NOTES/2026-01-25_Pivot_Mount_Assembly.md](../AI%20NOTES/2026-01-25_Pivot_Mount_Assembly.md) and [AI NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md](../AI%20NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md)); cross-tube mounts; SAE 8 ports; 3,000 PSI; chrome rod | Prince, Brand, Eaton, Custom Hoists. Surplus Center 4×8 or 4×10 cylinders if budget tight. | `HydraulicCylinderC` (double-acting); `A_1`, `A_2`, `s_l` parameters from final SKU. |
| **Bucket cylinder × 1** | **TODO_BENCH** — single cylinder for bucket curl/dump | Double-acting; smaller bore than arm cylinders (typically 2.5–3.0 in); shorter stroke (8–14 in); cross-tube or clevis; SAE 8 ports | Same vendors as arm cylinders. | `HydraulicCylinderC`; parameters TBD. |

---

## 6. Plumbing, filtration, cooling

*(Section numbering preserved across revisions: there is no §4 in the canonical BOM. The deleted §4 was a load-holding section required by the float-centre fallback; with tandem-centre spools per [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) it is no longer needed. The §4 hardware survives as the pre-BC-18 interim bill of materials documented in §3 above.)*


| Item | Canonical part | Key spec | Alternates | Hopsan mapping |
|---|---|---|---|---|
| **High-pressure hose** | **SAE 100R2 / EN 853 2SN, -8 (1/2 in ID) for pump-to-manifold and work lines** | 4,000 PSI working / 16,000 PSI burst on -8; -10 (5/8 in) for return | Parker 451TC, Gates G2, Eaton EC600 | Lumped pipe with friction loss in `HydraulicTubeF` if needed for transient analysis; usually omitted from steady-state sims. |
| **Hose fittings** | **SAE JIC 37° flare**, brass or steel | Reusable or crimp; sizes match hose IDs | Parker Triple-Lok, Eaton Aeroquip | N/A |
| **Suction line** | **SAE 100R4 -16 (1 in ID)** between reservoir and pump inlet | Low-pressure suction-rated; reinforced wire-helix to prevent collapse under vacuum | Generic suction hose | Modelled as ideal node in v0; refine if cavitation analysis needed. |
| **Return-line filter** | **In-tank or inline, 10 micron, 25 GPM rated, 25 PSI bypass** | Spin-on or cartridge element; pressure differential indicator; bypass to protect element on cold-start | Parker Racor, Donaldson P-CLAR series, Hydac RFL | Modelled as fixed pressure drop. |
| **Oil cooler** | **TODO_BENCH** — size for ~10–15% of total power as heat reject (~1.5–2.5 HP heat ≈ 4,000–6,500 BTU/hr) | Air-blast or fan-cooled; 30–40 GPM rated; 100 PSI burst; mounting brackets | AKG, Hayden, Setrab. Surplus Center fan-cooled units common. | Modelled as fixed thermal capacity in thermal-aware sims; out of scope for v0. |
| **Reservoir** | **TODO_BENCH** — target ~30 gallon (≈ 3 × pump GPM rule-of-thumb) | Steel weldment or polyethylene tank; sight gauge; 10 micron breather; baffle between suction and return; magnetic drain plug; cleanout cover | Buyer's Products UT-* steel tanks, custom weldment from frame steel | `HydraulicTankC`. |

**Hydraulic fluid:** ISO VG 46 anti-wear (AW46) hydraulic oil. ~30 gal initial fill plus ~5 gal change interval. NBR-compatible. Bio-based alternative (Mobil EAL EnviroSyn 46) is drop-in compatible and biodegradable for outdoor agricultural use.

---

## OPEN_SIZING_QUESTIONS

The following decisions block lock-in of the `TODO_BENCH` rows above. Each is one round of work or one operator decision.

1. **Exact pump SKU.** Need to verify Surplus Center / Northern Tool / Bailey Hydraulics in-stock inventory for an SAE-A, 1.4 cipr, 3,000 PSI gear pump. Candidates: Haldex Concentric 1001601, Parker PGP315, Casappa PLP10. Decision driver: lead time + price.
2. **Track motor displacement.** Depends on **track sprocket pitch diameter** (DESIGN-STRUCTURAL TBD) and target track speed (typical compact loader: 4–6 mph max). Once those land, displacement falls out of: $D_{\text{motor}} = \frac{\text{flow}_{\text{GPM}} \times 231}{\omega_{\text{sprocket, RPM}}}$.
3. **Arm cylinder bore + stroke.** Driven by [AI NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md](../AI%20NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md). Need final pivot geometry and target dump height to lock SKU.
4. **Bucket cylinder bore + stroke.** Driven by bucket geometry (bucket TBD in DESIGN-STRUCTURAL).
5. **Cooler sizing.** With tandem-centre spools and EFC ramp control (per [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md)) the pump idles through P→T at near-zero pressure — idle heat is negligible. Worst case is now sustained *operating* bypass when only one function is active: P_dump ≈ 250 PSI through EX port → ≈ 1.6 HP heat. Realistic cooler size: 6,500 BTU/hr fan-cooled. Confirm against ambient temperature spec (operator climate).
6. **Reservoir final volume + form factor.** 30 gal is the rule-of-thumb starting point; final volume depends on frame envelope (DESIGN-STRUCTURAL).
7. **Burkert 8605 vs Opta D1608S direct drive of EFC coil.** Check EFC coil current draw (Brand datasheet ~1.5–2 A typical) against Opta output rating. If Opta can drive directly, the Burkert is removable from the BOM and FLOW_VALVE_CONFIGURATION.md simplifies.
8. **Single vs dual EFC mode.** [FLOW_VALVE_CONFIGURATION.md](FLOW_VALVE_CONFIGURATION.md) treats this as a runtime D11-jumper choice. BOM currently lists single 10 GPM EFC as canonical; dual mode requires two 6 GPM EFCs + second Burkert. Final call deferred to bench testing.
9. **Tandem-pump architecture (deferred design alternative).** Switching from the current single-pump-with-EFC topology to a tandem gear pump (Honor Pumps 2DG1BU0606R-class, ~6+6 GPM SAE-A, ~$400) plus one EFC per pump section gives **inherent matched left/right track flow** without relying on EFC compensator balance — the standard skid-steer architecture used by Bobcat / John Deere / Cat compact track loaders. With tandem-centre spools (already canonical per §3) the implementation is clean: one series stack per pump section. Deferred until track-motor displacement and cylinder sizing land, because the per-section flow budget depends on those numbers. See chat session 2026-04-29 for full architectural analysis.

---

## Provenance

| Part | Source / page consulted | Date |
|---|---|---|
| Parker D1VW004CNKW (float, interim) | Hyspeco product listing + Grainger SKU 2NMU8 spec sheet | 2026-04-29 |
| Parker D1VW00*8*CNKW (tandem, canonical) | Parker D1VW family ordering code, spool-8 variant of decoded `004` SKU | 2026-04-29 |
| Brand Hydraulics EFC | brand-hyd.com/brand-product/efc/ | 2026-04-29 |
| Burkert 8605 | Existing [FLOW_VALVE_CONFIGURATION.md](FLOW_VALVE_CONFIGURATION.md) | (pre-existing) |
| B&S Vanguard 18 HP | shop.briggsandstratton.com Vanguard 305447/356447 family | 2026-04-29 |
| Soft-stop sequencing strategy | [SOFT_STOP_STRATEGY.md](SOFT_STOP_STRATEGY.md) (this round) | 2026-04-29 |
| Power-budget formula | Standard hydraulic power equation HP = GPM × PSI / 1714 | — |

This BOM will be the canonical parameter source for the [hopsan/](hopsan/) `.hmf` template work in BC-16.
