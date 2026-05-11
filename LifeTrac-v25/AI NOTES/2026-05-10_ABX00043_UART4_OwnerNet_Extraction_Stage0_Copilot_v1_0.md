# 2026-05-10 ABX00043 UART4 Owner-Net Extraction Stage0 (Copilot v1.0)

## Objective

Convert the open blocker from heuristic lane-group forcing to an evidence-backed owner-net map for the i.MX UART4 host path (`/dev/ttymxc3`) into the Murata CMWX1ZZABZ-078 L072 ingress.

## Evidence source used in this pass

- Arduino official user manual page for Portenta Max Carrier:
  - `https://docs.arduino.cc/tutorials/portenta-max-carrier/user-manual`

## Newly extracted, actionable topology facts

From the official user manual carrier topology table:

- `U23` is the Murata LPWAN module (`CMWX1ZZABZ-078`).
- `U16-U19` are `74LVC1G157` single 2-input multiplexers.
- `U8/U20/U21/U22` are `SN74LVC1T45` bi-directional level converters.
- `U10` is `SN74LVC1G125` single bus buffer gate.
- The board includes CN2/CN3 debug paths and explicit "sniffer channels" for UART lines.

## Interpretation for current blocker

The device inventory strongly implies that host-to-L072 direction can be controlled by one or more active gates (mux/level-shift/buffer), which is consistent with observed one-way behavior:

- ROM mode reachable and writable.
- User mode emits outbound traffic.
- Host ingress remains absent under Method G active probing.

This aligns with a route-control ownership problem (OE/SEL/EN ownership), not a pure framing or termios-only issue.

## Stage0 owner-net worksheet (to complete with schematic net names)

For each candidate component on the UART4-to-L072 route:

1. Record designator (`U16..U19`, `U8/U20/U21/U22`, `U10`).
2. Record function on path (mux / level-shift / buffer).
3. Record signal-side nets:
   - i.MX side TX/RX net names
   - Murata/L072 side TX/RX net names
4. Record control nets (OE/EN/SEL/DIR) and default strap state.
5. Record owner for each control net (i.MX GPIO, H747 GPIO, fixed strap, always-on logic).
6. Record expected polarity for "ingress enabled" state.

## Immediate experiment strategy once worksheet is filled

Narrow OpenOCD matrix to only owner nets that directly gate host->L072 ingress:

1. Baseline owner state replay (as-found state).
2. Single-net polarity flips, one at a time.
3. Pairwise owner-net combinations only when single-net flips are inconclusive.
4. For any signature change, run immediate repeat (`r2`) before accepting discriminator.

## Go criterion for this extraction stage

Move blocker from heuristic to evidence-backed control once all of the following are true:

- Exact schematic net names for i.MX UART4_TX/RX and Murata-side counterparts are recorded.
- Every control net on the path has an owner and polarity classification.
- Next OpenOCD matrix references only those owner nets.

## Current status

- Stage0 topology extraction: complete.
- Net-name and control-owner extraction: pending (requires direct schematic net annotation pass).
