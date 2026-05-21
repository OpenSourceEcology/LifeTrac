"""Walk-power falsification matrix analyzer (P1-4 / v3.0 §2 / v4.0 §3).

Reads the 3-pass output of `tools/walk_power_falsification_matrix.ps1`
(passes A, B, C) and prints a decision-rule verdict per the doc.

Pure stdlib. Run:
    py -3 LifeTrac-v25/tools/walk_power_matrix_analyze.py <matrix_dir>

where <matrix_dir> contains:
    matrix_meta.json
    pass_A/walk_power_tx_side.csv
    pass_A/walk_power_tx_side_perpacket.csv
    pass_B/...
    pass_C/...

Outputs a `verdict.md` next to matrix_meta.json so the AI NOTES doc can
cite the verdict file directly without re-running the analyzer.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path
from typing import Optional

# Cliff definition: a step has tx_per_pct > CLIFF_HIGH while the previous
# step was <= CLIFF_LOW. Matches what the pilot reported informally.
CLIFF_HIGH = 5.0
CLIFF_LOW = 1.0


def load_step_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def detect_cliff(rows: list[dict]) -> Optional[dict]:
    """Return the first cliff transition or None."""
    prev_per = None
    prev_dbm = None
    for r in rows:
        try:
            per = float(r.get("tx_per_pct", "") or 0.0)
            dbm = int(r.get("power_dbm_requested", "") or 0)
        except ValueError:
            continue
        if prev_per is not None and prev_per <= CLIFF_LOW and per > CLIFF_HIGH:
            return {
                "from_dbm": prev_dbm, "from_per_pct": prev_per,
                "to_dbm": dbm,        "to_per_pct": per,
                "abort_lbt_delta": int(r.get("radio_tx_abort_lbt_delta") or 0),
                "abort_air_delta": int(r.get("radio_tx_abort_airtime_delta") or 0),
            }
        prev_per = per
        prev_dbm = dbm
    return None


def per_pass_summary(pass_dir: Path) -> dict:
    step_csv = pass_dir / "walk_power_tx_side.csv"
    rows = load_step_csv(step_csv)
    cliff = detect_cliff(rows)
    total_pkts = sum(int(r.get("count_sent") or 0) for r in rows)
    total_ok   = sum(int(r.get("tx_done_ok") or 0) for r in rows)
    total_lbt  = sum(int(r.get("radio_tx_abort_lbt_delta") or 0) for r in rows)
    overall_per = (100.0 * (total_pkts - total_ok) / total_pkts) if total_pkts else 0.0
    return {
        "pass_dir": str(pass_dir),
        "row_count": len(rows),
        "total_packets": total_pkts,
        "overall_per_pct": round(overall_per, 3),
        "total_lbt_aborts": total_lbt,
        "cliff": cliff,
    }


def render_verdict(meta: dict, summaries: dict[str, dict]) -> str:
    out = []
    out.append("# Walk-power falsification matrix verdict")
    out.append("")
    out.append(f"- run_uuid: `{meta.get('run_uuid','?')}`")
    out.append(f"- git_sha_short: `{meta.get('git_sha_short','?')}`")
    out.append(f"- run_start_ts_local: `{meta.get('run_start_ts_local','?')}`")
    out.append("")
    out.append("## Per-pass summary")
    out.append("")
    out.append("| Pass | LBT | inter_s | total pkts | overall PER % | LBT aborts | Cliff |")
    out.append("|------|-----|---------|------------|---------------|------------|-------|")
    cliff_map = {}
    for p in meta.get("passes", []):
        name = p["name"]
        s = summaries.get(name, {})
        cliff_map[name] = s.get("cliff")
        c = s.get("cliff")
        cliff_str = (f"{c['from_dbm']}->{c['to_dbm']} dBm "
                     f"({c['from_per_pct']:.1f}->{c['to_per_pct']:.1f} %)"
                     if c else "none")
        out.append(f"| {name} | {p['lbt']} | {p['inter_cycle_s']} | "
                   f"{s.get('total_packets','?')} | "
                   f"{s.get('overall_per_pct','?')} | "
                   f"{s.get('total_lbt_aborts','?')} | {cliff_str} |")
    out.append("")

    a, b, c = cliff_map.get("A"), cliff_map.get("B"), cliff_map.get("C")
    out.append("## Decision rule application")
    out.append("")
    verdict_lines: list[str] = []

    # Host-cadence falsification
    if a is None and c is not None:
        verdict_lines.append(
            "- **Host-cadence hypothesis**: A clean, C cliff -> only differing "
            "variable is `inter_cycle_s` (A=0.05, C=0.02). "
            "**CONFIRMED**: the cliff IS sensitive to host cadence at the "
            "20 ms tick. (May still be downstream of a real RF effect that "
            "shows up only when ToA + cadence exceed a duty-cycle threshold.)"
        )
    elif a is not None and c is not None:
        verdict_lines.append(
            "- **Host-cadence hypothesis**: cliff persists at 50 ms inter-packet "
            "(>>> ToA + host overhead). **FALSIFIED**: cadence is not the "
            "root cause; a real RF/PA/firmware mechanism must be present."
        )
    else:
        verdict_lines.append(
            "- **Host-cadence hypothesis**: inconclusive (missing pass A or C)."
        )

    # LBT falsification
    if a is None and b is not None:
        b_lbt_jumps = (b.get("abort_lbt_delta", 0) > 0) if b else False
        if b_lbt_jumps:
            verdict_lines.append(
                "- **LBT-defer hypothesis**: A clean, B cliff with "
                f"`radio_tx_abort_lbt_delta={b.get('abort_lbt_delta')}` at the "
                "cliff step. **CONFIRMED**: enabling LBT reproduces the cliff "
                "and the LBT counter advances at the cliff."
            )
        else:
            verdict_lines.append(
                "- **LBT-defer hypothesis**: A clean, B cliff BUT no LBT abort "
                "counter jump. Suggests B's cliff is coincidental noise, not "
                "an LBT defer storm."
            )
    elif a is not None and b is not None:
        verdict_lines.append(
            "- **LBT-defer hypothesis**: A also has a cliff, so we cannot "
            "isolate LBT against the cadence-controlled baseline. Re-run with "
            "tighter inter-cycle isolation."
        )
    elif a is None and b is None:
        verdict_lines.append(
            "- **LBT-defer hypothesis**: neither LBT-off nor LBT-on (both at "
            "50 ms cadence) shows a cliff. **FALSIFIED**: LBT is not the "
            "trigger. The legacy cliff in pass C is a 20 ms-cadence artifact."
        )
    else:
        verdict_lines.append(
            "- **LBT-defer hypothesis**: inconclusive (missing pass A or B)."
        )

    # Combined rule: if A == B == C all clean, the whole walk_power pilot
    # cliff was a one-off / equipment issue.
    if a is None and b is None and c is None:
        verdict_lines.append(
            "- **Overall**: no cliff in any of the three passes. The original "
            "pilot cliff was not reproducible; treat the pilot finding as a "
            "transient (likely supply, antenna, or peer drift)."
        )

    # Combined rule: real RF
    if a is not None and b is not None and c is not None:
        b_lbt_flat = (b.get("abort_lbt_delta", 0) == 0) if b else True
        if b_lbt_flat:
            verdict_lines.append(
                "- **Overall**: cliff in all three passes AND no LBT counter "
                "jump. Cadence falsified + LBT falsified -> escalate to "
                "firmware/RF physical (PA compression, supply sag, calibration "
                "table miss). Recommend: CCIPR + Vdd_RF telemetry + "
                "PA_CONFIG/RegOcp dump at the cliff step."
            )

    out.extend(verdict_lines)
    out.append("")
    return "\n".join(out)


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: walk_power_matrix_analyze.py <matrix_dir>")
        return 2
    matrix_dir = Path(argv[1]).resolve()
    meta_path = matrix_dir / "matrix_meta.json"
    if not meta_path.exists():
        print(f"ERROR: {meta_path} not found")
        return 2
    meta = json.loads(meta_path.read_text(encoding="utf-8"))
    summaries: dict[str, dict] = {}
    for p in meta.get("passes", []):
        name = p["name"]
        pass_dir = matrix_dir / f"pass_{name}"
        summaries[name] = per_pass_summary(pass_dir)
    report = render_verdict(meta, summaries)
    out_path = matrix_dir / "verdict.md"
    out_path.write_text(report, encoding="utf-8")
    print(report)
    print(f"\nWrote: {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
