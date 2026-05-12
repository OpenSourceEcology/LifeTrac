#!/usr/bin/env python3
"""
analyze_rtt.py - W1-10b/W1-11 latency post-processor.

Reads the `tx_stdout.txt` + `rx_stdout.txt` artefacts produced by
`run_w1_10b_rx_pair_end_to_end.ps1` (or a manually-collected pair of
`method_h_stage2_tx_probe.py --probe tx_burst` / `--probe rx_listen`
captures) and computes the radio-link latency statistics needed to
confirm the planning estimates in
`DESIGN-CONTROLLER/RESEARCH-CONTROLLER/LATENCY_BUDGET.md` §1.

Specifically it extracts and characterizes:

  * **Time-on-air (toa_us)** — directly reported by the L072 firmware in
    every `__TX_DONE__` line. This is the row-#2 "LoRa TX air time"
    term in the budget. The PHY profile is currently SF7/BW125/CR4-5
    per `firmware/murata_l072/sx1276_modes.c::sx1276_modes_lora_init()`,
    so the budget prediction is ~30 ms (44 B on-air).

  * **Host-to-host TX-confirm latency (elapsed_ms)** — wall-clock
    measured by the TX-side probe between submitting the
    `TX_FRAME_REQ` (HostLink → L072 over LPUART) and receiving the
    `TX_DONE_URC` echo back. This subsumes budget rows #1 (encode), #2
    (air time), and the LPUART round-trip for the host control plane.

  * **RX inter-arrival jitter (timestamp_us delta)** — hardware
    timestamps captured by the L072 on PHY-IRQ. Should track the TX
    inter_cycle_s plus jitter on the order of one ToA. This is a
    cross-check that the receiver side does not introduce extra
    queueing delay.

Estimated **radio-link round-trip** (the only RTT meaningful at this
bench tier — there is no echo handler in firmware yet):

    rtt_radio_only_ms ≈ 2 * elapsed_ms_p50

Where `elapsed_ms` already includes 1× ToA + control-plane overhead.

What this **does not** measure (out of scope until the M7 / Opta /
hydraulic chain is on the bench): budget rows #5 (M7↔M4 IPC),
#6 (M4 100 Hz quant), #7 (Modbus 50 Hz slot), #8-#13 (RS-485, Opta,
SSR, coil, spool, cylinder).

Exit codes: 0 always when the analyzer ran (even if no frames were
captured); 2 only on a fatal CLI / I/O error. The W1-10b gates are
evaluated by the orchestrator, not here — this tool is informational.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

# Predicted SF7/BW125/CR4-5 ToA per LATENCY_BUDGET.md §1 row #2.
# Computed via the standard Semtech ToA formula for the canonical 44 B
# on-air control frame (8-symbol preamble, explicit header, CRC on,
# IH=0, LDRO off). Documented as "~30 ms" in the budget.
PREDICTED_TOA_SF7_BW125_MS = 30.0

# Predicted host control-plane overhead per LATENCY_BUDGET.md §1
# rows #1 + #3 + #4 (encode + demod + AES). The bench L072 probe path
# adds two LPUART hops (TX_FRAME_REQ down + TX_DONE_URC back) at
# 921600 8N1 (~0.04 ms / byte * ~16 B = ~0.6 ms each direction).
#
# **Bench-measured (W1-11 L-3 / 2026-05-12):** elapsed_ms - ToA across
# 2000 cycles at SF7/BW125 was p50 = 21 ms, p99 = 27 ms, p999 = 30 ms.
# That figure is the *bench-tier* HostLink round-trip (X8 ADB pipe +
# LPUART encode/decode + L072-side TX-cycle dispatch), NOT the
# production M7-direct path. The constant below is the bench p50; the
# production-path equivalent is still the ~5 ms originally pinned.
PREDICTED_HOST_OVERHEAD_MS = 21.0

# Phase-B observation window default; the budget gate is W4-00(b)
# "round-trip p50 within 5% of `lora_time_on_air_ms * 2 + 30 ms`".
# That 30 ms slop term is the M7 scheduler budget on the production
# Portenta path; on the bench L072 path the equivalent slop is the
# host-PC ADB / LPUART jitter, which is empirically ~10-30 ms.
W4_00B_SCHEDULER_SLOP_MS = 30.0


_TX_DONE_RE = re.compile(
    r"__TX_DONE__\s+idx=(?P<idx>\d+)\s+tx_id=0x(?P<tx_id>[0-9A-Fa-f]+)\s+"
    r"status=(?P<status>\d+)[^\n]*?toa_us=(?P<toa_us>\d+)[^\n]*?"
    r"elapsed_ms=(?P<elapsed_ms>[0-9]+(?:\.[0-9]+)?)[^\n]*?"
    r"payload_hex=(?P<payload_hex>[0-9A-Fa-f]+)"
)

_RX_FRAME_RE = re.compile(
    r"__RX_FRAME__\s+idx=(?P<idx>\d+)\s+rssi=(?P<rssi>-?\d+)\s+"
    r"snr=(?P<snr>-?\d+)\s+len=(?P<len>\d+)\s+"
    r"timestamp_us=(?P<timestamp_us>\d+)\s+payload_hex=(?P<payload_hex>[0-9A-Fa-f]+)"
)

# W1-11 L-1 ping-pong RTT (host-driven echo). One per cycle on the TX board.
_PINGPONG_RE = re.compile(
    r"__PINGPONG__\s+idx=(?P<idx>\d+)\s+tx_id=0x(?P<tx_id>[0-9A-Fa-f]+)\s+"
    r"status=(?P<status>\d+)[^\n]*?rtt_ms=(?P<rtt_ms>-?\d+(?:\.\d+)?)"
)


@dataclass
class TxRow:
    idx: int
    tx_id: int
    status: int
    toa_us: int
    elapsed_ms: float
    payload_hex: str


@dataclass
class RxRow:
    idx: int
    rssi: int
    snr: int
    length: int
    timestamp_us: int
    payload_hex: str


@dataclass
class Stats:
    n: int = 0
    min: float = 0.0
    p50: float = 0.0
    p90: float = 0.0
    p95: float = 0.0
    p99: float = 0.0
    p999: float = 0.0
    max: float = 0.0
    mean: float = 0.0
    stdev: float = 0.0
    samples: list[float] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "n": self.n,
            "min": round(self.min, 3),
            "p50": round(self.p50, 3),
            "p90": round(self.p90, 3),
            "p95": round(self.p95, 3),
            "p99": round(self.p99, 3),
            "p999": round(self.p999, 3),
            "max": round(self.max, 3),
            "mean": round(self.mean, 3),
            "stdev": round(self.stdev, 3),
        }


def _percentile(sorted_xs: list[float], pct: float) -> float:
    if not sorted_xs:
        return 0.0
    if len(sorted_xs) == 1:
        return sorted_xs[0]
    # Nearest-rank, conservative (round up). Fine for n in 100s.
    k = max(0, min(len(sorted_xs) - 1,
                   int(round(pct / 100.0 * (len(sorted_xs) - 1)))))
    return sorted_xs[k]


def summarize(values: Iterable[float]) -> Stats:
    xs = sorted(float(v) for v in values)
    if not xs:
        return Stats()
    return Stats(
        n=len(xs),
        min=xs[0],
        p50=_percentile(xs, 50),
        p90=_percentile(xs, 90),
        p95=_percentile(xs, 95),
        p99=_percentile(xs, 99),
        p999=_percentile(xs, 99.9),
        max=xs[-1],
        mean=statistics.fmean(xs),
        stdev=statistics.pstdev(xs) if len(xs) > 1 else 0.0,
        samples=xs,
    )


def parse_tx_dones(text: str) -> list[TxRow]:
    rows: list[TxRow] = []
    for m in _TX_DONE_RE.finditer(text):
        rows.append(TxRow(
            idx=int(m["idx"]),
            tx_id=int(m["tx_id"], 16),
            status=int(m["status"]),
            toa_us=int(m["toa_us"]),
            elapsed_ms=float(m["elapsed_ms"]),
            payload_hex=m["payload_hex"].lower(),
        ))
    return rows


def parse_rx_frames(text: str) -> list[RxRow]:
    rows: list[RxRow] = []
    for m in _RX_FRAME_RE.finditer(text):
        rows.append(RxRow(
            idx=int(m["idx"]),
            rssi=int(m["rssi"]),
            snr=int(m["snr"]),
            length=int(m["len"]),
            timestamp_us=int(m["timestamp_us"]),
            payload_hex=m["payload_hex"].lower(),
        ))
    return rows


def parse_pingpong_rtts(text: str) -> list[float]:
    """W1-11 L-1: extract per-cycle RTT (ms) from `__PINGPONG__` lines on
    the TX board's stdout. Cycles where the echo timed out emit
    `rtt_ms=-1` and are filtered out here.
    """
    rtts: list[float] = []
    for m in _PINGPONG_RE.finditer(text):
        v = float(m["rtt_ms"])
        if v >= 0:
            rtts.append(v)
    return rtts


def rx_inter_arrival_ms(rxs: list[RxRow]) -> list[float]:
    if len(rxs) < 2:
        return []
    sorted_rxs = sorted(rxs, key=lambda r: r.timestamp_us)
    deltas: list[float] = []
    for prev, curr in zip(sorted_rxs, sorted_rxs[1:]):
        delta_us = curr.timestamp_us - prev.timestamp_us
        # Guard against the L072 timestamp_us being a uint32 that wrapped
        # mid-burst (~71 minutes). Skip negative deltas.
        if delta_us > 0:
            deltas.append(delta_us / 1000.0)
    return deltas


def fmt_stats_row(label: str, s: Stats, unit: str = "ms") -> str:
    if s.n == 0:
        return f"  {label:<32s} (no samples)"
    return (f"  {label:<32s} n={s.n:<4d} "
            f"min={s.min:>7.2f} p50={s.p50:>7.2f} p90={s.p90:>7.2f} "
            f"p95={s.p95:>7.2f} p99={s.p99:>7.2f} p999={s.p999:>7.2f} "
            f"max={s.max:>7.2f} {unit}")


def build_report(evidence_dir: Path) -> tuple[str, dict]:
    tx_path = evidence_dir / "tx_stdout.txt"
    rx_path = evidence_dir / "rx_stdout.txt"
    tx_text = tx_path.read_text(encoding="utf-8", errors="replace") if tx_path.exists() else ""
    rx_text = rx_path.read_text(encoding="utf-8", errors="replace") if rx_path.exists() else ""

    txs = parse_tx_dones(tx_text)
    rxs = parse_rx_frames(rx_text)
    pingpong_rtts = parse_pingpong_rtts(tx_text)
    pingpong_ms = summarize(pingpong_rtts)

    toa_ms = summarize(t.toa_us / 1000.0 for t in txs if t.status == 0)
    elapsed_ms = summarize(t.elapsed_ms for t in txs if t.status == 0)
    iat_ms = summarize(rx_inter_arrival_ms(rxs))

    # Correlate RX frames back to TX frames by payload to compute
    # receive-side coverage at each percentile bucket of the TX-side
    # elapsed time. Useful for spotting "slow TX confirms also drop on
    # the air" patterns.
    tx_by_payload = {t.payload_hex: t for t in txs}
    matched_pairs = []
    for r in rxs:
        t = tx_by_payload.get(r.payload_hex)
        if t is not None:
            matched_pairs.append((t, r))

    # Estimated radio-link RTT = 2 * one-way host-perceived TX-confirm.
    # This is the bench-tier RTT surrogate; full RTT requires a firmware
    # echo handler (planned for the W4-00 production path on the H747).
    rtt_estimate_p50 = 2.0 * elapsed_ms.p50 if elapsed_ms.n else 0.0
    rtt_estimate_p99 = 2.0 * elapsed_ms.p99 if elapsed_ms.n else 0.0

    # Budget comparison.
    toa_match_pct = (
        100.0 * (toa_ms.p50 - PREDICTED_TOA_SF7_BW125_MS) / PREDICTED_TOA_SF7_BW125_MS
        if toa_ms.n else None
    )
    # W4-00(b): "p50 within 5% of `lora_time_on_air_ms * 2 + 30 ms`"
    # We don't have a paired-board echo, so apply the same gate to the
    # symmetric estimate (= 2 * elapsed_p50). The gate is informational
    # at this tier; W4-00 itself runs on the H747 production path.
    w4_00b_target_ms = 2.0 * (toa_ms.p50 if toa_ms.n else PREDICTED_TOA_SF7_BW125_MS) + W4_00B_SCHEDULER_SLOP_MS
    w4_00b_window_ms = 0.05 * w4_00b_target_ms
    w4_00b_within = (
        abs(rtt_estimate_p50 - w4_00b_target_ms) <= w4_00b_window_ms
        if elapsed_ms.n else None
    )

    # ----- text report -----
    lines: list[str] = []
    lines.append("=== W1-10b / W1-11 radio-link latency report ===")
    lines.append(f"Evidence dir: {evidence_dir}")
    lines.append(f"TX_DONE rows parsed: {len(txs)}  (status==OK: {sum(1 for t in txs if t.status == 0)})")
    lines.append(f"RX_FRAME rows parsed: {len(rxs)}  matched-to-TX-by-payload: {len(matched_pairs)}")
    lines.append("")
    lines.append("--- Time-on-air (LATENCY_BUDGET §1 row #2) ---")
    lines.append(fmt_stats_row("toa (firmware-reported)", toa_ms, "ms"))
    lines.append(f"  predicted SF7/BW125 ~{PREDICTED_TOA_SF7_BW125_MS:.1f} ms"
                 + (f"   measured-vs-predicted: {toa_match_pct:+.1f}%" if toa_match_pct is not None else ""))
    lines.append("")
    lines.append("--- Host-to-host TX-confirm (rows #1 + #2 + #3 + LPUART) ---")
    lines.append(fmt_stats_row("elapsed_ms (TX_FRAME_REQ -> TX_DONE_URC)",
                               elapsed_ms, "ms"))
    lines.append(f"  budget for this segment: ToA + ~{PREDICTED_HOST_OVERHEAD_MS:.1f} ms host overhead "
                 f"= ~{PREDICTED_TOA_SF7_BW125_MS + PREDICTED_HOST_OVERHEAD_MS:.1f} ms")
    lines.append("")
    lines.append("--- RX inter-arrival jitter (cross-check) ---")
    lines.append(fmt_stats_row("rx_iat (L072 hw timestamp_us)", iat_ms, "ms"))
    lines.append("  expected ~= TX inter_cycle_s + 1x ToA jitter")
    lines.append("")
    lines.append("--- Estimated radio-link RTT (bench-tier surrogate) ---")
    lines.append(f"  rtt_p50_estimate_ms = 2 * elapsed_p50 = {rtt_estimate_p50:.2f} ms")
    lines.append(f"  rtt_p99_estimate_ms = 2 * elapsed_p99 = {rtt_estimate_p99:.2f} ms")
    lines.append(f"  W4-00(b) target  = 2*toa_p50 + {W4_00B_SCHEDULER_SLOP_MS:.0f} ms slop "
                 f"= {w4_00b_target_ms:.2f} ms (+/- 5% = {w4_00b_window_ms:.2f} ms)")
    if w4_00b_within is None:
        lines.append("  W4-00(b) verdict  : NO_DATA")
    else:
        lines.append(f"  W4-00(b) verdict  : {'WITHIN_BUDGET' if w4_00b_within else 'OUTSIDE_BUDGET'}")
    lines.append("")
    if pingpong_ms.n:
        # W1-11 L-1: true (host-driven) ping-pong RTT, includes 2x ToA +
        # TX-side host overhead + RX-side host overhead. Production
        # firmware-echo RTT will be ~21 ms lower (one host overhead).
        lines.append("--- True ping-pong RTT (W1-11 L-1, host-driven echo) ---")
        lines.append(fmt_stats_row("pingpong_rtt_ms", pingpong_ms, "ms"))
        pp_target = w4_00b_target_ms
        pp_window = w4_00b_window_ms
        pp_within = abs(pingpong_ms.p50 - pp_target) <= pp_window
        lines.append(f"  W4-00(b) (true RTT) target = {pp_target:.2f} ms (+/- {pp_window:.2f} ms)")
        lines.append(f"  W4-00(b) (true RTT) verdict: {'WITHIN_BUDGET' if pp_within else 'OUTSIDE_BUDGET'}")
        lines.append("")
    lines.append("--- Reconciliation vs LATENCY_BUDGET.md §1 ---")
    lines.append("  This bench tier covers ONLY rows #1-#5 of the budget (encode +")
    lines.append("  air + demod + AES + IPC). Rows #6-#13 (M4 100 Hz quant, Modbus,")
    lines.append("  Opta SSR, coil ramp, spool shift, cylinder pressure) are NOT in")
    lines.append("  scope here and add the remaining ~70-160 ms typical / ~580 ms")
    lines.append("  worst case to the end-to-end stick->hydraulic figure.")
    lines.append("")

    # ----- JSON payload -----
    payload = {
        "evidence_dir": str(evidence_dir),
        "tx_done_count": len(txs),
        "tx_done_ok_count": sum(1 for t in txs if t.status == 0),
        "rx_frame_count": len(rxs),
        "matched_pair_count": len(matched_pairs),
        "toa_ms": toa_ms.to_dict(),
        "elapsed_ms": elapsed_ms.to_dict(),
        "rx_inter_arrival_ms": iat_ms.to_dict(),
        "rtt_estimate_ms": {
            "p50": round(rtt_estimate_p50, 3),
            "p99": round(rtt_estimate_p99, 3),
        },
        "budget_predictions": {
            "toa_sf7_bw125_ms": PREDICTED_TOA_SF7_BW125_MS,
            "host_overhead_ms": PREDICTED_HOST_OVERHEAD_MS,
            "w4_00b_target_ms": round(w4_00b_target_ms, 3),
            "w4_00b_window_ms": round(w4_00b_window_ms, 3),
        },
        "verdicts": {
            "toa_match_pct": (round(toa_match_pct, 2) if toa_match_pct is not None else None),
            "w4_00b_within_5pct": w4_00b_within,
            "w4_00b_within_5pct_pingpong": (
                bool(abs(pingpong_ms.p50 - w4_00b_target_ms) <= w4_00b_window_ms)
                if pingpong_ms.n else None
            ),
        },
        "pingpong_rtt_ms": pingpong_ms.to_dict() if pingpong_ms.n else None,
    }

    return "\n".join(lines), payload


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Post-process a W1-10b RX-pair evidence dir into a "
                    "radio-link latency report (LATENCY_BUDGET.md §1 "
                    "rows #1-#5 confirmation).")
    parser.add_argument("evidence_dir", type=Path,
                        help="Path to a `bench-evidence/W1-10b_rx_pair_<stamp>/` "
                             "directory (must contain tx_stdout.txt and "
                             "rx_stdout.txt).")
    parser.add_argument("--out-md", type=Path, default=None,
                        help="Markdown report path (default: <evidence_dir>/rtt_report.md)")
    parser.add_argument("--out-json", type=Path, default=None,
                        help="JSON report path (default: <evidence_dir>/rtt_report.json)")
    parser.add_argument("--no-stdout", action="store_true",
                        help="Suppress the human-readable report on stdout.")
    parser.add_argument("--merge-summary", action="store_true",
                        help="Merge the latency payload into <evidence_dir>/summary.json "
                             "under a `latency` key (orchestrator-friendly; non-fatal "
                             "if summary.json is absent or unreadable).")
    args = parser.parse_args(argv)

    if not args.evidence_dir.is_dir():
        print(f"FATAL: not a directory: {args.evidence_dir}", file=sys.stderr)
        return 2

    out_md = args.out_md or (args.evidence_dir / "rtt_report.md")
    out_json = args.out_json or (args.evidence_dir / "rtt_report.json")

    text, payload = build_report(args.evidence_dir)

    out_md.write_text(text + "\n", encoding="utf-8")
    out_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    merged_path = None
    if args.merge_summary:
        summary_path = args.evidence_dir / "summary.json"
        if summary_path.exists():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as e:
                print(f"WARN: cannot read {summary_path}: {e}", file=sys.stderr)
                summary = None
            if isinstance(summary, dict):
                summary["latency"] = payload
                summary_path.write_text(
                    json.dumps(summary, indent=2) + "\n", encoding="utf-8")
                merged_path = summary_path
        else:
            print(f"WARN: --merge-summary set but {summary_path} not found", file=sys.stderr)

    if not args.no_stdout:
        print(text)
        print(f"Report written: {out_md}")
        print(f"JSON written:   {out_json}")
        if merged_path is not None:
            print(f"Merged into:    {merged_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
