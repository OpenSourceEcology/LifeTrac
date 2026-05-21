"""Loss-pattern analysis for 2026-05-20 RF runs (T1/T2/T5).

Pure stdlib. Reads existing evidence dirs and prints:
  - dropped TX seq numbers (T1/T2) or fragment idx (T5)
  - position-in-burst distribution
  - RSSI/SNR distributions
  - inter-arrival outliers (potential duty-cycle backoff or retry)
"""
from __future__ import annotations
import re
import statistics
from pathlib import Path

ROOT = Path(__file__).resolve().parent

TX_LINE = re.compile(
    r"__TX_DONE__ idx=(\d+) tx_id=0x([0-9a-fA-F]+) status=(\d+)\([A-Z]+\) "
    r"toa_us=(\d+) elapsed_ms=([\d.]+) payload_hex=([0-9a-fA-F]+)"
)
RX_LINE = re.compile(
    r"__RX_FRAME__ idx=(\d+) rssi=(-?\d+) snr=(-?\d+) len=(\d+) "
    r"timestamp_us=(\d+) payload_hex=([0-9a-fA-F]+)"
)
SEQ_ASCII = re.compile(r"seq=(\d{4})")


def parse_tx(path: Path):
    if not path.exists():
        return []
    out = []
    for line in path.read_text(errors="replace").splitlines():
        m = TX_LINE.search(line)
        if m:
            out.append({
                "burst_idx": int(m.group(1)),
                "elapsed_ms": float(m.group(5)),
                "payload_hex": m.group(6),
            })
    return out


def parse_rx(path: Path):
    if not path.exists():
        return []
    out = []
    for line in path.read_text(errors="replace").splitlines():
        m = RX_LINE.search(line)
        if m:
            out.append({
                "rx_idx": int(m.group(1)),
                "rssi": int(m.group(2)),
                "snr": int(m.group(3)),
                "ts_us": int(m.group(5)),
                "payload_hex": m.group(6),
            })
    return out


def w1_seq(payload_hex: str):
    try:
        s = bytes.fromhex(payload_hex).decode("ascii", errors="replace")
    except ValueError:
        return None
    m = SEQ_ASCII.search(s)
    return int(m.group(1)) if m else None


def w2_frag(payload_hex: str):
    try:
        b = bytes.fromhex(payload_hex)
    except ValueError:
        return None, None
    if len(b) < 3 or b[0] != 0xFE:
        return None, None
    return b[1], b[2]


def report_w1(label: str, evdir: Path, tx_log: Path, rx_log: Path):
    print(f"\n{'=' * 72}\n{label}  ({evdir.name})\n{'=' * 72}")
    tx = parse_tx(tx_log)
    rx = parse_rx(rx_log)
    print(f"  tx_log={tx_log.name} TX_DONE={len(tx)}")
    print(f"  rx_log={rx_log.name} RX_FRAME={len(rx)}")

    tx_seqs = [w1_seq(t["payload_hex"]) for t in tx]
    rx_seqs = [w1_seq(r["payload_hex"]) for r in rx]
    tx_set = {s for s in tx_seqs if s is not None}
    rx_set = {s for s in rx_seqs if s is not None}
    dropped = sorted(tx_set - rx_set)
    orphan = sorted(rx_set - tx_set)
    print(f"  TX unique seqs={len(tx_set)} RX unique seqs={len(rx_set)}")
    print(f"  DROPPED seqs ({len(dropped)}): {dropped}")
    print(f"  ORPHAN  seqs ({len(orphan)}): {orphan}")

    if dropped and tx_set:
        lo, hi = min(tx_set), max(tx_set)
        n = hi - lo + 1
        first10 = sum(1 for d in dropped if d - lo < 10)
        last10 = sum(1 for d in dropped if hi - d < 10)
        middle = len(dropped) - first10 - last10
        print(f"  Position: first10={first10}  middle={middle}  last10={last10} (total burst={n})")

    if rx:
        rssis = sorted(r["rssi"] for r in rx)
        snrs = sorted(r["snr"] for r in rx)
        print(f"  RSSI min={rssis[0]} p10={rssis[len(rssis)//10]} med={statistics.median(rssis)} p90={rssis[-len(rssis)//10]} max={rssis[-1]}")
        print(f"  SNR  min={snrs[0]} med={statistics.median(snrs)} max={snrs[-1]}")

    if len(rx) >= 3:
        deltas = [(rx[i + 1]["ts_us"] - rx[i]["ts_us"]) / 1000.0 for i in range(len(rx) - 1)]
        med = statistics.median(deltas)
        outliers = [(i, d) for i, d in enumerate(deltas) if d > 1.5 * med]
        print(f"  Inter-arrival median={med:.1f} ms; outliers (>1.5x med) = {len(outliers)}")
        for i, d in outliers[:10]:
            sb, sa = w1_seq(rx[i]["payload_hex"]), w1_seq(rx[i + 1]["payload_hex"])
            gap = (sa - sb) if (sb is not None and sa is not None) else None
            print(f"    after rx_idx={rx[i]['rx_idx']:>3}  seq {sb}->{sa}  gap={gap}  delta={d:.1f} ms")

    if tx:
        el = sorted(t["elapsed_ms"] for t in tx)
        med_el = statistics.median(el)
        out_el = [(i, t) for i, t in enumerate(tx) if t["elapsed_ms"] > 1.5 * med_el]
        print(f"  TX elapsed_ms median={med_el:.1f}; outliers (>1.5x med) = {len(out_el)}")
        for i, t in out_el[:5]:
            s = w1_seq(t["payload_hex"])
            print(f"    burst_idx={t['burst_idx']} seq={s} elapsed={t['elapsed_ms']:.1f} ms")


def report_w2(label: str, evdir: Path):
    print(f"\n{'=' * 72}\n{label}  ({evdir.name})\n{'=' * 72}")
    rx = parse_rx(evdir / "rx_stdout.txt")
    print(f"  rx_stdout RX_FRAME={len(rx)}")
    if not rx:
        return
    frag_idxs = []
    by_frame = {}
    for r in rx:
        fr, fg = w2_frag(r["payload_hex"])
        frag_idxs.append((fr, fg))
        if fr is None:
            continue
        by_frame.setdefault(fr, set()).add(fg)
    for fr, frags in by_frame.items():
        if not frags:
            continue
        max_frag = max(frags)
        expected = set(range(0, max_frag + 1))
        missing = sorted(expected - frags)
        print(f"  frame seq=0x{fr:02x} fragments rx={len(frags)} highest_idx=0x{max_frag:02x} ({max_frag})")
        print(f"    missing frag idx: {missing}  (HEADER MISSING={0 in missing})")

    rssis = sorted(r["rssi"] for r in rx)
    snrs = sorted(r["snr"] for r in rx)
    print(f"  RSSI min={rssis[0]} med={statistics.median(rssis)} max={rssis[-1]}")
    print(f"  SNR  min={snrs[0]} med={statistics.median(snrs)} max={snrs[-1]}")

    deltas = [(rx[i + 1]["ts_us"] - rx[i]["ts_us"]) / 1000.0 for i in range(len(rx) - 1)]
    if deltas:
        med = statistics.median(deltas)
        outliers = [(i, d) for i, d in enumerate(deltas) if d > 1.5 * med]
        print(f"  Inter-arrival median={med:.1f} ms; outliers={len(outliers)}")
        for i, d in outliers[:10]:
            fb = w2_frag(rx[i]["payload_hex"])
            fa = w2_frag(rx[i + 1]["payload_hex"])
            print(f"    after rx_idx={rx[i]['rx_idx']:>3} frag {fb}->{fa}  delta={d:.1f} ms")


def main():
    base = ROOT
    # T1
    d1 = base / "W1-10b_rx_pair_2026-05-20_183616"
    report_w1("T1 W1-10b tx_burst", d1,
              d1 / "2D0A1209DABC240B_method_h_stage2_tx_burst.log",
              d1 / "2E2C1209DABC240B_method_h_stage2_rx_listen.log")
    # T2
    d2 = base / "W1-11_pingpong_2026-05-20_183815"
    report_w1("T2 W1-11 ping_pong", d2,
              d2 / "2D0A1209DABC240B_method_h_stage2_ping_pong.log",
              d2 / "2E2C1209DABC240B_method_h_stage2_rx_echo.log")
    # T5
    d5 = base / "W2-02_image_over_lora_2026-05-20_184021"
    report_w2("T5 W2-02 image-over-LoRa", d5)


if __name__ == "__main__":
    main()
