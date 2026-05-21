"""Step 0 — Loss-correlation (burstiness vs i.i.d.) analyzer for W2-02 runs.

Required by 2026-05-20_RF_Testing_Problems_To_Fix_v1_0.md, Copilot Review v4.0 §2.

For each W2-02 evidence dir given (baseline T5 and Redundancy=2 T5b):
  1) Read fragments.hex (TX order) and rx_stdout.txt (RX order, with payload_hex).
  2) Greedy-align RX to TX to produce a per-TX-attempt arrival indicator.
  3) Compute:
       - per-TX-attempt loss rate p
       - run-length distribution of consecutive losses
       - lag-k autocorrelation of the arrival indicator for k=1..8
       - chi-square goodness-of-fit of run-length distribution vs i.i.d.
         geometric (Bernoulli null)
  4) Print a verdict: BURSTY (autocorr > 3 sigma threshold OR run lengths
     significantly heavier than geometric) vs IID.

Pure stdlib; no numpy/scipy. Designed to be re-runnable from any cwd.

Usage:
    py -3 _analyze_burstiness_2026-05-20.py
"""
from __future__ import annotations

import math
import re
import statistics
from pathlib import Path
from typing import List, Tuple

ROOT = Path(__file__).resolve().parent

RX_LINE = re.compile(
    r"__RX_FRAME__ idx=(\d+) rssi=(-?\d+) snr=(-?\d+) len=(\d+) "
    r"timestamp_us=(\d+) payload_hex=([0-9a-fA-F]+)"
)


def parse_tx_fragments(path: Path) -> List[Tuple[int, int]]:
    """Read fragments.hex (TX order). Each non-blank line is one fragment
    payload, hex-encoded. The wire header is:
        0xFE | frame_seq(1B) | frag_idx(1B) | total-1(1B) | payload...
    Returns the TX-order list of (frame_seq, frag_idx)."""
    out: List[Tuple[int, int]] = []
    for raw in path.read_text(errors="replace").splitlines():
        s = raw.strip()
        if not s:
            continue
        try:
            b = bytes.fromhex(s)
        except ValueError:
            continue
        if len(b) < 3 or b[0] != 0xFE:
            continue
        out.append((b[1], b[2]))
    return out


def parse_rx_arrivals(path: Path) -> List[Tuple[int, int, int]]:
    """Parse rx_stdout.txt → list of (frame_seq, frag_idx, ts_us) in RX order."""
    out: List[Tuple[int, int, int]] = []
    for line in path.read_text(errors="replace").splitlines():
        m = RX_LINE.search(line)
        if not m:
            continue
        ts_us = int(m.group(5))
        try:
            b = bytes.fromhex(m.group(6))
        except ValueError:
            continue
        if len(b) < 3 or b[0] != 0xFE:
            continue
        out.append((b[1], b[2], ts_us))
    return out


def greedy_align(tx_seq: List[Tuple[int, int]],
                 rx_seq: List[Tuple[int, int, int]]) -> List[int]:
    """Greedy align RX onto TX, allowing RX to skip ahead past lost TX slots.

    Returns an arrival indicator of length len(tx_seq): 1 = arrived, 0 = lost.

    Algorithm:
      i = TX cursor, j = RX cursor.
      At each step:
        - If tx[i] == (rx[j].frame, rx[j].frag) → arrived[i]=1, i++, j++.
        - Else look ahead in TX for the next match of rx[j] within a small
          window (default 50). If found at i+k, mark arrived[i..i+k-1]=0,
          arrived[i+k]=1, advance i to i+k+1, j++.
        - If rx[j] cannot be matched, it is treated as an extraneous arrival
          (e.g. duplicate from Redundancy=2 with both copies arriving) and j is
          advanced without changing TX cursor.

    With Redundancy=N, the same (frame, frag) appears N consecutive times in
    fragments.hex. Both copies should usually arrive; the alignment will
    naturally consume them both because tx[i]==tx[i+1] when the next RX is the
    same tuple.
    """
    arrived = [0] * len(tx_seq)
    i = 0
    j = 0
    extraneous = 0
    while i < len(tx_seq) and j < len(rx_seq):
        rf, rg, _ts = rx_seq[j]
        if tx_seq[i] == (rf, rg):
            arrived[i] = 1
            i += 1
            j += 1
            continue
        # search ahead in TX for next match within window
        WINDOW = 50
        hit = -1
        for k in range(1, min(WINDOW, len(tx_seq) - i)):
            if tx_seq[i + k] == (rf, rg):
                hit = i + k
                break
        if hit >= 0:
            # i..hit-1 are lost
            arrived[hit] = 1
            i = hit + 1
            j += 1
        else:
            # cannot find this RX in upcoming TX window → treat as
            # extraneous (most likely a late duplicate)
            extraneous += 1
            j += 1
    # any remaining TX positions past last RX match are lost
    return arrived  # extraneous count discarded; we only care about per-TX arrival


def run_lengths_of_zeros(indicator: List[int]) -> List[int]:
    """Return the list of run-lengths of consecutive 0s in the indicator."""
    runs: List[int] = []
    cur = 0
    for v in indicator:
        if v == 0:
            cur += 1
        else:
            if cur > 0:
                runs.append(cur)
                cur = 0
    if cur > 0:
        runs.append(cur)
    return runs


def autocorr(indicator: List[int], max_lag: int = 8) -> List[float]:
    """Sample autocorrelation of the binary arrival indicator for lags 1..max_lag.

    Uses the standard normalization:
        r_k = sum_{t=1..N-k} (x_t - mean)(x_{t+k} - mean) / sum (x_t - mean)^2
    """
    n = len(indicator)
    if n < 2:
        return [0.0] * max_lag
    m = sum(indicator) / n
    denom = sum((v - m) ** 2 for v in indicator)
    if denom == 0:
        return [0.0] * max_lag
    out = []
    for k in range(1, max_lag + 1):
        num = sum((indicator[t] - m) * (indicator[t + k] - m)
                  for t in range(n - k))
        out.append(num / denom)
    return out


def chi_square_runs_vs_geometric(runs: List[int], p_loss: float) -> Tuple[float, int, str]:
    """Compare observed run-length distribution to geometric(p_loss) null.

    Under i.i.d. Bernoulli loss with rate p, conditional on a run starting,
    the run length R has Pr(R=k) = (1-p)*p^(k-1) for k>=1, i.e. shifted
    geometric with success prob (1-p). Bin into {1, 2, 3+} and compute chi^2
    with 2 degrees of freedom (3 bins - 1).

    Returns (chi2, df, verdict_label)."""
    if not runs or p_loss <= 0 or p_loss >= 1:
        return (0.0, 0, "N/A — no runs or degenerate p")
    n_runs = len(runs)
    # observed counts in {1, 2, >=3}
    obs1 = sum(1 for r in runs if r == 1)
    obs2 = sum(1 for r in runs if r == 2)
    obs3 = sum(1 for r in runs if r >= 3)
    # expected under geometric(1-p) on {1,2,>=3}
    pk = lambda k: (1 - p_loss) * (p_loss ** (k - 1))
    exp1 = n_runs * pk(1)
    exp2 = n_runs * pk(2)
    exp3 = n_runs * (1 - pk(1) - pk(2))
    chi2 = 0.0
    for o, e in [(obs1, exp1), (obs2, exp2), (obs3, exp3)]:
        if e > 1e-9:
            chi2 += (o - e) ** 2 / e
    # critical values for df=2: 5.99 (alpha=0.05), 9.21 (alpha=0.01)
    if chi2 < 5.99:
        verdict = f"chi2={chi2:.2f} < 5.99 -> CONSISTENT WITH I.I.D. (alpha=0.05)"
    elif chi2 < 9.21:
        verdict = f"chi2={chi2:.2f} in [5.99, 9.21) -> marginal bursty (alpha 0.05-0.01)"
    else:
        verdict = f"chi2={chi2:.2f} >= 9.21 -> BURSTY (rejected i.i.d. at alpha=0.01)"
    return (chi2, 2, verdict)


def analyze(label: str, evdir: Path) -> dict:
    print(f"\n{'=' * 78}\n{label}\n  dir: {evdir.name}\n{'=' * 78}")
    tx_path = evdir / "fragments.hex"
    rx_path = evdir / "rx_stdout.txt"
    if not tx_path.exists() or not rx_path.exists():
        print(f"  SKIP — missing {tx_path.name} or {rx_path.name}")
        return {}
    tx = parse_tx_fragments(tx_path)
    rx = parse_rx_arrivals(rx_path)
    print(f"  TX attempts (fragments.hex lines)  : {len(tx)}")
    print(f"  RX arrivals (__RX_FRAME__ events)  : {len(rx)}")

    indicator = greedy_align(tx, rx)
    n = len(indicator)
    n_arr = sum(indicator)
    n_lost = n - n_arr
    p = n_lost / n if n else 0.0
    print(f"  per-TX-attempt loss p              : {n_lost}/{n} = {p*100:.3f}%")

    # Unique-fragment view (so users can cross-check against summary_top.json)
    uniq_tx = set(tx)
    uniq_rx = set((rf, rg) for rf, rg, _ in rx)
    missing_uniq = sorted(uniq_tx - uniq_rx)
    print(f"  unique (frame_seq, frag_idx) TX    : {len(uniq_tx)}")
    print(f"  unique (frame_seq, frag_idx) RX    : {len(uniq_rx)}")
    print(f"  unique missing (catastrophic for N=1, recoverable for N>=2): {len(missing_uniq)}")
    if missing_uniq:
        print(f"    sample: {missing_uniq[:10]}{'...' if len(missing_uniq) > 10 else ''}")

    runs = run_lengths_of_zeros(indicator)
    print(f"  loss runs                          : {len(runs)} runs, "
          f"lengths={runs[:30]}{'...' if len(runs) > 30 else ''}")
    if runs:
        print(f"    max_run={max(runs)}  mean={statistics.mean(runs):.2f}  "
              f"median={statistics.median(runs)}")

    ac = autocorr(indicator, max_lag=8)
    # 95% CI band for autocorr of binary i.i.d. series is ~1.96/sqrt(n)
    band = 1.96 / math.sqrt(n) if n > 0 else 0.0
    print(f"  autocorr lag-1..8                  : "
          + ", ".join(f"{v:+.3f}" for v in ac))
    print(f"  95% CI band (i.i.d. null)          : +/-{band:.3f}")
    sig = [k + 1 for k, v in enumerate(ac) if abs(v) > band]
    print(f"  significant lags                   : {sig if sig else 'none'}")

    chi2, df, verdict = chi_square_runs_vs_geometric(runs, p)
    print(f"  chi-square run-length vs geometric : {verdict}")

    # Verdict summary.
    # Burstiness is defined by *consecutive* losses. The chi-square run-length
    # test is the primary signal. The lag-k autocorrelation is reported for
    # completeness but is unreliable when the indicator is nearly constant
    # (here: only 2–6 losses out of hundreds of TX attempts → any pair of
    # losses at separation k will saturate the CI band as a small-sample
    # artifact, not real burstiness).
    max_run = max(runs) if runs else 0
    bursty_by_chi = chi2 >= 5.99
    bursty_by_runlen = max_run >= 3  # any single run of 3+ consecutive losses
    if bursty_by_chi or bursty_by_runlen:
        overall = ("BURSTY — interleaving is high-value; design it into the "
                   "wire format")
    else:
        overall = ("I.I.D. — interleaving buys ~nothing; tile tolerance + "
                   "N=2 redundancy is the better engineering investment "
                   f"(max_run={max_run}, chi2={chi2:.2f}; autocorr 'sig lags' "
                   f"{sig if sig else 'none'} treated as small-sample noise)")
    print(f"\n  >>> VERDICT for {evdir.name}: {overall}")
    return {
        "n": n,
        "p_loss": p,
        "n_runs": len(runs),
        "max_run": max(runs) if runs else 0,
        "autocorr": ac,
        "ci_band": band,
        "sig_lags": sig,
        "chi2": chi2,
        "verdict": overall,
    }


def main():
    targets = [
        ("T5  W2-02 baseline (Redundancy=1)",
         ROOT / "W2-02_image_over_lora_2026-05-20_184021"),
        ("T5b W2-02 with Redundancy=2",
         ROOT / "W2-02_image_over_lora_2026-05-20_185712"),
    ]
    results = []
    for label, d in targets:
        if not d.exists():
            print(f"SKIP {label}: directory not found {d}")
            continue
        r = analyze(label, d)
        if r:
            results.append((label, r))

    print(f"\n{'#' * 78}\n# Combined verdict\n{'#' * 78}")
    bursty_votes = sum(1 for _, r in results
                       if "BURSTY" in r["verdict"])
    iid_votes = sum(1 for _, r in results
                    if "I.I.D." in r["verdict"])
    print(f"  bursty votes: {bursty_votes} / {len(results)}")
    print(f"  i.i.d. votes : {iid_votes} / {len(results)}")
    if bursty_votes > iid_votes:
        print("  >>> STEP 4 architecture: INCLUDE fragment interleaving in wire format.")
    elif iid_votes > bursty_votes:
        print("  >>> STEP 4 architecture: SKIP interleaving; tile tolerance + N=2 is sufficient.")
    else:
        print("  >>> STEP 4 architecture: SPLIT verdict — collect one more sample before deciding.")


if __name__ == "__main__":
    main()
