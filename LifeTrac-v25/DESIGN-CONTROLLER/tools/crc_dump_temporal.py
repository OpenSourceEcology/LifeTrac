"""crc_dump_temporal.py — temporal + population analysis of RX_CRC_DUMP captures.

Answers RS-11.6 item 1: are the corrupt receptions periodic or bursty, and do
they correlate with our own transmissions?

Reads one or more radio-monitor archives and parses four line kinds out of
`rx_daemon.log`:

    crc_dump:  ts, irq, rx_len, snr, rssi, dump      -- a corrupt reception
    rx_rf:     ts, n, rssi[min/med/max], snr[...]    -- healthy-frame window
    air_gap:   ts, cmd_tx_ok                         -- base's own TX count
    stats:     ts, rx_frames, ...                    -- run progress

IMPORTANT — clocks. The tractor and base board clocks are far apart (in the
2026-08-16 archive tx_daemon.log is stamped 8 days off from rx_daemon.log), so
tx/rx timestamps are NOT comparable. Everything here stays inside
rx_daemon.log, which is a single clock. That is sufficient for the
self-interference question because the interfering transmitter of interest is
the base's own radio, and the base IS the RX board.

Usage:
    py -3 crc_dump_temporal.py <archive_dir> [<archive_dir> ...]
    py -3 crc_dump_temporal.py --all <bench-evidence-dir>
"""

from __future__ import annotations

import argparse
import math
import pathlib
import re
import sys
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------- parsing

TS_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}),(\d{3})")

CRC_RE = re.compile(
    r"crc_dump:\s+irq=(?P<irq>0x[0-9a-fA-F]+)\s+"
    r"rx_len=(?P<rx_len>\d+)\s+"
    r"snr=(?P<snr>-?[\d.]+)\s+"
    r"rssi=(?P<rssi>-?\d+)\s+"
    r"dump=(?P<dump>[0-9a-fA-F]+)"
)

RXRF_RE = re.compile(
    r"rx_rf:\s+n_rssi=(?P<n_rssi>\d+)\s+n_snr=(?P<n_snr>\d+)\s+"
    r"rssi\[min/med/max\]=(?P<rmin>-?\d+)/(?P<rmed>-?\d+)/(?P<rmax>-?\d+)\s+"
    r"snr\[min/med/max\]=(?P<smin>-?[\d.]+)/(?P<smed>-?[\d.]+)/(?P<smax>-?[\d.]+)"
)

CMD_RE = re.compile(r"air_gap:.*cmd_tx_ok=(?P<ok>\d+)\s+cmd_tx_fail=(?P<fail>\d+)")


def _ts_seconds(line: str) -> float | None:
    """Absolute seconds-of-day from a log line's leading timestamp."""
    m = TS_RE.match(line.lstrip("﻿"))
    if not m:
        return None
    hh, mm, ss = m.group(1).split(" ")[1].split(":")
    return int(hh) * 3600 + int(mm) * 60 + int(ss) + int(m.group(2)) / 1000.0


@dataclass
class Capture:
    t: float           # seconds since first parsed event in this archive
    wall: float        # seconds-of-day
    irq: int
    rx_len: int
    snr: float
    rssi: int
    dump: bytes


@dataclass
class Window:
    t: float
    n: int
    rssi_min: int
    rssi_med: int
    rssi_max: int
    snr_min: float
    snr_med: float
    snr_max: float


@dataclass
class Archive:
    name: str
    captures: list[Capture] = field(default_factory=list)
    windows: list[Window] = field(default_factory=list)
    cmd_marks: list[tuple[float, int]] = field(default_factory=list)
    t0_wall: float = 0.0

    @property
    def duration(self) -> float:
        ends = [c.t for c in self.captures] + [w.t for w in self.windows]
        return max(ends) if ends else 0.0


def parse_archive(path: pathlib.Path) -> Archive:
    log = path / "rx_daemon.log"
    arch = Archive(name=path.name)
    t0: float | None = None

    for raw in log.read_text(encoding="utf-8", errors="replace").splitlines():
        if not raw.strip():
            continue
        wall = _ts_seconds(raw)
        if wall is None:
            continue
        if t0 is None:
            t0 = wall
            arch.t0_wall = wall
        # guard against a midnight wrap inside a 300 s run
        rel = wall - t0
        if rel < -43200:
            rel += 86400

        m = CRC_RE.search(raw)
        if m:
            arch.captures.append(
                Capture(
                    t=rel,
                    wall=wall,
                    irq=int(m.group("irq"), 16),
                    rx_len=int(m.group("rx_len")),
                    snr=float(m.group("snr")),
                    rssi=int(m.group("rssi")),
                    dump=bytes.fromhex(m.group("dump")),
                )
            )
            continue

        m = RXRF_RE.search(raw)
        if m:
            arch.windows.append(
                Window(
                    t=rel,
                    n=int(m.group("n_rssi")),
                    rssi_min=int(m.group("rmin")),
                    rssi_med=int(m.group("rmed")),
                    rssi_max=int(m.group("rmax")),
                    snr_min=float(m.group("smin")),
                    snr_med=float(m.group("smed")),
                    snr_max=float(m.group("smax")),
                )
            )
            continue

        m = CMD_RE.search(raw)
        if m:
            arch.cmd_marks.append((rel, int(m.group("ok"))))

    return arch


# ------------------------------------------------------- statistical tools


def fano(times: np.ndarray, duration: float, bin_w: float) -> tuple[float, float]:
    """Fano factor (var/mean of per-bin counts). 1.0 == Poisson, >1 == clustered.

    Returns (F, mean_count). Meaningless when mean_count is tiny, so the caller
    should filter on it.
    """
    nbins = max(1, int(math.floor(duration / bin_w)))
    counts, _ = np.histogram(times, bins=nbins, range=(0.0, nbins * bin_w))
    mu = counts.mean()
    if mu <= 0:
        return float("nan"), 0.0
    return float(counts.var() / mu), float(mu)


def rayleigh(times: np.ndarray, period: float) -> tuple[float, float]:
    """Rayleigh test for phase concentration at a trial period.

    Folds event times modulo `period` and measures how concentrated the
    resulting phases are. Returns (R, p) where R in [0,1] is the mean resultant
    length and p = exp(-N*R^2) is the single-trial significance under a uniform
    null. NOTE: p is uncorrected; a scan over many periods needs a
    multiple-comparison correction (see scan_periods).
    """
    n = len(times)
    if n < 4 or period <= 0:
        return 0.0, 1.0
    phi = 2.0 * np.pi * (np.mod(times, period) / period)
    c = np.cos(phi).sum()
    s = np.sin(phi).sum()
    r = math.hypot(c, s) / n
    return r, math.exp(-n * r * r)


def scan_periods(
    times: np.ndarray, duration: float, tmin: float, tmax: float, n_trial: int = 4000
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Rayleigh-scan a log-spaced grid of trial periods.

    Returns (periods, R values, best-dict). The best-dict carries a
    Bonferroni-style corrected p using the number of INDEPENDENT trials, which
    for a fold test is ~duration/period_resolution rather than n_trial.
    """
    periods = np.logspace(np.log10(tmin), np.log10(tmax), n_trial)
    rs = np.empty_like(periods)
    ps = np.empty_like(periods)
    for i, p in enumerate(periods):
        rs[i], ps[i] = rayleigh(times, float(p))

    i = int(np.argmin(ps))
    # Independent-trial count: folding resolves periods to ~P^2/T spacing, so
    # the number of distinct foldings across the scanned band is roughly
    # T*(1/tmin - 1/tmax).
    n_indep = max(1.0, duration * (1.0 / tmin - 1.0 / tmax))
    p_corr = min(1.0, ps[i] * n_indep)
    best = {
        "period": float(periods[i]),
        "R": float(rs[i]),
        "p_raw": float(ps[i]),
        "n_indep": float(n_indep),
        "p_corrected": float(p_corr),
    }
    return periods, rs, best


def shuffled_null(
    times: np.ndarray, duration: float, tmin: float, tmax: float,
    n_iter: int = 200, seed: int = 20260816,
) -> tuple[float, float]:
    """Null distribution for the fold scan, preserving the renewal structure.

    THE TRAP THIS EXISTS TO AVOID: a process with regular-ish spacing produces
    high Rayleigh R when folded at ~its mean inter-arrival, with no external
    clock involved. Scanning for the best period will therefore "find" one at
    roughly the mean gap every time. To claim a real periodicity we need the
    observed best-R to beat a null that has the SAME inter-arrival
    distribution but no absolute phase coherence -- which is exactly what
    shuffling the intervals gives.

    Returns (null_best_R_95th_percentile, fraction of nulls beating observed).
    """
    rng = np.random.default_rng(seed)
    iat = np.diff(times)
    if len(iat) < 4:
        return float("nan"), float("nan")
    obs_r = scan_periods(times, duration, tmin, tmax, n_trial=800)[2]["R"]

    best_rs = np.empty(n_iter)
    for k in range(n_iter):
        perm = rng.permutation(iat)
        synth = np.concatenate([[times[0]], times[0] + np.cumsum(perm)])
        best_rs[k] = scan_periods(synth, duration, tmin, tmax, n_trial=800)[2]["R"]

    return float(np.percentile(best_rs, 95)), float((best_rs >= obs_r).mean())


def refine_period(times: np.ndarray, p_lo: float, p_hi: float,
                  n_grid: int = 400000) -> dict:
    """Fine Rayleigh search for the period that best phase-aligns `times`.

    Reports the cycle-index each event falls on and the residual in cycles.
    A residual RMS well under 0.1 cycle means the events sit on a hard grid --
    the signature of a fixed-cadence emitter rather than a random process.
    """
    grid = np.linspace(p_lo, p_hi, n_grid)
    best_p, best_r = p_lo, -1.0
    for p in grid:
        phi = 2.0 * np.pi * np.mod(times, p) / p
        r = math.hypot(np.cos(phi).sum(), np.sin(phi).sum()) / len(times)
        if r > best_r:
            best_r, best_p = r, float(p)

    rel = times - times[0]
    cycles = rel / best_p
    nearest = np.round(cycles)
    resid = cycles - nearest
    _, p_val = rayleigh(times, best_p)
    return {
        "period": best_p,
        "R": best_r,
        "p": p_val,
        "cycles": nearest.astype(int),
        "resid_cycles": resid,
        "resid_rms": float(np.sqrt((resid ** 2).mean())),
        "resid_max": float(np.abs(resid).max()),
    }


def burst_segments(times: np.ndarray, gap: float) -> list[tuple[float, int]]:
    """Split events into bursts separated by more than `gap` seconds."""
    if len(times) == 0:
        return []
    out: list[tuple[float, int]] = []
    start = times[0]
    count = 1
    for prev, cur in zip(times[:-1], times[1:]):
        if cur - prev > gap:
            out.append((float(start), count))
            start, count = cur, 1
        else:
            count += 1
    out.append((float(start), count))
    return out


# --------------------------------------------------------------- reporting


def describe_populations(caps: list[Capture]) -> str:
    """Cluster captures by RSSI into the regimes visible in the leg-1 data."""
    rssi = np.array([c.rssi for c in caps], dtype=float)
    snr = np.array([c.snr for c in caps], dtype=float)
    lines = []
    lines.append(f"  n={len(caps)}  rssi mean={rssi.mean():.1f} med={np.median(rssi):.0f} "
                 f"min={rssi.min():.0f} max={rssi.max():.0f}")
    lines.append(f"           snr  mean={snr.mean():+.2f} med={np.median(snr):+.1f} "
                 f"min={snr.min():+.1f} max={snr.max():+.1f}")
    return "\n".join(lines)


def corruption_shape(caps: list[Capture]) -> dict:
    """Where inside the payload does corruption land, and is it burst or scatter?

    We do not have the clean reference payload, so we cannot diff byte-for-byte.
    What we CAN measure without a reference is the run-length structure of
    plausible-vs-implausible bytes and the position of the WEBP/RIFF magic when
    it survives, which is what RS-11.5 used. Reported here as raw material
    rather than a verdict.
    """
    lens = np.array([c.rx_len for c in caps])
    riff = sum(1 for c in caps if b"RIFF" in c.dump)
    webp = sum(1 for c in caps if b"WEBP" in c.dump)
    return {
        "n": len(caps),
        "rx_len_hist": {int(v): int(n) for v, n in zip(*np.unique(lens, return_counts=True))},
        "riff_intact": riff,
        "webp_intact": webp,
    }


def analyse(arch: Archive, out=sys.stdout) -> None:
    caps = arch.captures
    if not caps:
        print(f"{arch.name}: no crc_dump captures", file=out)
        return

    t = np.array([c.t for c in caps])
    dur = arch.duration

    print(f"\n{'=' * 78}", file=out)
    print(f"ARCHIVE {arch.name}", file=out)
    print(f"{'=' * 78}", file=out)
    print(f"span {dur:.1f} s   captures {len(caps)}   "
          f"healthy windows {len(arch.windows)}", file=out)

    # -- populations
    print("\n-- corrupt population (all) --", file=out)
    print(describe_populations(caps), file=out)

    if arch.windows:
        wm = np.array([w.rssi_med for w in arch.windows], dtype=float)
        ws = np.array([w.snr_med for w in arch.windows], dtype=float)
        print(f"\n-- healthy windows -- rssi med-of-med={np.median(wm):.0f} "
              f"(range {wm.min():.0f}..{wm.max():.0f})  "
              f"snr med-of-med={np.median(ws):+.1f}", file=out)
        # settle detection: first window whose median is within 2 dB of the
        # run's modal median
        modal = float(np.median(wm[len(wm) // 2:]))
        settled_idx = next((i for i, v in enumerate(wm) if abs(v - modal) <= 2.0), 0)
        t_settle = arch.windows[settled_idx].t
        print(f"   link settles at t={t_settle:.0f}s "
              f"(modal healthy median {modal:.0f} dBm; "
              f"{settled_idx} warm-up window(s) excluded below)", file=out)
    else:
        t_settle = 0.0

    settled = [c for c in caps if c.t >= t_settle]
    if len(settled) != len(caps):
        print(f"\n-- corrupt population (settled only, t>={t_settle:.0f}s) --", file=out)
        print(describe_populations(settled), file=out)

    # -- burstiness
    print("\n-- burstiness (Fano factor; 1.0 = Poisson, >1 = clustered) --", file=out)
    for w in (1.0, 2.0, 5.0, 10.0, 20.0, 30.0):
        f, mu = fano(t, dur, w)
        if mu >= 0.5:
            print(f"   bin {w:5.1f}s   F={f:6.3f}   mean/bin={mu:5.2f}", file=out)

    iat = np.diff(t)
    if len(iat):
        cv = iat.std() / iat.mean()
        print(f"   inter-arrival: mean={iat.mean():.2f}s med={np.median(iat):.2f}s "
              f"min={iat.min():.3f}s CV={cv:.3f}  (CV=1 Poisson)", file=out)

    # -- periodicity
    print("\n-- periodicity (Rayleigh fold scan) --", file=out)
    tmin, tmax = 0.05, dur / 3.0
    _, _, best = scan_periods(t, dur, tmin=tmin, tmax=tmax)
    print(f"   best period {best['period']:.4f}s  R={best['R']:.3f}  "
          f"p_raw={best['p_raw']:.2e}  n_indep~{best['n_indep']:.0f}  "
          f"p_corrected={best['p_corrected']:.2e}", file=out)
    print(f"   (mean inter-arrival is {iat.mean():.2f}s -- a best period near "
          f"that value is the renewal artifact, not a clock)", file=out)

    null95, frac = shuffled_null(t, dur, tmin, tmax)
    print(f"   interval-shuffled null: best-R 95th pct={null95:.3f}, "
          f"observed R={best['R']:.3f}, "
          f"fraction of nulls >= observed = {frac:.3f}", file=out)
    if frac < 0.05:
        verdict = "PERIODIC beyond renewal structure (p<0.05 vs shuffled null)"
    else:
        verdict = ("NOT periodic -- the fold peak is fully explained by the "
                   "inter-arrival distribution alone")
    print(f"   verdict: {verdict}", file=out)

    # candidate cadences we care about
    print("\n   targeted tests at known system cadences:", file=out)
    for label, period in (
        ("fragment ToA ~0.100s", 0.100),
        ("fragment slot ~0.117s", 0.117),
        ("synth frame 0.500s", 0.500),
        ("train boundary ~1.85s", 1.85),
        ("1 s", 1.0),
        ("10 s stats window", 10.0),
    ):
        r, p = rayleigh(t, period)
        flag = "  <-- concentrated" if p < 0.01 else ""
        print(f"     {label:24s} R={r:.3f} p={p:.3f}{flag}", file=out)

    # -- bursts
    print("\n-- burst structure --", file=out)
    for gap in (0.5, 1.0, 2.0):
        segs = burst_segments(t, gap)
        multi = [s for s in segs if s[1] > 1]
        biggest = max((s[1] for s in segs), default=0)
        print(f"   gap>{gap:.1f}s: {len(segs)} bursts, "
              f"{len(multi)} multi-event, largest={biggest}", file=out)

    # -- the interference-dominated subset. Selecting on SNR rather than RSSI
    #    is deliberate: an absolute-RSSI cut is a moving target across runs
    #    (in runs whose whole median sits near -60 dBm it selects half the
    #    population and means nothing), whereas strongly negative SNR is the
    #    physical signature of noise power rivalling the wanted signal, and is
    #    comparable across runs regardless of link level.
    #    BOTH halves of the signature are required. An SNR-only cut drags in
    #    the near-noise-floor captures (-113 dBm at -12 dB SNR), which are
    #    ordinary weak-signal corruption and destroy any grid structure; an
    #    RSSI-only cut is not comparable across runs at different link levels.
    #    The interferer population is the one that is SIMULTANEOUSLY stronger
    #    than the run's own healthy median and dirtier than the healthy SNR.
    HOT_SNR = -5.0
    if arch.windows:
        wm = np.array([w.rssi_med for w in arch.windows], dtype=float)
        ref_rssi = float(np.median(wm[len(wm) // 2:]))
    else:
        ref_rssi = float(np.median([c.rssi for c in caps]))
    hot = [c for c in caps if c.snr < HOT_SNR and c.rssi > ref_rssi]
    print(f"\n   [interferer cut: snr < {HOT_SNR:+.0f} dB AND rssi > "
          f"{ref_rssi:.0f} dBm (this run's healthy median)]", file=out)
    if len(hot) >= 4:
        ht = np.array([c.t for c in hot])
        hr = np.array([c.rssi for c in hot], dtype=float)
        hs = np.array([c.snr for c in hot], dtype=float)
        print(f"\n-- HOT cluster (rssi > -60 dBm), n={len(hot)} --", file=out)
        print(f"   rssi med={np.median(hr):.0f} (range {hr.min():.0f}..{hr.max():.0f})  "
              f"snr med={np.median(hs):+.1f} (range {hs.min():+.1f}..{hs.max():+.1f})",
              file=out)
        print(f"   times: {', '.join(f'{v:.1f}' for v in ht)}", file=out)
        hiat = np.diff(ht)
        if len(hiat):
            print(f"   inter-arrival: mean={hiat.mean():.1f}s med={np.median(hiat):.1f}s "
                  f"min={hiat.min():.1f}s max={hiat.max():.1f}s", file=out)
        print(f"   first at t={ht.min():.1f}s, last at t={ht.max():.1f}s "
              f"(run span {dur:.0f}s)", file=out)

        # Is the hot cluster on a fixed grid? Search a broad band so the
        # answer is not assumed; report residuals so it can be judged.
        fit = refine_period(ht, 1.0, 40.0)
        print(f"   GRID FIT: period={fit['period']:.4f}s  R={fit['R']:.3f}  "
              f"p={fit['p']:.2e}", file=out)
        print(f"     cycle index : "
              f"{', '.join(str(v) for v in fit['cycles'])}", file=out)
        print(f"     residual    : "
              f"{', '.join(f'{v:+.3f}' for v in fit['resid_cycles'])}", file=out)
        print(f"     residual RMS={fit['resid_rms']:.4f} cycles, "
              f"max={fit['resid_max']:.4f} cycles", file=out)
        if fit["resid_rms"] < 0.05:
            print("     => events lie on a HARD GRID: fixed-cadence emitter",
                  file=out)
        elif fit["resid_rms"] < 0.12:
            print("     => quasi-periodic (grid with jitter)", file=out)
        else:
            print("     => no convincing grid", file=out)

    # -- self-interference: our own command TX vs corrupt arrivals
    if len(arch.cmd_marks) > 1:
        print("\n-- self-TX correlation (base's own command TXs) --", file=out)
        wt = np.array([m[0] for m in arch.cmd_marks])
        wc = np.array([m[1] for m in arch.cmd_marks], dtype=float)
        d_cmd = np.diff(wc)
        edges = wt
        n_crc, _ = np.histogram(t, bins=edges)
        if d_cmd.sum() == 0:
            print("   base sent no commands after the first window -- "
                  "self-TX cannot explain any of these captures", file=out)
        elif len(d_cmd) > 2 and d_cmd.std() > 0 and n_crc.std() > 0:
            r = float(np.corrcoef(d_cmd, n_crc)[0, 1])
            print(f"   per-window cmd_tx delta vs corrupt count: r={r:+.3f} "
                  f"(n={len(d_cmd)} windows, {int(d_cmd.sum())} total cmd TX)", file=out)
        else:
            print(f"   insufficient variance to correlate "
                  f"({int(d_cmd.sum())} total cmd TX)", file=out)

    # -- corruption SHAPE, compared between populations.
    #    We have no clean reference payload, so damaged bytes cannot be
    #    localised directly. Two reference-free integrity proxies survive:
    #      rx_len == 255  -> the LoRa explicit header demodulated correctly,
    #                        so damage did not reach the header;
    #      RIFF magic     -> a 4-byte known-plaintext landmark inside the
    #                        payload survived.
    #    Comparing those between the interferer and bulk populations
    #    distinguishes LOCALISED damage from DISTRIBUTED damage.
    print("\n-- corruption shape (integrity proxies by population) --", file=out)
    hot_ids = {id(c) for c in hot}
    for label, group in (("interferer", hot),
                         ("bulk", [c for c in caps if id(c) not in hot_ids])):
        if not group:
            continue
        full = sum(1 for c in group if c.rx_len == 255)
        riff = sum(1 for c in group if b"RIFF" in c.dump)
        print(f"   {label:11s} n={len(group):4d}  header intact (rx_len=255) "
              f"{100.0 * full / len(group):5.1f}%   RIFF landmark intact "
              f"{100.0 * riff / len(group):5.1f}%", file=out)

    shape = corruption_shape(caps)
    print(f"   rx_len hist {shape['rx_len_hist']}", file=out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="+", type=pathlib.Path)
    ap.add_argument("--all", action="store_true",
                    help="treat paths as bench-evidence dirs and sweep every "
                         "archive that contains crc_dump captures")
    args = ap.parse_args()

    targets: list[pathlib.Path] = []
    if args.all:
        for root in args.paths:
            for d in sorted(root.glob("radio_monitor_*")):
                log = d / "rx_daemon.log"
                if log.is_file() and "crc_dump" in log.read_text(
                        encoding="utf-8", errors="replace"):
                    targets.append(d)
    else:
        targets = args.paths

    archives = [parse_archive(p) for p in targets]
    for a in archives:
        analyse(a)

    # ---- cross-run view
    if len(archives) > 1:
        print(f"\n{'=' * 78}", file=sys.stdout)
        print("CROSS-RUN", file=sys.stdout)
        print(f"{'=' * 78}", file=sys.stdout)
        for a in archives:
            if not a.captures:
                continue
            r = np.array([c.rssi for c in a.captures], dtype=float)
            s = np.array([c.snr for c in a.captures], dtype=float)
            hh = int(a.t0_wall // 3600)
            mm = int((a.t0_wall % 3600) // 60)
            hot = int((s < -5.0).sum())
            print(f"  {a.name[15:30]:16s} start {hh:02d}:{mm:02d}  n={len(a.captures):4d}  "
                  f"rssi med={np.median(r):6.1f}  snr med={np.median(s):+5.1f}  "
                  f"snr<-5dB={hot:3d}", file=sys.stdout)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
