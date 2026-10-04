"""Offline screen of candidate relabel metrics on the pass-1 features."""
import json
import sys

D = json.load(open("features.json"))


def s(a, b):
    return a / b if b > 0 else 0.0


METRICS = {
    "lab": lambda f: s(f["relab"], f["labelled"]),
    "val": lambda f: s(f["relab"], f["valid"]),
    "l1v": lambda f: s(f["relab"], f["l1valid"]),
    "mc": lambda f: s(f["relab_mc"], f["labelled"]),
    "sym": lambda f: s(f["relab"] + f["gone"], f["labelled"] + f["prev_labelled"]),
    "cov": lambda f: s(f["relab_cov"], f["labelled"]),
    "pix": lambda f: s(f["pix_chg"], f["labelled"]),
    "pixb": lambda f: s(f["pix_chg_both"], f["both"]),
    "pixmc": lambda f: s(f["pix_mc"], f["labelled"]),
    "pixv": lambda f: s(f["pix_chg"], f["valid"]),
}

WARM = 1          # frame index from which a no-change frame counts (frame 0 has no features)


def table(metrics=("lab", "val", "mc", "cov", "pix", "pixmc")):
    print("%-22s %-8s" % ("scenario", "kind") + "".join(" %15s" % m for m in metrics))
    for name, d in D.items():
        cut = d["cut"]
        fs = d["feats"]
        row = "%-22s %-8s" % (name, d["kind"])
        for m in metrics:
            fn = METRICS[m]
            vals = [fn(f) if f else None for f in fs]
            other = [v for i, v in enumerate(vals) if v is not None and i >= WARM
                     and (cut is None or not (cut <= i <= cut + 1))]
            mx = max(other) if other else 0.0
            if cut is not None:
                row += " %6.2f|%4.2f,%4.2f" % (mx, vals[cut], vals[cut + 1])
            else:
                p90 = sorted(other)[int(0.9 * (len(other) - 1))]
                row += " %6.2f|p90 %4.2f" % (mx, p90)
        print(row)


def sweep(metric, thresholds, extra=None, label=None):
    """For each threshold: no-change fires (frames), cuts caught within 0-1 frames, misses."""
    fn = METRICS[metric]
    out = []
    for t in thresholds:
        fires = 0
        nframes = 0
        caught = missed = 0
        false_in_cut = 0
        missed_names = []
        for name, d in D.items():
            fs = d["feats"]
            cut = d["cut"]
            for i, f in enumerate(fs):
                if f is None or i < WARM:
                    continue
                hit = fn(f) > t and (extra is None or extra(f))
                if cut is None:
                    nframes += 1
                    fires += hit
                elif cut <= i <= cut + 1:
                    pass
                else:
                    false_in_cut += hit
            if cut is not None:
                ok = any(fn(fs[i]) > t and (extra is None or extra(fs[i])) for i in (cut, cut + 1))
                caught += ok
                if not ok:
                    missed_names.append(name)
                missed += not ok
        out.append((t, fires, nframes, false_in_cut, caught, missed, missed_names))
        print("%-18s t=%.2f  no-change fires %4d/%d   cut-scen. false %3d   cuts caught %2d missed %2d %s"
              % (label or metric, t, fires, nframes, false_in_cut, caught, missed, missed_names))
    return out


if __name__ == "__main__":
    table()
