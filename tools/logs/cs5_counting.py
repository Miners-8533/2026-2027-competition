"""Replays the CS Pass Study logs (CS5 = the color sensor nearest the intake, which elements only TRANSIT) through several counting algorithms and scores them.

    python tools/logs/cs5_counting.py                      # the runs listed in RUNS, a table of algorithms x runs
    python tools/logs/cs5_counting.py --run 200607 -v      # one run, the detections one by one against the counter's events
    python tools/logs/cs5_counting.py --fast --log robot-logs/2026-11-13/101500_bringup.log -v   # a FAST READ run (the cs5f column), see below

Input: the bringup logs of the CS Pass Study (robot-logs/<day>/<time>_bringup.log) with the cs5_* / cs4_* columns (one row per loop, one read per sensor).

The EMPTY baselines are taken from the empty stretches, not from the study's own (sometimes contaminated) Start baselines: CS5's raw brightness sum and proximity count when nothing is
in front of it, measured as the median of the first 40 rows of each run (the runs start with the track empty and the readings agree to 1 percent).

TRUTH (there is no hand count): the elements that entered a run = the sum of the "N held" of every SALVO started plus what the counter still held at the end, minus what it held at the
start. It is the counter's own count (about 10 percent unreliable: a flicker at the end sensor), so read the scores as a comparison of the algorithms, not as an exact error. The second
check is TIMING: a good detection comes shortly BEFORE a rise of the counter's count (the element reaches CS1 to CS4 after passing CS5); `matched` counts the detections that have a counter
rise within MATCH_AFTER_S after them. Reads while the pusher is moving (and a margin after) are ignored: the arm sweeps past the sensor then.
"""
import argparse
import os
import re
import statistics as st

import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "robot-logs")
RUNS = {  # run: (day, time); only runs with elements fed through the intake and the cs5 columns
    "200552": "2026-11-12/200552",
    "200607": "2026-11-12/200607",
    "200731": "2026-11-12/200731",
    "200831": "2026-11-12/200831",
}
MATCH_AFTER_S = 1.5       # a detection is matched when the counter's count rises within this long after it
ARM_MARGIN_S = 0.30       # reads this long after the pusher leaves HOLD are ignored too
BASELINE_ROWS = 40


class Run:
    pass


def load(name):
    path = name if os.path.exists(name) else os.path.join(ROOT, RUNS[name] + "_bringup.log")
    hdr = None
    rows = []
    ev = []
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("H,"):
            hdr = line.rstrip("\n").split(",")
        elif line.startswith("S,") and hdr:
            p = line.rstrip("\n").split(",")
            if len(p) >= len(hdr):
                rows.append(dict(zip(hdr, p)))
        elif line.startswith("E,"):
            p = line.rstrip("\n").split(",", 2)
            ev.append((float(p[1]), p[2]))
    r = Run()
    r.name = os.path.basename(name).split("_")[0] if os.path.exists(name) else name
    r.t = np.array([float(x["t"]) for x in rows])
    age = np.array([float(x["cs5_age_ms"]) for x in rows]) / 1000.0
    r.read_t = r.t - age                                   # when the CS5 read finished
    r.rgb = np.array([[float(x["cs5_raw_r"]), float(x["cs5_raw_g"]), float(x["cs5_raw_b"])] for x in rows])
    r.sum = r.rgb.sum(axis=1)
    r.prox = np.array([float(x["cs5_prox"]) for x in rows])
    r.fresh = np.array([x["cs5_fresh"] == "1" for x in rows])
    r.blue = r.rgb[:, 2] / np.maximum(r.sum, 1e-9)
    r.pusher = [x["pusher_state"] for x in rows]
    r.count = np.array([int(x["count"]) if x["count"].lstrip("-").isdigit() else -1 for x in rows])
    r.intake_amps = np.array([float(x["intake_amps"]) if x["intake_amps"] not in ("", "-") else 0.0 for x in rows])
    # the baselines from the first rows (the track is empty then)
    r.p0 = float(np.median(r.prox[:BASELINE_ROWS]))
    r.psig = max(1.4826 * float(np.median(np.abs(r.prox[:BASELINE_ROWS] - r.p0))), 2.0)
    r.s0 = float(np.median(r.sum[:BASELINE_ROWS]))
    r.b0 = float(np.median(r.blue[:BASELINE_ROWS]))
    # the mask: pusher not at HOLD, and a margin after it was
    moving = np.array([s not in ("HOLD", "") for s in r.pusher])
    mask = moving.copy()
    last = -1e9
    for i in range(len(r.t)):
        if moving[i]:
            last = r.t[i]
        elif r.t[i] - last < ARM_MARGIN_S:
            mask[i] = True
    r.mask = mask
    # the fast proximity samples (the cs5f column), on the log clock
    r.fast_t = np.zeros(0)
    r.fast_p = np.zeros(0)
    r.fast_mask = np.zeros(0, bool)
    if hdr and "cs5f" in hdr:
        ft, fp = [], []
        for x in rows:
            for tok in x.get("cs5f", "").split():
                try:
                    v, ms = tok.split("@")
                    fp.append(int(v))
                    ft.append(float(ms) / 1000.0)
                except ValueError:
                    pass
        if ft:
            study = np.array([float(x["study_t"]) for x in rows])
            offset = float(np.median(r.t - study))            # log clock minus study clock
            r.fast_t = np.array(ft) + offset
            r.fast_p = np.array(fp, float)
            idx = np.clip(np.searchsorted(r.t, r.fast_t), 0, len(r.t) - 1)
            r.fast_mask = mask[idx]
    # truth
    shot = sum(int(re.search(r"(\d) held", m).group(1)) for _, m in ev if m.startswith("SALVO started"))
    c0 = int(r.count[0]) if r.count[0] >= 0 else 0
    c1 = int(r.count[-1]) if r.count[-1] >= 0 else 0
    r.truth = shot + c1 - c0
    r.shot = shot
    r.rises = [a for a, m in ev if m.startswith("ELEMENTS") and int(m.split()[3]) > int(m.split()[1])]
    r.rise_amount = [int(m.split()[3]) - int(m.split()[1]) for a, m in ev if m.startswith("ELEMENTS") and int(m.split()[3]) > int(m.split()[1])]
    return r


# ---- the algorithms: each returns a list of (time, multiplicity) detections ----

def edges(on_mask, t, mask):
    """Rising edges of a boolean series with a Schmitt trigger already applied; reads in the pusher mask are skipped."""
    out = []
    prev = False
    for i in range(len(t)):
        if mask[i]:
            prev = False
            continue
        if on_mask[i] and not prev:
            out.append((t[i], 1))
        prev = on_mask[i]
    return out


def schmitt(x, hi, lo):
    on = np.zeros(len(x), bool)
    state = False
    for i, v in enumerate(x):
        if not state and v >= hi:
            state = True
        elif state and v < lo:
            state = False
        on[i] = state
    return on


def alg_sum_edge(r, thr=0.25):
    x = (r.sum - r.s0) / r.s0
    return edges(schmitt(x, thr, thr * 0.5), r.read_t, r.mask)


def alg_prox_edge(r, hi=25.0, lo=10.0):
    x = r.prox - r.p0
    return edges(schmitt(x, hi, lo), r.read_t, r.mask)


def alg_prox_z_edge(r, hi=6.0, lo=3.0):
    x = (r.prox - r.p0) / r.psig
    return edges(schmitt(x, hi, lo), r.read_t, r.mask)


def alg_prox_peaks(r, height=25.0, sep_s=0.10):
    from scipy.signal import find_peaks
    x = np.where(r.mask, 0.0, r.prox - r.p0)
    pk, _ = find_peaks(x, height=height, prominence=height * 0.6)
    out = []
    for i in pk:
        if not out or r.read_t[i] - out[-1][0] >= sep_s:
            out.append((r.read_t[i], 1))
    return out


def combo(r):
    """One score from the proximity rise, the brightness rise (relative) and the blue-share drop, each in units of its own empty noise."""
    zp = (r.prox - r.p0) / r.psig
    zs = (r.sum - r.s0) / (0.02 * r.s0)                       # 2 percent of the empty brightness is the unit
    zb = (r.b0 - r.blue) / 0.005
    return 0.5 * zp + 0.3 * zs + 0.2 * zb


def alg_combo_edge(r, hi=8.0, lo=4.0):
    return edges(schmitt(combo(r), hi, lo), r.read_t, r.mask)


def alg_combo_peaks(r, height=8.0, sep_s=0.10):
    from scipy.signal import find_peaks
    x = np.where(r.mask, 0.0, combo(r))
    pk, _ = find_peaks(x, height=height, prominence=height * 0.5)
    out = []
    for i in pk:
        if not out or r.read_t[i] - out[-1][0] >= sep_s:
            out.append((r.read_t[i], 1))
    return out


def alg_area(r, hi=25.0, lo=10.0, per_element=None):
    """Events by the proximity Schmitt trigger; each event counts round(area / area-per-element) elements, at least one (the area is the baseline-subtracted proximity integral)."""
    x = r.prox - r.p0
    on = schmitt(x, hi, lo)
    events = []
    i = 0
    n = len(x)
    while i < n:
        if on[i] and not r.mask[i]:
            j = i
            area = 0.0
            while j < n and on[j]:
                dt = (r.read_t[j] - r.read_t[j - 1]) if j > 0 else 0.056
                area += max(x[j], 0.0) * min(dt, 0.12)
                j += 1
            events.append((r.read_t[i], area))
            i = j
        else:
            i += 1
    if not events:
        return []
    ref = per_element if per_element else float(np.median([a for _, a in events]))
    return [(t, max(1, int(round(a / ref)))) for t, a in events]


_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


# ---- the fast-read algorithms: over r.fast_t / r.fast_p (hundreds of samples per second) ----

def fast_baseline(r, seconds=1.5):
    n = max(20, int(np.searchsorted(r.fast_t, r.fast_t[0] + seconds)))
    p0 = float(np.median(r.fast_p[:n]))
    sig = max(1.4826 * float(np.median(np.abs(r.fast_p[:n] - p0))), 1.0)
    return p0, sig


def fast_series(r, smooth=3):
    """The proximity above its empty level, in sigmas, median-filtered over `smooth` samples (kills a single-sample spike)."""
    p0, sig = fast_baseline(r)
    x = (r.fast_p - p0) / sig
    if smooth > 1:
        from scipy.ndimage import median_filter
        x = median_filter(x, size=smooth, mode="nearest")
    return x


def fast_events(r, hi=6.0, lo=3.0, smooth=3, min_gap_s=0.03):
    """(start time, width s, peak sigma, area sigma*s) of each stretch above the Schmitt trigger, skipping the pusher-moving reads; events closer than min_gap_s are merged."""
    x = fast_series(r, smooth)
    t = r.fast_t
    on = False
    ev = []
    for i in range(len(x)):
        if r.fast_mask[i]:
            on = False
            continue
        if not on and x[i] >= hi:
            on = True
            a = i
        elif on and x[i] < lo:
            on = False
            ev.append([t[a], t[i - 1] - t[a], float(x[a:i].max()), float(_trapz(np.maximum(x[a:i], 0.0), t[a:i]))])
    merged = []
    for e in ev:
        if merged and e[0] - (merged[-1][0] + merged[-1][1]) < min_gap_s:
            m = merged[-1]
            m[1] = e[0] + e[1] - m[0]
            m[2] = max(m[2], e[2])
            m[3] += e[3]
        else:
            merged.append(list(e))
    return merged


def alg_fast_edge(r, hi=6.0, lo=3.0, smooth=3):
    return [(e[0], 1) for e in fast_events(r, hi, lo, smooth)]


def alg_fast_area(r, hi=6.0, lo=3.0, smooth=3, long_s=0.6):
    """Each event counts round(area / the median area of the others), at least one; an event longer than long_s is a dwell (a hand, a stuck element) and counts as one."""
    ev = fast_events(r, hi, lo, smooth)
    if not ev:
        return []
    ref = float(np.median([e[3] for e in ev if e[1] < long_s] or [e[3] for e in ev]))
    return [(e[0], 1 if e[1] >= long_s else max(1, int(round(e[3] / ref)))) for e in ev]


FAST_ALGS = [
    ("fast proximity edge, 4 sigma", alg_fast_edge, dict(hi=4.0, lo=2.0)),
    ("fast proximity edge, 6 sigma", alg_fast_edge, {}),
    ("fast proximity edge, 10 sigma", alg_fast_edge, dict(hi=10.0, lo=5.0)),
    ("fast proximity edge, 6 sigma, no smoothing", alg_fast_edge, dict(smooth=1)),
    ("fast proximity edge 6 sigma with area multiplicity", alg_fast_area, {}),
    ("fast proximity edge 4 sigma with area multiplicity", alg_fast_area, dict(hi=4.0, lo=2.0)),
]


def report_fast(runs, verbose):
    for r in runs:
        if len(r.fast_t) < 50:
            print("%s: no fast samples (the cs5f column is missing or empty)" % r.name)
            continue
        dt = np.diff(r.fast_t) * 1000.0
        p0, sig = fast_baseline(r)
        print("%s: %d fast samples over %.1f s: %.0f Hz, interval median %.1f ms, p90 %.1f, p99 %.1f, max %.0f ms, %d over 20 ms; empty proximity %.0f +/- %.1f; truth %d (shot %d), counter rises %d" % (
            r.name, len(r.fast_t), r.fast_t[-1] - r.fast_t[0], 1000.0 / max(np.median(dt), 1e-6), np.median(dt), np.percentile(dt, 90), np.percentile(dt, 99), dt.max(),
            int((dt > 20.0).sum()), p0, sig, r.truth, r.shot, len(r.rises)))
        for label, fn, kw in FAST_ALGS:
            det = fn(r, **kw)
            n, m = score(r, det)
            ev = fast_events(r, kw.get("hi", 6.0), kw.get("lo", 3.0), kw.get("smooth", 3))
            widths = [e[1] * 1000.0 for e in ev]
            print("   %-52s detected %2d / truth %2d (matched %2d)   event width median %.0f ms, p90 %.0f ms%s" % (
                label, n, r.truth, m, np.median(widths) if widths else 0.0, np.percentile(widths, 90) if widths else 0.0,
                "   multi: %s" % [m for _, m in det if m > 1] if fn is alg_fast_area else ""))
            if verbose and "6 sigma" == label.split(", ")[-1] and fn is alg_fast_edge:
                print("     detections:", ", ".join("%.2f" % t for t, _ in det))
                print("     counter rises:", ", ".join("%.2f(+%d)" % (t, k) for t, k in zip(r.rises, r.rise_amount)))


ALGS = [
    ("brightness sum edge +25%", alg_sum_edge, {}),
    ("proximity edge +25", alg_prox_edge, {}),
    ("proximity edge +15 (more sensitive)", alg_prox_edge, dict(hi=15.0, lo=6.0)),
    ("proximity edge, 6 sigma", alg_prox_z_edge, {}),
    ("proximity peaks (>25, 100 ms apart)", alg_prox_peaks, {}),
    ("proximity peaks (>15, 100 ms apart)", alg_prox_peaks, dict(height=15.0)),
    ("combined score edge", alg_combo_edge, {}),
    ("combined score peaks", alg_combo_peaks, {}),
    ("proximity edge +15 with area multiplicity", alg_area, dict(hi=15.0, lo=6.0)),
]


def score(r, det):
    n = sum(m for _, m in det)
    matched = 0
    for t, m in det:
        if any(0.0 <= a - t <= MATCH_AFTER_S for a in r.rises):
            matched += 1
    return n, matched


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", help="one run only")
    ap.add_argument("--fast", action="store_true", help="replay a FAST READ run (the cs5f column)")
    ap.add_argument("--log", help="a bringup log file (instead of the runs in RUNS)")
    ap.add_argument("-v", action="store_true", help="list the detections against the counter's rises")
    a = ap.parse_args()
    names = [a.log] if a.log else [a.run] if a.run else list(RUNS)
    runs = [load(n) for n in names]
    if a.fast:
        report_fast(runs, a.v)
        return
    print("Runs: " + "; ".join("%s: truth %d (shot %d), counter rises %d, CS5 empty prox %.0f +/- %.1f, brightness %.4f" % (r.name, r.truth, r.shot, len(r.rises), r.p0, r.psig, r.s0) for r in runs))
    print()
    print("%-46s" % "algorithm (count detected / truth, matched detections)" + "".join("%18s" % r.name for r in runs) + "%14s" % "TOTAL")
    for label, fn, kw in ALGS:
        cells = []
        tot_n = tot_t = tot_m = 0
        for r in runs:
            det = fn(r, **kw)
            n, m = score(r, det)
            cells.append("%d/%d (%d)" % (n, r.truth, m))
            tot_n += n
            tot_t += r.truth
            tot_m += m
        print("%-46s" % label + "".join("%18s" % c for c in cells) + "%14s" % ("%d/%d (%d)" % (tot_n, tot_t, tot_m)))
        if a.v and a.run:
            r = runs[0]
            det = fn(r, **kw)
            print("   detections:", ", ".join("%.2f%s" % (t, "x%d" % m if m > 1 else "") for t, m in det))
            print("   counter rises:", ", ".join("%.2f(+%d)" % (t, k) for t, k in zip(r.rises, r.rise_amount)))


if __name__ == "__main__":
    main()
