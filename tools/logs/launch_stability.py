"""Flywheel speed stability during salvos, from the run logs under robot-logs/ (TeleOp and auto logs).

    python tools/logs/launch_stability.py                 # every salvo in every TeleOp / auto log
    python tools/logs/launch_stability.py --since 2026-11-03
    python tools/logs/launch_stability.py --out some_folder

Writes PNG figures to robot-logs/plots/ and prints a summary. A salvo runs from the log's "SALVO started" event to "PUSHER state SALVO -> HOLD"; the rows used are the
100 ms sample rows ("S," lines) from 0.4 s before the start to 0.8 s after the end. The speed is the hub's flywheel velocity in ticks/s (fly_vel); the error is
(fly_vel - fly_target) / fly_target in percent.

Figures:
  salvo_overlay.png   every salvo, error against time since the salvo started
  salvo_phase.png     the error by phase of the salvo (0 = start, 1 = the pusher is back at HOLD), pooled: where in the salvo is the speed high or low
  salvo_detail.png    the newest salvos one by one, with the actual sample points, the pusher angle and when the controller was at full power
  salvo_summary.png   per salvo: the speed at the start, the lowest and highest during, the highest in the 0.5 s after
"""
import argparse
import glob
import math
import os
import statistics
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    header, rows, events = None, [], []
    for line in open(path, encoding="utf-8", errors="ignore"):
        if line.startswith("H,"):
            header = line.rstrip("\n").split(",")[1:]
        elif line.startswith("S,") and header:
            parts = line.rstrip("\n").split(",")[1:]
            if len(parts) != len(header):
                continue
            try:
                rows.append({k: v for k, v in zip(header, parts)})
            except ValueError:
                continue
        elif line.startswith("E,"):
            p = line.rstrip("\n").split(",", 2)
            if len(p) == 3:
                try:
                    events.append((float(p[1]), p[2]))
                except ValueError:
                    pass
    return rows, events


def f(row, key, default=float("nan")):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return default


def salvos_in(path):
    rows, events = load(path)
    if not rows or "fly_vel" not in rows[0]:
        return []
    out = []
    for i, (t0, text) in enumerate(events):
        if not text.startswith("SALVO started"):
            continue
        t1 = next((t for t, tx in events[i + 1:] if tx.startswith("PUSHER state SALVO -> HOLD")), t0 + 1.6)
        sel = [r for r in rows if t0 - 0.4 <= f(r, "t") <= t1 + 0.8]
        if len(sel) < 6:
            continue
        target = statistics.median([f(r, "fly_target") for r in sel if f(r, "fly_target") > 0] or [0.0])
        if target <= 0:
            continue
        out.append({"file": path, "t0": t0, "t1": t1, "target": target, "rows": sel})
    return out


def err(row, target):
    return (f(row, "fly_vel") - target) / target * 100.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join("robot-logs"))
    ap.add_argument("--since", default="0000-00-00", help="only run folders from this date (YYYY-MM-DD) on")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out_dir = args.out or os.path.join(args.root, "plots")
    os.makedirs(out_dir, exist_ok=True)

    files = []
    for kind in ("teleop", "auto"):
        files += sorted(glob.glob(os.path.join(args.root, "*-*-*", "*_%s.log" % kind)))
    files = [p for p in files if os.path.basename(os.path.dirname(p)) >= args.since]
    salvos = []
    for p in files:
        salvos += salvos_in(p)
    if not salvos:
        sys.exit("No salvos found in %d log(s) under %s" % (len(files), args.root))
    print("%d salvos in %d logs" % (len(salvos), len(files)))

    # ---- resolution of what the logs hold ----
    dts, vels = [], []
    for s in salvos:
        ts = [f(r, "t") for r in s["rows"]]
        dts += [b - a for a, b in zip(ts, ts[1:])]
        vels += [f(r, "fly_vel") for r in s["rows"] if not math.isnan(f(r, "fly_vel"))]
    dts.sort()
    steps = sorted(set(round(abs(b - a)) for a, b in zip(sorted(set(vels)), sorted(set(vels))[1:]))) if vels else []
    on20 = sum(1 for v in vels if abs(v / 20.0 - round(v / 20.0)) < 1e-6) / max(1, len(vels))
    print("RESOLUTION: sample spacing in salvos median %.0f ms (p10 %.0f, p90 %.0f); salvo length median %.2f s -> about %.0f rows per salvo"
          % (1000 * statistics.median(dts), 1000 * dts[int(0.1 * len(dts))], 1000 * dts[int(0.9 * len(dts))],
             statistics.median([s["t1"] - s["t0"] for s in salvos]), statistics.median([s["t1"] - s["t0"] for s in salvos]) / statistics.median(dts)))
    print("            fly_vel values: %.0f%% are multiples of 20 ticks/s (the smallest step the hub reports looks like 20 ticks/s = %.1f%% of 1350)" % (100 * on20, 100 * 20 / 1350))

    # ---- per salvo numbers ----
    stats = []
    off = 0
    for s in salvos:
        T = s["target"]
        during = [r for r in s["rows"] if s["t0"] <= f(r, "t") <= s["t1"]]
        after = [r for r in s["rows"] if s["t1"] < f(r, "t") <= s["t1"] + 0.5]
        start_row = min(s["rows"], key=lambda r: abs(f(r, "t") - s["t0"]))
        if not during:
            continue
        if err(start_row, T) < -50.0:
            off += 1   # the flywheel was not spinning (a pusher test): not a launch
            continue
        e = [err(r, T) for r in during]
        stats.append({"s": s, "start": err(start_row, T), "min": min(e), "max": max(e), "std": statistics.pstdev(e) if len(e) > 1 else 0.0,
                      "after": max([err(r, T) for r in after] or [float("nan")])})
    if off:
        print("            (%d salvo(s) with the flywheel stopped were left out)" % off)
    print("SALVO SPEED vs target (percent): at start %+.1f median (range %+.1f to %+.1f); lowest during %+.1f median (worst %+.1f); highest during %+.1f median (worst %+.1f); "
          "highest in the 0.5 s after %+.1f median (worst %+.1f)" % (
              statistics.median([x["start"] for x in stats]), min(x["start"] for x in stats), max(x["start"] for x in stats),
              statistics.median([x["min"] for x in stats]), min(x["min"] for x in stats),
              statistics.median([x["max"] for x in stats]), max(x["max"] for x in stats),
              statistics.median([x["after"] for x in stats if not math.isnan(x["after"])]), max(x["after"] for x in stats if not math.isnan(x["after"]))))
    sd = [x["std"] for x in stats]
    print("            standard deviation within a salvo: median %.1f %%, worst %.1f %% (one measurement step is %.1f %%)" % (statistics.median(sd), max(sd), 100 * 20 / 1350))

    # ---- figure 1: overlay ----
    fig, ax = plt.subplots(figsize=(11, 6))
    cmap = plt.get_cmap("viridis")
    for i, x in enumerate(stats):
        s = x["s"]
        tt = [f(r, "t") - s["t0"] for r in s["rows"]]
        ee = [err(r, s["target"]) for r in s["rows"]]
        ax.plot(tt, ee, "-", lw=0.8, alpha=0.45, color=cmap(i / max(1, len(stats) - 1)))
    ax.axvline(0, color="k", lw=1)
    ax.set_ylim(-30, 20)
    ax.text(0.02, 18.5, "salvo starts", fontsize=8, va="top")
    for band in (2, 5):
        ax.axhline(band, color="r", lw=0.6, ls=":")
        ax.axhline(-band, color="r", lw=0.6, ls=":")
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("seconds since the salvo started")
    ax.set_ylabel("flywheel speed against target (%)")
    ax.set_title("Flywheel speed during %d salvos (older = purple, newer = yellow; dotted = +-2 and +-5 percent)" % len(stats))
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "salvo_overlay.png"), dpi=130)
    plt.close(fig)

    # ---- figure 2: by phase ----
    bins = 12
    phase = [[] for _ in range(bins)]
    for x in stats:
        s = x["s"]
        span = max(0.2, s["t1"] - s["t0"])
        for r in s["rows"]:
            p = (f(r, "t") - s["t0"]) / span
            if 0.0 <= p < 1.0:
                phase[int(p * bins)].append(err(r, s["target"]))
    fig, ax = plt.subplots(figsize=(11, 5.5))
    data = [b if b else [float("nan")] for b in phase]
    ax.boxplot(data, positions=[(i + 0.5) / bins for i in range(bins)], widths=0.06, showfliers=True, flierprops={"markersize": 3})
    ax.axhline(0, color="gray", lw=0.8)
    for q in (0.25, 0.5, 0.75):
        ax.axvline(q, color="gray", lw=0.4, ls=":")
    ax.set_xlim(0, 1)
    ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
    ax.set_ylim(-10, 16)
    ax.set_xlabel("phase of the salvo (0 = it starts, 1 = the pusher is back at HOLD); the four pollen leave in turn along this axis")
    ax.set_ylabel("flywheel speed against target (%)")
    ax.set_title("Where in the salvo is the speed high or low (all salvos pooled, %d rows)" % sum(len(b) for b in phase))
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "salvo_phase.png"), dpi=130)
    plt.close(fig)

    # ---- figure 3: the newest salvos, one by one ----
    newest = stats[-8:]
    cols = 4
    nrows = int(math.ceil(len(newest) / cols))
    fig, axes = plt.subplots(nrows, cols, figsize=(16, 3.6 * nrows), squeeze=False)
    for ax, x in zip(axes.flat, newest):
        s = x["s"]
        tt = [f(r, "t") - s["t0"] for r in s["rows"]]
        vv = [f(r, "fly_vel") for r in s["rows"]]
        bang = [f(r, "fly_power") for r in s["rows"]]
        for t, p in zip(tt, bang):
            if p >= 0.9:
                ax.axvspan(t - 0.05, t + 0.05, color="orange", alpha=0.25, lw=0)
        ax.plot(tt, vv, "o-", ms=4, lw=1, color="C0")
        ax.axhline(s["target"], color="k", lw=0.8)
        ax.axvline(0, color="k", lw=0.6)
        ax.axvline(s["t1"] - s["t0"], color="k", lw=0.6, ls="--")
        ax.set_ylim(s["target"] * 0.93, s["target"] * 1.07)
        ax.set_title(os.path.basename(os.path.dirname(s["file"])) + " " + os.path.basename(s["file"])[:6] + "  t=%.0f s  start %+.1f%%" % (s["t0"], x["start"]), fontsize=8)
        ax2 = ax.twinx()
        ax2.plot(tt, [f(r, "pusher_deg") for r in s["rows"]], ".-", color="gray", lw=0.6, ms=3, alpha=0.7)
        ax2.set_ylim(0, 400)
        ax2.tick_params(labelsize=6)
        ax.tick_params(labelsize=7)
    fig.suptitle("Newest salvos: speed (blue dots = the logged samples, black line = target, orange = the controller was at full power, grey = pusher angle 0 to 400 deg)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "salvo_detail.png"), dpi=110)
    plt.close(fig)

    # ---- figure 4: summary ----
    fig, ax = plt.subplots(figsize=(11, 5.5))
    xs = list(range(len(stats)))
    ax.plot(xs, [x["start"] for x in stats], "o", label="at the start", color="C0")
    ax.plot(xs, [x["min"] for x in stats], "v", label="lowest during", color="C3")
    ax.plot(xs, [x["max"] for x in stats], "^", label="highest during", color="C2")
    ax.plot(xs, [x["after"] for x in stats], "s", label="highest in the 0.5 s after", color="C1", ms=4)
    for i, x in enumerate(stats):
        if x["start"] > 4.0:
            ax.annotate(os.path.basename(os.path.dirname(x["s"]["file"]))[5:] + " " + os.path.basename(x["s"]["file"])[:6] + " t=%.0f" % x["s"]["t0"], (i, x["start"]), fontsize=7, xytext=(4, 4), textcoords="offset points")
    ax.set_ylim(-10, 20)
    ax.axhline(0, color="gray", lw=0.8)
    ax.set_xlabel("salvo number (oldest first)")
    ax.set_ylabel("flywheel speed against target (%)")
    ax.legend()
    ax.set_title("Per salvo: how far the speed strays from the target")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "salvo_summary.png"), dpi=130)
    plt.close(fig)
    print("wrote 4 figures to", out_dir)


if __name__ == "__main__":
    main()
