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
  salvo_pollen.png    per pollen (added 2026-11-07): the speed as each of the four pollen leaves its slot sensor, the speed around that moment, and when each leaves
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
    # ---- figure 5: per pollen ----
    per = pollen_exits(stats)
    if per:
        names = ["pollen %d (CS%d)" % (k + 1, k + 1) for k in range(4)]
        fig, (axa, axb, axc) = plt.subplots(1, 3, figsize=(17, 5.6))
        data = [[e["err"] for e in per[k]] or [float("nan")] for k in range(4)]
        axa.boxplot(data, positions=range(1, 5), widths=0.5, showfliers=True, flierprops={"markersize": 3})
        for k in range(4):
            axa.plot([k + 1 + 0.08 * ((i * 7) % 5 - 2) for i in range(len(per[k]))], [e["err"] for e in per[k]], ".", color="C0", alpha=0.35, ms=4)
        axa.axhline(0, color="gray", lw=0.8)
        axa.set_ylim(-12, 12)
        axa.set_xticks(range(1, 5))
        axa.set_xticklabels(["%d\n(n=%d)" % (k + 1, len(per[k])) for k in range(4)])
        axa.set_xlabel("pollen number (the slot it was in; 1 leaves first)")
        axa.set_ylabel("flywheel speed against target (%) at the moment the pollen leaves its sensor")
        axa.set_title("Speed as each pollen leaves")
        # speed against time around each pollen's exit
        lo, hi, w = -0.30, 0.30, 0.06
        nb = int(round((hi - lo) / w))
        for k in range(4):
            bins = [[] for _ in range(nb)]
            for e in per[k]:
                for t, v in e["trace"]:
                    j = int((t - lo) / w)
                    if 0 <= j < nb:
                        bins[j].append(v)
            xs_ = [lo + (j + 0.5) * w for j in range(nb) if len(bins[j]) >= 5]
            med = [statistics.fmean(bins[j]) for j in range(nb) if len(bins[j]) >= 5]
            axb.plot(xs_, med, "o-", ms=3, lw=1.3, label=names[k], color="C%d" % k)
        axb.axvline(0, color="k", lw=0.8)
        axb.axhline(0, color="gray", lw=0.8)
        axb.set_xlabel("seconds from the moment the pollen leaves its sensor")
        axb.set_ylabel("mean speed against target (%)")
        axb.set_title("Speed around each pollen's exit (mean)")
        axb.legend(fontsize=8)
        # when each pollen leaves
        data2 = [[e["t_since_start"] for e in per[k]] or [float("nan")] for k in range(4)]
        axc.boxplot(data2, positions=range(1, 5), widths=0.5, showfliers=True, flierprops={"markersize": 3})
        axc.set_xticks(range(1, 5))
        axc.set_xlabel("pollen number")
        axc.set_ylabel("seconds after the salvo started")
        axc.set_title("When each pollen leaves its sensor")
        fig.suptitle("Per pollen, from the salvos that started with all four slots filled and the count at 4 (%d salvos); the slot sensors show a pollen as P, in motion as O, gone as '.'" % len(per[0]))
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, "salvo_pollen.png"), dpi=130)
        plt.close(fig)
        print("PER POLLEN (full four-pollen salvos): speed against target when the pollen leaves its sensor, and the lowest speed within 0.1 s after")
        for k in range(4):
            if not per[k]:
                continue
            ee = [e["err"] for e in per[k]]
            mm = [e["low_after"] for e in per[k]]
            tt = [e["t_since_start"] for e in per[k]]
            print("  pollen %d: n=%d  at exit %+.1f %% median (range %+.1f to %+.1f); lowest within 0.1 s after %+.1f %% median (worst %+.1f); leaves %.2f s after the salvo starts (median)"
                  % (k + 1, len(ee), statistics.median(ee), min(ee), max(ee), statistics.median(mm), min(mm), statistics.median(tt)))
    print("wrote 5 figures to", out_dir)


def pollen_exits(stats):
    """For every salvo that started with the count at 4 and all four slot sensors reading something, the time each pollen leaves its sensor:
    the row after the last 'P' the slot shows during the salvo (a slot that never showed a P: its first '.'), pollen number = slot number.
    Returns four lists of dicts: err (speed against target at that moment), low_after (lowest error in the next 0.1 s), t_since_start, trace [(seconds from exit, error)]."""
    per = [[] for _ in range(4)]
    for x in stats:
        s = x["s"]
        if x["min"] < -20.0 or x["start"] < -20.0:
            continue   # the flywheel was dying (a low battery or a lost hub): not a stability sample
        rows = s["rows"]
        pre = [r for r in rows if f(r, "t") < s["t0"]]
        if not pre or pre[-1].get("count") != "4" or "." in pre[-1].get("slots", ".") or len(pre[-1]["slots"]) != 4:
            continue
        sel = [r for r in rows if s["t0"] <= f(r, "t") <= s["t1"] + 0.3]
        if len(sel) < 10:
            continue
        found = []
        for k in range(4):
            last_p = max([i for i, r in enumerate(sel) if r["slots"][k] == "P"] or [-1])
            if last_p >= 0:
                idx = last_p + 1
            else:
                idx = next((i for i, r in enumerate(sel) if r["slots"][k] == "."), None)
            if idx is None or idx >= len(sel):
                found = None
                break
            found.append(idx)
        if found is None:
            continue
        for k, idx in enumerate(found):
            te = f(sel[idx], "t")
            trace = [(f(r, "t") - te, err(r, s["target"])) for r in rows if abs(f(r, "t") - te) <= 0.35]
            after = [err(r, s["target"]) for r in rows if te <= f(r, "t") <= te + 0.1]
            per[k].append({"err": err(sel[idx], s["target"]), "low_after": min(after) if after else float("nan"),
                           "t_since_start": te - s["t0"], "trace": trace})
    return per


if __name__ == "__main__":
    main()
