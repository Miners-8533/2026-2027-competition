"""The drivers' own trips to a launch, from the labeled matches of an event: the baseline of the dash SPEED GATE (specs/dashes.md, target T9).

    python -I tools/dash/hand_trips.py --event "Hartland Qualifier 2026" [--out build/dashsim/hand_trips.csv]

A trip is the "find spot" part of a TeleOp cycle (tools/logs/cycle_times.py): from the last pickup that mattered to arriving within 6 in of the place the
next salvo was fired from. For each one this writes where it started (pose and field velocity), which cell was launched at, where the salvo was fired from,
how long the drive took and how far it was in a straight line. A cycle in which a Hive Dash ran is marked (dash = 1): it is not a hand-driven trip.

tools/dash/DashSim.java reads the CSV (run-sim.cmd trips ...) and drives the real Hive Dash from the same starts, so the two times can be compared trip by trip.
"""
import argparse
import csv
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "logs"))
import cycle_times as ct  # noqa: E402

TARGET = re.compile(r"target ((?:BLUE|RED)_(?:REAR|AUDIENCE))")


def pose_rows(path):
    """(t, x, y, heading deg) of every sample row."""
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("S,"):
                p = line.split(",", 9)
                try:
                    out.append((float(p[1]), float(p[5]), float(p[6]), float(p[7])))
                except (ValueError, IndexError):
                    pass
    return out


def at(rows, t):
    best = None
    for r in rows:
        if r[0] <= t:
            best = r
        else:
            break
    return best


def velocity(rows, t, window=0.25):
    """Field velocity (in/s) over the window that ends at t."""
    a, b = at(rows, t - window), at(rows, t)
    if a is None or b is None or b[0] - a[0] < 0.05:
        return 0.0, 0.0
    dt = b[0] - a[0]
    return (b[1] - a[1]) / dt, (b[2] - a[2]) / dt


def trips(path, radius):
    start, end, salvos, locks, full_on, full_off, elements = ct.teleop(path)
    ev = ct.events(path)
    launches = [(t, TARGET.search(m).group(1)) for t, m in ev if m.startswith("LAUNCH requested") and TARGET.search(m)]
    dashes = [t for t, m in ev if m.startswith("HIVE DASH")]
    samples = ct.pose_samples(path)
    rows = pose_rows(path)
    out = []
    for i in range(len(salvos) - 1):
        s, n = salvos[i], salvos[i + 1]
        if s["done"] is None:
            continue
        lk = [t for t in locks if s["done"] <= t <= n["start"]]
        if not lk:
            continue
        lock_on = lk[-1]
        reach, _final = ct.reach_time(elements, s["done"], lock_on)
        arr = ct.arrival_time(samples, s["done"], n["start"], radius)
        if arr is None:
            continue
        arrive = max(arr, reach)
        cell = next((c for t, c in reversed(launches) if t <= n["start"] + 1e-6), None)
        a, b = at(rows, reach), at(rows, n["start"])
        if cell is None or a is None or b is None:
            continue
        vx, vy = velocity(rows, reach)
        out.append({
            "t": reach, "x": a[1], "y": a[2], "hdg_deg": a[3], "vx": vx, "vy": vy, "cell": cell,
            "fire_x": b[1], "fire_y": b[2], "find_s": arrive - reach, "at_spot_s": max(0.0, lock_on - arrive), "aim_s": n["start"] - max(arrive, lock_on),
            "dist_in": ((a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2) ** 0.5, "dash": int(any(s["done"] <= t <= n["start"] for t in dashes)),
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True)
    ap.add_argument("--radius", type=float, default=6.0)
    ap.add_argument("--out", default=os.path.join("build", "dashsim", "hand_trips.csv"))
    args = ap.parse_args()
    labels = os.path.join(ct.ROOT, "events", args.event, "labels.csv")
    all_rows = []
    with open(labels, encoding="utf-8") as f:
        for lab in csv.DictReader(f):
            if not lab["log"].endswith("_teleop"):
                continue
            path = os.path.join(ct.ROOT, lab["log"] + ".log")
            if not os.path.exists(path):
                continue
            for r in trips(path, args.radius):
                r["match"] = lab["label"]
                all_rows.append(r)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    cols = ["match", "t", "x", "y", "hdg_deg", "vx", "vy", "cell", "fire_x", "fire_y", "find_s", "at_spot_s", "aim_s", "dist_in", "dash"]
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in all_rows:
            w.writerow([r["match"], "%.2f" % r["t"], "%.1f" % r["x"], "%.1f" % r["y"], "%.1f" % r["hdg_deg"], "%.1f" % r["vx"], "%.1f" % r["vy"], r["cell"],
                        "%.1f" % r["fire_x"], "%.1f" % r["fire_y"], "%.2f" % r["find_s"], "%.2f" % r["at_spot_s"], "%.2f" % r["aim_s"], "%.1f" % r["dist_in"], r["dash"]])
    hand = [r for r in all_rows if not r["dash"]]
    moved = [r for r in hand if r["dist_in"] >= 12.0]
    print("%d trips in %s, %d of them with a Hive Dash; %d hand-driven trips of 12 in or more" % (len(all_rows), args.event, len(all_rows) - len(hand), len(moved)))
    if moved:
        find = [r["find_s"] for r in moved]
        dist = [r["dist_in"] for r in moved]
        speed = [r["dist_in"] / r["find_s"] for r in moved if r["find_s"] > 0.2]
        print("  drive time: median %.2f s, mean %.2f s, 90th %.2f s" % (statistics.median(find), statistics.mean(find), sorted(find)[int(0.9 * (len(find) - 1))]))
        print("  straight-line distance: median %.0f in, mean %.0f in" % (statistics.median(dist), statistics.mean(dist)))
        if speed:
            print("  straight-line speed: median %.0f in/s" % statistics.median(speed))
        for cell in sorted(set(r["cell"] for r in moved)):
            sub = [r for r in moved if r["cell"] == cell]
            print("  %-14s %3d trips, median %.2f s over %.0f in" % (cell, len(sub), statistics.median([r["find_s"] for r in sub]), statistics.median([r["dist_in"] for r in sub])))
    print("CSV: " + args.out)


if __name__ == "__main__":
    main()
