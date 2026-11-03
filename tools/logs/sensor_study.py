"""Summarizes a Color Sensor Study log (the BRINGUP kind), take by take.

    python tools/logs/sensor_study.py                      # the newest bringup log under robot-logs/
    python tools/logs/sensor_study.py path\\to\\bringup.log

For each take: the type, the pollen really inside (known), the length, and how the counter did against the truth: the share of the time the
sensors lit now (raw_count) and the reported count (count) were below, equal to and above the known number. Then the same pooled by known count
(the STILL and JOSTLE takes), with the false full rate (reported 4 while fewer were inside). For STILL takes with pollen, the median reading of
each sensor; for SWEEP takes, each stretch where a sensor was lit with the pusher angle it covered, so the pollen and the arm passing can be told apart.
"""
import glob
import os
import statistics
import sys
from collections import Counter, defaultdict

N = 4


def newest_bringup(root):
    paths = glob.glob(os.path.join(root, "robot-logs", "*", "*_bringup.log"))
    if not paths:
        latest = os.path.join(root, "robot-logs", "latest", "bringup.log")
        return latest if os.path.exists(latest) else None
    return max(paths, key=os.path.getmtime)


def read_rows(path):
    header = None
    rows = []
    events = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("H,"):
                header = line.split(",")[1:]
            elif line.startswith("E,"):
                parts = line.split(",", 2)
                if len(parts) == 3:
                    events.append((float(parts[1]), parts[2]))
            elif line.startswith("S,") and header:
                values = line.split(",")[1:]
                if len(values) != len(header):
                    continue
                row = dict(zip(header, values))
                try:
                    row["_t"] = float(row["t"])
                except ValueError:
                    continue
                rows.append(row)
    return rows, events


def num(row, key, default=float("nan")):
    try:
        return float(row[key])
    except (KeyError, ValueError):
        return default


def share(counter, total):
    return 100.0 * counter / total if total else 0.0


def compare(rows, field):
    """Counts of rows where the field is below / equal to / above the known number."""
    c = Counter()
    for r in rows:
        v, k = int(num(r, field)), int(num(r, "known"))
        c["below" if v < k else "equal" if v == k else "above"] += 1
    return c


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    path = sys.argv[1] if len(sys.argv) > 1 else newest_bringup(root)
    if not path or not os.path.exists(path):
        sys.exit("No bringup log found. Pass the path of a Color Sensor Study log.")
    rows, events = read_rows(path)
    print("Log:", path, "(%d rows)" % len(rows))
    takes = defaultdict(list)
    for r in rows:
        take = int(num(r, "take", 0))
        if take > 0:
            takes[take].append(r)
    if not takes:
        sys.exit("No takes in this log.")

    pooled = defaultdict(list)
    print("\nTAKES")
    print("take type    known intake        len   sensors lit now: below/equal/above   reported: below/equal/above")
    for take in sorted(takes):
        tr = takes[take]
        t0, t1 = tr[0]["_t"], tr[-1]["_t"]
        intake = "/".join(sorted(set(r["intake"] for r in tr)))
        knowns = sorted(set(int(num(r, "known")) for r in tr))
        raw, rep = compare(tr, "raw_count"), compare(tr, "count")
        n = len(tr)
        print("%4d %-7s %-5s %-12s %5.1fs   %5.1f / %5.1f / %5.1f %%          %5.1f / %5.1f / %5.1f %%" % (
            take, tr[0]["type"], "/".join(map(str, knowns)), intake, t1 - t0,
            share(raw["below"], n), share(raw["equal"], n), share(raw["above"], n),
            share(rep["below"], n), share(rep["equal"], n), share(rep["above"], n)))
        if tr[0]["type"] in ("STILL", "JOSTLE") and len(knowns) == 1:
            pooled[knowns[0]].extend(tr)

    if pooled:
        print("\nPOOLED BY POLLEN INSIDE (STILL and JOSTLE takes)")
        print("known  rows   raw count seen (share of the time)          reported count (share of the time)")
        for k in sorted(pooled):
            tr = pooled[k]
            raw = Counter(int(num(r, "raw_count")) for r in tr)
            rep = Counter(int(num(r, "count")) for r in tr)
            fmt = lambda c: "  ".join("%d:%4.1f%%" % (v, share(c[v], len(tr))) for v in sorted(c))
            print("%5d %5d   %-42s %s" % (k, len(tr), fmt(raw), fmt(rep)))
            if k < 4:
                full = sum(1 for r in tr if int(num(r, "count")) >= 4)
                print("      false FULL (reported 4 with %d inside): %.1f %% of the time" % (k, share(full, len(tr))))

    for take in sorted(takes):
        tr = takes[take]
        kind, known = tr[0]["type"], int(num(tr[0], "known"))
        if kind == "STILL" and known > 0:
            print("\nTAKE %d, STILL with %d inside: median reading per sensor" % (take, known))
            for i in range(1, N + 1):
                col = lambda name: statistics.median(num(r, "cs%d_%s" % (i, name)) for r in tr)
                print("  CS%d dist %5.2f cm  alpha %.3f  rgb %.2f / %.2f / %.2f  shift %.3f  lit %3.0f %%" % (
                    i, col("dist"), col("alpha"), col("r"), col("g"), col("b"), col("shift"), 100.0 * statistics.mean(num(r, "cs%d_on" % i) for r in tr)))
        if kind == "SWEEP":
            print("\nTAKE %d, SWEEP with %d inside: where each sensor was lit (pusher angle is the unwrapped degrees from HOLD; backward is negative)" % (take, known))
            for i in range(1, N + 1):
                stretches, start = [], None
                for idx, r in enumerate(tr):
                    on = num(r, "cs%d_on" % i) > 0.5
                    if on and start is None:
                        start = idx
                    if (not on or idx == len(tr) - 1) and start is not None:
                        seg = tr[start:idx + 1 if on else idx]
                        stretches.append(seg)
                        start = None
                print("  CS%d: %d stretch(es) lit" % (i, len(stretches)))
                for seg in stretches:
                    angles = [num(r, "pusher_unwrapped_deg") for r in seg]
                    print("     %5.1f-%5.1f s  pusher %7.1f to %7.1f deg  dist %.2f  alpha %.3f  shift %.3f  slots %s" % (
                        seg[0]["_t"], seg[-1]["_t"], min(angles), max(angles),
                        statistics.median(num(r, "cs%d_dist" % i) for r in seg), statistics.median(num(r, "cs%d_alpha" % i) for r in seg),
                        max(num(r, "cs%d_shift" % i) for r in seg), seg[len(seg) // 2]["slots"]))


if __name__ == "__main__":
    main()
