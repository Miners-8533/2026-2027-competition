"""Summarizes a Color Sensor Study log (the BRINGUP kind): the four guided steps, in order.

    python tools/logs/sensor_study.py                      # the newest bringup log under robot-logs/
    python tools/logs/sensor_study.py path\\to\\bringup.log

Prints, step by step (EMPTY_STILL, EMPTY_SWEEP, FULL_STILL, FULL_SWEEP): the length, how the counter did against the pollen really inside (the
share of the time the sensors lit now and the reported count were below / equal to / above it), the median reading of each sensor for the still
steps, and the RESULT / BASELINE lines the opmode wrote (the lit stretches with the pusher angle, and the empty baseline as the three lines to paste
into ElementCounter). The lit stretches are judged against the empty baseline measured in the same run, not the one stored in ElementCounter.
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
                    try:
                        events.append((float(parts[1]), parts[2]))
                    except ValueError:
                        pass
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

    for take in sorted(takes):
        tr = takes[take]
        t0, t1 = tr[0]["_t"], tr[-1]["_t"]
        kind, known = tr[0]["type"], int(num(tr[0], "known"))
        n = len(tr)
        raw, rep = compare(tr, "raw_count"), compare(tr, "count")
        print("\n=== TAKE %d  %s  (%d pollen inside)  %.1f s ===" % (take, kind, known, t1 - t0))
        print("  sensors lit now (against the STORED baseline) below / equal / above the known: %.0f / %.0f / %.0f %%;  reported count: %.0f / %.0f / %.0f %%" % (
            share(raw["below"], n), share(raw["equal"], n), share(raw["above"], n),
            share(rep["below"], n), share(rep["equal"], n), share(rep["above"], n)))
        if kind.endswith("_STILL"):
            run = [r for r in tr if r["phase"] == "RUN" and num(r, "take_s") >= 1.0]
            if run:
                print("  median reading per sensor (stored-baseline shift in the last column):")
                for i in range(1, N + 1):
                    col = lambda name: statistics.median(num(r, "cs%d_%s" % (i, name)) for r in run)
                    print("    CS%d dist %5.2f cm  alpha %.3f  rgb %.3f / %.3f / %.3f  shift %.3f" % (
                        i, col("dist"), col("alpha"), col("r"), col("g"), col("b"), col("shift")))
        # the opmode's own lines that fall inside this take
        for t, text in events:
            if t0 - 0.5 <= t <= t1 + 5.0 and (text.startswith("RESULT ") or text.startswith("BASELINE")):
                print("  " + text)

    print("\nThe BASELINE lines (from the empty step) go into ElementCounter's BASE_ALPHA, BASE_DIST and BASE_SHARES, then rebuild.")


if __name__ == "__main__":
    main()
