"""Post-match health check of the element sensors' empty baselines, from the run logs (competition or practice).

    python tools/logs/baseline_health.py                         # every TeleOp / auto log under robot-logs/
    python tools/logs/baseline_health.py --since 2026-11-05      # only runs from that day on (e.g. the competition day)
    python tools/logs/baseline_health.py path\to\teleop.log      # one log

Two sources, the better one first:
  * COUNTER EMPTY lines (from 2026-11-04 on): once a second while the track reads empty, the four colour shifts from the stored empty baselines. Presence turns ON at a shift of 0.05
    and OFF below 0.03, so a sensor whose empty shift sits near 0.03 is one bad lighting change away from reading a phantom pollen (the 2026-11-03 false full was CS2 at 0.060).
  * for older logs, what the count does after a salvo: the track is empty then, so if the count is still above 0, with the same sensors lit, from 1 to 3 s after the salvo ended
    (the intake is stopped or the robot is full-empty by then), the lit sensor is a phantom (the 2026-11-03 CS2 case: the count fell to 1, not 0, after every salvo).
    ('O' and 'A' at CS4 are the pusher arm's edge and are not counted.)

Verdict per sensor: OK (median shift under 0.02), WATCH (0.02 to 0.03, or a phantom after 1 salvo), RE-MEASURE (0.03 or more, or a phantom after 2 or more salvos).
To re-measure: Color Sensor Study, an empty STILL take in the pit, copy the medians into ElementCounter (see design/color-sensor-study.md). CS4 sees the pusher arm part of the time,
so its shift is split by the `arm` flag and judged on the `clear` rows.
"""
import argparse
import glob
import os
import re
import statistics

EMPTY = re.compile(r"COUNTER EMPTY shift ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+) \| alpha ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+) \| dist ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+) \| arm (near CS4|clear)")
BASE = re.compile(r"COUNTER baselines: (.*?) \| presence ON")


def scan(path):
    shifts = [[], [], [], []]
    shifts_arm = []
    base = None
    phantom = [0, 0, 0, 0]
    salvos_seen = 0
    rows, salvo_ends = [], []
    header = None
    for line in open(path, encoding="utf-8", errors="ignore"):
        if line.startswith("H,"):
            header = line.rstrip("\n").split(",")[1:]
        elif line.startswith("E,"):
            p = line.rstrip("\n").split(",", 2)
            if len(p) < 3:
                continue
            m = EMPTY.search(p[2])
            if m:
                v = [float(x) for x in m.groups()[:4]]
                if m.group(13) == "clear":
                    for i in range(4):
                        shifts[i].append(v[i])
                else:
                    shifts_arm.append(v)
                continue
            b = BASE.search(p[2])
            if b:
                base = b.group(1)
            if p[2].startswith("SALVO done"):
                try:
                    salvo_ends.append(float(p[1]))
                except ValueError:
                    pass
        elif line.startswith("S,") and header:
            parts = line.rstrip("\n").split(",")[1:]
            if len(parts) != len(header):
                continue
            r = dict(zip(header, parts))
            try:
                rows.append((float(r["t"]), int(r["count"]), r.get("slots", ""), r.get("launch", r.get("fire"))))
            except (KeyError, ValueError):
                pass
    for k, t1 in enumerate(salvo_ends):
        nxt = salvo_ends[k + 1] if k + 1 < len(salvo_ends) else 1e9
        win = [x for x in rows if t1 + 1.0 <= x[0] <= min(t1 + 3.0, nxt - 0.5) and x[3] == "idle"]
        if len(win) < 5:
            continue
        salvos_seen += 1
        if all(x[1] > 0 for x in win) and len(set(x[2] for x in win)) == 1 and len(win[0][2]) == 4:
            for i, ch in enumerate(win[0][2]):
                if ch in "PRB" or (ch == "O" and i < 3):
                    phantom[i] += 1
    return shifts, shifts_arm, base, phantom, salvos_seen


def verdict(median, phantom_salvos):
    if median >= 0.03 or phantom_salvos >= 2:
        return "RE-MEASURE"
    if median >= 0.02 or phantom_salvos == 1:
        return "WATCH"
    return "OK"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="*")
    ap.add_argument("--root", default="robot-logs")
    ap.add_argument("--since", default="0000-00-00")
    args = ap.parse_args()
    files = args.logs
    if not files:
        for kind in ("teleop", "auto"):
            files += glob.glob(os.path.join(args.root, "*-*-*", "*_%s.log" % kind))
        files = sorted([f for f in files if os.path.basename(os.path.dirname(f)) >= args.since], key=lambda p: (os.path.basename(os.path.dirname(p)), os.path.basename(p)))
    all_shifts = [[], [], [], []]
    all_phantom = [0, 0, 0, 0]
    all_salvos = 0
    print("%-22s %-7s %s" % ("run", "rows", "per sensor: median empty shift (95th percentile) | salvos that left a phantom, of the salvos checked"))
    for f in files:
        shifts, shifts_arm, base, phantom, salvos_seen = scan(f)
        name = os.path.basename(os.path.dirname(f)) + " " + os.path.basename(f)[:6]
        n = len(shifts[0])
        if n:
            cells = ["CS%d %.3f (%.3f)" % (i + 1, statistics.median(shifts[i]), sorted(shifts[i])[int(0.95 * (n - 1))]) for i in range(4)]
        else:
            cells = ["no COUNTER EMPTY lines"]
        ph = ", ".join("CS%d %d" % (i + 1, c) for i, c in enumerate(phantom) if c) or "none"
        print("%-22s %-7d %s | phantom after a salvo: %s of %d" % (name, n, "  ".join(cells), ph, salvos_seen))
        for i in range(4):
            all_shifts[i] += shifts[i]
            all_phantom[i] += phantom[i]
        all_salvos += salvos_seen
    print()
    for i in range(4):
        s = all_shifts[i]
        ph = all_phantom[i]
        if s:
            med = statistics.median(s)
            p95 = sorted(s)[int(0.95 * (len(s) - 1))]
            print("CS%d: %s   median empty shift %.3f, 95th percentile %.3f over %d empty seconds; left a phantom after %d of %d salvos" % (i + 1, verdict(med, ph), med, p95, len(s), ph, all_salvos))
        elif all_salvos:
            print("CS%d: %s   (no shift lines; judged on the salvos only) left a phantom after %d of %d salvos" % (i + 1, verdict(0.0, ph), ph, all_salvos))
        else:
            print("CS%d: no empty-track data in these logs" % (i + 1))
    print("\nThe thresholds: presence ON at a shift of 0.05, OFF below 0.03. RE-MEASURE the baselines in the pit (Color Sensor Study, empty STILL take) when any sensor says so.")


if __name__ == "__main__":
    main()
