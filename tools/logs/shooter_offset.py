"""Summarizes the good-salvo marks of a Shooter Offset Finder log (the BRINGUP kind).

    python tools/logs/shooter_offset.py                    # the newest bringup log under robot-logs/
    python tools/logs/shooter_offset.py path\to\bringup.log

Each MARK good carries the pose at the last LAUNCH and "cell center N in left of the front line": the sideways distance from the robot's front line to the cell's
center. For a salvo that went through the center with the shooter lined up, that is the shooter's sideways offset from the pose point (positive = left) plus any
hive or aim error. Per cell this prints the marks, their mean and spread. When both cells of a hive were marked (one shot facing +X, one facing -X) it also splits the
two: with the robot facing +X the number is  s - t,  facing -X it is  s + t,  where s is the shooter's offset (left positive, the same at both headings, robot frame)
and t is a sideways error of the hive's cell center in the field (+Y positive, the same at both cells of the hive). So  s = mean of the two,  t = half their difference.
A cell marked at two or more distances (8 in or more apart) also gets a straight-line fit of the number against distance: the slope is the aim angle error.
"""
import glob
import math
import os
import re
import statistics
import sys
from collections import defaultdict

MARK = re.compile(r"E,([\d.]+),MARK (good|landed LEFT|landed RIGHT) \| AT THE LAST (?:FIRE|LAUNCH): (?:FIRE|LAUNCH) pose \(([-\d.]+), ([-\d.]+), ([-\d.]+) deg\), cell (\w+) "
                  r"\(([-\d.]+), ([-\d.]+)\) ([\d.]+) in, bearing ([-\d.]+) deg, cell center ([-+\d.]+) deg / ([-+\d.]+) in left of the front line")


def newest(root):
    paths = glob.glob(os.path.join(root, "robot-logs", "*", "*_bringup.log"))
    return max(paths, key=os.path.getmtime) if paths else None


def fit(points):
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    sxx = sum((x - mx) ** 2 for x in xs)
    return (sum((x - mx) * (y - my) for x, y in points) / sxx if sxx else 0.0), mx, my


def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    path = sys.argv[1] if len(sys.argv) > 1 else newest(root)
    if not path or not os.path.exists(path):
        sys.exit("No bringup log found. Pass the path of a Shooter Offset Finder log.")
    rows = []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = MARK.search(line)
        if m:
            t, outcome, x, y, h, cell, cx, cy, d, b, e, left = m.groups()
            rows.append(dict(t=float(t), outcome=outcome, x=float(x), y=float(y), h=float(h), cell=cell, d=float(d), err=float(e), left=float(left)))
    print("Log:", path, "(%d marks)" % len(rows))
    good = [r for r in rows if r["outcome"] == "good"]
    if not good:
        sys.exit("No good marks.")
    by = defaultdict(list)
    for r in good:
        by[r["cell"]].append(r)
    means = {}
    print("\nGOOD SALVOS BY CELL")
    for cell in sorted(by):
        rs = by[cell]
        lefts = [r["left"] for r in rs]
        means[cell] = statistics.mean(lefts)
        sd = statistics.pstdev(lefts) if len(lefts) > 1 else 0.0
        print("%-14s n=%2d  cell center left of the front line: mean %+.2f in, sd %.2f, range %+.2f to %+.2f; heading %.0f to %.0f deg; distance %.1f to %.1f in" % (
            cell, len(rs), means[cell], sd, min(lefts), max(lefts), min(r["h"] for r in rs), max(r["h"] for r in rs), min(r["d"] for r in rs), max(r["d"] for r in rs)))
        for r in rs:
            print("      %7.1fs pose (%.2f, %.2f, %.1f)  %5.1f in  left %+.2f in" % (r["t"], r["x"], r["y"], r["h"], r["d"], r["left"]))
        if max(r["d"] for r in rs) - min(r["d"] for r in rs) >= 8.0 and len(rs) >= 3:
            slope, mx, my = fit([(r["d"], r["left"]) for r in rs])
            print("      fit against distance: slope %+.3f in/in = aim angle error %+.2f deg; at 0 in distance %+.2f in (the offset)" % (slope, math.degrees(math.atan(slope)), my - slope * mx))
    print("\nSPLIT OF THE SHOOTER OFFSET s (robot frame) AND THE HIVE ERROR t (field +Y), per hive")
    for hive, rear, aud in (("BLUE hive", "BLUE_REAR", "BLUE_AUDIENCE"), ("RED hive", "RED_REAR", "RED_AUDIENCE")):
        # the rear cell is shot facing +X, the audience cell facing -X
        if rear in means and aud in means:
            s = (means[rear] + means[aud]) / 2.0
            t = (means[aud] - means[rear]) / 2.0
            print("%s: rear (facing +X) %+.2f, audience (facing -X) %+.2f  =>  s = %+.2f in (left positive), t = %+.2f in (the cell centers sit this far toward +Y of where the code puts them)" % (hive, means[rear], means[aud], s, t))
        else:
            print("%s: needs good marks at both its cells (rear and audience)" % hive)
    if all(c in means for c in ("BLUE_REAR", "BLUE_AUDIENCE", "RED_REAR", "RED_AUDIENCE")):
        s_all = statistics.mean([(means["BLUE_REAR"] + means["BLUE_AUDIENCE"]) / 2.0, (means["RED_REAR"] + means["RED_AUDIENCE"]) / 2.0])
        print("both hives: s = %+.2f in. If the two hives give about the same s and t has the same sign for both (in +Y), the hive error is a field effect; if t flips sign between the hives, it is a heading or alliance-side effect." % s_all)


if __name__ == "__main__":
    main()
