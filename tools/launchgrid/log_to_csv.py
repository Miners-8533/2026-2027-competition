"""Turns the GRID POINT lines of a Launch Grid Tuner run (a bringup log) into the CSV make_table.py reads.

    python tools/launchgrid/log_to_csv.py robot-logs/2026-11-14/101500_bringup.log tools/launchgrid/data/2026-11-14_BLUE_REAR_retest_101500.csv

One row per GRID POINT line: the spot NUMBER in the full grid (a retest keeps its numbers; extra spots are 101, 102, ...), the spot, the status, the speed and the aim the operator
left it with, the salvos fired and the pose reached. make_table.py reads every tools/launchgrid/data/*_BLUE_REAR_*.csv in name order and a later file REPLACES an earlier one's row
for the same spot number (so a retest corrects the first run); extra spots (101 and up) are added to the table as spots of their own.
"""
import csv
import math
import re
import sys

PATTERN = re.compile(
    r"GRID POINT (\d+) of \d+: spot \(([-\d.]+), ([-\d.]+)\)( slid)?, ([\d.]+) in, "
    r"(GOOD: speed (\d+) ticks/s \(map (\d+) ([+-]\d+) trim\), aim heading ([-\d.]+) deg \(geometric ([-\d.]+) ([+-][\d.]+) trim\)|NO SHOT possible), "
    r"(\d+) salvo\(s\) fired.*pose \(([-\d.]+), ([-\d.]+), ([-\d.]+) deg\)")
HEADER = ["spot", "spot_x", "spot_y", "distance_in", "status", "speed_ticks_s", "map_speed", "speed_trim", "aim_heading_deg", "geometric_heading_deg", "angle_trim_deg",
          "salvos_fired", "pose_x", "pose_y", "pose_heading_deg", "off_spot_in"]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    n = 0
    with open(dst, "w", newline="") as out:
        w = csv.writer(out)
        w.writerow(HEADER)
        for line in open(src, encoding="utf-8", errors="replace"):
            m = PATTERN.search(line)
            if not m:
                continue
            good = m.group(6).startswith("GOOD")
            x, y = float(m.group(2)), float(m.group(3))
            px, py, ph = float(m.group(14)), float(m.group(15)), float(m.group(16))
            w.writerow([m.group(1), x, y, m.group(5), "GOOD" if good else "NO_SHOT"] + ([m.group(7), m.group(8), m.group(9), m.group(10), m.group(11), m.group(12)] if good else [""] * 6)
                       + [m.group(13), px, py, ph, round(math.hypot(px - x, py - y), 1)])
            n += 1
    print("wrote %d spots to %s" % (n, dst))


if __name__ == "__main__":
    main()
