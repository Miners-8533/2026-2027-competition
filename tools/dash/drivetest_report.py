"""Drive Characterization report (design/dash-planner-grid.md, "Drive characterization").

Reads a drivetest.log (opmodes/tuning/DriveCharacterization: a row every loop with the pose, the field velocity, the commanded robot powers, the wheel powers,
the battery and one motor current per loop) and prints, per test and segment, what the drive did in the ROBOT frame: the speed along and across the
robot and the turn rate at the end of each segment, the rise to 63 percent for steps, the deceleration after a drop in power, and the heading and
sideways drift of the straight runs. Usage (repo root):

    python -I tools/dash/drivetest_report.py [robot-logs/latest/drivetest.log] [--csv out.csv]

The --csv option writes the rows in the robot frame (t, test, seg, phase, cmd_fwd, cmd_left, cmd_turn, v_fwd, v_left, w, vbat, heading) for fitting.
"""
import math
import re
import sys

EV_TEST = re.compile(r"DRIVE TEST (\d+) of \d+: (.*?); start")
EV_SEG = re.compile(r"DRIVE TEST (\d+) segment (\d+) of (\d+): .*-> robot powers forward ([-+\d.]+), left ([-+\d.]+), turn ([-+\d.]+)")
EV_FENCE = re.compile(r"DRIVE TEST (\d+) GEOFENCE \(([^)]*)\)")


def load(path):
    head, rows, events = None, [], []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("H,"):
                head = line.split(",")[1:]
            elif line.startswith("S,") and head:
                p = line.split(",")[1:]
                if len(p) != len(head):
                    continue
                r = {}
                for k, v in zip(head, p):
                    try:
                        r[k] = float(v)
                    except ValueError:
                        r[k] = v
                rows.append(r)
            elif line.startswith("E,"):
                p = line.split(",", 2)
                events.append((float(p[1]), p[2]))
    return rows, events


def robot_frame(r):
    h = math.radians(r["hdg_deg"])
    c, s = math.cos(h), math.sin(h)
    return r["vx"] * c + r["vy"] * s, -r["vx"] * s + r["vy"] * c, r["w_dps"]


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def main():
    args = [a for a in sys.argv[1:]]
    csv = None
    if "--csv" in args:
        i = args.index("--csv")
        csv = args[i + 1]
        del args[i:i + 2]
    path = args[0] if args else "robot-logs/latest/drivetest.log"
    rows, events = load(path)
    names, fences = {}, {}
    for t, e in events:
        m = EV_TEST.search(e)
        if m:
            names[int(m.group(1))] = m.group(2)
        m = EV_FENCE.search(e)
        if m:
            fences[int(m.group(1))] = m.group(2)
    for r in rows:
        r["vf"], r["vl"], r["w"] = robot_frame(r)
    dts = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])]
    dts.sort()
    print(f"{path}: {len(rows)} rows, loop median {1000 * dts[len(dts) // 2]:.1f} ms; battery {min(r['vbat'] for r in rows):.2f} to {max(r['vbat'] for r in rows):.2f} V")
    if csv:
        with open(csv, "w", encoding="utf-8") as f:
            f.write("t,test,seg,phase,cmd_fwd,cmd_left,cmd_turn,v_fwd,v_left,w_dps,vbat,hdg_deg,x,y\n")
            for r in rows:
                f.write(f"{r['t']:.3f},{int(r['test'])},{int(r['seg'])},{r['phase']},{r['cmd_fwd']:.3f},{r['cmd_left']:.3f},{r['cmd_turn']:.3f},{r['vf']:.2f},{r['vl']:.2f},{r['w']:.1f},{r['vbat']:.2f},{r['hdg_deg']:.2f},{r['x']:.2f},{r['y']:.2f}\n")
    # the RUN rows by test and segment
    by = {}
    for r in rows:
        if r["phase"] != "RUN":
            continue
        by.setdefault(int(r["test"]), {}).setdefault(int(r["seg"]), []).append(r)
    for test in sorted(by):
        print(f"\nTEST {test}: {names.get(test, '?')}" + (f"   [GEOFENCE: {fences[test]}]" if test in fences else ""))
        segs = by[test]
        prev_end = None
        for seg in sorted(segs):
            rs = segs[seg]
            t0, t1 = rs[0]["t"], rs[-1]["t"]
            cf, cl, ct = rs[0]["cmd_fwd"], rs[0]["cmd_left"], rs[0]["cmd_turn"]
            tail = [r for r in rs if r["t"] >= t1 - 0.15]
            vf, vl, w = mean([r["vf"] for r in tail]), mean([r["vl"] for r in tail]), mean([r["w"] for r in tail])
            vb = mean([r["vbat"] for r in rs])
            line = f"  seg {seg}: {t1 - t0:4.2f} s  cmd fwd {cf:+.2f} left {cl:+.2f} turn {ct:+.2f}  end: fwd {vf:+6.1f} left {vl:+6.1f} in/s, turn {w:+6.1f} deg/s  ({vb:.2f} V)"
            # rise: time to 63 percent of the end speed along the commanded direction
            spd = [math.hypot(r["vf"], r["vl"]) for r in rs]
            end = math.hypot(vf, vl)
            start = prev_end if prev_end is not None else spd[0]
            if abs(end - start) > 8:
                target = start + 0.63 * (end - start)
                for r, s in zip(rs, spd):
                    if (end > start and s >= target) or (end < start and s <= target):
                        line += f"; 63% after {r['t'] - t0:.2f} s"
                        break
            # deceleration over the first part of a slowdown (speed from 90 to 30 percent of the start)
            if prev_end is not None and end < 0.5 * prev_end and prev_end > 15:
                pts = [(r["t"] - t0, s) for r, s in zip(rs, spd) if 0.3 * prev_end <= s <= 0.9 * prev_end]
                if len(pts) >= 3:
                    n = len(pts)
                    mt, ms = mean([p[0] for p in pts]), mean([p[1] for p in pts])
                    num = sum((p[0] - mt) * (p[1] - ms) for p in pts)
                    den = sum((p[0] - mt) ** 2 for p in pts)
                    if den > 0:
                        line += f"; slows at {-num / den:.0f} in/s^2"
            # straight runs: heading change and sideways drift
            if abs(ct) < 1e-3 and t1 - t0 > 0.5:
                dh = rs[-1]["hdg_deg"] - rs[0]["hdg_deg"]
                dh = (dh + 180) % 360 - 180
                line += f"; heading {dh:+.1f} deg"
            print(line)
            prev_end = end
    print()
    for t, e in events:
        if "SLOW LOOP" in e:
            print(f"  {t:.2f} {e}")


if __name__ == "__main__":
    main()
