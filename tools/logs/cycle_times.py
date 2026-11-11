"""Cycle time analysis for the labeled matches of an event.

    python tools/logs/cycle_times.py --event "Hartland Qualifier 2026"

Reads robot-logs/events/<event>/labels.csv (made by event_day.py label) and writes robot-logs/events/<event>/cycle_times.md.

TeleOp cycle = salvo start to the next salvo start. Two moments split it besides the salvos and the lock:
  pickup  when the stack LAST rose to the count it held when the lock came on (the last pickup that mattered)
  arrival when the robot came within --radius (6 in) of the spot it fired the next salvo from and stayed there (from the pose
          samples; turning in place and small corrections stay inside the radius)
The parts, which add up to the cycle:
  salvo     SALVO started to SALVO done (the pusher turning, the flywheel shooting)
  collect   SALVO done to the pickup: driving out, intaking, filling the stack
  find spot the pickup to the arrival: the drive to the launch spot with the load, locked or not
              lock drive  the part of find spot with the target lock already on (a Hive Dash counts as a lock)
              find in     straight-line distance from the pickup to the launch spot
  at spot   at the launch spot before the lock came on: stopped, not yet locked (0 when the lock came on before the arrival)
  aim       the later of the arrival and the lock coming on, to the next SALVO started (turning onto the target, flywheel band wait)
  lock      lock on to SALVO started as before (= lock drive + aim), for comparison with older reports
  gather    SALVO done to the lock that led to the next salvo (a lock the driver released again counts as gather)
  full      of the gather time, how long the stack already held 4 (FULL) before the lock: the driver was ready and not yet locked
The time before the first salvo (from RUN STARTED) and after the last salvo is reported on its own.

Auto: the AUTO TIMING lines, summed by kind (drive, aim, launch, flower jostle, waits).
"""
import argparse
import csv
import os
import re
import statistics

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "robot-logs")
ROW = re.compile(r"^E,([0-9.]+),(.*)$")
HELD = re.compile(r"(\d) held")
ELEM = re.compile(r"ELEMENTS (\d) -> (\d)")


def events(path):
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            m = ROW.match(line.rstrip("\n"))
            if m:
                out.append((float(m.group(1)), m.group(2)))
    return out


def log_end(path):
    last = 0.0
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("S,") or line.startswith("E,"):
                try:
                    last = float(line.split(",", 2)[1])
                except ValueError:
                    pass
    return last


def teleop(path):
    ev = events(path)
    start = next((t for t, m in ev if m.startswith("RUN STARTED")), 0.0)
    end = log_end(path)
    salvos = []  # dict: start, done, held
    for t, m in ev:
        if m.startswith("SALVO started"):
            h = HELD.search(m)
            salvos.append({"start": t, "done": None, "held": int(h.group(1)) if h else 0})
        elif m.startswith("SALVO done") and salvos and salvos[-1]["done"] is None:
            salvos[-1]["done"] = t
    locks = [t for t, m in ev if m.startswith("TARGET LOCK on") or m.startswith("HIVE DASH start")]
    full_on = [t for t, m in ev if m.startswith("WARNING FULL")]
    full_off = [t for t, m in ev if m.startswith("WARNING cleared: FULL")]
    elements = []
    for t, m in ev:
        mm = ELEM.match(m)
        if mm:
            elements.append((t, int(mm.group(1)), int(mm.group(2))))
    return start, end, salvos, locks, full_on, full_off, elements


def pose_samples(path):
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if line.startswith("S,"):
                p = line.split(",", 8)
                try:
                    out.append((float(p[1]), float(p[5]), float(p[6])))
                except (ValueError, IndexError):
                    pass
    return out


def arrival_time(samples, a, b, radius):
    """When the robot came within radius of its pose at b and stayed there, looking only in [a, b]; a if it never left."""
    fire = pose_at(samples, b)
    if fire is None:
        return None
    arr = a
    prev_out = False
    for st, x, y in samples:
        if st < a:
            continue
        if st > b:
            break
        out = ((x - fire[0]) ** 2 + (y - fire[1]) ** 2) ** 0.5 > radius
        if prev_out and not out:
            arr = st
        prev_out = out
    return arr


def pose_at(samples, t):
    best = None
    for st, x, y in samples:
        if st <= t:
            best = (x, y)
        else:
            break
    return best


def count_at(elements, t):
    c = 0
    for et, a, b in elements:
        if et <= t:
            c = b
        else:
            break
    return c


def reach_time(elements, done, lock_on):
    """When the stack LAST rose to the count it held at the lock (the last pickup that mattered; a count that flickers down and
    back up counts its last rise), scanning from just after the salvo's own drop."""
    t0 = done
    for et, a, b in elements:
        if done <= et <= done + 0.6:
            t0 = et  # the count falling after the salvo
    final = count_at(elements, lock_on)
    reach = t0
    for et, a, b in elements:
        if t0 < et <= lock_on and a < final <= b:
            reach = et
    return reach, final


def full_time_between(a, b, full_on, full_off):
    """Seconds in [a, b] during which the FULL warning was up."""
    tot = 0.0
    pts = sorted([(t, 1) for t in full_on] + [(t, 0) for t in full_off])
    on = False
    for t, st in pts:
        if t <= a:
            on = bool(st)
    cur = a if on else None
    for t, st in pts:
        if t <= a or t >= b:
            continue
        if st and cur is None:
            cur = t
        elif not st and cur is not None:
            tot += t - cur
            cur = None
    if cur is not None:
        tot += b - cur
    return tot


def teleop_stats(path, radius):
    start, end, salvos, locks, full_on, full_off, elements = teleop(path)
    samples = pose_samples(path)
    rows = []
    for i in range(len(salvos) - 1):
        s, n = salvos[i], salvos[i + 1]
        if s["done"] is None:
            continue
        lk = [t for t in locks if s["done"] <= t <= n["start"]]
        lock_on = lk[-1] if lk else None  # the LAST lock before the salvo: an earlier one the driver released again belongs to the gather
        cycle = n["start"] - s["start"]
        salvo_t = s["done"] - s["start"]
        if lock_on is None:
            continue  # lock held across the salvo; the split is not defined
        gather = lock_on - s["done"]
        lock_t = n["start"] - lock_on
        full = full_time_between(s["done"], lock_on, full_on, full_off)
        reach, final = reach_time(elements, s["done"], lock_on)
        collect = reach - s["done"]
        arr = arrival_time(samples, s["done"], n["start"], radius)
        if arr is None:
            arr = lock_on  # no pose samples: fall back to the old split (transit = find spot, lock = aim)
        arrive = max(arr, reach)  # still picking up after reaching the spot: the find starts at the pickup
        find = arrive - reach
        lock_drive = max(0.0, arrive - lock_on)
        at_spot = max(0.0, lock_on - arrive)
        aim = n["start"] - max(arrive, lock_on)
        a, b = pose_at(samples, reach), pose_at(samples, n["start"])
        dist = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 if a and b else float("nan")
        rows.append({"cycle": cycle, "salvo": salvo_t, "gather": gather, "collect": collect, "find": find, "lock_drive": lock_drive,
                     "at_spot": at_spot, "aim": aim, "dist": dist, "lock": lock_t, "full": min(full, gather), "held": n["held"]})
    return {
        "length": end - start,
        "salvos": len(salvos),
        "pollen": sum(s["held"] for s in salvos),
        "first": (salvos[0]["start"] - start) if salvos else None,
        "tail": (end - salvos[-1]["done"]) if salvos and salvos[-1]["done"] else None,
        "cycles": rows,
    }


def auto_stats(path):
    kinds = {"drive": 0.0, "aim": 0.0, "launch": 0.0, "jostle": 0.0, "wait": 0.0, "square": 0.0, "other": 0.0}
    total = None
    steps = []
    for t, m in events(path):
        if not m.startswith("AUTO TIMING"):
            continue
        mm = re.match(r"AUTO TIMING total ([0-9.]+) s", m)
        if mm:
            total = float(mm.group(1))
            continue
        mm = re.match(r"AUTO TIMING (.*?)\s{2,}([0-9.]+) s\s+\(", m)
        if not mm:
            continue
        name, sec = mm.group(1), float(mm.group(2))
        steps.append((name, sec))
        if name.startswith("drive"):
            kinds["drive"] += sec
        elif name.startswith("aim"):
            kinds["aim"] += sec
        elif name.startswith("launch"):
            kinds["launch"] += sec
        elif "jostle" in name:
            kinds["jostle"] += sec
        elif name.startswith("square"):
            kinds["square"] += sec
        elif name.startswith("wait") or "(wait)" in name:
            kinds["wait"] += sec
        else:
            kinds["other"] += sec
    return total, kinds, steps


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def med(v):
    return statistics.median(v) if v else float("nan")


def parts(c):
    """The per-part means of a list of cycles, as table cells from salvo s to full wait s."""
    return "%.2f | %.1f | %.1f | %.1f | %.0f | %.1f | %.1f | %.1f | %.1f" % (
        mean([x["salvo"] for x in c]), mean([x["collect"] for x in c]), mean([x["find"] for x in c]), mean([x["lock_drive"] for x in c]),
        mean([x["dist"] for x in c if x["dist"] == x["dist"]]), mean([x["at_spot"] for x in c]), mean([x["aim"] for x in c]),
        mean([x["lock"] for x in c]), mean([x["full"] for x in c]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True)
    ap.add_argument("--root", default=ROOT)
    ap.add_argument("--radius", type=float, default=6.0, help="inches from the launch spot that count as arrived")
    a = ap.parse_args()
    ev_dir = os.path.join(a.root, "events", a.event)
    matches = {}
    order = []
    with open(os.path.join(ev_dir, "labels.csv"), encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            lab = r["log"].replace("\\", "/")
            day, rest = lab.split("/")
            kind = "auto" if rest.endswith("_auto") else "teleop"
            if r["label"] not in matches:
                matches[r["label"]] = {}
                order.append(r["label"])
            matches[r["label"]][kind] = os.path.join(a.root, day, rest + ".log")

    md = ["# %s: cycle times\n" % a.event]
    md.append("Cycle = salvo start to the next salvo start = salvo + collect + find spot + at spot + aim.\n")
    md.append("- salvo: shooting. collect: SALVO done to the last pickup that mattered.")
    md.append("- find spot: that pickup to arriving within %.0f in of the spot the next salvo was fired from (lock drive = the part of it with the target lock already on; find in = straight-line distance)." % a.radius)
    md.append("- at spot: at the launch spot, not yet locked. aim: the later of arriving and locking, to SALVO started (turning onto the target, flywheel band wait).")
    md.append("- lock: lock on to SALVO started as in older reports (= lock drive + aim). full wait: part of SALVO done to the lock with the stack already at 4.\n")

    tele = {}
    for lab in order:
        p = matches[lab].get("teleop")
        if p and os.path.exists(p):
            tele[lab] = teleop_stats(p, a.radius)

    md.append("## TeleOp per match\n")
    md.append("| match | salvos | pollen | pollen/min | cycle mean s | cycle median s | salvo s | collect s | find spot s | lock drive s | find in | at spot s | aim s | lock s | full wait s | first salvo at s | tail after last s |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    allc = []
    for lab in order:
        s = tele.get(lab)
        if not s:
            continue
        c = s["cycles"]
        allc += c
        md.append("| %s | %d | %d | %.1f | %.1f | %.1f | %s | %.1f | %.1f |" % (
            lab, s["salvos"], s["pollen"], 60.0 * s["pollen"] / max(s["length"], 1.0),
            mean([x["cycle"] for x in c]), med([x["cycle"] for x in c]), parts(c),
            s["first"] if s["first"] is not None else float("nan"), s["tail"] if s["tail"] is not None else float("nan")))
    if allc:
        md.append("| **all** | | | | %.1f | %.1f | %s | | |" % (mean([x["cycle"] for x in allc]), med([x["cycle"] for x in allc]), parts(allc)))

    md.append("\n## TeleOp cycles by pollen shot\n")
    md.append("| pollen in the salvo | cycles | cycle mean s | collect mean s | find spot mean s | at spot mean s | aim mean s |")
    md.append("|---|---|---|---|---|---|---|")
    for h in range(1, 5):
        c = [x for x in allc if x["held"] == h]
        if c:
            md.append("| %d | %d | %.1f | %.1f | %.1f | %.1f | %.1f |" % (h, len(c), mean([x["cycle"] for x in c]), mean([x["collect"] for x in c]),
                                                              mean([x["find"] for x in c]), mean([x["at_spot"] for x in c]), mean([x["aim"] for x in c])))
    md.append("\n(The pollen count is that of the salvo that ENDS the cycle.)\n")

    md.append("## Every TeleOp cycle\n")
    md.append("cycle s (collect / find spot, of it locked / at spot / aim)\n")
    for lab in order:
        s = tele.get(lab)
        if s:
            md.append("- **%s**: %s" % (lab, ", ".join("%.1f (%.1f / %.1f, %.1f / %.1f / %.1f)" % (
                x["cycle"], x["collect"], x["find"], x["lock_drive"], x["at_spot"], x["aim"]) for x in s["cycles"])))
    md.append("\n## Auto per match\n")
    md.append("| match | total s | drive | aim | launch | square | jostle | wait | other | steps |")
    md.append("|---|---|---|---|---|---|---|---|---|---|")
    for lab in order:
        p = matches[lab].get("auto")
        if not p or not os.path.exists(p):
            continue
        total, k, steps = auto_stats(p)
        md.append("| %s | %s | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f | %d |" % (
            lab, "%.1f" % total if total is not None else "cut off", k["drive"], k["aim"], k["launch"], k["square"], k["jostle"], k["wait"], k["other"], len(steps)))
    md.append("\nTotal is what the auto itself reported (a stopped auto has no total). Seconds per kind add up to less than the total when steps overlap.\n")

    # auto step table for matches with the same step names (Cybugs)
    names = []
    for lab in order:
        p = matches[lab].get("auto")
        if p and os.path.exists(p):
            for n, _ in auto_stats(p)[2]:
                if n not in names:
                    names.append(n)
    md.append("## Auto steps (seconds)\n")
    md.append("| step | " + " | ".join(order) + " |")
    md.append("|---|" + "---|" * len(order))
    per = {}
    for lab in order:
        p = matches[lab].get("auto")
        per[lab] = dict(auto_stats(p)[2]) if p and os.path.exists(p) else {}
    for n in names:
        md.append("| %s | %s |" % (n, " | ".join(("%.2f" % per[l][n]) if n in per[l] else "" for l in order)))

    text = "\n".join(md) + "\n"
    out = os.path.join(ev_dir, "cycle_times.md")
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(text)
    print("wrote", out)


if __name__ == "__main__":
    main()
