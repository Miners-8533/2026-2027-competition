"""Cycle time analysis for the labeled matches of an event.

    python tools/logs/cycle_times.py --event "Hartland Qualifier 2026"

Reads robot-logs/events/<event>/labels.csv (made by event_day.py label) and writes robot-logs/events/<event>/cycle_times.md.

TeleOp cycle = salvo start to the next salvo start, split in three:
  salvo   SALVO started to SALVO done (the pusher turning, the flywheel shooting)
  gather  SALVO done to the lock that led to the next salvo (a lock the driver released again counts as gather): driving, intaking, filling the stack
          split in two at the moment the stack LAST rose to the count it held when the lock came on (the last pickup that mattered):
  collect   SALVO done to that moment: driving out, intaking, filling the stack
  transit   that moment to the lock coming on: the drive to the launch spot with the load (dist = how far the robot moved in that time, from the pose samples)
  lock    target lock on to the next SALVO started (align, flywheel band wait, hold)
  full    of the gather time, how long the stack already held 4 (FULL) before the lock: the driver was ready and not yet locked
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


def teleop_stats(path):
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
        transit = lock_on - reach
        a, b = pose_at(samples, reach), pose_at(samples, lock_on)
        dist = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 if a and b else float("nan")
        rows.append({"cycle": cycle, "salvo": salvo_t, "gather": gather, "collect": collect, "transit": transit, "dist": dist, "lock": lock_t, "full": min(full, gather), "held": n["held"]})
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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True)
    ap.add_argument("--root", default=ROOT)
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
    md.append("Cycle = salvo start to the next salvo start. salvo = shooting, gather = SALVO done to the driver's next target lock, lock = lock on to SALVO started, full = part of the gather the stack already held 4.\n")

    tele = {}
    for lab in order:
        p = matches[lab].get("teleop")
        if p and os.path.exists(p):
            tele[lab] = teleop_stats(p)

    md.append("## TeleOp per match\n")
    md.append("| match | salvos | pollen | pollen/min | cycle mean s | cycle median s | salvo s | collect s | transit s | transit in | lock s | full wait s | first salvo at s | tail after last s |")
    md.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    allc = []
    for lab in order:
        s = tele.get(lab)
        if not s:
            continue
        c = s["cycles"]
        allc += c
        md.append("| %s | %d | %d | %.1f | %.1f | %.1f | %.2f | %.1f | %.1f | %.0f | %.1f | %.1f | %.1f | %.1f |" % (
            lab, s["salvos"], s["pollen"], 60.0 * s["pollen"] / max(s["length"], 1.0),
            mean([x["cycle"] for x in c]), med([x["cycle"] for x in c]), mean([x["salvo"] for x in c]),
            mean([x["collect"] for x in c]), mean([x["transit"] for x in c]), mean([x["dist"] for x in c if x["dist"] == x["dist"]]), mean([x["lock"] for x in c]), mean([x["full"] for x in c]),
            s["first"] if s["first"] is not None else float("nan"), s["tail"] if s["tail"] is not None else float("nan")))
    if allc:
        md.append("| **all** | | | | %.1f | %.1f | %.2f | %.1f | %.1f | %.0f | %.1f | %.1f | | |" % (
            mean([x["cycle"] for x in allc]), med([x["cycle"] for x in allc]), mean([x["salvo"] for x in allc]),
            mean([x["collect"] for x in allc]), mean([x["transit"] for x in allc]), mean([x["dist"] for x in allc if x["dist"] == x["dist"]]), mean([x["lock"] for x in allc]), mean([x["full"] for x in allc])))

    md.append("\n## TeleOp cycles by pollen shot\n")
    md.append("| pollen in the salvo | cycles | cycle mean s | collect mean s | transit mean s | lock mean s |")
    md.append("|---|---|---|---|---|---|")
    for h in range(1, 5):
        c = [x for x in allc if x["held"] == h]
        if c:
            md.append("| %d | %d | %.1f | %.1f | %.1f | %.1f |" % (h, len(c), mean([x["cycle"] for x in c]), mean([x["collect"] for x in c]), mean([x["transit"] for x in c]), mean([x["lock"] for x in c])))
    md.append("\n(The pollen count is that of the salvo that ENDS the cycle.)\n")

    md.append("## Every TeleOp cycle\n")
    for lab in order:
        s = tele.get(lab)
        if s:
            md.append("- **%s** cycle s: %s" % (lab, ", ".join("%.1f" % x["cycle"] for x in s["cycles"])))
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
