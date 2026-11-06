"""Competition-day logs: label the runs with their match, then a report per match and for the day that shows what stands out.

Pull the logs first, as usual (python tools/logs/pull_logs.py). The raw logs are never changed; labels are a separate file.

    python tools/logs/event_day.py label  --event "Regional 2026" [--since 2026-11-05]        # ask for the match of each unlabeled run, oldest first
    python tools/logs/event_day.py label  --event "Regional 2026" --set 2026-11-05/101530 Q12  # label the run group that contains that log (no questions)
    python tools/logs/event_day.py label  --event "Regional 2026" --auto-number Q --since ...   # Q1, Q2, ... in order (for a rehearsal)
    python tools/logs/event_day.py report --event "Regional 2026" [--since 2026-11-05]        # a report per labeled match and the day report

Output goes to robot-logs/events/<event>/: labels.csv (log, label, note: edit it by hand to clean labels up; the label is free text), <label>.md per match and day.md.

A MATCH is an auto run followed by a TeleOp run within 90 s of its end (the field starts the TeleOp by itself); a lone auto or TeleOp is its own group. Bring-up and pit-check
runs are ignored. The report reads each run's log: the auto's timing and what timed out, the launches, the launch band, Hive Dash, hold position, the pusher lock, the element count's
flicker, the flywheel speed at the start of each salvo, the faults (Pinpoint, backup mode, a lost flywheel encoder, a crash, no end marker), the battery, slow loops, and the
Dashboard values that changed. "Stands out" compares each match with the day's median once there are 4 or more matches, and always lists the hard faults.
"""
import argparse
import csv
import datetime
import glob
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import launch_stability as LS
except Exception:   # matplotlib missing: the salvo numbers are skipped
    LS = None

HEADER = re.compile(r"# (.+?), opened (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d) \(hub clock\)")
PAIR_GAP_S = 90.0


# ---------------------------------------------------------------- reading a run
class Run:
    pass


def read_run(path):
    r = Run()
    r.path = path
    r.rel = os.path.join(os.path.basename(os.path.dirname(path)), os.path.basename(path)).replace("\\", "/")
    r.key = r.rel[:-4]
    r.kind = "auto" if path.endswith("_auto.log") else "teleop" if path.endswith("_teleop.log") else None
    r.events, r.rows, r.header = [], [], None
    first = None
    for line in open(path, encoding="utf-8", errors="ignore"):
        if first is None:
            first = line
        if line.startswith("H,"):
            r.header = line.rstrip("\n").split(",")[1:]
        elif line.startswith("S,") and r.header:
            p = line.rstrip("\n").split(",")[1:]
            if len(p) == len(r.header):
                r.rows.append(dict(zip(r.header, p)))
        elif line.startswith("E,"):
            p = line.rstrip("\n").split(",", 2)
            if len(p) == 3:
                try:
                    r.events.append((float(p[1]), p[2]))
                except ValueError:
                    pass
    m = HEADER.match(first or "")
    r.name = m.group(1) if m else os.path.basename(path)
    r.opened = datetime.datetime.strptime(m.group(2), "%Y-%m-%d %H:%M:%S") if m else None
    r.duration = max([t for t, _ in r.events] + [float(x.get("t", 0)) for x in r.rows[-1:]] + [0.0])
    r.ended = any(tx.startswith("RUN ENDED") for _, tx in r.events)
    r.crashed = any(tx.startswith("CRASH:") for _, tx in r.events)
    return r


def find_runs(root, since):
    runs = []
    for kind in ("auto", "teleop"):
        for p in glob.glob(os.path.join(root, "*-*-*", "*_%s.log" % kind)):
            if os.path.basename(os.path.dirname(p)) >= since:
                r = read_run(p)
                if r.opened:
                    runs.append(r)
    runs.sort(key=lambda r: r.opened)
    return runs


def group_runs(runs):
    """Pairs each auto with the TeleOp that follows it; everything else is a group of one."""
    used, groups = set(), []
    for i, r in enumerate(runs):
        if i in used:
            continue
        g = [r]
        used.add(i)
        if r.kind == "auto":
            end = r.opened + datetime.timedelta(seconds=r.duration)
            for j in range(i + 1, len(runs)):
                if j in used:
                    continue
                nxt = runs[j]
                gap = (nxt.opened - end).total_seconds()
                if nxt.kind == "teleop" and -30.0 <= gap <= PAIR_GAP_S:
                    g.append(nxt)
                    used.add(j)
                    break
                if gap > PAIR_GAP_S:
                    break
        groups.append(g)
    return groups


# ---------------------------------------------------------------- labels
def labels_path(root, event):
    d = os.path.join(root, "events", re.sub(r"[^\w\- ]+", "_", event).strip())
    os.makedirs(d, exist_ok=True)
    return d, os.path.join(d, "labels.csv")


def load_labels(path):
    out = {}
    if os.path.exists(path):
        for row in csv.DictReader(open(path, encoding="utf-8", newline="")):
            out[row["log"]] = (row.get("label", ""), row.get("note", ""))
    return out


def save_labels(path, labels):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["log", "label", "note"])
        for k in sorted(labels):
            w.writerow([k, labels[k][0], labels[k][1]])


def group_label(g, labels):
    for r in g:
        if r.key in labels and labels[r.key][0]:
            return labels[r.key][0]
    return None


def summary_line(g):
    parts = []
    for r in g:
        sal = sum(1 for _, tx in r.events if tx.startswith("SALVO started"))
        al = next((re.search(r"alliance (\w+)", tx).group(1) for _, tx in r.events if tx.startswith("SETUP") and "alliance" in tx), "?")
        parts.append("%s %s %s %.0f s, %d salvos, %s" % (r.opened.strftime("%H:%M:%S"), r.kind, al, r.duration, sal, "ended" if r.ended else "NO END MARKER"))
    return "; ".join(parts)


def cmd_label(args):
    runs = find_runs(args.root, args.since)
    groups = group_runs(runs)
    d, lp = labels_path(args.root, args.event)
    labels = load_labels(lp)
    if args.set:
        key, label = args.set
        key = key.replace("\\", "/")
        for g in groups:
            if any(r.key == key or r.key.startswith(key) for r in g):
                for r in g:
                    labels[r.key] = (label, "")
                save_labels(lp, labels)
                print("labeled %s as %s" % (", ".join(r.key for r in g), label))
                return
        sys.exit("No run starting with %s found." % key)
    todo = [g for g in groups if group_label(g, labels) is None and not any(r.key in labels for r in g)]
    if args.auto_number:
        n = args.start
        for g in todo:
            for r in g:
                labels[r.key] = ("%s%d" % (args.auto_number, n), "")
            print("%s%d  %s" % (args.auto_number, n, summary_line(g)))
            n += 1
        save_labels(lp, labels)
        return
    print("%d unlabeled run group(s). Type the match (free text, for example Q12 or SF2-1), 'practice' for a practice run, Enter to skip, 'q' to stop.\n" % len(todo))
    for g in todo:
        print(summary_line(g))
        try:
            ans = input("  match> ").strip()
        except EOFError:
            break
        if ans.lower() == "q":
            break
        if not ans:
            continue
        for r in g:
            labels[r.key] = (ans, "")
        save_labels(lp, labels)
    print("labels in", lp)


# ---------------------------------------------------------------- analysis of one run
def count_ev(events, prefix):
    return sum(1 for _, tx in events if tx.startswith(prefix))


def analyze(r):
    ev = r.events
    a = {"run": r, "flags": [], "notes": []}
    a["name"] = r.name
    a["alliance"] = next((re.search(r"alliance (\w+)", tx).group(1) for _, tx in ev if tx.startswith("SETUP") and "alliance" in tx), "?")
    a["duration"] = r.duration
    a["salvos"] = count_ev(ev, "SALVO started")
    a["launch_requests"] = count_ev(ev, "COMMAND launch queued")
    a["launch_held"] = count_ev(ev, "LAUNCH held")
    a["launch_dropped"] = sum(1 for _, tx in ev if tx.startswith("NOTICE Launch dropped"))
    a["rejects"] = count_ev(ev, "COMMAND reject started")
    volts = [float(x["volt"]) for x in r.rows if x.get("volt") not in (None, "", "nan")]
    a["min_volt"] = min(volts) if volts else float("nan")
    loops = [float(x["loop_max_ms"]) for x in r.rows if x.get("loop_max_ms")]
    a["max_loop_ms"] = max(loops) if loops else float("nan")
    a["slow_loops"] = count_ev(ev, "SLOW LOOP")
    held = []
    for _, tx in ev:
        m = re.match(r"SALVO started: .*?, (\d+) held", tx)
        if m:
            held.append(int(m.group(1)))
    a["held_at_salvo"] = held
    # hard faults
    if r.crashed:
        a["flags"].append("CRASHED")
    if not r.ended and not r.crashed:
        a["flags"].append("no end marker (power loss or the app stopped hard)")
    for prefix, text in (("PINPOINT fault", "Pinpoint fault"), ("BACKUP MODE ENTERED", "BACKUP MODE entered"), ("FLYWHEEL ENCODER LOST", "flywheel encoder lost")):
        n = count_ev(ev, prefix)
        if n:
            a["flags"].append("%s (%d)" % (text, n))
    if a["launch_dropped"]:
        a["flags"].append("%d launch(es) dropped" % a["launch_dropped"])
    if count_ev(ev, "PUSHER LOCK FAILED"):
        a["flags"].append("pusher lock failed %d time(s)" % count_ev(ev, "PUSHER LOCK FAILED"))
    if count_ev(ev, "HOLD dropped"):
        a["flags"].append("hold position dropped (pose jump) %d time(s)" % count_ev(ev, "HOLD dropped"))
    # auto
    if r.kind == "auto":
        m = next((re.search(r"total ([\d.]+) s", tx) for _, tx in ev if tx.startswith("AUTO TIMING total")), None)
        a["auto_total"] = float(m.group(1)) if m else float("nan")
        a["drive_timeouts"] = sum(1 for _, tx in ev if tx.startswith("DRIVE TIMED OUT"))
        a["aim_timeouts"] = sum(1 for _, tx in ev if tx.startswith("AIM TIMED OUT"))
        a["skipped"] = [tx for _, tx in ev if tx.startswith("AUTO SKIP salvo")]
        if not r.ended or r.duration < 29.0 and not m:
            a["flags"].append("the auto did not report its total time (cut short?)")
        if a["skipped"]:
            a["flags"].append("salvo skipped: " + "; ".join(s.replace("AUTO SKIP salvo: ", "") for s in a["skipped"]))
        jost = next((tx for _, tx in ev if tx.startswith("AUTO JOSTLE ended")), None)
        a["jostle"] = jost
    # teleop
    if r.kind == "teleop":
        a["dash_starts"] = count_ev(ev, "HIVE DASH start")
        a["dash_refused"] = [tx for _, tx in ev if tx.startswith("HIVE DASH refused")]
        a["dash_cancelled"] = count_ev(ev, "HIVE DASH cancelled")
        a["dash_finished"] = count_ev(ev, "HIVE DASH finished")
        a["holds"] = count_ev(ev, "HOLD latched")
        peaks = [float(m.group(1)) for _, tx in ev for m in [re.search(r"peak error ([\d.]+) in", tx)] if m and tx.startswith("HOLD released")]
        a["hold_peak_in"] = max(peaks) if peaks else 0.0
        a["lock_locked"] = count_ev(ev, "PUSHER LOCKED")
        a["lock_unlocks"] = count_ev(ev, "PUSHER UNLOCKED")
        a["target_locks"] = count_ev(ev, "TARGET LOCK on")
        a["auto_launches"] = count_ev(ev, "AUTO LAUNCH:") - count_ev(ev, "AUTO LAUNCH gave up")
        a["reports"] = count_ev(ev, "REPORT landed")
    # count flicker: reversals within 2 s away from a salvo
    pat = re.compile(r"ELEMENTS (\d+) -> (\d+)")
    changes = [(t, int(m.group(1)), int(m.group(2))) for t, tx in ev for m in [pat.search(tx)] if m]
    sal_t = [t for t, tx in ev if tx.startswith("SALVO done") or tx.startswith("REJECT done") or tx.startswith("SALVO started")]
    rev = 0
    for i in range(len(changes) - 1):
        x, y = changes[i], changes[i + 1]
        if y[0] - x[0] < 2.0 and not any(abs(x[0] - s) < 2.5 for s in sal_t) and x[1] != 0 and x[2] != 0 and ((x[2] > x[1] and y[2] < y[1]) or (x[2] < x[1] and y[2] > y[1])):
            rev += 1
    a["count_reversals"] = rev
    a["count_reversals_per_min"] = rev / max(r.duration / 60.0, 0.1)
    # baseline health (COUNTER EMPTY lines)
    em = re.compile(r"COUNTER EMPTY shift ([\d.]+) ([\d.]+) ([\d.]+) ([\d.]+) .*arm clear")
    sh = [[], [], [], []]
    for _, tx in ev:
        m = em.search(tx)
        if m:
            for i in range(4):
                sh[i].append(float(m.group(i + 1)))
    a["empty_shift_median"] = [statistics.median(s) if s else None for s in sh]
    for i, s in enumerate(a["empty_shift_median"]):
        if s is not None and s >= 0.03:
            a["flags"].append("CS%d empty shift %.3f is at or above the 0.03 OFF threshold: re-measure the baselines" % (i + 1, s))
    # flywheel at the salvo starts
    a["start_err"] = []
    if LS is not None:
        try:
            for s in LS.salvos_in(r.path):
                st = min(s["rows"], key=lambda x: abs(LS.f(x, "t") - s["t0"]))
                e = LS.err(st, s["target"])
                if e > -50:
                    a["start_err"].append(e)
        except Exception:
            pass
    out_of_band = [e for e in a["start_err"] if abs(e) > 4.0]
    if out_of_band:
        a["flags"].append("%d salvo(s) started outside +-4%% of the setpoint (%s)" % (len(out_of_band), ", ".join("%+.1f%%" % e for e in out_of_band)))
    a["config_changes"] = [tx[len("CONFIG CHANGED "):] for _, tx in ev if tx.startswith("CONFIG CHANGED ")]
    # events worth reading
    keys = ("WARNING ", "NOTICE ", "PINPOINT fault", "PINPOINT recovered", "PINPOINT START mode RESEEDED_AFTER_FAILED_ADOPT", "PINPOINT INIT manual reset", "BACKUP MODE", "FLYWHEEL ENCODER LOST", "CRASH", "AUTO SKIP", "HIVE DASH refused", "HIVE DASH cancelled", "PUSHER LOCK FAILED",
            "LAUNCH held", "HOLD dropped", "DRIVE TIMED OUT", "AIM TIMED OUT", "AUTO JOSTLE ended", "COMMAND reject started")
    seen = {}
    for t, tx in ev:
        if tx.startswith("WARNING cleared"):
            continue
        if any(tx.startswith(k) for k in keys):
            base = re.sub(r"[\d.]+", "#", tx)[:90]
            if base in seen:
                seen[base][1] += 1
            else:
                seen[base] = [(t, tx), 1]
    a["notable"] = [(v[0][0], v[0][1], v[1]) for v in seen.values()][:40]
    return a


# ---------------------------------------------------------------- reports
def fmt(v, spec="%.1f"):
    return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else spec % v


def match_report(label, g):
    parts = [analyze(r) for r in g]
    L = ["# %s" % label, ""]
    L.append("Runs: " + "; ".join("%s (%s, %s, %.0f s)" % (a["run"].rel, a["run"].kind, a["alliance"], a["duration"]) for a in parts))
    L.append("")
    flags = [(a["run"].kind, f) for a in parts for f in a["flags"]]
    L.append("## Flags")
    L += ["- **%s:** %s" % x for x in flags] or ["- none"]
    L.append("")
    for a in parts:
        r = a["run"]
        L.append("## %s (%s)" % (r.kind, a["name"]))
        rows = [("duration", fmt(a["duration"]) + " s"), ("salvos", a["salvos"]), ("pollen held at each salvo start", ", ".join(map(str, a["held_at_salvo"])) or "-"),
                ("launch requests / held for the speed band / dropped", "%d / %d / %d" % (a["launch_requests"], a["launch_held"], a["launch_dropped"])),
                ("flywheel speed at each salvo start (% of setpoint)", ", ".join("%+.1f" % e for e in a["start_err"]) or "-"),
                ("rejects", a["rejects"]), ("battery minimum", fmt(a["min_volt"], "%.2f") + " V"),
                ("slowest loop / slow loops", "%s ms / %d" % (fmt(a["max_loop_ms"], "%.0f"), a["slow_loops"])),
                ("element count reversals (per minute)", "%d (%.1f)" % (a["count_reversals"], a["count_reversals_per_min"]))]
        if r.kind == "auto":
            rows += [("auto total time", fmt(a.get("auto_total")) + " s"), ("drive legs that timed out / aim steps that timed out", "%d / %d" % (a["drive_timeouts"], a["aim_timeouts"])),
                     ("flower jostle", a.get("jostle") or "-")]
        else:
            rows += [("Hive Dash starts / finished / cancelled / refused", "%d / %d / %d / %d" % (a["dash_starts"], a["dash_finished"], a["dash_cancelled"], len(a["dash_refused"]))),
                     ("hold position latched / largest push resisted", "%d / %.1f in" % (a["holds"], a["hold_peak_in"])), ("pusher lock locked / unlocked", "%d / %d" % (a["lock_locked"], a["lock_unlocks"])),
                     ("target locks / auto launches / operator reports", "%d / %d / %d" % (a["target_locks"], a["auto_launches"], a["reports"]))]
        if any(s is not None for s in a["empty_shift_median"]):
            rows.append(("empty colour shift per sensor (median; OFF threshold 0.03)", " ".join("CS%d %s" % (i + 1, fmt(s, "%.3f")) for i, s in enumerate(a["empty_shift_median"]))))
        L += ["| item | value |", "|---|---|"] + ["| %s | %s |" % x for x in rows] + [""]
        if a["config_changes"]:
            L += ["Dashboard values changed since the previous run:"] + ["- " + c for c in a["config_changes"][:30]] + [""]
        if a["notable"]:
            L += ["Worth reading:"] + ["- %.1f s  %s%s" % (t, tx[:200], " (x%d)" % n if n > 1 else "") for t, tx, n in a["notable"]] + [""]
    return "\n".join(L), parts


def standouts(matches):
    """matches: list of (label, parts). Compares the numbers with the day's median when there are enough matches."""
    lines = []
    if len(matches) < 4:
        return ["Fewer than 4 matches: no comparison with the day's median yet (the hard faults are in each match's Flags)."]
    metrics = [("auto", "auto_total", "auto total time (s)", 1.0, 3.0), ("auto", "drive_timeouts", "auto drive legs that timed out", 1.0, 1.0),
               ("auto", "salvos", "auto salvos", 1.0, 1.0), ("teleop", "salvos", "TeleOp salvos", 1.0, 3.0), ("teleop", "count_reversals_per_min", "count reversals per minute", 1.0, 2.0),
               ("teleop", "min_volt", "battery minimum (V)", -1.0, 0.4), ("auto", "min_volt", "auto battery minimum (V)", -1.0, 0.4), ("teleop", "max_loop_ms", "slowest loop (ms)", 1.0, 25.0),
               ("teleop", "launch_held", "launches held for the speed band", 1.0, 2.0)]
    for kind, key, title, sign, floor in metrics:
        vals = []
        for label, parts in matches:
            for a in parts:
                v = a.get(key)
                if a["run"].kind == kind and isinstance(v, (int, float)) and not math.isnan(v):
                    vals.append((label, v))
        if len(vals) < 4:
            continue
        med = statistics.median(v for _, v in vals)
        mad = statistics.median(abs(v - med) for _, v in vals) or floor / 2.0
        for label, v in vals:
            d = (v - med) * sign
            if d > floor and d / (1.4826 * mad) > 2.5:
                lines.append("- **%s**: %s %.2f against the day's median %.2f" % (label, title, v, med))
    return lines or ["Nothing stands out against the day's median."]


def cmd_report(args):
    runs = find_runs(args.root, args.since)
    groups = group_runs(runs)
    d, lp = labels_path(args.root, args.event)
    labels = load_labels(lp)
    matches, unlabeled = [], []
    for g in groups:
        lab = group_label(g, labels)
        (matches if lab else unlabeled).append((lab, g))
    if not matches:
        sys.exit("No labeled runs yet: run the label command first.")
    day = ["# %s: day report" % args.event, "", "%d labeled match group(s), %d unlabeled run group(s) (practice or test)." % (len(matches), len(unlabeled)), ""]
    analyzed = []
    names = {}
    for lab, g in matches:
        text, parts = match_report(lab, g)
        n = names.get(lab, 0)
        names[lab] = n + 1
        fname = re.sub(r"[^\w\-]+", "_", lab) + ("" if n == 0 else "_%d" % (n + 1)) + ".md"
        open(os.path.join(d, fname), "w", encoding="utf-8").write(text)
        analyzed.append((lab, parts))
    day += ["## Stands out", ""] + standouts(analyzed) + [""]
    day += ["## Hard faults", ""]
    faults = [("%s" % lab, f) for lab, parts in analyzed for a in parts for f in a["flags"]]
    day += ["- **%s**: %s" % x for x in faults] or ["- none"]
    day += ["", "## Every match", "", "| match | auto | auto s | auto salvos | held at each | TeleOp s | TeleOp salvos | launches held | dash start/refused | count rev/min | min V | flags |", "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for lab, parts in analyzed:
        au = next((a for a in parts if a["run"].kind == "auto"), None)
        te = next((a for a in parts if a["run"].kind == "teleop"), None)
        flags = sum(len(a["flags"]) for a in parts)
        volts = [a["min_volt"] for a in parts if not math.isnan(a["min_volt"])]
        day.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %d |" % (
            lab, au["alliance"] if au else (te["alliance"] if te else "-"), fmt(au["auto_total"]) if au else "-", au["salvos"] if au else "-",
            ("/".join(map(str, au["held_at_salvo"])) if au else "-") or "-", fmt(te["duration"], "%.0f") if te else "-", te["salvos"] if te else "-", te["launch_held"] if te else "-",
            ("%d/%d" % (te["dash_starts"], len(te["dash_refused"]))) if te else "-", fmt(te["count_reversals_per_min"]) if te else "-", fmt(min(volts), "%.2f") if volts else "-", flags))
    changed = [(lab, c) for lab, parts in analyzed for a in parts for c in a["config_changes"]]
    day += ["", "## Dashboard values changed between runs", ""] + (["- **%s**: %s" % x for x in changed[:80]] or ["- none logged"])
    if unlabeled:
        day += ["", "## Unlabeled runs", ""] + ["- " + summary_line(g) for _, g in unlabeled]
    open(os.path.join(d, "day.md"), "w", encoding="utf-8").write("\n".join(day) + "\n")
    print("wrote %d match report(s) and day.md to %s" % (len(analyzed), d))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["label", "report"])
    ap.add_argument("--event", required=True, help="the event name (a folder under robot-logs/events/)")
    ap.add_argument("--root", default="robot-logs")
    ap.add_argument("--since", default="0000-00-00", help="only runs from this day on")
    ap.add_argument("--set", nargs=2, metavar=("LOG", "LABEL"), help="label the run group that contains LOG (for example 2026-11-05/101530)")
    ap.add_argument("--auto-number", metavar="PREFIX", help="label the unlabeled run groups PREFIX1, PREFIX2, ... in order")
    ap.add_argument("--start", type=int, default=1, help="the first number for --auto-number")
    args = ap.parse_args()
    if args.command == "label":
        cmd_label(args)
    else:
        cmd_report(args)


if __name__ == "__main__":
    main()
