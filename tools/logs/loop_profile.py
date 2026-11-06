"""Where the loop time goes: reads the LOOP PROFILE lines (util/LoopProfiler.java) of run logs.

    python tools/logs/loop_profile.py                       # the newest TeleOp and auto logs under robot-logs/
    python tools/logs/loop_profile.py path\\to\\teleop.log   # one log
    python tools/logs/loop_profile.py --windows <log>       # every 5 s window of a log, not just the whole run
    python tools/logs/loop_profile.py --busiest <log>       # the 5 slowest windows with their sections

Each LOOP PROFILE line looks like

    LOOP PROFILE RUN 120.3 s 4012 loops | slow 40 60 80: 301 40 3 | loop 28.6 74.2 | other 2.1 | pose 8.1 21.0 1.00 | mechanisms 9.7 40.1 1.00 | mech.counter 4.2 16.0 1.00 | ...

After "loop" (average and longest loop, ms) and "other" (loop time outside every top-level section: the opmode's own logic) each section is: its average
per LOOP in ms, its longest single call, and its calls per loop. Sections can nest: mech.* are inside mechanisms, and in an auto pose and drive_write are inside
action; nested ones are shown indented and are not counted again in "other". "RUN" covers the whole run, "window" lines 5 s each.
"""
import glob
import os
import sys


def newest(root, kind):
    paths = glob.glob(os.path.join(root, "robot-logs", "*", "*_%s.log" % kind))
    return max(paths, key=os.path.getmtime) if paths else None


def parse_line(text):
    """Returns (kind, seconds, loops, slow, loop_avg, loop_max, other, sections) or None. sections: list of (name, avg, max, calls_per_loop)."""
    if not text.startswith("LOOP PROFILE "):
        return None
    parts = [p.strip() for p in text.split(" | ")]
    head = parts[0].split()          # LOOP PROFILE <kind> <secs> s <loops> loops
    try:
        kind, secs, loops = head[2], float(head[3]), int(head[5])
        slow = [int(x) for x in parts[1].split(":")[1].split()]
        loop_avg, loop_max = [float(x) for x in parts[2].split()[1:3]]
        other = float(parts[3].split()[1])
        sections = []
        counts = []
        for p in parts[4:]:
            f = p.split()
            if f[0] == "count":
                counts.append((f[1], float(f[2])))
            else:
                sections.append((f[0], float(f[1]), float(f[2]), float(f[3])))
    except (IndexError, ValueError):
        return None
    return kind, secs, loops, slow, loop_avg, loop_max, other, sections, counts


def read(path):
    out = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            if not line.startswith("E,"):
                continue
            parts = line.rstrip("\n").split(",", 2)
            if len(parts) < 3:
                continue
            p = parse_line(parts[2])
            if p:
                out.append((float(parts[1]),) + p)
    return out


def show(entry, title):
    t, kind, secs, loops, slow, loop_avg, loop_max, other, sections, counts = entry
    print("%s  (%.1f s, %d loops; loops over 40 / 60 / 80 ms: %d / %d / %d)" % (title, secs, loops, slow[0], slow[1], slow[2]))
    print("   %-16s %8s %8s %10s" % ("section", "avg ms", "max ms", "calls/loop"))
    print("   %-16s %8.1f %8.1f" % ("LOOP", loop_avg, loop_max))
    names = {s[0]: s for s in sections}

    def parent(name):
        if name.startswith("mech.") and "mechanisms" in names:
            return "mechanisms"
        if name in ("pose", "drive_write") and "action" in names:   # in an auto the drive legs run inside the action
            return "action"
        return name

    def key(s):
        p = parent(s[0])
        return (-names[p][1], p, 0 if p == s[0] else 1, -s[1])

    for name, avg, mx, calls in sorted(sections, key=key):
        print("   %-16s %8.1f %8.1f %10.2f" % (("  " + name) if parent(name) != name else name, avg, mx, calls))
    print("   %-16s %8.1f   (in no top-level section: the opmode's own logic)" % ("other", other))
    sent = dict(counts).get("drive_sent")
    skipped = dict(counts).get("drive_skipped")
    if sent is not None or skipped is not None:
        sent, skipped = sent or 0.0, skipped or 0.0
        total = sent + skipped
        print("   drive writes per loop: %.2f sent, %.2f skipped by the %s (%.0f %% skipped)" % (
            sent, skipped, "write deadband", 100.0 * skipped / total if total else 0.0))
    for name, per_loop in counts:
        if name not in ("drive_sent", "drive_skipped"):
            print("   count %-12s %.2f per loop" % (name, per_loop))


def main():
    args = sys.argv[1:]
    windows = "--windows" in args
    busiest = "--busiest" in args
    paths = [a for a in args if not a.startswith("--")]
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    if not paths:
        paths = [p for p in (newest(root, "teleop"), newest(root, "auto")) if p]
    if not paths:
        sys.exit("No logs found: pass a path, or pull the logs first (python tools/logs/pull_logs.py).")
    for path in paths:
        entries = read(path)
        print("\n=== %s" % path)
        if not entries:
            print("   no LOOP PROFILE lines (a build from before 2026-11-05, or the run was too short for one window)")
            continue
        runs = [e for e in entries if e[1] == "RUN"]
        wins = [e for e in entries if e[1] == "window"]
        if busiest:
            for e in sorted(wins, key=lambda e: -e[5])[:5]:
                show(e, "window ending at %.1f s" % e[0])
            continue
        if windows:
            for e in wins:
                show(e, "window ending at %.1f s" % e[0])
        if runs:
            show(runs[-1], "WHOLE RUN")
        elif wins:
            print("   no RUN line (the run did not end cleanly); the last window:")
            show(wins[-1], "last window")


if __name__ == "__main__":
    main()
