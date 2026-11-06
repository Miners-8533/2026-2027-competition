"""What the Dashboard values were in each run, and what changed between runs (from the CONFIG lines the opmodes write at Start; see ConfigDump).

    python tools/logs/config_changes.py                 # every run with CONFIG lines, newest last: the values edited since the previous run
    python tools/logs/config_changes.py --since 2026-11-04
    python tools/logs/config_changes.py --value Flywheel.FEED_FORWARD    # that value in every run

Logs from before 2026-11-04 have no CONFIG lines. The first run of an app session shows the compiled defaults unless the Dashboard was edited before it; a later run
shows `CONFIG CHANGED name: old -> new` for every value edited in between.
"""
import argparse
import glob
import os
import re


def read(path):
    values, changes = {}, []
    for line in open(path, encoding="utf-8", errors="ignore"):
        if not line.startswith("E,"):
            continue
        p = line.rstrip("\n").split(",", 2)
        if len(p) < 3 or not p[2].startswith("CONFIG "):
            continue
        text = p[2][len("CONFIG "):]
        if text.startswith("CHANGED "):
            changes.append(text[len("CHANGED "):])
            continue
        m = re.match(r"(\w+) (.*)", text)
        if m and "=" in m.group(2) and not text.startswith("first run") and "value(s) changed" not in text:
            for part in m.group(2).split(", "):
                if "=" in part:
                    k, v = part.split("=", 1)
                    values[m.group(1) + "." + k] = v
    return values, changes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="robot-logs")
    ap.add_argument("--since", default="0000-00-00")
    ap.add_argument("--value", default=None)
    args = ap.parse_args()
    files = sorted(glob.glob(os.path.join(args.root, "*-*-*", "*_*.log")), key=lambda p: (os.path.basename(os.path.dirname(p)), os.path.basename(p)))
    shown = 0
    for f in files:
        day = os.path.basename(os.path.dirname(f))
        if day < args.since:
            continue
        values, changes = read(f)
        if not values:
            continue
        name = day + " " + os.path.basename(f)[:6] + " " + os.path.basename(f)[7:-4]
        if args.value:
            print("%-32s %s = %s" % (name, args.value, values.get(args.value, "(not logged)")))
        else:
            print("%s: %d values logged, %d changed since the previous run" % (name, len(values), len(changes)))
            for c in changes:
                print("    " + c)
        shown += 1
    if not shown:
        print("No CONFIG lines found (runs from 2026-11-04 on write them).")


if __name__ == "__main__":
    main()
