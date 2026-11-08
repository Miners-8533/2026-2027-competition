#!/usr/bin/env python3
"""
Pull the robot's run logs off the Control Hub, keep every run, and index them.

The robot (util/RunLog.java) keeps the last 10 runs of each kind on the hub, in /sdcard/FIRST/runlogs/ (auto, teleop,
pitcheck, bringup; as NAME.log, NAME_prev.log, NAME_prev2.log ... NAME_prev9.log). This tool copies them off over adb and files each
run by the time it was opened, so nothing is lost when the hub rotates them:

    robot-logs/
      index.md                     one row per run, newest first, with a summary of each
      index.json                   the same, for tools
      latest/teleop.log            the newest and second newest run of each kind, under fixed names
      latest/teleop_prev.log          (so "read the last TeleOp log" is always the same path)
      2026-11-02/233015_teleop.log every run, by the date and time on its first line (the hub's clock)
      matchlogs/                   the SDK's own match logs, when --matchlogs is given

Running it again is safe: a run already stored is skipped, and a run that was still being written is replaced by the longer copy.
After filing, it DELETES from the hub every log it has confirmed is stored here (the stored file is read back and must contain the
pulled bytes) AND is finished (it has its end marker, or it has not been written to for 5 minutes, e.g. the robot lost power). A log
that may still be written to is left on the hub. Nothing is ever deleted from robot-logs. Use --keep-on-hub to delete nothing.

Usage (hub on USB, from the repo root):
    python tools/logs/pull_logs.py
    python tools/logs/pull_logs.py --list                  just print the index
    python tools/logs/pull_logs.py --from-dir SOME_FOLDER  file logs from a folder you already pulled (no hub needed; deletes nothing)
    python tools/logs/pull_logs.py --keep-on-hub           pull and file, but leave the logs on the hub
    python tools/logs/pull_logs.py --matchlogs             also pull /sdcard/FIRST/matchlogs
    python tools/logs/pull_logs.py --serial 5462f57f0f18d316   when more than one device is attached
    python tools/logs/pull_logs.py --dest D:/robot-logs    store somewhere else

Only the Python standard library and adb are needed.
"""

import argparse
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HUB_RUNLOGS = "/sdcard/FIRST/runlogs"
HUB_MATCHLOGS = "/sdcard/FIRST/matchlogs"
KINDS = ("auto", "teleop", "pitcheck", "bringup", "waypoints", "drivetest")
LOG_NAME_RE = re.compile(r"^(?P<kind>auto|teleop|pitcheck|bringup|waypoints|drivetest)(?:_prev\d*)?\.log$")
IDLE_SECONDS = 300   # a log with no end marker that has not been written to for this long is treated as finished (power loss)
REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DEFAULT_DEST = os.path.join(REPO, "robot-logs")

BUILD_RE = re.compile(r"^#\s*Build:\s*commit\s+(?P<commit>\S+).*?(?:fingerprint\s+(?P<fp>[0-9a-f]+))?\s*$")
HEADER_RE = re.compile(r"^#\s*(?P<name>.*?),\s*opened\s+(?P<date>\d{4}-\d\d-\d\d)\s+(?P<time>\d\d:\d\d:\d\d)")


# ---------------------------------------------------------------- adb

def find_adb():
    found = shutil.which("adb")
    if found:
        return found
    candidates = []
    for var in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        if os.environ.get(var):
            candidates.append(os.path.join(os.environ[var], "platform-tools", "adb.exe"))
            candidates.append(os.path.join(os.environ[var], "platform-tools", "adb"))
    local = os.environ.get("LOCALAPPDATA")
    if local:
        candidates.append(os.path.join(local, "Android", "Sdk", "platform-tools", "adb.exe"))
    candidates.append(os.path.expanduser("~/Library/Android/sdk/platform-tools/adb"))
    candidates.append(os.path.expanduser("~/Android/Sdk/platform-tools/adb"))
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def run_adb(adb, serial, *args):
    cmd = [adb] + (["-s", serial] if serial else []) + list(args)
    return subprocess.run(cmd, capture_output=True, text=True, errors="replace")


def pick_device(adb, serial):
    out = run_adb(adb, None, "devices").stdout.splitlines()[1:]
    devices = [line.split()[0] for line in out if line.strip() and line.split()[1] == "device"]
    if serial:
        if serial not in devices:
            sys.exit("Device %s is not attached (attached: %s)." % (serial, ", ".join(devices) or "none"))
        return serial
    if not devices:
        sys.exit("No device found. Plug the Control Hub in over USB (or run 'adb connect <ip>:5555'), then try again.")
    if len(devices) > 1:
        sys.exit("More than one device is attached (%s). Pick one with --serial." % ", ".join(devices))
    return devices[0]


def pull(adb, serial, hub_path, into):
    result = run_adb(adb, serial, "pull", hub_path, into)
    if result.returncode != 0:
        sys.exit("adb pull %s failed:\n%s%s" % (hub_path, result.stdout, result.stderr))


# ---------------------------------------------------------------- reading one log

def kind_of(filename):
    m = LOG_NAME_RE.match(os.path.basename(filename))
    return m.group("kind") if m else None


def read_lines(path):
    with open(path, encoding="utf-8", errors="replace") as f:
        return [line.rstrip("\n").rstrip("\r") for line in f]


def to_float(text):
    try:
        return float(text)
    except ValueError:
        return float("nan")


def summarize(lines):
    """Everything the index shows about one run, read from its lines."""
    s = {"name": "", "opened": "", "duration_s": 0.0, "end": "NO END MARKER", "alliance": "", "start": "",
         "salvos": 0, "pusher_checks": 0, "warnings": 0, "slow_loops": 0, "backup_mode": False,
         "flywheel_encoder_lost": False, "pinpoint_faults": 0, "marks": 0, "min_volts": None, "max_loop_ms": 0.0, "events": 0, "build": ""}
    if lines:
        m = HEADER_RE.match(lines[0])
        if m:
            s["name"] = m.group("name")
            s["opened"] = "%s %s" % (m.group("date"), m.group("time"))
    for line in lines[:4]:
        b = BUILD_RE.match(line)   # "# Build: commit 28935fd (integration), built ... , clean | UNCOMMITTED changes: ..., fingerprint 0a12ac51" (from 2026-11-07)
        if b:
            s["build"] = b.group("commit") + ("+" + b.group("fp") if b.group("fp") else "")
            break
    cols = None
    for line in lines:
        kind = line.split(",", 1)[0]
        if kind == "H":
            cols = line.split(",")
        elif kind == "S" and cols:
            parts = line.split(",")
            row = dict(zip(cols[1:], parts[1:]))
            t = to_float(row.get("t", "nan"))
            if t == t:
                s["duration_s"] = max(s["duration_s"], t)
            v = to_float(row.get("volt", "nan"))
            if v == v and v > 1.0:
                s["min_volts"] = v if s["min_volts"] is None else min(s["min_volts"], v)
            lm = to_float(row.get("loop_max_ms", "nan"))
            if lm == lm:
                s["max_loop_ms"] = max(s["max_loop_ms"], lm)
        elif kind == "E":
            parts = line.split(",", 2)
            if len(parts) < 3:
                continue
            s["events"] += 1
            t = to_float(parts[1])
            if t == t:
                s["duration_s"] = max(s["duration_s"], t)
            text = parts[2]
            if text.startswith("RUN ENDED"):
                s["end"] = "ended normally"
            elif text.startswith("CRASH:"):
                s["end"] = "CRASHED"
            elif text.startswith("SETUP profile"):
                m = re.search(r"alliance (BLUE|RED)", text)
                if m:
                    s["alliance"] = m.group(1)
                m = re.search(r"start pose (default|from the AUTO) \(([^)]*)\)", text)
                if m:
                    s["start"] = "%s (%s)" % (m.group(1), m.group(2))
            elif text.startswith("SALVO started"):
                s["salvos"] += 1
            elif text.startswith("MARK ") or text.startswith("CHECK "):
                s["marks"] += 1
            elif text.startswith("PUSHER CHECK") and ("started" in text or "SEEK" in text or "OK" in text):
                s["pusher_checks"] += 1
            elif text.startswith("WARNING") and "cleared" not in text:
                s["warnings"] += 1
            elif text.startswith("SLOW LOOP"):
                s["slow_loops"] += 1
            elif text.startswith("BACKUP MODE ENTERED"):
                s["backup_mode"] = True
            elif text.startswith("FLYWHEEL ENCODER LOST"):
                s["flywheel_encoder_lost"] = True
            elif text.startswith("PINPOINT fault"):
                s["pinpoint_faults"] += 1
    s["duration_s"] = round(s["duration_s"], 1)
    s["max_loop_ms"] = round(s["max_loop_ms"], 1)
    if s["min_volts"] is not None:
        s["min_volts"] = round(s["min_volts"], 2)
    return s


# ---------------------------------------------------------------- the store

def stored_name(kind, opened):
    date, time = opened.split(" ")
    return os.path.join(date, "%s_%s.log" % (time.replace(":", ""), kind))


def file_in(dest, source_path, kind):
    """Copy one log into the store. Returns ('new' | 'updated' | 'skipped', its path inside the store)."""
    lines = read_lines(source_path)
    m = HEADER_RE.match(lines[0]) if lines else None
    if not m:
        opened = datetime.datetime.fromtimestamp(os.path.getmtime(source_path)).strftime("%Y-%m-%d %H:%M:%S")
    else:
        opened = "%s %s" % (m.group("date"), m.group("time"))
    rel = stored_name(kind, opened)
    target = os.path.join(dest, rel)
    os.makedirs(os.path.dirname(target), exist_ok=True)
    if not os.path.exists(target):
        shutil.copyfile(source_path, target)
        return "new", rel
    if os.path.getsize(source_path) > os.path.getsize(target):   # the run was still being written when it was first pulled
        shutil.copyfile(source_path, target)
        return "updated", rel
    return "skipped", rel


def is_complete(lines):
    """True when the run wrote its end marker (it ended or crashed), so nothing more will be added to it."""
    return any(line.startswith("E,") and (",RUN ENDED" in line or ",CRASH:" in line) for line in lines)


def stored_has(dest, rel, pulled_path):
    """True when the stored file starts with exactly the bytes that were pulled (it is the same run, as long or longer)."""
    with open(pulled_path, "rb") as f:
        pulled = f.read()
    stored_path = os.path.join(dest, rel)
    if not os.path.isfile(stored_path):
        return False
    with open(stored_path, "rb") as f:
        stored = f.read()
    return len(pulled) > 0 and stored.startswith(pulled)


def hub_stat(adb, serial, hub_path):
    """(size, modified seconds) of a file on the hub, or None when the hub's stat command is not usable."""
    out = run_adb(adb, serial, "shell", "stat", "-c", "%s %Y", hub_path)
    parts = out.stdout.split()
    if out.returncode != 0 or len(parts) != 2 or not all(p.isdigit() for p in parts):
        return None
    return int(parts[0]), int(parts[1])


def hub_now(adb, serial):
    out = run_adb(adb, serial, "shell", "date", "+%s").stdout.strip()
    return int(out) if out.isdigit() else None


def delete_from_hub(adb, serial, dest, filed):
    """Delete from the hub the logs confirmed stored here that are finished. `filed` is a list of (hub file name, pulled path, stored path)."""
    deleted, left = [], []
    now = hub_now(adb, serial)
    for name, pulled_path, rel in filed:
        hub_path = HUB_RUNLOGS + "/" + name
        if not stored_has(dest, rel, pulled_path):
            left.append((name, "not confirmed stored"))
            continue
        pulled_size = os.path.getsize(pulled_path)
        stat = hub_stat(adb, serial, hub_path)
        if stat is not None and stat[0] != pulled_size:
            left.append((name, "changed on the hub since it was pulled (still running?)"))
            continue
        if not is_complete(read_lines(pulled_path)):
            idle = (now - stat[1]) if (stat is not None and now is not None) else None
            if idle is None or idle < IDLE_SECONDS:
                left.append((name, "no end marker yet (may still be running)"))
                continue
        result = run_adb(adb, serial, "shell", "rm", "-f", hub_path)
        if result.returncode == 0:
            deleted.append(name)
        else:
            left.append((name, "rm failed: " + (result.stderr or result.stdout).strip()))
    return deleted, left


def scan_store(dest):
    """Every stored run, newest first, with its summary."""
    runs = []
    for day in sorted(os.listdir(dest)) if os.path.isdir(dest) else []:
        day_dir = os.path.join(dest, day)
        if not re.match(r"\d{4}-\d\d-\d\d$", day) or not os.path.isdir(day_dir):
            continue
        for name in sorted(os.listdir(day_dir)):
            m = re.match(r"(\d{6})_(\w+)\.log$", name)
            if not m:
                continue
            path = os.path.join(day_dir, name)
            summary = summarize(read_lines(path))
            summary["kind"] = m.group(2)
            summary["file"] = "%s/%s" % (day, name)
            summary["size_kb"] = round(os.path.getsize(path) / 1024.0, 1)
            runs.append(summary)
    runs.sort(key=lambda r: r["file"], reverse=True)
    return runs


def write_latest(dest, runs):
    latest = os.path.join(dest, "latest")
    os.makedirs(latest, exist_ok=True)
    for kind in KINDS:
        of_kind = [r for r in runs if r["kind"] == kind]
        for i, suffix in enumerate(("", "_prev")):
            target = os.path.join(latest, kind + suffix + ".log")
            if i < len(of_kind):
                shutil.copyfile(os.path.join(dest, of_kind[i]["file"]), target)
            elif os.path.exists(target):
                os.remove(target)


def flags(r):
    out = []
    if r["end"] != "ended normally":
        out.append(r["end"])
    if r["backup_mode"]:
        out.append("BACKUP MODE")
    if r["flywheel_encoder_lost"]:
        out.append("FLYWHEEL ENCODER LOST")
    if r["pinpoint_faults"]:
        out.append("%d Pinpoint fault(s)" % r["pinpoint_faults"])
    return ", ".join(out)


def write_index(dest, runs):
    with open(os.path.join(dest, "index.json"), "w", encoding="utf-8") as f:
        json.dump(runs, f, indent=1)
    lines = ["# Robot run logs", "",
             "%d run(s), newest first. Times are the hub's clock (it may be wrong). Run `python tools/logs/pull_logs.py` to add new ones." % len(runs),
             "", "| Opened | Kind | Opmode | Build | Length | Ended | Alliance | Salvos | Marks | Min volts | Max loop ms | Slow loops | Flags | File |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for r in runs:
        lines.append("| %s | %s | %s | %s | %.0f s | %s | %s | %d | %d | %s | %.0f | %d | %s | [%s](%s) |" % (
            r["opened"], r["kind"], r["name"], r.get("build") or "-", r["duration_s"], r["end"], r["alliance"] or "-", r["salvos"], r.get("marks", 0),
            "-" if r["min_volts"] is None else "%.1f" % r["min_volts"], r["max_loop_ms"], r["slow_loops"], flags(r) or "-",
            r["file"], r["file"]))
    with open(os.path.join(dest, "index.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def print_index(runs, limit=12):
    if not runs:
        print("No runs stored yet.")
        return
    print("%-19s %-9s %-26s %-17s %7s  %-15s %-5s %s" % ("opened", "kind", "opmode", "build", "length", "ended", "salv.", "flags"))
    for r in runs[:limit]:
        print("%-19s %-9s %-26s %-17s %6.0fs  %-15s %5d %s" % (r["opened"], r["kind"], r["name"][:26], (r.get("build") or "-")[:17], r["duration_s"], r["end"], r["salvos"], flags(r)))
    if len(runs) > limit:
        print("... and %d older (see index.md)" % (len(runs) - limit))


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Pull, store and index the robot's run logs.")
    ap.add_argument("--dest", default=DEFAULT_DEST, help="where to store the logs (default: robot-logs/ in the repo)")
    ap.add_argument("--serial", help="adb serial, when more than one device is attached")
    ap.add_argument("--from-dir", help="file the logs already in this folder (the runlogs folder of an earlier adb pull) instead of using the hub")
    ap.add_argument("--keep-on-hub", action="store_true", help="do not delete the logs from the hub after they are stored")
    ap.add_argument("--matchlogs", action="store_true", help="also pull the SDK's match logs from the hub")
    ap.add_argument("--list", action="store_true", help="only print the index of what is stored")
    args = ap.parse_args()
    dest = os.path.abspath(args.dest)

    if args.list:
        print_index(scan_store(dest), limit=10 ** 6)
        return

    tmp = tempfile.mkdtemp(prefix="runlogs_")
    try:
        if args.from_dir:
            source = os.path.abspath(args.from_dir)
            if os.path.basename(source) != "runlogs" and os.path.isdir(os.path.join(source, "runlogs")):
                source = os.path.join(source, "runlogs")
        else:
            adb = find_adb()
            if not adb:
                sys.exit("adb not found. Install the Android platform-tools, or put adb on PATH.")
            serial = pick_device(adb, args.serial)
            pull(adb, serial, HUB_RUNLOGS, tmp)
            source = os.path.join(tmp, "runlogs")
            if args.matchlogs:
                match_dest = os.path.join(dest, "matchlogs")
                os.makedirs(match_dest, exist_ok=True)
                mtmp = os.path.join(tmp, "matchlogs_pull")
                os.makedirs(mtmp)
                pull(adb, serial, HUB_MATCHLOGS, mtmp)
                for root, _dirs, files in os.walk(mtmp):
                    for name in files:
                        target = os.path.join(match_dest, name)
                        if not os.path.exists(target) or os.path.getsize(os.path.join(root, name)) > os.path.getsize(target):
                            shutil.copyfile(os.path.join(root, name), target)
        if not os.path.isdir(source):
            sys.exit("No runlogs folder found at %s" % source)

        os.makedirs(dest, exist_ok=True)
        counts = {"new": 0, "updated": 0, "skipped": 0}
        filed = []
        for name in sorted(os.listdir(source)):
            kind = kind_of(name)
            if not name.endswith(".log") or kind is None:
                continue
            result, rel = file_in(dest, os.path.join(source, name), kind)
            counts[result] += 1
            filed.append((name, os.path.join(source, name), rel))
            if result != "skipped":
                print("%-8s %s  <- %s" % (result, rel, name))
        runs = scan_store(dest)
        write_latest(dest, runs)
        write_index(dest, runs)
        print("%d new, %d updated, %d already stored. %d run(s) in %s" % (counts["new"], counts["updated"], counts["skipped"], len(runs), dest))
        print()
        print_index(runs)
        if not args.from_dir and not args.keep_on_hub:
            deleted, left = delete_from_hub(adb, serial, dest, filed)
            print()
            print("Deleted from the hub (confirmed stored here): %s" % (", ".join(deleted) if deleted else "nothing"))
            for name, why in left:
                print("Left on the hub: %s (%s)" % (name, why))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    main()
