"""Lists the @Config classes that ConfigDump does not write into the run log (so a new tunable class does not go unlogged by accident).

    python tools/logs/check_config_list.py

Bring-up, verify and auto-build opmodes are left out on purpose (see ConfigDump).
"""
import glob
import os
import re

SRC = os.path.join("TeamCode", "src", "main", "java", "org", "firstinspires", "ftc", "teamcode")
SKIP_DIRS = ("bringup", "verify", "autobuild")
SKIP_FILES = ()


def main():
    dump = open(os.path.join(SRC, "util", "ConfigDump.java"), encoding="utf-8").read()
    listed = set(re.findall(r"(\w+)\.class", dump))
    missing = []
    for path in glob.glob(os.path.join(SRC, "**", "*.java"), recursive=True):
        parts = path.split(os.sep)
        if any(d in parts for d in SKIP_DIRS) or os.path.basename(path) in SKIP_FILES:
            continue
        text = open(path, encoding="utf-8", errors="ignore").read()
        if re.search(r"^@Config", text, re.M):
            name = os.path.basename(path)[:-5]
            if name not in listed:
                missing.append(name)
    if missing:
        print("@Config classes missing from ConfigDump.CLASSES: " + ", ".join(sorted(missing)))
    else:
        print("Every @Config class (outside bring-up, verify and auto-build) is in ConfigDump.")


if __name__ == "__main__":
    main()
