"""Official match results for an event from the FIRST FTC Events API, written as video/fim_scores.json.

    set FTC_API_USER=<your FTC Events API username>
    set FTC_API_TOKEN=<your authorization key>
    python tools/video/fetch_fim.py --event "Hartland Qualifier 2026" --code USMIHOQ --season 2026 --inspect
    python tools/video/fetch_fim.py --event "Hartland Qualifier 2026" --code USMIHOQ --season 2026            (compare only)
    python tools/video/fetch_fim.py --event "Hartland Qualifier 2026" --code USMIHOQ --season 2026 --write    (replace the file)

The key comes from ftc-events.firstinspires.org (API / Services: register for the FTC Events API). It is read from the
environment only, never written anywhere; keep it out of git. Requests go to ftc-api.firstinspires.org only.

What it does:
  1. GET /v2.0/<season>/matches/<code>?tournamentLevel=qual|playoff  (teams, stations, start times)
     GET /v2.0/<season>/scores/<code>/qual|playoff                    (the per-alliance score breakdown)
     and keeps the raw answers in video/fim_raw_first.json (to read FIRST's own field names later).
  2. Maps FIRST's breakdown to the fields the pipeline reads (FIELDS below: fuse_match uses the tips and total, event_log
     the LEAVE / park per station and the end state). The first fetches of this season were made through FTCScout, which
     renames some of FIRST's fields; each of ours lists the names it may have in FIRST's answer. --inspect prints FIRST's
     names for one match and which of ours were found, so a missing one can be added to FIELDS.
  3. Compares every mapped value with the current fim_scores.json and lists any difference. Only --write replaces the
     file (the old one is kept as fim_scores.prev.json), and only when every field was found and nothing differs;
     --accept-differences writes anyway, once each difference has been checked on the event's match pages.

The output has the same shape as before: {"source", "matches": [{"id", "matchNum", "series", "tournamentLevel" ("Quals" or
"DoubleElim"), "actualStartTime", "teams": [{"teamNumber", "alliance" ("Red" / "Blue"), "station" ("One" / "Two")}],
"scores": {"red": {...}, "blue": {...}}}]}.
"""
import argparse, base64, datetime, json, os, shutil, sys, urllib.error, urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
API = "https://ftc-api.firstinspires.org/v2.0"

# our field -> the names it may have in FIRST's score breakdown (first found wins); FIRST's 2026 names checked on the
# first run with a key (2026-11-13, Hartland): teleop* for FTCScout's dc*, foulPointsCommitted. A new season: --inspect.
FIELDS = {
    "autoHiveTips": ["autoHiveTips", "autoTips"],
    "dcHiveTips": ["dcHiveTips", "teleopHiveTips", "teleOpHiveTips", "dcTips", "teleopTips"],
    "totalPoints": ["totalPoints"],
    "dcHivePollenNectar": ["dcHivePollenNectar", "teleopHivePollenNectar"],
    "dcGardenPollenNectar": ["dcGardenPollenNectar", "teleopGardenPollenNectar"],
    "dcFlowerPoints": ["dcFlowerPoints", "teleopFlowerPoints"],
}
# per-robot LEAVE and park: FIRST has true / false per station (2026: autoRobot1Leave ...), FTCScout had the points;
# kept as points (LEAVE 3, PARK 5, game manual) so the file reads as before (Hartland: true = 3 / 5, false = 0 in all 16)
ROBOT_POINTS = {
    "autoLeave1": ("autoRobot1Leave", 3), "autoLeave2": ("autoRobot2Leave", 3),
    "autoParkPoints1": ("autoRobot1Park", 5), "autoParkPoints2": ("autoRobot2Park", 5),
    "dcParkPoints1": ("teleopRobot1Park", 5), "dcParkPoints2": ("teleopRobot2Park", 5),
}
# points the alliance got from the OTHER alliance's fouls: FIRST lists what each alliance committed
FOULS_COMMITTED = ["penaltyPointsCommitted", "foulPointsCommitted"]
LEVEL = {"qual": "Quals", "playoff": "DoubleElim"}
STATION = {"1": "One", "2": "Two"}


def get(path, user, token):
    req = urllib.request.Request(API + path, headers={
        "Authorization": "Basic " + base64.b64encode(f"{user}:{token}".encode()).decode(),
        "Accept": "application/json", "User-Agent": "ftc-event-tools"})
    try:
        with urllib.request.urlopen(req, timeout=30) as r:
            return json.load(r)
    except urllib.error.HTTPError as e:
        if e.code == 401:
            sys.exit("401 from the FTC Events API: the username or key is wrong (or the key is not activated yet)")
        sys.exit(f"{e.code} from the FTC Events API for {path}: {e.read()[:300]!r}")


def pick(d, names):
    for n in names:
        if n in d:
            return n, d[n]
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True, help="the event folder under robot-logs/events")
    ap.add_argument("--code", required=True, help="FIRST event code, e.g. USMIHOQ")
    ap.add_argument("--season", required=True, type=int, help="the season year in FIRST's URLs, e.g. 2026")
    ap.add_argument("--inspect", action="store_true", help="print FIRST's field names for one match and what was found")
    ap.add_argument("--write", action="store_true", help="replace fim_scores.json (only when every field was found and nothing differs)")
    ap.add_argument("--accept-differences", action="store_true",
                    help="with --write: replace the file although values differ (after checking each one on the match pages)")
    a = ap.parse_args()
    user, token = os.environ.get("FTC_API_USER"), os.environ.get("FTC_API_TOKEN")
    if not user or not token:
        sys.exit("set FTC_API_USER and FTC_API_TOKEN (the FTC Events API username and authorization key) first")
    vdir = os.path.join(ROOT, "robot-logs", "events", a.event, "video")
    if not os.path.isdir(vdir):
        sys.exit(f"no event folder {vdir}")

    raw, out, missing = {}, [], set()
    for lvl in ("qual", "playoff"):
        matches = get(f"/{a.season}/matches/{a.code}?tournamentLevel={lvl}", user, token)
        scores = get(f"/{a.season}/scores/{a.code}/{lvl}", user, token)
        raw[lvl] = {"matches": matches, "scores": scores}
        by_key = {(m.get("series", 0), m["matchNumber"]): m for m in matches.get("matches", [])}
        for sc in scores.get("matchScores", scores.get("MatchScores", [])):
            key = (sc.get("matchSeries", 0), sc["matchNumber"])
            m = by_key.get(key, {})
            al = {x["alliance"].lower(): x for x in sc.get("alliances", [])}
            if a.inspect and not out:
                print(f"FIRST match record keys: {sorted(m)}")
                print(f"FIRST score record keys: {sorted(k for k in sc if k != 'alliances')}")
                for side, x in al.items():
                    print(f"FIRST {side} alliance fields: {sorted(x)}")
            res = {}
            for side, x in al.items():
                r = {}
                for ours, names in FIELDS.items():
                    n, v = pick(x, names)
                    if n is None:
                        missing.add(ours)
                    else:
                        r[ours] = v
                for ours, (name, pts) in ROBOT_POINTS.items():
                    if ours in x:
                        r[ours] = x[ours]
                    elif name in x:
                        r[ours] = pts if x[name] else 0
                    else:
                        missing.add(ours)
                other = al.get("red" if side == "blue" else "blue", {})
                n, v = pick(other, FOULS_COMMITTED)
                if n is None:
                    missing.add("penaltyPointsByOpp")
                else:
                    r["penaltyPointsByOpp"] = v
                res[side] = r
            teams = []
            for t in m.get("teams", []):
                st = str(t.get("station", ""))   # "Red1", "Blue2"
                teams.append({"teamNumber": t["teamNumber"], "alliance": "Red" if st.startswith("Red") else "Blue", "station": STATION.get(st[-1:], st)})
            out.append({"id": None, "matchNum": sc["matchNumber"], "series": sc.get("matchSeries", 0), "tournamentLevel": LEVEL[lvl],
                        "actualStartTime": m.get("actualStartTime"), "teams": teams, "scores": res})
    json.dump(raw, open(os.path.join(vdir, "fim_raw_first.json"), "w"), indent=1)
    print(f"{len(out)} matches from FIRST ({sum(m['tournamentLevel'] == 'Quals' for m in out)} qualification, "
          f"{sum(m['tournamentLevel'] == 'DoubleElim' for m in out)} playoff); raw answers in fim_raw_first.json")
    if missing:
        print("NOT FOUND in FIRST's breakdown (add their names to FIELDS / FOULS_COMMITTED): " + ", ".join(sorted(missing)))
    if a.inspect:
        print("fields found for the first match: " + ", ".join(f"{k}={v}" for k, v in out[0]["scores"].get("blue", {}).items()) if out else "no matches")

    # compare with the current file, match by match (only the matches it has)
    cur_path = os.path.join(vdir, "fim_scores.json")
    diffs, compared = [], 0
    if os.path.exists(cur_path):
        cur = json.load(open(cur_path))
        new_by = {(m["tournamentLevel"], m["series"] if m["tournamentLevel"] == "DoubleElim" else m["matchNum"]): m for m in out}
        for m in cur["matches"]:
            key = (m["tournamentLevel"], m["series"] if m["tournamentLevel"] == "DoubleElim" else m["matchNum"])
            n = new_by.get(key)
            label = ("Q" if key[0] == "Quals" else "M") + str(key[1])
            if n is None:
                diffs.append(f"{label}: not in FIRST's answer"); continue
            compared += 1
            if sorted((t["teamNumber"], t["alliance"], t["station"]) for t in m["teams"]) != sorted((t["teamNumber"], t["alliance"], t["station"]) for t in n["teams"]):
                diffs.append(f"{label}: teams or stations differ")
            for side in ("red", "blue"):
                for k in list(FIELDS) + list(ROBOT_POINTS) + ["penaltyPointsByOpp"]:
                    if k in n["scores"].get(side, {}) and m["scores"][side].get(k) != n["scores"][side][k]:
                        diffs.append(f"{label} {side} {k}: file {m['scores'][side].get(k)!r}, FIRST {n['scores'][side][k]!r}")
        print(f"compared {compared} matches with {os.path.basename(cur_path)} ({cur.get('source', '?')}): " + (f"{len(diffs)} differences" if diffs else "no differences"))
        for d in diffs:
            print("  " + d)
    if a.write:
        if missing:
            sys.exit("not written: some fields were not found (see above)")
        if diffs and not a.accept_differences:
            sys.exit("not written: values differ from the current file (a wrong field mapping, or a real correction); check them on "
                     f"ftc-events.firstinspires.org/{a.season}/{a.code}/qualifications/<n> and rerun with --accept-differences")
        if os.path.exists(cur_path):
            shutil.copy(cur_path, os.path.join(vdir, "fim_scores.prev.json"))
        src = f"ftc-api.firstinspires.org v2.0, event {a.code} {a.season}, fetched {datetime.datetime.now(datetime.timezone.utc):%Y-%m-%d %H:%M} UTC"
        json.dump({"source": src, "matches": out}, open(cur_path, "w"), indent=1)
        print(f"wrote {cur_path} (the previous file is fim_scores.prev.json)")


if __name__ == "__main__":
    main()
