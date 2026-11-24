"""One event log for a whole event: every timed thing the robot logs, video frames and scoreboard gave, on the match clock.

    python tools/video/event_log.py --event "Hartland Qualifier 2026"

Reads robot-logs/events/<event>/video/<label>_fused.json (fuse_match.py) and <label>_overlay.csv, and writes
robot-logs/events/<event>/video/events.csv: one row per event, long format, for any later aggregate analysis.

Columns
  match         Q5, M10, ...
  event         what happened (see EVENTS below)
  actor         8533, partner, partner?, red, blue, field
  alliance      red / blue: whose side the event belongs to
  t_s           match time in seconds from the start of AUTO (TeleOp starts at about 38 s); the event's start
  t_end_s       end of an event that lasts (a salvo, a hive swing); blank for an instant
  duration_s    t_end_s - t_s
  source        log (8533's robot log), frames (the match video), scoreboard (the broadcast overlay), fim (official), derived
  uncertainty_s how far t_s may be off, from the method (see UNCERTAINTY below); an estimate, not a measurement
  detail        short free text: counts, cells, credit, notes
  clip_s        the same moment in the match clip, to find it in the video

Not in it: how many balls a salvo put in a cell, and whether a given ball scored. The broadcast camera cannot count
balls in a cell (they hide each other) and cannot see balls in flight across the field.
"""
import argparse, csv, glob, json, math, os, re, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "logs"))
import cycle_times as CT                          # noqa: E402
from fuse_match import logs_for, fim_match, START_CELL, OTHER_CELL   # noqa: E402

# Field geometry, field inches, from the robot code (blue frame where it differs; red is the same turned 180 degrees)
FLOWERS = {"rear wall": (-68.6, -24.0), "Blue Wall": (-24.0, 68.6), "Red Wall": (24.0, -68.6), "audience wall": (68.6, 24.0)}  # Ramp.FLOWERS
FLOWER_NEAR_IN = 24.0                              # Ramp.CLEAR_DISTANCE_IN: the robot centre this close is "at the flower"
LOADING_BLUE = (23.9, 46.6, 59.1, 70.1)            # ParkDash: x min, x max, y min, y max
ROBOT_HALF_IN = 9.0                                # an 18 in robot: only part of it needs to be in a zone
GARDEN_BLUE = (-71.0, -46.0, 40.0, 71.0)           # around DuoAutos' garden pickup (-59.8, 46 to 58.5)
HIVE_Y_BLUE, CELL_FACE_X = 12.75, 21.5             # the hive's centre line and its cells' opening faces (HiveTarget)
LAUNCH_HALF_Y, LAUNCH_MAX_X = 30.0, 62.0           # a launch area: outside a cell face, within this of the hive's line
ZONE_MIN_S = 0.3                                   # zone visits shorter than this are dropped
STALL_CMD, STALL_SPEED_IN_S, STALL_MIN_S = 0.4, 3.0, 0.5   # driving commanded this hard, moving slower than this, this long
LEAVE_MIN_IN = 6.0                                 # moved this far from the start pose = left the wall (log-based LEAVE)
SETTLE_S = 1.0                                     # pollen of a salvo come to rest within this after it ends (+1 s)
PICKUP_FLICKER_S = 0.3                             # an element count rise undone within this is the counter flickering

# how far each kind of time can be off (seconds). Log events: the log-to-video fit is good to ~0.1 s; where the fit was
# weak an offset of 0 was assumed, and good fits ranged -0.6 to +0.2 s except a late-started AUTO (+6.2 s), so 1 s.
UNCERTAINTY = {
    "log": 0.1, "log_weak_fit": 1.0,
    "frames": 0.1,              # a frame-to-frame event at 10-30 fps
    "tip_start": 0.5,           # the start of a hive swing: balls landing just before it can pull it up to ~0.7 s early
    "tip_end": 0.2,
    "tip_scoreboard_only": 2.0, # a tip the frames missed, placed 1.4 s before the counter moved (lags seen: -0.7 to 7.5 s)
    "scoreboard": 1.5,          # the score bar is entered by people: tip counters lagged 0-3 s, once 7.5 s
    "period": 0.1,              # period starts read off the overlay clock
    "cell_count_time": 1.0,     # a count change is seen within about a second (2 readings a second, median of 3)
}


def stable_changes(rows, col, hold_s=1.0):
    """(t, from, to) for each value of an overlay column that held at least hold_s (drops misreads and 1-frame glitches)."""
    segs = []
    for r in rows:
        v = r.get(col, "")
        if not v.isdigit(): continue
        t, v = float(r["t"]), int(v)
        if segs and segs[-1][0] == v: segs[-1][2] = t
        else: segs.append([v, t, t])
    st = [s for s in segs if s[2] - s[1] >= hold_s]
    return [(b[1], a[0], b[0]) for a, b in zip(st, st[1:]) if b[0] != a[0]]


def samples(path):
    """(t, x, y, cmd_fwd, cmd_left) for every sample row of a run log."""
    hdr, out = None, []
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("H,"): hdr = line.strip().split(",")[1:]
        elif line.startswith("S,") and hdr:
            v = dict(zip(hdr, line.strip().split(",")[1:]))
            try: out.append((float(v["t"]), float(v["x"]), float(v["y"]), float(v.get("cmd_fwd") or 0), float(v.get("cmd_left") or 0)))
            except (ValueError, KeyError): pass
    return out


REAL_NAME_FOR_RED = {"rear wall": "audience wall", "audience wall": "rear wall", "Blue Wall": "Red Wall", "Red Wall": "Blue Wall"}


def zone_of(x, y, ally):
    """The named area the robot centre (x, y) is in, or None for the open field. Alliance areas are taken in the blue
    frame (red turned 180 degrees); flowers are named as they are on the field."""
    bx, by = (x, y) if ally == "blue" else (-x, -y)
    for name, (fx, fy) in FLOWERS.items():
        if math.hypot(bx - fx, by - fy) <= FLOWER_NEAR_IN:
            return f"{name if ally == 'blue' else REAL_NAME_FOR_RED[name]} flower"
    x0, x1, y0, y1 = LOADING_BLUE
    if x0 - ROBOT_HALF_IN <= bx <= x1 + ROBOT_HALF_IN and y0 - ROBOT_HALF_IN <= by <= y1 + ROBOT_HALF_IN: return "our loading zone"
    x0, x1, y0, y1 = GARDEN_BLUE
    if x0 <= bx <= x1 and y0 <= by <= y1: return "our garden"
    if CELL_FACE_X <= abs(bx) <= LAUNCH_MAX_X and abs(by - HIVE_Y_BLUE) <= LAUNCH_HALF_Y:
        return "audience launch area" if x > 0 else "rear launch area"
    return None


def salvo_added(d, s, salvos, ally):
    """Pollen seen added to the up cell's resting count by this salvo: the count 1 s after it ends minus the count just
    before it started. None when a tip or another salvo falls in between, or the cell was not counted."""
    ser = d.get("cell_pollen")
    if not ser or s.get("done") is None: return None
    t0, t1 = s["start"] - 0.3, s["done"] + 1.0 + SETTLE_S
    if any(t0 <= (t.get("start") or t["t"]) <= t1 + 1.0 for t in d["tips"] if t["alliance"] == ally): return None
    if any(o is not s and t0 < o["start"] <= t1 for o in salvos): return None
    before = [v for t, v in ser if s["start"] - 1.5 <= t <= t0]
    after = [v for t, v in ser if s["done"] + 1.0 <= t <= t1]
    if not before or not after: return None
    return int(round(np.median(after) - before[-1]))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--event", required=True); a = ap.parse_args()
    vdir = os.path.join(ROOT, "robot-logs", "events", a.event, "video")
    views = json.load(open(os.path.join(vdir, "video_config.json"))).get("views", {})
    fim = json.load(open(os.path.join(vdir, "fim_scores.json")))
    out = []
    for path in sorted(glob.glob(os.path.join(vdir, "*_fused.json"))):
        d = json.load(open(path))
        L, a0, ally = d["label"], d["auto_start_clip"], d["alliance"]
        other = "blue" if ally == "red" else "red"
        mt = lambda x: "" if x is None else round(x - a0, 2)

        def row(event, actor, alliance, t, t_end=None, source="", unc="", detail=""):
            out.append({"match": L, "event": event, "actor": actor, "alliance": alliance, "t_s": mt(t),
                        "t_end_s": mt(t_end), "duration_s": "" if t_end is None or t is None else round(t_end - t, 2),
                        "source": source, "uncertainty_s": unc, "detail": detail, "clip_s": "" if t is None else round(t, 2)})

        def log_unc(period):
            return UNCERTAINTY["log_weak_fit"] if d["alignment"].get(period, {}).get("weak") else UNCERTAINTY["log"]

        # periods (overlay clock)
        tele = d["teleop_start_clip"]
        row("AUTO start", "field", "", a0, source="scoreboard", unc=UNCERTAINTY["period"], detail="overlay clock leaves 2:30")
        row("AUTO end", "field", "", a0 + 30, source="derived", unc=UNCERTAINTY["period"], detail="AUTO start + 30 s")
        row("TeleOp start", "field", "", tele, source="scoreboard", unc=UNCERTAINTY["period"], detail="overlay clock leaves 2:00")
        row("match end", "field", "", tele + 120, source="derived", unc=UNCERTAINTY["period"], detail="TeleOp start + 120 s")

        # video notes
        v = views.get(L, {})
        if v.get("camera_moved"): row("video note", "field", "", None, source="frames", detail="the camera moved during the match")
        if not d.get("landing_video_ok", True): row("video note", "field", "", None, source="frames", detail="pollen landings not read: the hive is not visible well enough")
        if (d.get("held_frame_share") or 0) > 0.005: row("video note", "field", "", None, source="frames", detail=f"the stream held frames ({d['held_frame_share']:.1%} of the match)")
        for k, al in d["alignment"].items():
            if al.get("weak"): row("video note", "field", "", None, source="derived", detail=f"{k} log-to-video fit weak (r {al['r']:.2f}): log times assume offset 0, +-1 s")

        # hive tips (frames; scoreboard-only where the frames missed one)
        for t in sorted(d["tips"], key=lambda t: t["t"]):
            actor = t["alliance"] if t["alliance"] != ally else {"8533": "8533", "8533 + partner": "8533 + partner", "partner (seen)": "partner",
                                                                 "partner?": "partner?"}.get(t.get("credit", ""), t.get("credit", ""))
            det = f"tip {t['n']} of {t['alliance']} hive; {t['period']}"
            if t["alliance"] == ally:
                det += f"; credit {t.get('credit')}; {t.get('our_salvos')} of our salvos ({t.get('our_elements')} elements held) since the previous tip"
                if t.get("pollen_before_tip") is not None: det += f"; about {t['pollen_before_tip']:.0f} pollen seen resting in the cell just before it tipped (at least)"
            if t["source"] == "frames, not counted by FiM": det += "; NOT counted by FiM or the scoreboard"
            if t["source"] == "frames" and t.get("start") is not None:
                row("hive tip", actor, t["alliance"], t["start"], t["end"], "frames", UNCERTAINTY["tip_start"], det + f"; swing peak {mt(t['t'])} s")
            elif t["source"].startswith("frames") and t.get("start") is not None:
                row("hive tip", actor, t["alliance"], t["start"], t["end"], "frames", UNCERTAINTY["tip_start"], det + f"; swing peak {mt(t['t'])} s")
            else:
                row("hive tip", actor, t["alliance"], t["t"], None, "scoreboard", UNCERTAINTY["tip_scoreboard_only"], det + "; the frames missed it: time from the scoreboard, no swing")
            if t.get("scoreboard_t") is not None and t["source"] == "frames":
                row("tip on scoreboard", "field", t["alliance"], t["scoreboard_t"], source="scoreboard", unc=UNCERTAINTY["frames"],
                    detail=f"tip {t['n']} counter rises {t.get('scoreboard_lag')} s after the swing peak")

        # 8533 salvos (log) and their pollen landing (frames)
        for i, s in enumerate(sorted(d["salvos"], key=lambda s: s["start"])):
            st = s.get("stack", "") or ""
            comp = f"pollen {st.count('P')}, red nectar {st.count('R')}, blue nectar {st.count('B')}, unclassified {st.count('O')}"
            det = f"salvo {i + 1}; {s['period']}; held {s['held']} ({comp}; stack {st or '?'})" + (f"; aimed {s['target']}, up cell {s['up_cell']}" if s.get("target") else "")
            if s.get("right_cell") is False: det += "; AT THE DOWN CELL"
            added = salvo_added(d, s, sorted(d["salvos"], key=lambda x: x["start"]), ally)
            if added is not None: det += f"; about {added:+d} pollen seen added to the resting count (at least; launched {s['held']})"
            row("8533 salvo", "8533", ally, s["start"], s["done"], "log", log_unc(s["period"]), det)
            if s.get("landing_seen"):
                row("8533 landing", "8533", ally, s["start"] + s["landing_delay_s"], source="frames", unc=UNCERTAINTY["frames"],
                    detail=f"{s.get('landing_kind') or 'pollen'}; salvo {i + 1}: first ball seen arriving in the up cell, {s['landing_delay_s']} s after the salvo started")
            elif s.get("landing_seen") is False:
                row("8533 no landing seen", "8533", ally, s["start"], source="frames", unc="",
                    detail=f"salvo {i + 1}: no pollen or nectar seen arriving within 2 s of the salvo ending (a miss, or not visible)")

        # pollen arriving in our up cell with no salvo of ours (frames): most likely the partner
        for b in d.get("other_landings", []):
            row("partner landing", "partner?", ally, b["start"], b["end"], "frames", UNCERTAINTY["frames"],
                detail=f"{b.get('kind', 'pollen')}; {b['flights']} flight(s) seen, no 8533 salvo or tip explains it: most likely the partner's salvo "
                       "arriving (it fired about 1 s earlier, like ours), sometimes a ball bouncing past")

        # 8533 cycle phases and mechanism events (log)
        for i, c in enumerate(d["cycles"]):
            det = f"cycle {i + 1}: salvo {mt(c['start'])}-{mt(c['done'])}, last pickup {mt(c['reach'])}"
            if c.get("arrive") is not None: det += f", at the launch spot {mt(c['arrive'])}"
            det += f", lock {mt(c['lock_on'])}, next salvo {mt(c['next'])}; next salvo held {c['held_next']}"
            row("8533 cycle", "8533", ally, c["start"], c["next"], "log", log_unc("teleop"), det)
        for m in d["marks"]:
            if m["kind"] in ("auto step",): continue
            row(f"8533 {m['kind']}", "8533", ally, m["t"], source="log", unc=log_unc(m["period"]), detail=m["text"][:120])

        # ---- from the robot logs: zone visits, pickups, stalls (8533 only)
        logs = logs_for(os.path.dirname(vdir), L)

        def to_clip(kind, t):
            x = d["alignment"][kind]; return x["period_start_clip"] + (t - x["run_started_log_t"]) + x["delta_s"]

        for kind in ("auto", "teleop"):
            if kind not in logs or kind not in d["alignment"]: continue
            S = samples(logs[kind]); unc = log_unc(kind)
            if not S: continue
            cur, t0 = None, None
            for (t, x, y, _, _) in S + [(S[-1][0] + 1e-3, None, None, 0, 0)]:
                z = zone_of(x, y, ally) if x is not None else "__end"
                if z != cur:
                    if cur and cur != "__end" and t - t0 >= ZONE_MIN_S:
                        row("8533 in zone", "8533", ally, to_clip(kind, t0), to_clip(kind, t), "log", unc, f"{cur}; {kind}")
                    cur, t0 = z, t
            ev = CT.events(logs[kind])
            elem = []
            for t, msg in ev:
                m_ = re.match(r"ELEMENTS (\d) -> (\d)", msg)
                if m_: elem.append((t, int(m_.group(1)), int(m_.group(2))))
            for i, (t, a_, b_) in enumerate(elem):
                if b_ <= a_: continue
                if any(tt - t <= PICKUP_FLICKER_S and bb <= a_ for tt, aa, bb in elem[i + 1:]): continue
                px = min(S, key=lambda q: abs(q[0] - t))
                z = zone_of(px[1], px[2], ally)
                row("8533 pickup", "8533", ally, to_clip(kind, t), source="log", unc=unc,
                    detail=f"element count {a_} -> {b_} (+{b_ - a_}); {kind}; {z or 'open field'} at ({px[1]:.0f}, {px[2]:.0f}) in")
            # our own LEAVE / park, from the pose, to set beside FiM's per-station fields (Hartland Q24: FiM has robot 2 not
            # leaving, which by station would be 8533, while 8533's log shows it drove its whole auto)
            if kind == "auto":
                moved = max(math.hypot(q[1] - S[0][1], q[2] - S[0][2]) for q in S)
                row("8533 LEAVE (log)", "8533", ally, a0 + 30, source="log", unc=unc,
                    detail=f"{'yes' if moved >= LEAVE_MIN_IN else 'no'}: moved up to {moved:.0f} in from its start pose")
            last = S[-1]
            z = zone_of(last[1], last[2], ally)
            row(f"8533 {'AUTO' if kind == 'auto' else 'TeleOp'} park (log)", "8533", ally, to_clip(kind, last[0]), source="log", unc=unc,
                detail=f"{'yes' if z == 'our loading zone' else 'no'}: last pose ({last[1]:.0f}, {last[2]:.0f}) in, {z or 'open field'}")
            if kind == "teleop":   # AUTO squares on walls and jostles flowers on purpose, so stalls are TeleOp only
                run = []
                for k_, ((ta, xa, ya, fa, la), (tb, xb, yb, fb, lb)) in enumerate(zip(S, S[1:])):
                    v = math.hypot(xb - xa, yb - ya) / max(1e-3, tb - ta)
                    stuck = math.hypot(fb, lb) >= STALL_CMD and v < STALL_SPEED_IN_S
                    if stuck: run.append((tb, xb, yb, math.hypot(fb, lb)))
                    if run and (not stuck or k_ == len(S) - 2):
                        if run[-1][0] - run[0][0] >= STALL_MIN_S:
                            z = zone_of(run[0][1], run[0][2], ally)
                            row("8533 stall", "8533", ally, to_clip(kind, run[0][0]), to_clip(kind, run[-1][0]), "log", unc,
                                f"driving commanded (up to {max(r[3] for r in run):.2f}) but under {STALL_SPEED_IN_S:.0f} in/s; {z or 'open field'} "
                                f"at ({run[0][1]:.0f}, {run[0][2]:.0f}) in; what blocked it is not in the log")
                        run = []

        # ---- how many pollen rest in our up cell (frames), as steps; our nectar from the rules and the landings
        if d.get("cell_pollen"):
            prev = None
            for t, v in d["cell_pollen"]:
                v = int(round(v))
                if v != prev:
                    row("up cell pollen", ally, ally, t, source="frames", unc=UNCERTAINTY["cell_count_time"],
                        detail=f"{v} pollen seen resting in our up cell (at least: balls behind the front row are hidden; good to about +-1-2)"
                               + ("" if prev is None else f"; was {prev}"))
                    prev = v
        elif d.get("cell_pollen") is None and not d.get("landing_video_ok", True):
            row("video note", "field", "", None, source="frames", detail="up cell pollen not counted: the video does not show the hive well enough")
        cn = d.get("cell_nectar")
        if cn:
            row("up cell nectar", ally, ally, a0, cn["until"], "derived", 0,
                f"{cn['preloaded']} of our nectar placed in the starting up cell (rules), until our hive's first tip dumps them")
            row("up cell nectar", ally, ally, cn["until"], tele + 120, "derived", "",
                f"after the first tip: nectar seen landing in our up cell {cn['seen_landing']} time(s) (counting nectar at rest is not reliable: "
                "its colour is the frame bars')")

        # ---- our hive's up cell, as intervals between our tips (a tip ends when its swing comes to rest)
        cell, start_t = START_CELL[ally], a0
        for t in sorted((t for t in d["tips"] if t["alliance"] == ally), key=lambda t: t["t"]):
            flip = t["end"] if t.get("end") is not None else t["t"]
            row("up cell", ally, ally, start_t, flip, "derived", UNCERTAINTY["tip_end"], f"{cell} up (our hive)")
            cell, start_t = OTHER_CELL[cell], flip
        row("up cell", ally, ally, start_t, tele + 120, "derived", UNCERTAINTY["tip_end"], f"{cell} up (our hive)")

        # ---- official outcomes (FiM), placed when each is assessed
        fm = fim_match(fim, L)
        if fm:
            sc = fm["scores"][ally]
            by_station = {t["station"]: t["teamNumber"] for t in fm["teams"] if t["alliance"].lower() == ally}
            for st_i, st in ((1, "One"), (2, "Two")):
                team = str(by_station.get(st, "?"))
                row("LEAVE", team, ally, a0 + 30, source="fim", unc=0, detail="yes" if sc[f"autoLeave{st_i}"] else "no")
                row("AUTO park", team, ally, a0 + 30, source="fim", unc=0, detail="yes" if sc[f"autoParkPoints{st_i}"] else "no")
                row("TeleOp park", team, ally, tele + 120, source="fim", unc=0, detail="yes" if sc[f"dcParkPoints{st_i}"] else "no")
            row("end state", "field", ally, tele + 120, source="fim", unc=0,
                detail=f"our up cell held {sc['dcHivePollenNectar']} elements at rest; garden {sc['dcGardenPollenNectar']}; flower points {sc['dcFlowerPoints']}; "
                       f"penalty points from the opponents {sc['penaltyPointsByOpp']}; total {sc['totalPoints']}")

        # score changes (scoreboard, lagged and entered by people)
        ov = os.path.join(vdir, f"{L}_overlay.csv")
        if os.path.exists(ov):
            rows = list(csv.DictReader(open(ov)))
            sb_tips = [t["scoreboard_t"] for t in d["tips"] if t["alliance"] == ally and t.get("scoreboard_t") is not None]
            for al in ("red", "blue"):
                for t, f, to in stable_changes(rows, f"{al}_score"):
                    if t < a0 - 1 or t > tele + 125: continue
                    dlt, kind = to - f, ""
                    if al == ally:   # what kind of points, from the timing (ours only: no opposing-alliance analysis yet)
                        if dlt > 0 and dlt % 20 == 0 and any(abs(t - x) <= 2.0 for x in sb_tips): kind = "hive tip" + (f" x{dlt // 20}" if dlt > 20 else "")
                        elif dlt < 0: kind = "correction"
                        elif dlt in (3, 6) and t <= tele + 2: kind = "LEAVE"
                        elif dlt in (5, 10) and (t <= tele + 2 or t >= tele + 118): kind = "park"
                        elif t >= tele + 118: kind = "end of match (park, cells, flowers, garden)"
                        else: kind = "other (flower, garden, opponents' foul or a correction)"
                    row("score change", al, al, t, source="scoreboard", unc=UNCERTAINTY["scoreboard"],
                        detail=f"{f} -> {to} ({dlt:+d})" + (f"; {kind}" if kind else ""))

    out.sort(key=lambda r: (r["match"], r["t_s"] if r["t_s"] != "" else -1))
    cols = ["match", "event", "actor", "alliance", "t_s", "t_end_s", "duration_s", "source", "uncertainty_s", "detail", "clip_s"]
    p = os.path.join(vdir, "events.csv")
    with open(p, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols); w.writeheader(); w.writerows(out)
    from collections import Counter
    print(f"{len(out)} events -> {p}")
    for k, n in sorted(Counter(r["event"] for r in out).items(), key=lambda x: -x[1]): print(f"  {n:5d}  {k}")


if __name__ == "__main__":
    main()
