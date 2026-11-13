"""One match's robot log + match video + FiM score, on one clock.

    python tools/video/fuse_match.py --event "Hartland Qualifier 2026" --label Q5

Needs: robot-logs/events/<event>/labels.csv (event_day.py label), robot-logs/events/<event>/video/video_config.json
(clip folder, field corners in the picture, hive boxes) and video/fim_scores.json (fetch_fim.py). Writes
video/<label>_fused.json and prints a summary. Intermediate results are cached in video/ (<label>_overlay.csv,
<label>_tips.json, <label>_align.json); delete one to recompute it.

Sources and how much each is trusted:
  robot log  (our robot: salvos, lock, intake, pose)                   - primary truth for what OUR robot did
  frames     (physical HIVE TIPs, robot motion for the clock alignment) - real-time evidence of what happened on the field
  scoreboard (overlay clock, tip counters, score)                       - match clock and tip ORDER only: the
                                                                         counters are entered by people and lag (0-3 s)
  FiM        (official breakdown)                                       - final counts to check the others against

Clock: video time = period start (clock leaving 2:30 / 2:00 on the overlay) + (log t - RUN STARTED) + delta, with delta
fitted from the logged robot speed against the motion at the robot's projected position in the picture.
"""
import argparse, csv, glob, json, os, re, subprocess, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "logs"))
import align_log, hive_tips, pollen_landings, cell_count   # noqa: E402
import cycle_times as CT             # noqa: E402

TIP_WINDOW_AFTER_SALVO = (-0.5, 4.0)   # a tip this long after one of our salvos ENDS (or during it) is credited to the salvo
LAND_AFTER_DONE_S = 2.0                # a landing starting between salvo start and this long after it ends belongs to the salvo
OURS_SETTLE_S = 3.5                    # balls of our salvo keep rolling/bouncing this long after it ends (not partner shots)
TIP_SETTLE = (-2.5, 4.0)               # flights this long around a tip are the tip moving balls, not shots
LANDING_REF_HIVE_W_PX = 185.0         # hive box width in the framing pollen_landings was tuned on (Hartland Q5)
LANDING_MIN_FRAME_TIPS = 0.5           # landings are only read where at least this share of our tips was found in the frames
MIX_MERGE_S = 0.6                      # pollen and nectar landings this close are one volley
PARTNER_BEFORE_TIP_S = 4.0             # a partner landing this long before a "partner?" tip makes it "partner (seen)"
WEAK_FIT_R = 0.35                      # below this the log-to-video fit is not trusted
PAIR_MAX_S = 10.0                      # a scoreboard rise this long after a frame tip can still be its registration
SB_LAG_GUESS = 1.4                     # a tip the frames missed is placed this long before its scoreboard rise (Q5 median)
START_CELL = {"blue": "BLUE_REAR", "red": "RED_AUDIENCE"}   # TeleOpCompetition / autos: the up cell at match start
OTHER_CELL = {"BLUE_REAR": "BLUE_AUDIENCE", "BLUE_AUDIENCE": "BLUE_REAR", "RED_REAR": "RED_AUDIENCE", "RED_AUDIENCE": "RED_REAR"}


def clip_for(label, clip_dir):
    m = re.match(r"([A-Za-z]+)(\d+)$", label)
    pat = f"{m.group(1).upper()}{int(m.group(2)):02d}_*.mp4"
    hits = glob.glob(os.path.join(clip_dir, pat))
    if len(hits) != 1: sys.exit(f"clip for {label}: {hits or 'none'} matching {pat} in {clip_dir}")
    return hits[0]


def logs_for(event_dir, label):
    out = {}
    for r in csv.DictReader(open(os.path.join(event_dir, "labels.csv"), encoding="utf-8")):
        if r["label"].strip() == label:
            kind = "auto" if r["log"].endswith("_auto") else "teleop" if r["log"].endswith("_teleop") else None
            if kind: out[kind] = os.path.join(ROOT, "robot-logs", r["log"] + ".log")
    return out


def fim_match(fim, label):
    lvl = "Quals" if label.upper().startswith("Q") else "DoubleElim"
    n = int(re.sub(r"\D", "", label))
    for m in fim["matches"]:
        if m["tournamentLevel"] == lvl and (m["matchNum"] == n if lvl == "Quals" else m["series"] == n):
            return m
    return None


def periods(overlay_rows):
    """Clip times of the AUTO start and TELEOP start from the overlay clock (it shows 2:29 one second in)."""
    seen_auto = auto = tele = None
    for r in overlay_rows:
        c, t = r["clock"], float(r["t"])
        if auto is None and c == "2:29": auto = t - 1.0
        if auto is not None and c in ("0:08", "0:07", "0:06"): seen_auto = True
        if seen_auto and tele is None and c == "1:59": tele = t - 1.0
    return auto, tele


def overlay_tip_changes(rows, col, hold_s=1.5):
    """(clip time, count) for each rise of the scoreboard tip counter. A value must stay up hold_s to count, so a
    scorekeeper's correction (M10: 2 -> 3 -> 2 within 0.9 s) and single misread frames are ignored."""
    segs = []   # [value, first t, last t]
    for r in rows:
        v = r.get(col, "")
        if not v.isdigit(): continue
        t, v = float(r["t"]), int(v)
        if segs and segs[-1][0] == v: segs[-1][2] = t
        else: segs.append([v, t, t])
    stable = [s for s in segs if s[2] - s[1] >= hold_s]
    out = []
    for a, b in zip(stable, stable[1:]):
        for k in range(a[0] + 1, b[0] + 1):
            out.append((b[1], k))
    return out


def log_events(path):
    ev = CT.events(path)
    start = next((t for t, m in ev if m.startswith("RUN STARTED")), 0.0)
    salvos, cur_target = [], None
    for t, m in ev:
        if m.startswith("LAUNCH requested"):
            mm = re.search(r"target (\w+)", m); cur_target = mm.group(1) if mm else cur_target
        elif m.startswith("SALVO started"):
            h = re.search(r"(\d) held \((\S+)\)", m)
            salvos.append({"start": t, "done": None, "held": int(h.group(1)) if h else None, "stack": h.group(2) if h else "", "target": cur_target})
        elif m.startswith("SALVO done") and salvos and salvos[-1]["done"] is None:
            salvos[-1]["done"] = t
    marks = []
    for t, m in ev:
        for key, name in (("TARGET LOCK on", "lock on"), ("TARGET LOCK off", "lock off"), ("COMMAND ramp deploy", "ramp deploy"),
                          ("COMMAND ramp stow", "ramp stow"), ("INTAKE mode", "intake"), ("HIVE DASH start", "hive dash"),
                          ("PARK DASH", "park dash"), ("WARNING FULL", "full"), ("AUTO TIMING", "auto step"), ("CRASH", "crash")):
            if m.startswith(key): marks.append({"t": t, "kind": name, "text": m[:160]}); break
    return start, salvos, marks, ev


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True); ap.add_argument("--label", required=True)
    ap.add_argument("--team", type=int, default=8533)
    a = ap.parse_args()
    ev_dir = os.path.join(ROOT, "robot-logs", "events", a.event)
    vdir = os.path.join(ev_dir, "video")
    cfg = json.load(open(os.path.join(vdir, "video_config.json")))
    fim = json.load(open(os.path.join(vdir, "fim_scores.json")))
    clip = clip_for(a.label, cfg["clip_dir"])
    view = cfg.get("views", {}).get(a.label)
    if view is None: sys.exit(f"no camera view for {a.label}: run tools/video/calibrate_views.py first")
    cfg["hive_rois"], cfg["field_corners_px"] = view["hive_rois"], view["field_corners_px"]   # this match's framing
    logs = logs_for(ev_dir, a.label)
    fm = fim_match(fim, a.label)
    mine = next(t for t in fm["teams"] if t["teamNumber"] == a.team)
    alliance = mine["alliance"].lower()
    other = "blue" if alliance == "red" else "red"
    partners = [t["teamNumber"] for t in fm["teams"] if t["alliance"].lower() == alliance and t["teamNumber"] != a.team]
    opponents = [t["teamNumber"] for t in fm["teams"] if t["alliance"].lower() == other]

    # 1. overlay (clock, scores, tip counters)
    ov_path = os.path.join(vdir, f"{a.label}_overlay.csv")
    if not os.path.exists(ov_path):
        subprocess.run([sys.executable, "-I", os.path.join(HERE, "overlay_ocr.py"), "read", clip, "--glyphs", os.path.join(HERE, "glyphs_fim2026"),
                        "--start", "120", "--end", "340", "--fps", "10", "--out", ov_path], check=True)
    rows = list(csv.DictReader(open(ov_path)))
    auto_start, tele_start = periods(rows)
    if auto_start is None or tele_start is None: sys.exit(f"could not read the period starts from the overlay clock ({auto_start}, {tele_start})")

    # 2. physical tips from the frames
    tips_path = os.path.join(vdir, f"{a.label}_tips.json")
    if not os.path.exists(tips_path):
        m, crops = hive_tips.motion(clip, auto_start - 5, tele_start + 128, cfg["hive_rois"])
        tips = hive_tips.find_tips(m, crops, list(cfg["hive_rois"]), auto_start, tele_start + 125)
        json.dump({"clip": clip, "match_start": auto_start, "tips": tips, "held_frame_share": hive_tips.held_share(m, auto_start, tele_start + 125)},
                  open(tips_path, "w"), indent=1)
    tj = json.load(open(tips_path)); phys = tj["tips"]

    # 3. log <-> video clock
    c = cfg["field_corners_px"]; F = cfg["field_half_in"]
    Hm = cv2.getPerspectiveTransform(np.float32([[F, -F], [F, F], [-F, -F], [-F, F]]),
                                     np.float32([c["near_red"], c["near_blue"], c["far_red"], c["far_blue"]]))
    al_path = os.path.join(vdir, f"{a.label}_align.json")
    al = json.load(open(al_path)) if os.path.exists(al_path) else {}
    for kind, pstart, dur, search in (("teleop", tele_start, 120.0, (-2.0, 4.0)), ("auto", auto_start, 30.0, (-2.0, 12.0))):
        if kind in logs and kind not in al:
            d, r, started, scan = align_log.align(logs[kind], clip, Hm, pstart, dur, 10.0, search)
            al[kind] = {"period_start_clip": pstart, "run_started_log_t": started, "delta_s": d, "r": r, "fitted_delta_s": d}
    for kind, x in al.items():
        # a weak fit (the robot small, far or hidden in the picture) is not trusted: use 0, which every good fit is near
        # unless the drive team started late (Q5 AUTO +6.2 s at r 0.84)
        x.setdefault("fitted_delta_s", x["delta_s"])   # caches written before this rule hold the fit in delta_s
        x["weak"] = not (x["r"] == x["r"] and x["r"] >= WEAK_FIT_R)
        x["delta_s"] = 0.0 if x["weak"] else x["fitted_delta_s"]
    json.dump(al, open(al_path, "w"), indent=1)

    def to_clip(kind, t):
        x = al[kind]; return round(x["period_start_clip"] + (t - x["run_started_log_t"]) + x["delta_s"], 2)

    # 4. our robot from the logs
    out = {"event": a.event, "label": a.label, "team": a.team, "alliance": alliance, "partners": partners, "opponents": opponents,
           "clip": clip, "auto_start_clip": auto_start, "teleop_start_clip": tele_start, "alignment": al, "salvos": [], "marks": [],
           "cycles": [], "pose": []}
    out_note_held = None
    for kind in ("auto", "teleop"):
        if kind not in logs: continue
        start, salvos, marks, ev = log_events(logs[kind])
        for s in salvos:
            out["salvos"].append({"period": kind, "start": to_clip(kind, s["start"]), "done": to_clip(kind, s["done"]) if s["done"] else None,
                                  "held": s["held"], "stack": s["stack"], "target": s["target"]})
        for mk in marks:
            out["marks"].append({"period": kind, "t": to_clip(kind, mk["t"]), "kind": mk["kind"], "text": mk["text"]})
        for st, x, y in CT.pose_samples(logs[kind])[::5]:
            out["pose"].append([to_clip(kind, st), round(x, 1), round(y, 1)])
        if kind == "teleop":
            for cyc in teleop_cycles(logs[kind]):
                c = {k: (to_clip(kind, v) if k in ("start", "done", "reach", "arrive", "lock_on", "next") else v) for k, v in cyc.items()}
                c["pickups"] = [[to_clip(kind, t), jump, sp] for t, jump, sp in cyc["pickups"]]
                out["cycles"].append(c)

    # 5. tips: physical (frames), reconciled per alliance and period against the official FiM count;
    #    the scoreboard only pairs (for its lag) and fills a tip the frames missed
    fs = fm["scores"]
    out["fim"] = {al_: {"auto_tips": fs[al_]["autoHiveTips"], "teleop_tips": fs[al_]["dcHiveTips"], "total": fs[al_]["totalPoints"]} for al_ in ("red", "blue")}
    reg = {al_: overlay_tip_changes(rows, f"{al_}_tips") for al_ in ("red", "blue")}
    period_of = lambda t: "auto" if t < tele_start else "teleop"   # a tip finished in the 8 s transition counts for AUTO
    out["tips"], out["tip_notes"] = [], []
    dropped_tips = []
    for al_ in ("red", "blue"):
        frames_ = sorted((dict(t) for t in phys if t["hive"] == f"{al_}_hive"), key=lambda t: t["t"])
        rises = list(reg[al_])
        for t in frames_:   # pair in order with the first unused scoreboard rise up to 10 s later (Q31 showed ~7.5 s)
            cand = [x for x in rises if -1.0 <= x[0] - t["t"] <= PAIR_MAX_S]
            sb = min(cand, key=lambda x: x[0]) if cand else None
            if sb: rises.remove(sb)
            t["scoreboard_t"] = sb[0] if sb else None
            t["source"] = "frames"
        kept = []
        for per in ("auto", "teleop"):
            want = out["fim"][al_][f"{per}_tips"]
            fr = [t for t in frames_ if period_of(t["t"]) == per]
            while len(fr) > want:   # too many: drop the weakest unpaired one (no scoreboard rise near it)
                pool = [t for t in fr if t["scoreboard_t"] is None] or fr
                worst = min(pool, key=lambda t: t["peak"] * t["change"])
                fr.remove(worst)
                dropped_tips.append((al_, worst, len(out["tip_notes"])))
                out["tip_notes"].append(f"{al_} {per}: dropped a frame tip at {worst['t']:.1f} s (FiM has {want}; no scoreboard rise near it)" if worst["scoreboard_t"] is None
                                        else f"{al_} {per}: dropped a frame tip at {worst['t']:.1f} s (FiM has {want}; weakest)")
            spare = [x for x in rises if period_of(x[0] - SB_LAG_GUESS) == per]
            while len(fr) < want and spare:   # too few: a scoreboard rise no frame tip explains fills it, flagged
                x = spare.pop(0); rises.remove(x)
                fr.append({"t": round(x[0] - SB_LAG_GUESS, 2), "scoreboard_t": x[0], "source": "scoreboard only"})
                out["tip_notes"].append(f"{al_} {per}: tip at about {x[0] - SB_LAG_GUESS:.1f} s taken from the scoreboard (frames missed it)")
            if len(fr) < want:
                out["tip_notes"].append(f"{al_} {per}: {want - len(fr)} FiM tip(s) found in neither the frames nor the scoreboard; the up cell after it is uncertain")
            kept += fr
        kept.sort(key=lambda z: z["t"])
        for i, z in enumerate(kept):
            out["tips"].append({"alliance": al_, "n": i + 1, "t": z["t"], "period": period_of(z["t"]), "source": z["source"],
                                "start": z.get("start"), "end": z.get("end"), "duration_s": z.get("duration_s"),
                                "duration_note": z.get("duration_note"), "scoreboard_t": z["scoreboard_t"],
                                "scoreboard_lag": round(z["scoreboard_t"] - z["t"], 2) if z["scoreboard_t"] is not None and z["source"] == "frames" else None})

    # 6. our alliance: up cell, salvos per tip, credit
    cell = START_CELL[alliance]

    def wrong_count(tips_):
        n = 0
        for sv in out["salvos"]:
            up = cell
            for _ in range(sum(1 for t in tips_ if t["t"] <= sv["start"])): up = OTHER_CELL[up]
            if sv["target"] and sv["target"] != up: n += 1
        return n

    # A frame tip FiM does not count may still have happened: if our robot's aim (log, the most trusted source) only makes
    # sense with it, keep it (Hartland Q24: a red tip at 262.8 s is plain in the frames, missing on the scoreboard and in FiM,
    # and without it 9 of our salvos would have been aimed at the down cell).
    ours = sorted((t for t in out["tips"] if t["alliance"] == alliance), key=lambda t: t["t"])
    for al_, d, note_i in dropped_tips:
        if al_ != alliance: continue
        base = wrong_count(ours)
        if base == 0: break
        cand = {"alliance": al_, "n": 0, "t": d["t"], "period": period_of(d["t"]), "source": "frames, not counted by FiM",
                "start": d.get("start"), "end": d.get("end"), "duration_s": d.get("duration_s"), "duration_note": d.get("duration_note"),
                "scoreboard_t": None, "scoreboard_lag": None}
        trial = sorted(ours + [cand], key=lambda t: t["t"])
        if wrong_count(trial) < base:
            out["tips"].append(cand); ours = trial
            out["tip_notes"][note_i] = (f"{al_} {cand['period']}: kept a frame tip at {d['t']:.1f} s that FiM and the scoreboard do not count: "
                                        f"our robot's aim matches the up cell only with it ({base} salvos at the down cell without it)")
    for i, t in enumerate(ours): t["n"] = i + 1
    out["tips"].sort(key=lambda t: (t["alliance"], t["t"]))
    bounds = [auto_start - 1] + [t["t"] for t in ours] + [tele_start + 125]
    for s in out["salvos"]:
        k = sum(1 for t in ours if t["t"] <= s["start"])   # tips before this salvo
        up = cell
        for _ in range(k): up = OTHER_CELL[up]
        s["up_cell"] = up
        s["right_cell"] = (s["target"] == up) if s["target"] else None
        s["tip_index"] = k + 1                               # the tip this salvo is working toward
    for i, t in enumerate(ours):
        mine_s = [s for s in out["salvos"] if s["tip_index"] == i + 1]
        t["our_salvos"] = len(mine_s)
        t["our_elements"] = sum(s["held"] or 0 for s in mine_s)
        last = max((s for s in mine_s if s["done"]), key=lambda s: s["done"], default=None)
        if last:
            lag = round(t["t"] - last["done"], 2)
            t["after_our_salvo_s"] = lag
            t["credit"] = "8533" if TIP_WINDOW_AFTER_SALVO[0] <= lag <= TIP_WINDOW_AFTER_SALVO[1] or last["start"] <= t["t"] <= last["done"] else "partner?"
        else:
            t["credit"] = "partner?"
        t["since_previous_tip_s"] = round(t["t"] - bounds[i], 2) if i > 0 else None

    # 7. pollen and nectar landing in our up cell (frames): per salvo, and landings no salvo of ours explains (partner volleys).
    #    Nectar = our alliance's colour only (opponents' nectar is not ours to control, G408).
    land_path = os.path.join(vdir, f"{a.label}_landings.json")
    lj = json.load(open(land_path)) if os.path.exists(land_path) else {}
    hx0, hy0, hx1, hy1 = cfg["hive_rois"][f"{alliance}_hive"]
    # ball size scales with the hive's size in the picture; pollen_landings is calibrated on Hartland Q5's framing, where a
    # hive box was 185 px wide (a view's own "zoom" is relative to its field's reference, which for field 2 is not Q5)
    zoom = (hx1 - hx0) / LANDING_REF_HIVE_W_PX
    box = [hx0, max(262, hy0 - 20), hx1, int(hy0 + 0.6 * (hy1 - hy0))]   # the up cell sits in the top of the hive box
    if "bursts" not in lj:
        fl, bu = pollen_landings.landings(clip, box, zoom, auto_start, tele_start + 122, "pollen")
        lj.update({"box": box, "zoom": zoom, "flights": fl, "bursts": bu})
    if "nectar_bursts" not in lj:
        fl, bu = pollen_landings.landings(clip, box, zoom, auto_start, tele_start + 122, alliance)
        lj.update({"nectar_colour": alliance, "nectar_flights": fl, "nectar_bursts": bu})
    json.dump(lj, open(land_path, "w"), indent=1)
    bursts = [dict(b, kind="pollen") for b in lj["bursts"]] + [dict(b, kind="nectar") for b in lj["nectar_bursts"]]
    frame_share = sum(1 for t in ours if t["source"].startswith("frames")) / max(1, len(ours))
    out["landing_video_ok"] = frame_share >= LANDING_MIN_FRAME_TIPS and (tj.get("held_frame_share") or 0) < 0.005
    if not out["landing_video_ok"]:
        bursts = []   # the picture cannot show balls arriving (Hartland M3: hive tops under the score bar; M7: the stream held frames)
        out["tip_notes"].append("landings not read: the video does not show the hive well enough in this match")
    tip_ts = [t["t"] for t in ours]

    def mix(bs):
        k = {b["kind"] for b in bs}
        return "mix" if len(k) == 2 else (k.pop() if k else None)

    for sv in out["salvos"]:
        if sv["done"] is None: continue
        bs = [b for b in bursts if sv["start"] <= b["start"] <= sv["done"] + LAND_AFTER_DONE_S]
        sv["landing_seen"] = bool(bs) if out["landing_video_ok"] else None
        sv["landing_delay_s"] = round(min(b["start"] for b in bs) - sv["start"], 2) if bs else None
        sv["landing_kind"] = mix(bs) if out["landing_video_ok"] else None   # pollen / nectar / mix (what was SEEN arriving)

    def explained(b):
        if any(TIP_SETTLE[0] <= b["start"] - t <= TIP_SETTLE[1] for t in tip_ts): return "tip"
        if any(sv["start"] <= b["start"] <= sv["done"] + OURS_SETTLE_S for sv in out["salvos"] if sv["done"]): return "ours"
        return None

    # other landings: pollen and nectar bursts no salvo or tip explains, merged when they overlap (one volley of both = a mix)
    other = sorted((b for b in bursts if explained(b) is None), key=lambda b: b["start"])
    groups = []
    for b in other:
        if groups and b["start"] - groups[-1][-1]["end"] <= MIX_MERGE_S: groups[-1].append(b)
        else: groups.append([b])
    out["other_landings"] = [{"start": min(b["start"] for b in g), "end": max(b["end"] for b in g), "flights": sum(b["flights"] for b in g),
                              "kind": mix(g)} for g in groups]
    for t in ours:
        seen = [b for b in out["other_landings"] if 0 <= t["t"] - b["start"] <= PARTNER_BEFORE_TIP_S]
        t["partner_landing_before_s"] = round(t["t"] - max(b["start"] for b in seen), 2) if seen else None
        if t["credit"] == "partner?" and seen: t["credit"] = "partner (seen)"
        elif t["credit"] == "8533" and seen: t["credit"] = "8533 + partner"
    # 8. how many pollen rest in our up cell over time (frames), and our nectar from the rules and the landings
    cc_path = os.path.join(vdir, f"{a.label}_cellcount.json")
    if out["landing_video_ok"]:
        if not os.path.exists(cc_path):
            skip = [((t.get("start") or t["t"]) - 0.5, (t.get("end") or t["t"]) + 1.0) for t in ours]
            ser = cell_count.series(clip, cfg["hive_rois"][f"{alliance}_hive"], zoom, auto_start, tele_start + 125, 2.0, skip)
            json.dump({"series": ser}, open(cc_path, "w"))
        ser = json.load(open(cc_path))["series"]
        out["cell_pollen"] = ser
        for t in ours:   # the count just before the swing started: what the cell held when it tipped (an "at least")
            t0 = (t.get("start") or t["t"]) - 0.5
            before = [v for tt, v in ser if t0 - 1.5 <= tt <= t0]
            t["pollen_before_tip"] = max(before) if before else None
    else:
        out["cell_pollen"] = None
    first_tip = min((t["t"] for t in ours), default=None)
    out["cell_nectar"] = {"preloaded": 3, "until": first_tip,
                          "seen_landing": sum(1 for b in out["other_landings"] if b.get("kind") in ("nectar", "mix"))
                                          + sum(1 for sv in out["salvos"] if sv.get("landing_kind") in ("nectar", "mix"))}
    out["held_frame_share"] = tj.get("held_frame_share")
    path = os.path.join(vdir, f"{a.label}_fused.json")
    json.dump(out, open(path, "w"), indent=1)

    # summary
    print(f"{a.label}: {alliance} with {partners} vs {opponents}; auto at {auto_start:.1f}, teleop at {tele_start:.1f} (clip s); stream held frames {out['held_frame_share']}")
    for k, v in al.items(): print(f"  clock {k}: delta {v['delta_s']:+.1f} s (fit {v['fitted_delta_s']:+.1f} s, r {v['r']:.2f}{', WEAK: using 0' if v['weak'] else ''})")
    for al_ in ("red", "blue"):
        n_a = sum(1 for t in out["tips"] if t["alliance"] == al_ and t["period"] == "auto"); n_t = sum(1 for t in out["tips"] if t["alliance"] == al_ and t["period"] == "teleop")
        f = out["fim"][al_]
        n_sb = sum(1 for t in out["tips"] if t["alliance"] == al_ and t["source"] != "frames")
        print(f"  {al_} tips (from scoreboard {n_sb}): auto {n_a} teleop {n_t}  | FiM: auto {f['auto_tips']} teleop {f['teleop_tips']}  {'OK' if (n_a, n_t) == (f['auto_tips'], f['teleop_tips']) else 'MISMATCH'}")
    for n_ in out["tip_notes"]: print("  NOTE", n_)
    ls = [sv for sv in out["salvos"] if sv.get("landing_seen") is not None]
    if not out["landing_video_ok"]: print("  landings: not read (video not good enough)")
    from collections import Counter
    print(f"  landings seen for {sum(sv['landing_seen'] for sv in ls)}/{len(ls)} salvos {dict(Counter(sv.get('landing_kind') for sv in ls if sv['landing_seen']))}; "
          "other (partner?) landings: " + " ".join(f"{b['start']:.1f}({b['kind']})" for b in out["other_landings"]))
    wrong = [s for s in out["salvos"] if s["right_cell"] is False]
    print(f"  our salvos {len(out['salvos'])}, at the wrong (down) cell: {len(wrong)}")
    for t in ours:
        print(f"  tip {t['n']} {t['period']:6} at {t['t']:6.1f} [{t['source']}] took {t['duration_s']} s  scoreboard +{t['scoreboard_lag']}  our salvos {t['our_salvos']} ({t['our_elements']} el)  "
              f"after our salvo {t.get('after_our_salvo_s')} s  credit {t['credit']}")
    print(f"wrote {path}")


def teleop_cycles(path, radius=6.0):
    """The cycle split of tools/logs/cycle_times.py, kept as times (log clock) instead of durations. arrive = arriving
    within radius of the spot the next salvo was fired from (never before reach); find_in = straight-line inches from
    reach to that spot. travel_s / lineup_s / stopped_s / pickup_s = collect (done to reach) split by CT.collect_split, and
    pickups = [[t, count jump, speed in/s]] of the count rises in it."""
    start, end, salvos, locks, full_on, full_off, elements = CT.teleop(path)
    samples = CT.pose_samples(path)
    drive = CT.drive_samples(path)
    out = []
    for i in range(len(salvos) - 1):
        s, n = salvos[i], salvos[i + 1]
        if s["done"] is None: continue
        lk = [t for t in locks if s["done"] <= t <= n["start"]]
        if not lk: continue
        lock_on = lk[-1]
        reach, final = CT.reach_time(elements, s["done"], lock_on)
        arr = CT.arrival_time(samples, s["done"], n["start"], radius)
        arrive = max(lock_on if arr is None else arr, reach)
        a, b = CT.pose_at(samples, reach), CT.pose_at(samples, n["start"])
        find_in = round(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5, 1) if a and b else None
        part, picks = CT.collect_split(drive, elements, s["done"], reach)
        out.append({"start": s["start"], "done": s["done"], "reach": reach, "arrive": arrive, "lock_on": lock_on, "next": n["start"], "find_in": find_in,
                    "held_next": n["held"], "full_s": round(CT.full_time_between(s["done"], lock_on, full_on, full_off), 2),
                    "travel_s": round(part["travel"], 2), "lineup_s": round(part["lineup"], 2), "stopped_s": round(part["stopped"], 2),
                    "pickup_s": round(part["pickup"], 2), "pickups": picks})
    return out


if __name__ == "__main__":
    main()
