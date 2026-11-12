"""Combine every <label>_fused.json of an event into the CSV files the cycle-analysis dashboard reads.

    python tools/video/export_dash.py --event "Hartland Qualifier 2026"

Writes robot-logs/events/<event>/video/dash/{salvos,tips,cycles,tip_counts,alignment,matches,partner_landings,hive_pollen}.csv.
All times are match time: seconds from the start of AUTO (TeleOp starts at 38 s).
"""
import argparse, csv, glob, json, os, re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
# as in fuse_match.py (not imported: it pulls in OpenCV): each hive's up cell at match start, and the cell that comes up when it tips
START_CELL = {"blue": "BLUE_REAR", "red": "RED_AUDIENCE"}
OTHER_CELL = {"BLUE_REAR": "BLUE_AUDIENCE", "BLUE_AUDIENCE": "BLUE_REAR", "RED_REAR": "RED_AUDIENCE", "RED_AUDIENCE": "RED_REAR"}


def order(label):
    m = re.match(r"([A-Za-z]+)(\d+)", label)
    return (0 if m.group(1).upper() == "Q" else 1, int(m.group(2)))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--event", required=True); a = ap.parse_args()
    vdir = os.path.join(ROOT, "robot-logs", "events", a.event, "video")
    views = json.load(open(os.path.join(vdir, "video_config.json"))).get("views", {})
    fused = sorted((json.load(open(p)) for p in glob.glob(os.path.join(vdir, "*_fused.json"))), key=lambda d: order(d["label"]))
    out = {k: [] for k in ("salvos", "tips", "cycles", "tip_counts", "alignment", "matches", "partner_landings", "hive_pollen")}
    for d in fused:
        L, a0 = d["label"], d["auto_start_clip"]
        mt = lambda x: "" if x is None else round(x - a0, 2)
        sal = sorted(d["salvos"], key=lambda s: s["start"])
        for i, s in enumerate(sal):
            out["salvos"].append([L, i + 1, s["period"], mt(s["start"]), mt(s["done"]), s["held"], s["target"], s["up_cell"], s["right_cell"], s["tip_index"], s["start"],
                                  "" if s.get("landing_seen") is None else s["landing_seen"], "" if s.get("landing_delay_s") is None else s["landing_delay_s"], s.get("landing_kind") or ""])
        for ts, v in d.get("cell_pollen") or []:   # cell_count: pollen seen resting in our up cell, 2 a second (an "at least"); none where the video is too poor
            out["hive_pollen"].append([L, mt(ts), v])
        for b in d.get("other_landings", []):
            out["partner_landings"].append([L, mt(b["start"]), mt(b["end"]), b["flights"], round(b["start"], 2), b.get("kind", "pollen")])
        up = dict(START_CELL)   # a tip dumps its hive's up cell, then the other cell is up (the same walk fuse_match uses for salvos)
        for t in sorted(d["tips"], key=lambda t: t["t"]):
            cell = up[t["alliance"]]; up[t["alliance"]] = OTHER_CELL[cell]
            out["tips"].append([L, t["alliance"], t["n"], t["period"], t.get("source", "frames"), mt(t["t"]), mt(t.get("start")), mt(t.get("end")),
                                "" if t.get("duration_s") is None else t["duration_s"], t.get("duration_note") or "", mt(t["scoreboard_t"]), "" if t["scoreboard_lag"] is None else t["scoreboard_lag"],
                                t.get("our_salvos", ""), t.get("our_elements", ""), t.get("after_our_salvo_s", ""), t.get("credit", ""), t["t"], cell])
        for i, c in enumerate(d["cycles"]):
            # a cycle on the dashboard runs from the end of one salvo to the end of the next, in the order the driver lives it:
            # collect, find spot, at spot, aim, salvo (the salvo that ENDS the cycle, so salvo_s and held are that salvo's).
            # The parts are the split of tools/logs/cycle_times.py: find spot = reach to arrive (lock drive = the part after the
            # lock came on), at spot = arrived and not yet locked, aim = the later of arrive and lock on, to the salvo starting
            arr = c.get("arrive", c["lock_on"])   # fused files written before the arrival was kept: the old transit / lock split
            es = next((s for s in sal if abs(s["start"] - c["next"]) < 0.02 and s["done"] is not None), None)
            if es is None:
                continue   # the ending salvo never finished (cut off at the match end): no whole cycle
            end = es["done"]
            out["cycles"].append([L, i + 1, mt(c["done"]), round(c["reach"] - c["done"], 2),
                                  round(arr - c["reach"], 2), round(max(0.0, arr - c["lock_on"]), 2), round(max(0.0, c["lock_on"] - arr), 2),
                                  round(c["next"] - max(arr, c["lock_on"]), 2), round(end - c["next"], 2), round(c["next"] - c["lock_on"], 2), c.get("find_in", ""),
                                  round(end - c["done"], 2), c["held_next"], c["full_s"],
                                  es["target"] or "", es["up_cell"] or "", "" if es["right_cell"] is None else es["right_cell"]])
        for al in ("red", "blue"):
            fr = [t for t in d["tips"] if t["alliance"] == al and t.get("source", "frames").startswith("frames")]
            na = sum(1 for t in fr if t["period"] == "auto")
            nt = sum(1 for t in fr if t["period"] == "teleop")
            f = d["fim"][al]
            out["tip_counts"].append([L, al, na, nt, f["auto_tips"], f["teleop_tips"], f["total"]])
        for k, v in d["alignment"].items():
            out["alignment"].append([L, k, v["delta_s"], round(v["r"], 3), v.get("fitted_delta_s", v["delta_s"]), v.get("weak", False), v["period_start_clip"]])
        out["matches"].append([L, d["alliance"], "+".join(map(str, d["partners"])), "+".join(map(str, d["opponents"])),
                               d["fim"][d["alliance"]]["total"], d["fim"]["red" if d["alliance"] == "blue" else "blue"]["total"],
                               round(a0, 2), os.path.basename(d["clip"]), views.get(L, {}).get("field", ""), views.get(L, {}).get("camera_moved", False), d.get("held_frame_share", ""),
                               " | ".join(d.get("tip_notes", []))])
    cols = {
        "salvos": ["match", "salvo", "period", "start_s", "end_s", "held", "target_cell", "up_cell", "right_cell", "for_tip", "clip_s", "landing_seen", "landing_delay_s", "landing_kind"],
        "partner_landings": ["match", "start_s", "end_s", "flights", "clip_s", "kind"],
        "hive_pollen": ["match", "t_s", "pollen_seen"],
        "tips": ["match", "alliance", "tip", "period", "source", "frame_s", "tip_start_s", "tip_end_s", "tip_duration_s", "duration_note", "scoreboard_s", "scoreboard_lag_s", "our_salvos", "our_elements", "after_our_salvo_s", "credit", "clip_s", "cell"],
        "cycles": ["match", "cycle", "start_s", "collect_s", "find_s", "lock_drive_s", "at_spot_s", "aim_s", "salvo_s", "lock_s", "find_in", "cycle_s", "held", "full_wait_s", "target_cell", "up_cell", "right_cell"],
        "tip_counts": ["match", "alliance", "frame_auto_tips", "frame_teleop_tips", "fim_auto_tips", "fim_teleop_tips", "fim_total"],
        "alignment": ["match", "period", "offset_s", "fit_r", "fitted_offset_s", "weak_fit", "period_start_clip_s"],
        "matches": ["match", "alliance", "partner", "opponents", "our_score", "their_score", "auto_start_clip_s", "clip", "field", "camera_moved", "held_frame_share", "tip_notes"],
    }
    os.makedirs(os.path.join(vdir, "dash"), exist_ok=True)
    for k, rows in out.items():
        with open(os.path.join(vdir, "dash", f"{k}.csv"), "w", newline="") as f:
            w = csv.writer(f); w.writerow(cols[k]); w.writerows(rows)
        print(f"{k}.csv: {len(rows)} rows")


if __name__ == "__main__":
    main()
