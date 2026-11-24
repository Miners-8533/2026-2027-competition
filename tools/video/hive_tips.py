"""Find the physical HIVE TIPs in a match clip from the video frames (not the scoreboard, which lags ~1.4 s).

    python tools/video/hive_tips.py CLIP.mp4 --config video_config.json --start 140 --end 320 --out tips.json

A tip is the whole HIVE swinging over (~1 s of large motion inside one hive's box). Per frame (10 fps, half
resolution) the share of changed pixels is measured in each hive box. A motion peak counts as a tip when the hive box
looks different 2 s after the peak than 2 s before (the hive came to rest the other way up) while the rest of the
picture does not: a broadcast dissolve, replay or camera cut changes the whole picture, a person walking past the
hive changes neither for long. Peaks of one hive closer than MIN_GAP_S merge.
Tuned on Hartland 2026 Q5 (6 of 6 tips) and M10 (21 of 21 tips, red and blue, including two at the same moment).
Each filmed tip also gets its swing's start, end and duration (motion above its baseline + 15% of its rise). held_frame_share in the
output is how much of the match the stream held frames instead of moving (a stalled stream hides tips).
Check new events with contact sheets.
"""
import argparse, json, os, subprocess
import numpy as np
import cv2

FF = os.path.join(os.environ.get("LOCALAPPDATA", ""), r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
SC = 0.5
FPS = 10
PEAK_MIN = 0.20        # smoothed share of changed pixels in the hive box (a Q10 auto tip peaked at 0.299)
STATE_CHANGE_MIN = 20.0 # mean grey-level difference of the hive box 2 s before vs 2 s after: a tip leaves it changed (real tips 25-45)
HIVE_OVER_SCENE = 2.0  # ... and the hive box must change this many times more than the rest of the picture
SCENE_CHANGE_MAX = 20.0  # ... and the rest of the picture must stay put (Q5/M10 tips: 7-14; a dissolve into a replay: 31)
STATE_GAP_S = 2.0
MIN_GAP_S = 3.0
EDGE_FRAC = 0.15       # the swing is where the box's motion is above its baseline plus this share of the way to its peak
                       # (0.3 missed the slow first tilt: M10 237.8 s measured 0.7 s, ~1.6 s by eye)
STILL_SHARE = 0.003    # below this share of changed pixels in the whole picture, the frame is a held (repeated) frame


def motion(clip, start, end, rois):
    w, h = int(1920 * SC), int(1080 * SC)
    cmd = [FF if os.path.exists(FF) else "ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-i", clip, "-t", f"{end - start:.2f}",
           "-vf", f"fps={FPS},scale={w}:{h},format=gray", "-f", "rawvideo", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    prev, out, n, crops = None, [], 0, {k: [] for k in rois}
    mask = np.ones((54, 96), bool)
    for x0, y0, x1, y1 in rois.values():
        mask[int(y0 / 20):int(y1 / 20) + 1, int(x0 / 20):int(x1 / 20) + 1] = False
    mask[:int(260 / 20) + 1] = False   # the score bar
    crops["_scene"] = []
    while True:
        b = p.stdout.read(w * h)
        if len(b) < w * h: break
        g = cv2.GaussianBlur(np.frombuffer(b, np.uint8).reshape(h, w), (5, 5), 0).astype(np.int16)
        for k, (x0, y0, x1, y1) in rois.items():
            crops[k].append(cv2.resize(g[int(y0 * SC):int(y1 * SC), int(x0 * SC):int(x1 * SC)].astype(np.uint8), (48, 60)).astype(np.int16))
        crops["_scene"].append(cv2.resize(g.astype(np.uint8), (96, 54), interpolation=cv2.INTER_AREA).astype(np.int16)[mask])
        if prev is not None:
            d = np.abs(g - prev) > 12
            row = [start + n / FPS, float(d[int(260 * SC):].mean())]   # below the score bar
            for x0, y0, x1, y1 in rois.values():
                row.append(float(d[int(y0 * SC):int(y1 * SC), int(x0 * SC):int(x1 * SC)].mean()))
            out.append(row)
        prev, n = g, n + 1
    p.wait()
    return np.array(out), {k: v[1:] for k, v in crops.items()}   # crops[k][i] is the frame of out[i]


def _edges(raw, j):
    """Start and end index of the swing around peak j, on the box's motion smoothed over 0.3 s. Motion in the box before a
    tip is not zero (people behind the clear hive, the last balls arriving), so the threshold is the local baseline (median
    of the 1 s ending 2 s before the peak) plus EDGE_FRAC of the way to the swing's peak; the swing lasts while motion stays
    above it (3 frames below end it; motion dips mid-swing, which cut Hartland M10's 237.8 s tip to 0.6 s instead of ~1.6 s).
    Checked by eye: Q5 tip 2 rotates from ~191.1 s, rests at 192.4 s; Q5 tip 5 ~276.9 to 278.5 s; M10 237.8 s tip ~236.5 to 238.1 s."""
    sm = np.convolve(raw, np.ones(3) / 3, "same")
    w = sm[max(0, j - 30):max(1, j - 20)]
    base = float(np.median(w)) if len(w) else 0.0
    top = float(sm[max(0, j - 3):j + 4].max())
    th = base + EDGE_FRAC * max(0.0, top - base)
    below = sm < th
    i0 = j
    while i0 > 2 and not (below[i0 - 1] and below[i0 - 2] and below[i0 - 3]): i0 -= 1
    i1 = j
    while i1 < len(sm) - 3 and not (below[i1 + 1] and below[i1 + 2] and below[i1 + 3]): i1 += 1
    return i0, i1


def find_tips(m, crops, names, t0=-1e9, t1=1e9):
    """Tips of each hive whose swing was filmed, with its start, end and duration. Where the stream held frames (Hartland M7:
    the picture stood still for seconds while the overlay ran) a tip is not filmed and is not found here; looking for the
    hive's state jumping between two pictures instead was tried and gave many false tips (balls piling into a cell, people
    behind the clear hive), so fuse_match fills such tips from the scoreboard, flagged."""
    k = np.ones(5) / 5
    gap = int(STATE_GAP_S * FPS)
    still = m[:, 1] < STILL_SHARE          # the whole picture did not change from the previous frame: a held frame
    tips = []
    for i, name in enumerate(n for n in names if not n.startswith('_')):
        raw = m[:, 2 + i]
        s = np.convolve(raw, k, "same")

        def state(j):
            change = float(np.abs(crops[name][j + gap] - crops[name][j - gap]).mean())
            scene = float(np.abs(crops["_scene"][j + gap] - crops["_scene"][j - gap]).mean())
            return change, scene, change / max(scene, 0.5)

        def ok(change, scene, ratio):
            return change >= STATE_CHANGE_MIN and ratio >= HIVE_OVER_SCENE and scene <= SCENE_CHANGE_MAX

        kept = []
        for j in [j for j in range(2, len(s) - 2) if s[j] == s[j - 2:j + 3].max() and s[j] >= PEAK_MIN]:
            if not (t0 <= m[j, 0] <= t1) or j < gap or j + gap >= len(s): continue
            change, scene, ratio = state(j)
            if not ok(change, scene, ratio): continue
            i0, i1 = _edges(raw, j)
            held = float(still[i0:i1 + 1].mean()) if i1 >= i0 else 0.0
            tip = {"t": round(float(m[j, 0]), 2), "kind": "motion", "peak": round(float(s[j]), 3), "change": round(change, 1),
                   "scene_change": round(scene, 1), "ratio": round(float(ratio), 1),
                   "start": round(float(m[i0, 0]), 2), "end": round(float(m[i1, 0]), 2), "duration_s": round(float(m[i1, 0] - m[i0, 0]), 2),
                   "held_frames_share": round(held, 2), "time_uncertainty_s": round(1.0 / FPS, 2)}
            if held > 0.3: tip["duration_note"] = "the stream held frames during the swing; the duration is uncertain"
            if kept and m[j, 0] - kept[-1]["t"] < MIN_GAP_S:
                if s[j] > kept[-1]["peak"]: kept[-1] = tip
                continue
            kept.append(tip)
        kept.sort(key=lambda x: x["t"])
        for x in kept: x["hive"] = name
        tips += kept
    return sorted(tips, key=lambda x: x["t"])


def held_share(m, t0, t1):
    """Share of the match's frames (10 fps) where the whole picture did not change: the stream held a frame."""
    sel = (m[:, 0] >= t0) & (m[:, 0] <= t1)
    return round(float((m[sel, 1] < STILL_SHARE).mean()), 3) if sel.any() else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clip"); ap.add_argument("--config", required=True)
    ap.add_argument("--start", type=float, required=True); ap.add_argument("--end", type=float, required=True)
    ap.add_argument("--match-start", type=float, help="clip time the auto starts; tips outside the match (+5 s) are dropped")
    ap.add_argument("--out")
    a = ap.parse_args()
    rois = json.load(open(a.config))["hive_rois"]
    m, crops = motion(a.clip, a.start, a.end, rois)
    t0, t1 = (a.match_start, a.match_start + 158 + 5) if a.match_start is not None else (-1e9, 1e9)
    tips = find_tips(m, crops, list(rois), t0, t1)
    for t in tips: print(f"{t['t']:7.2f}  {t['hive']:10} {t['kind']:6} peak {t['peak']:.2f}  hive change {t['change']}  scene change {t['scene_change']}  "
                         f"duration {t['duration_s']}  +-{t['time_uncertainty_s']}")
    if a.out:
        json.dump({"clip": a.clip, "match_start": a.match_start, "tips": tips, "held_frame_share": held_share(m, t0, t1),
                   "params": {"PEAK_MIN": PEAK_MIN, "STATE_CHANGE_MIN": STATE_CHANGE_MIN, "HIVE_OVER_SCENE": HIVE_OVER_SCENE,
                              "SCENE_CHANGE_MAX": SCENE_CHANGE_MAX}}, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
