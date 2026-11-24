"""Pollen landing in a hive's up cell, from the video frames: when balls arrive, not how many stay.

    python tools/video/pollen_landings.py CLIP.mp4 --box x0 y0 x1 y1 --zoom 1.0 --start 140 --end 320 [--out landings.json]

Plain motion in the cell's area also picks up people standing behind the clear hive (Hartland Q5 236-239 s: a field staff
member in a yellow shirt), and the shirts are the same yellow as pollen (hue ~20, saturation ~125, value ~225), so colour
alone cannot tell them apart. Shape can: a pollen ball is a compact round blob of a known size, a shirt is large and ragged.

Per frame (FPS, full resolution, only the box): yellow mask -> blobs; a blob counts as a ball when its size fits a pollen
ball at this match's zoom, it is round enough (fill, aspect with motion blur allowed, circularity), and it is isolated:
the thin band around it is not yellow. That last test is what rejects shirts, which the hive's frame bars and the
shirts' own dark panels chop into ball-sized pieces that continue just past the bar. A ball is
MOVING when no ball was seen within R px of it in the last MEMORY_S (resting balls and flickering shirt pieces are ignored). Moving-ball detections
closer than MERGE_S in time form one landing burst: {start, end, moving_balls}.

NECTAR (kind "red" or "blue"): the same tracking on red / blue blobs of a nectar's size (3.6 in, 1.65x a pollen's area).
Nectar is the colour of its own hive's frame bars, so a colour mask alone merges them; the frame does not move between
tips, so for nectar only colour that is NEW counts: each pixel's colour history is kept (BG_TAU_S), and a pixel is
foreground when it is nectar-coloured now but was not for most of the last moments. Tips (the frame swings) are left to
the caller to exclude.
"""
import argparse, json, os, subprocess
import numpy as np
import cv2

FF = os.path.join(os.environ.get("LOCALAPPDATA", ""), r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
FPS = 30
HSV_LO, HSV_HI = (12, 80, 120), (40, 255, 255)
BALL_AREA = (90, 900)     # px at zoom 1.0 (a resting pollen ball in the audience cell is ~22 px across, ~350 px)
# nectar colours, sampled on the nectar placed in each up cell at the start of Hartland M10 (blue hue ~110, red ~177)
NECTAR_HSV = {"blue": [((100, 100, 90), (122, 255, 255))],
              "red": [((168, 50, 150), (180, 255, 255)), ((0, 50, 150), (6, 255, 255))]}
NECTAR_AREA = (120, 1500) # px at zoom 1.0: 1.65x a pollen's (smaller in the rear cell, farther away)
NECTAR_MAX_MISS = 4       # nectar drops out of the mask more often where it crosses its own colour's frame bars ...
NECTAR_MIN_OFF_REST = 1   # ... so its tracks may miss more frames (checked on a real pollen flight recoloured to nectar blue,
                          # Hartland Q5 261.4-262.6 s: found 261.9-262.5 s with these, nothing with the pollen settings)
BG_TAU_S = 2.0            # how long a pixel's colour history is (nectar: colour present this long is background)
BG_ON = 0.3               # a pixel that was nectar-coloured more than this share of its history is background
MIN_FILL = 0.45           # blob area / bounding box area
MAX_ASPECT = 2.6          # motion blur stretches a flying ball
GATE_PX = 90              # at zoom 1.0: how far a new track's first step may reach (a fast ball near the opening)
PRED_PX = 30              # how far from its predicted position a moving track's next sighting may be
REST_PX = 12              # a resting track only takes a sighting this close
REST_FRAMES = 6           # a track this many frames (0.2 s) within REST_PX marks a resting spot ...
REST_MEMORY_S = 3.0       # ... remembered this long
MIN_OFF_REST = 2          # a flight must pass through at least this many points away from resting spots (Hartland Q10:
                          # flickering detections along a row of resting balls linked into fast straight "flights")
MAX_MISS = 2              # frames a track may go unseen (behind a frame bar, a blur too faint) before it ends
MIN_TRACK = 3             # a flight needs at least this many sightings ...
MIN_DISP_PX = 30          # ... covering at least this distance at zoom 1.0 ...
MIN_STRAIGHT = 0.6        # ... fairly straight (net distance / path length) ...
MIN_STEP_PX = 8           # ... and moving at least this much per frame at zoom 1.0 (Q5: a real landing 12.8, a shirt piece 4.8)
MERGE_S = 0.6
MIN_CIRC = 0.55           # 4 pi area / perimeter^2 (a disc is 1.0; a ragged shirt piece is far lower)
RING_PX = 5               # at zoom 1.0: the band looked at just outside a candidate ball ...
MAX_RING_YELLOW = 0.12    # ... may be at most this yellow (a shirt cut into pieces by the cell frame continues past the bar)


def colour_mask(bgr, kind):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    if kind == "pollen": return cv2.inRange(hsv, HSV_LO, HSV_HI)
    m = None
    for lo_, hi_ in NECTAR_HSV[kind]:
        r = cv2.inRange(hsv, lo_, hi_); m = r if m is None else (m | r)
    return m


def balls_in(bgr, zoom, mask=None, area=None):
    m = colour_mask(bgr, "pollen") if mask is None else mask
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m, 8)
    area = area or BALL_AREA
    lo, hi = area[0] * zoom * zoom, area[1] * zoom * zoom
    ring_px = max(3, int(round(RING_PX * zoom)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * ring_px + 1, 2 * ring_px + 1))
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if not (lo <= a <= hi): continue
        if a / float(w * h) < MIN_FILL or max(w, h) / max(1, min(w, h)) > MAX_ASPECT: continue
        x0, y0 = max(0, x - 2 * ring_px), max(0, y - 2 * ring_px)
        sub = (lab[y0:y + h + 2 * ring_px, x0:x + w + 2 * ring_px] == i).astype(np.uint8)
        cs, _ = cv2.findContours(sub, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        per = cv2.arcLength(max(cs, key=cv2.contourArea), True) if cs else 0
        circ = 4 * np.pi * a / (per * per) if per else 0
        if circ < MIN_CIRC: continue
        ring = cv2.dilate(sub, k) & (1 - sub)          # a thin band just outside the blob
        other = m[y0:y0 + sub.shape[0], x0:x0 + sub.shape[1]] > 0
        if ring.sum() and float((other & (ring > 0)).sum()) / ring.sum() > MAX_RING_YELLOW: continue   # more yellow just past it: a shirt piece
        out.append((float(cen[i][0]), float(cen[i][1]), int(a)))
    return out


def _off_rest(seg_pts, rest, zoom):
    """How many of these points lie away from every spot where a ball was resting around then."""
    r2 = (REST_PX * zoom) ** 2
    n = 0
    for t, x, y in seg_pts:
        if all(not (abs(t - rt) <= REST_MEMORY_S and (x - rx) ** 2 + (y - ry) ** 2 <= r2) for rx, ry, rt in rest): n += 1
    return n


def _flight(tr, zoom, rest=(), min_off=None):
    """The fast part of a track, if it is a ball in flight: the longest run of steps of at least MIN_STEP_PX that covers at
    least MIN_DISP_PX fairly straight. Judging only the fast run keeps a ball that flies in and then rests (or bounces).
    Returns (start t, end t, n points) or None."""
    if len(tr) < MIN_TRACK: return None
    pts = np.array([(x, y) for _, x, y in tr]); ts = [t for t, _, _ in tr]
    steps = np.hypot(*np.diff(pts, axis=0).T)
    fast = steps >= MIN_STEP_PX * zoom
    best = None; i = 0
    while i < len(fast):
        if not fast[i]: i += 1; continue
        j = i
        while j + 1 < len(fast) and fast[j + 1]: j += 1
        seg = pts[i:j + 2]; disp = float(np.hypot(*(seg[-1] - seg[0]))); path = float(steps[i:j + 1].sum())
        if (j - i + 1 >= MIN_TRACK - 1 and disp >= MIN_DISP_PX * zoom and disp / path >= MIN_STRAIGHT
                and _off_rest(tr[i:j + 2], rest, zoom) >= (MIN_OFF_REST if min_off is None else min_off)):
            if best is None or disp > best[3]: best = (ts[i], ts[j + 1], j - i + 2, disp)
        i = j + 1
    return best[:3] if best else None


def landings(clip, box, zoom, start, end, kind="pollen"):
    """Flights of ball-shaped blobs of this kind (pollen, red, blue) in the box. Returns (flights, bursts); a flight is
    (first t, last t, n points)."""
    x0, y0, x1, y1 = [int(v) for v in box]
    x0, y0 = max(0, x0) // 2 * 2, max(0, y0) // 2 * 2
    w, h = (x1 - x0) // 2 * 2, (y1 - y0) // 2 * 2   # even: ffmpeg rounds an odd crop of 4:2:0 video, which skewed every row read
    return track(read_frames(clip, x0, y0, w, h, start, end), x0, y0, zoom, start, kind)


def read_frames(clip, x0, y0, w, h, start, end):
    p = subprocess.Popen([FF if os.path.exists(FF) else "ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-i", clip, "-t", f"{end - start:.2f}",
                          "-vf", f"fps={FPS},crop={w}:{h}:{x0}:{y0}", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], stdout=subprocess.PIPE)
    while True:
        b = p.stdout.read(w * h * 3)
        if len(b) < w * h * 3: break
        yield np.frombuffer(b, np.uint8).reshape(h, w, 3)
    p.wait()


def track(frames, x0, y0, zoom, start, kind="pollen"):
    """The tracking itself, on any sequence of box images (FPS apart, the first at `start`; x0, y0 = the box's corner)."""
    gate2 = (GATE_PX * zoom) ** 2
    max_miss = MAX_MISS if kind == "pollen" else NECTAR_MAX_MISS
    min_off = MIN_OFF_REST if kind == "pollen" else NECTAR_MIN_OFF_REST
    live, flights, n = [], [], 0          # live: list of tracks, each a list of (t, x, y) with a miss counter
    rest = []                             # (x, y, t) where a ball sat still; a "flight" hopping along a row of resting balls is not one
    bg = None                             # nectar: each pixel's share of recent frames in which it was nectar-coloured
    for im in frames:
        t = start + n / FPS; n += 1
        if kind == "pollen":
            found = balls_in(im, zoom)
        else:
            cm = (colour_mask(im, kind) > 0).astype(np.float32)
            bg = cm.copy() if bg is None else bg + (cm - bg) / (BG_TAU_S * FPS)
            fg = ((cm > 0) & (bg < BG_ON)).astype(np.uint8) * 255   # nectar-coloured now, but not for most of the last moments
            found = balls_in(im, zoom, mask=fg, area=NECTAR_AREA)
        cur = [(cx + x0, cy + y0, a) for cx, cy, a in found]   # picture coordinates
        used = set()

        def nearest(cx_, cy_, r2):
            best, bd = None, r2
            for i, (cx, cy, _) in enumerate(cur):
                if i in used: continue
                d2 = (cx - cx_) ** 2 + (cy - cy_) ** 2
                if d2 < bd: best, bd = i, d2
            return best

        def moving(tr):
            return len(tr["pts"]) >= 2 and np.hypot(tr["pts"][-1][1] - tr["pts"][-2][1], tr["pts"][-1][2] - tr["pts"][-2][2]) >= MIN_STEP_PX * zoom

        # association in order: resting tracks take only a detection right where they are; moving tracks take one near their
        # predicted next position; one-point tracks may reach far (the first step of a fast ball). Greedy matching without
        # this let a resting ball's track zig-zag through every incoming ball (Hartland Q5 229.9-230.9 s), failing straightness.
        for tr in sorted(live, key=lambda tr: (moving(tr), len(tr["pts"]) == 1)):
            (_, px, py) = tr["pts"][-1]
            if moving(tr):
                (_, qx, qy) = tr["pts"][-2]; k = nearest(2 * px - qx, 2 * py - qy, (PRED_PX * zoom) ** 2)
            elif len(tr["pts"]) == 1:
                k = nearest(px, py, gate2)
            else:
                k = nearest(px, py, (REST_PX * zoom) ** 2)
                if k is None: k = nearest(px, py, gate2)   # a resting ball can be knocked into motion
            if k is None: tr["miss"] += 1
            else:
                used.add(k); tr["miss"] = 0; tr["pts"].append((round(t, 3), cur[k][0], cur[k][1]))
        for i, (cx, cy, _) in enumerate(cur):
            if i not in used: live.append({"pts": [(round(t, 3), cx, cy)], "miss": 0})
        for tr in live:   # a track that has not moved for REST_FRAMES frames marks a resting spot
            q = tr["pts"][-REST_FRAMES:]
            if len(q) == REST_FRAMES and max(np.hypot(x - q[-1][1], y - q[-1][2]) for _, x, y in q) <= REST_PX * zoom:
                rest.append((q[-1][1], q[-1][2], q[-1][0]))
        rest = [r for r in rest if t - r[2] <= REST_MEMORY_S]
        keep = []
        for tr in live:
            if tr["miss"] > max_miss:
                f = _flight(tr["pts"], zoom, rest, min_off)
                if f: flights.append(f)
            else: keep.append(tr)
        live = keep
    for tr in live:
        f = _flight(tr["pts"], zoom, rest, min_off)
        if f: flights.append(f)
    flights.sort()
    bursts = []
    for f0, f1, k in flights:
        if bursts and f0 - bursts[-1]["end"] <= MERGE_S:
            bursts[-1]["end"] = max(bursts[-1]["end"], f1); bursts[-1]["flights"] += 1
        else:
            bursts.append({"start": f0, "end": f1, "flights": 1})
    return flights, bursts


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("clip"); ap.add_argument("--box", type=float, nargs=4, required=True); ap.add_argument("--zoom", type=float, default=1.0)
    ap.add_argument("--start", type=float, required=True); ap.add_argument("--end", type=float, required=True); ap.add_argument("--out")
    ap.add_argument("--kind", choices=("pollen", "red", "blue"), default="pollen")
    a = ap.parse_args()
    fl, bu = landings(a.clip, a.box, a.zoom, a.start, a.end, a.kind)
    for b in bu: print(f"{b['start']:7.2f}-{b['end']:7.2f}  flights {b['flights']}")
    if a.out: json.dump({"clip": a.clip, "box": a.box, "zoom": a.zoom, "flights": fl, "bursts": bu}, open(a.out, "w"), indent=1)
