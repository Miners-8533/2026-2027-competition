"""Estimated number of pollen (and our nectar) resting in a hive's up cell, from the video frames.

    import cell_count; cell_count.estimate(clip, hive_box, zoom, times, kind="pollen", ref_t=...)

Balls at rest sit on the cell floor, mostly in one row, so the cell floor band is where they are counted, by the width of
each yellow (or our nectar colour) cluster about one ball high, divided by one ball's diameter at this match's zoom. A second
row hidden behind the first is not seen, so it UNDER-counts as the cell fills: read it as "at least".
Checked against FiM's count of elements resting in the up cell at the end of each match (Hartland: 6 of 6 matches exact).
"""
import subprocess, os
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
import sys; sys.path.insert(0, HERE)
import pollen_landings as P   # noqa: E402

BAND = (0.38, 0.72)        # the up cell's floor as a share of the hive box height (Hartland framing)
BALL_D_1 = {"pollen": 21.0, "nectar": 27.0}   # one resting ball's diameter at zoom 1.0 (px): pollen 2.8 in, nectar 3.6 in
MAX_H_BALLS = 1.7          # a cluster taller than this many ball diameters is not a row of resting balls
MIN_H_BALLS = 0.15         # ... or shorter than this (with the tag strip in front only the balls' tops show: 4-8 px)
TAPE_W_BALLS = 2.5         # a cluster wider than this many ball diameters but under 0.4 of one high is the caution tape
PACK = 0.85                # balls touching in a row take this share of a diameter each in the picture (perspective overlap)
MIN_FILL = 0.35


def band_box(hive_box):
    x0, y0, x1, y1 = hive_box
    h = y1 - y0
    return [x0 // 2 * 2, int(y0 + BAND[0] * h) // 2 * 2, x1 // 2 * 2, int(y0 + BAND[1] * h) // 2 * 2]


def frame(clip, t, box):
    x0, y0, x1, y1 = box
    r = subprocess.run([P.FF if os.path.exists(P.FF) else "ffmpeg", "-v", "error", "-ss", f"{t:.2f}", "-i", clip, "-frames:v", "1",
                        "-vf", f"crop={x1 - x0}:{y1 - y0}:{x0}:{y0}", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], capture_output=True)
    return np.frombuffer(r.stdout, np.uint8).reshape(y1 - y0, x1 - x0, 3)


def count_in(img, ref, zoom, kind="pollen", colour="pollen"):
    """(estimated count, details) of resting balls in img (a band crop). Balls at rest sit side by side in a row and the
    cell's tag strip can hide their lower halves, so they are counted by WIDTH: each yellow (or nectar-coloured) cluster
    about one ball high adds (its width / one ball's diameter). Shirts (taller) and the caution tape (thinner) are left out.
    ref is not used for the count (balls already in the cell after a tip are real); it is kept for callers that want it."""
    m = (P.colour_mask(img, colour) > 0).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, _, st, _ = cv2.connectedComponentsWithStats(m, 8)
    d1 = BALL_D_1[kind] * zoom
    total, parts = 0.0, []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if h < MIN_H_BALLS * d1 or h > MAX_H_BALLS * d1: continue   # a fragment, or a shirt / a person
        if w > TAPE_W_BALLS * d1 and h < 0.4 * d1: continue          # the caution tape behind the hive
        if w < 0.5 * d1 or a / float(w * h) < MIN_FILL: continue
        k = max(1, int(round(w / (PACK * d1))))
        total += k; parts.append((int(x), int(y), int(w), int(h), k))
    return total, parts


def estimate(clip, hive_box, zoom, t, ref_t, kind="pollen", colour="pollen"):
    bb = band_box(hive_box)
    return count_in(frame(clip, t, bb), frame(clip, ref_t, bb) if ref_t is not None else None, zoom, kind, colour)


def series(clip, hive_box, zoom, start, end, fps=2.0, skip=()):
    """[(t, estimated pollen count)] every 1/fps s, the median of each reading and its two neighbours (balls in flight and
    people passing come and go). Readings inside any (t0, t1) of skip (tip swings) are left out."""
    x0, y0, x1, y1 = band_box(hive_box)
    w, h = x1 - x0, y1 - y0
    p = subprocess.Popen([P.FF if os.path.exists(P.FF) else "ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-i", clip, "-t", f"{end - start:.2f}",
                          "-vf", f"fps={fps},crop={w}:{h}:{x0}:{y0}", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], stdout=subprocess.PIPE)
    raw, n = [], 0
    while True:
        b = p.stdout.read(w * h * 3)
        if len(b) < w * h * 3: break
        t = start + n / fps; n += 1
        if any(a <= t <= c for a, c in skip): continue
        raw.append((round(t, 2), count_in(np.frombuffer(b, np.uint8).reshape(h, w, 3), None, zoom)[0]))
    p.wait()
    out = []
    for i, (t, _) in enumerate(raw):
        win = [v for _, v in raw[max(0, i - 1):i + 2]]
        out.append((t, float(np.median(win))))
    return out
