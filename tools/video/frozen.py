"""Find stretches where the broadcast's camera picture is frozen while the overlay keeps updating.

    python tools/video/frozen.py CLIP.mp4 --start 140 --end 320 [--json out.json]

Hartland M3/M7: the field picture stood still for long stretches while the score bar clock ran, so nothing on the
field (tips, robots) can be read from those seconds. Per frame (10 fps) the picture below the score bar is compared
with the previous frame; a run of at least MIN_S with almost no change is a freeze.
"""
import argparse, json, os, subprocess
import numpy as np
import cv2

FF = os.path.join(os.environ.get("LOCALAPPDATA", ""), r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
FPS, STILL, MIN_S = 10, 0.0015, 1.0   # share of pixels changing by more than 8 grey levels


def freezes(clip, start, end):
    w, h = 480, 270
    p = subprocess.Popen([FF if os.path.exists(FF) else "ffmpeg", "-v", "error", "-ss", f"{start:.2f}", "-i", clip, "-t", f"{end - start:.2f}",
                          "-vf", f"fps={FPS},scale={w}:{h},format=gray", "-f", "rawvideo", "-"], stdout=subprocess.PIPE)
    prev, n, still = None, 0, []
    while True:
        b = p.stdout.read(w * h)
        if len(b) < w * h: break
        g = np.frombuffer(b, np.uint8).reshape(h, w)[66:].astype(np.int16)   # below the score bar
        if prev is not None:
            still.append((start + n / FPS, float((np.abs(g - prev) > 8).mean()) < STILL))
        prev, n = g, n + 1
    p.wait()
    runs, cur = [], None
    for t, s in still:
        if s and cur is None: cur = t
        if not s and cur is not None:
            if t - cur >= MIN_S: runs.append([round(cur, 1), round(t, 1)])
            cur = None
    if cur is not None and still[-1][0] - cur >= MIN_S: runs.append([round(cur, 1), round(still[-1][0], 1)])
    return runs


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("clip"); ap.add_argument("--start", type=float, default=140); ap.add_argument("--end", type=float, default=320)
    ap.add_argument("--json"); a = ap.parse_args()
    r = freezes(a.clip, a.start, a.end)
    print(os.path.basename(a.clip)[:3], f"frozen {sum(b - a_ for a_, b in r):.1f} s in {len(r)} stretches:", " ".join(f"{a_}-{b}" for a_, b in r))
    if a.json: json.dump(r, open(a.json, "w"))
