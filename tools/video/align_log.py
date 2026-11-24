"""Find the time offset between a robot RunLog and a match clip.

    python align_log.py --log teleop.log --clip Q05.mp4 --H H.npy --clip-start 185.2 --out align.json

Model: clip_time = clip_start + (log_t - RUN_STARTED) + delta. For each candidate delta, the logged pose is projected
into the image (field->image homography H) and the frame-to-frame motion energy in a box around that point is
correlated with the logged robot speed. The best delta maximizes the correlation.
"""
import argparse, json, os, re, subprocess
import numpy as np
import cv2

FF = os.path.join(os.environ.get("LOCALAPPDATA", ""), r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
SC = 0.5   # work at 960x540


def read_log(path):
    hdr, rows, started = None, [], None
    for line in open(path, encoding="utf-8", errors="replace"):
        if line.startswith("H,"): hdr = line.strip().split(",")[1:]
        elif line.startswith("S,") and hdr:
            v = line.strip().split(",")[1:]
            if len(v) >= len(hdr):
                d = dict(zip(hdr, v))
                try: rows.append((float(d["t"]), float(d["x"]), float(d["y"]), float(d["hdg_deg"])))
                except ValueError: pass
        elif started is None and re.match(r"^E,[0-9.]+,RUN STARTED", line):
            started = float(line.split(",")[1])
    return started, np.array(rows)


def motion_frames(clip, start, end, fps):
    w, h = int(1920 * SC), int(1080 * SC)
    cmd = [FF, "-v", "error", "-ss", f"{start:.2f}", "-i", clip, "-t", f"{end - start:.2f}",
           "-vf", f"fps={fps},scale={w}:{h},format=gray", "-f", "rawvideo", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    prev, out, n = None, [], 0
    while True:
        b = p.stdout.read(w * h)
        if len(b) < w * h: break
        g = cv2.GaussianBlur(np.frombuffer(b, np.uint8).reshape(h, w), (5, 5), 0).astype(np.int16)
        if prev is not None:
            out.append((start + n / fps, np.abs(g - prev).astype(np.uint8)))
        prev, n = g, n + 1
    p.wait()
    return out


def align(log, clip, Hm, clip_start, duration=120.0, fps=10.0, search=(-2.0, 4.0)):
    """Returns (delta_s, r, run_started_log_t, scan)."""
    started, S = read_log(log)
    t_rel = S[:, 0] - started
    spd = np.hypot(np.gradient(S[:, 1], S[:, 0]), np.gradient(S[:, 2], S[:, 0]))
    frames = motion_frames(clip, clip_start + search[0] - 0.5, clip_start + duration + search[1] + 0.5, fps)
    ft = np.array([f[0] for f in frames])
    half = 12.0

    def energy(ti, x, y):
        k = int(np.argmin(np.abs(ft - ti)))
        if abs(ft[k] - ti) > 0.6 / fps: return np.nan
        pts = np.float32([[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]]).reshape(-1, 1, 2)
        q = cv2.perspectiveTransform(pts, Hm).reshape(-1, 2) * SC
        x0, y0 = np.clip(q.min(0).astype(int), 0, None); x1, y1 = q.max(0).astype(int)
        roi = frames[k][1][y0:max(y1, y0 + 1), x0:max(x1, x0 + 1)]
        return float((roi > 12).mean()) if roi.size else np.nan

    grid = np.arange(0.5, min(duration, t_rel[-1]) - 0.5, 1 / fps)
    sp = np.interp(grid, t_rel, spd)
    xs, ys = np.interp(grid, t_rel, S[:, 1]), np.interp(grid, t_rel, S[:, 2])
    scan = []
    for d in np.arange(search[0], search[1] + 1e-9, 0.1):
        e = np.array([energy(clip_start + g + d, x, y) for g, x, y in zip(grid, xs, ys)])
        m = ~np.isnan(e)
        r = float(np.corrcoef(e[m], sp[m])[0, 1]) if m.sum() > 50 else float("nan")
        scan.append((round(float(d), 2), r))
    best = max(scan, key=lambda z: -1 if np.isnan(z[1]) else z[1])
    return best[0], best[1], started, scan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", required=True); ap.add_argument("--clip", required=True); ap.add_argument("--H", required=True)
    ap.add_argument("--clip-start", type=float, required=True, help="clip time of the period start (teleop: clock leaves 2:00)")
    ap.add_argument("--duration", type=float, default=120); ap.add_argument("--fps", type=float, default=10)
    ap.add_argument("--search", type=float, nargs=2, default=(-2.0, 4.0)); ap.add_argument("--out")
    a = ap.parse_args()
    started, S = read_log(a.log)
    Hm = np.load(a.H)
    t_rel = S[:, 0] - started
    spd = np.hypot(np.gradient(S[:, 1], S[:, 0]), np.gradient(S[:, 2], S[:, 0]))
    frames = motion_frames(a.clip, a.clip_start + a.search[0] - 0.5, a.clip_start + a.duration + a.search[1] + 0.5, a.fps)
    ft = np.array([f[0] for f in frames])
    half = 12.0   # inches around the robot centre

    def energy(ti, x, y):
        k = int(np.argmin(np.abs(ft - ti)))
        if abs(ft[k] - ti) > 0.11: return np.nan
        pts = np.float32([[x - half, y - half], [x + half, y - half], [x + half, y + half], [x - half, y + half]]).reshape(-1, 1, 2)
        q = cv2.perspectiveTransform(pts, Hm).reshape(-1, 2) * SC
        x0, y0 = np.clip(q.min(0).astype(int), 0, None); x1, y1 = q.max(0).astype(int)
        roi = frames[k][1][y0:max(y1, y0 + 1), x0:max(x1, x0 + 1)]
        return float((roi > 12).mean()) if roi.size else np.nan

    # sample the log at the video rate
    grid = np.arange(0.5, a.duration - 0.5, 1 / a.fps)
    xs, ys, sp = (np.interp(grid, t_rel, S[:, i]) for i in (1, 2)), None, np.interp(grid, t_rel, spd)
    xs, ys = np.interp(grid, t_rel, S[:, 1]), np.interp(grid, t_rel, S[:, 2])
    results = []
    for d in np.arange(a.search[0], a.search[1] + 1e-9, 0.1):
        e = np.array([energy(a.clip_start + g + d, x, y) for g, x, y in zip(grid, xs, ys)])
        m = ~np.isnan(e)
        r = float(np.corrcoef(e[m], sp[m])[0, 1]) if m.sum() > 50 else float("nan")
        results.append((round(float(d), 2), r))
    best = max(results, key=lambda z: -1 if np.isnan(z[1]) else z[1])
    print("delta s -> correlation:", " ".join(f"{d:+.1f}:{r:.3f}" for d, r in results))
    print(f"BEST delta {best[0]:+.2f} s (r={best[1]:.3f}); clip_time = {a.clip_start:.2f} + (log_t - {started:.3f}) + {best[0]:+.2f}")
    if a.out:
        json.dump({"log": a.log, "clip": a.clip, "run_started_log_t": started, "period_start_clip_t": a.clip_start,
                   "delta_s": best[0], "r": best[1], "scan": results}, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
