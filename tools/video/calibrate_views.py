"""Per-match camera calibration: map the reference match's field corners and hive boxes into every other match.

    python tools/video/calibrate_views.py --event "Hartland Qualifier 2026" [--labels Q10,Q24]

The broadcast camera holds still during a match but is re-aimed and re-zoomed between matches. A camera that only
turns and zooms maps one picture onto another by a homography, so SIFT features on the static parts of the picture
(field perimeter, hive frames, walls, banners) give it: frame of the reference match -> frame of this match.
The reference corners (video_config.json "field_corners_px", read by eye on the reference match) and hive boxes are
pushed through it and written to video_config.json under "views": {label: {...}}. A check picture per match
(views/<label>.jpg) shows the projected field grid and hive boxes; look at every one.
"""
import argparse, json, os, re, subprocess, sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)
from fuse_match import clip_for, logs_for   # noqa: E402

FF = os.path.join(os.environ.get("LOCALAPPDATA", ""), r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
MOVED_PX = 12.0   # field corners differing more than this between the two halves of a match = camera bumped or re-aimed
TAKE_AT = tuple(185.0 + 12.0 * i for i in range(10))   # clip seconds (TeleOp in every clip); matches from all of them are pooled


def frame(clip, t):
    w, h = 1920, 1080
    r = subprocess.run([FF if os.path.exists(FF) else "ffmpeg", "-v", "error", "-ss", f"{t:.2f}", "-i", clip, "-frames:v", "1",
                        "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], capture_output=True)
    return np.frombuffer(r.stdout, np.uint8).reshape(h, w, 3) if len(r.stdout) == w * h * 3 else None


def register(ref, imgs):
    """SIFT + ratio test on each sample frame, all correspondences pooled (the camera holds still during a match, while
    people and robots move, so only the static background matches consistently), then one MAGSAC homography.
    One frame alone gave ~40 inliers and an unstable fit on a zoomed match (Hartland Q24: x0.85 or x1.16 run to run);
    ORB was tried first and failed on that zoom change outright."""
    sift = cv2.SIFT_create(8000)
    mask = np.zeros(ref.shape[:2], np.uint8); mask[262:, :] = 255     # below the score bar
    k1, d1 = sift.detectAndCompute(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY), mask)
    fl = cv2.FlannBasedMatcher(dict(algorithm=1, trees=5), dict(checks=64))
    P1, P2 = [], []
    for img in imgs:
        k2, d2 = sift.detectAndCompute(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), mask)
        if d2 is None: continue
        for a, b in (x for x in fl.knnMatch(d1, d2, k=2) if len(x) == 2):
            if a.distance < 0.7 * b.distance:
                P1.append(k1[a.queryIdx].pt); P2.append(k2[a.trainIdx].pt)
    if len(P1) < 30: return None, 0
    H, inl = cv2.findHomography(np.float32(P1), np.float32(P2), cv2.USAC_MAGSAC, 4.0, maxIters=20000, confidence=0.9999)
    return H, int(inl.sum()) if inl is not None else 0


def draw(img, corners, hive_rois, half, path):
    out = img.copy()
    F = half
    Hm = cv2.getPerspectiveTransform(np.float32([[F, -F], [F, F], [-F, -F], [-F, F]]),
                                     np.float32([corners["near_red"], corners["near_blue"], corners["far_red"], corners["far_blue"]]))
    P = lambda x, y: tuple(int(v) for v in cv2.perspectiveTransform(np.float32([[[x, y]]]), Hm)[0, 0])
    for v in np.arange(-72, 73, 24):
        cv2.line(out, P(v, -F), P(v, F), (0, 255, 255), 2); cv2.line(out, P(-F, v), P(F, v), (0, 255, 255), 2)
    for name, (x0, y0, x1, y1) in hive_rois.items():
        cv2.rectangle(out, (int(x0), int(y0)), (int(x1), int(y1)), (0, 0, 255) if name.startswith("red") else (255, 0, 0), 3)
    cv2.imwrite(path, cv2.resize(out, (960, 540)))


def field_for(label):
    """Field rule for this event: odd quals field 1, even quals field 2; playoffs: decided by registration."""
    m = re.match(r"([A-Za-z]+)(\d+)$", label)
    if m.group(1).upper() == "Q": return "1" if int(m.group(2)) % 2 else "2"
    return None


def fit(ref, imgs, f, cfg):
    H, n = register(ref, imgs)
    if H is None: return None
    mp = lambda pt: [round(float(v), 1) for v in cv2.perspectiveTransform(np.float32([[pt]]), H)[0, 0]]
    corners = {k: mp(v) for k, v in f["field_corners_px"].items() if k != "note"}
    rois = {}
    for name, (x0, y0, x1, y1) in f["hive_rois"].items():
        q = cv2.perspectiveTransform(np.float32([[[x0, y0]], [[x1, y0]], [[x1, y1]], [[x0, y1]]]), H).reshape(-1, 2)
        rois[name] = [round(float(q[:, 0].min())), round(float(q[:, 1].min())), round(float(q[:, 0].max())), round(float(q[:, 1].max()))]
    return {"H": H, "n": n, "corners": corners, "rois": rois, "zoom": float(np.sqrt(abs(np.linalg.det(H[:2, :2]))))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True); ap.add_argument("--labels")
    a = ap.parse_args()
    ev_dir = os.path.join(ROOT, "robot-logs", "events", a.event); vdir = os.path.join(ev_dir, "video")
    cfg_path = os.path.join(vdir, "video_config.json"); cfg = json.load(open(cfg_path))
    refs = {k: frame(clip_for(f["reference_label"], cfg["clip_dir"]), f["reference_clip_s"]) for k, f in cfg["fields"].items()}
    labels = a.labels.split(",") if a.labels else sorted({r.split(",")[1].strip() for r in open(os.path.join(ev_dir, "labels.csv")).read().splitlines()[1:] if r.split(",")[1].strip()})
    os.makedirs(os.path.join(vdir, "views"), exist_ok=True)
    views = cfg.setdefault("views", {})
    for L in labels:
        clip = clip_for(L, cfg["clip_dir"])
        imgs = [im for im in (frame(clip, t) for t in TAKE_AT) if im is not None]
        fixed = field_for(L)
        tries = {k: fit(refs[k], imgs, cfg["fields"][k], cfg) for k in ([fixed] if fixed else cfg["fields"])}
        tries = {k: v for k, v in tries.items() if v}
        if not tries: print(f"{L}: registration failed"); continue
        fk = max(tries, key=lambda k: tries[k]["n"])
        best = tries[fk]
        # bump check: fit the first and the second half of the samples on their own and compare the field corners
        half = len(imgs) // 2
        A, B = fit(refs[fk], imgs[:half], cfg["fields"][fk], cfg), fit(refs[fk], imgs[half:], cfg["fields"][fk], cfg)
        drift = max(float(np.hypot(A["corners"][k][0] - B["corners"][k][0], A["corners"][k][1] - B["corners"][k][1])) for k in A["corners"]) if A and B else float("nan")
        moved = not (drift <= MOVED_PX)
        if moved and B:
            best = B   # the later half covers most of TeleOp (Hartland M3: bumped ~10 px around the end of AUTO, then still)
        views[L] = {"field": fk, "field_corners_px": best["corners"], "hive_rois": best["rois"],
                    "from": f"field {fk} reference {cfg['fields'][fk]['reference_label']} via homography ({best['n']} pooled inliers from {len(imgs)} frames, zoom x{best['zoom']:.2f})",
                    "half_drift_px": round(drift, 1), "camera_moved": moved,
                    "tried": {k: v["n"] for k, v in tries.items()}}
        draw(imgs[half], best["corners"], best["rois"], cfg["field_half_in"], os.path.join(vdir, "views", f"{L}.jpg"))
        print(f"{L}: field {fk} ({', '.join(f'f{k} {v[chr(110)]}' for k, v in tries.items())} inliers), zoom x{best['zoom']:.2f}, "
              f"first vs second half {drift:.1f} px{'  CAMERA MOVED: using the later-half fit; check views/' + L + '.jpg' if moved else ''}")
    json.dump(cfg, open(cfg_path, "w"), indent=2)


if __name__ == "__main__":
    main()
