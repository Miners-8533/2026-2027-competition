"""Refine a field's corner marks in the picture from the robot's own logged path.

    python tools/video/calibrate_from_log.py --event "Novi Qualifier 2026" --label Q9 \
        --corners "122,878 1490,1062 770,592 1535,700"        (rough: near_red near_blue far_red far_blue, by eye)

Where a camera looks across the field from a corner, the floor corners are hard to mark by eye (the walls are clear panels
between rails, and people stand in front). The robot log knows where the robot was at every moment of the TeleOp, and the
picture moves where the robot is: for a candidate set of corners, the logged pose is projected into the picture and the
frame-to-frame motion in a box around it is correlated with the logged speed (the same measure align_log uses for the
clock). The corners are moved (Nelder-Mead, from the rough marks) to make that correlation highest, with the clock offset
fitted before and after. The result is printed, drawn (video/views/<label>_fromlog.jpg: the field grid, and the logged
track over a frame) and, with --write, stored as that match's view and as its field's reference in video_config.json.

near = the audience wall (+X), far = the rear wall (-X), red = the Red Wall (-Y), blue = the Blue Wall (+Y).
"""
import argparse, csv, json, os, subprocess, sys
import numpy as np
import cv2
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "logs"))
import align_log, fuse_match as F   # noqa: E402

HALF_BOX_IN = 12.0
MAX_MOVE_PX = 160.0     # a corner may move this far from its rough mark


def homography(c, half):
    return cv2.getPerspectiveTransform(np.float32([[half, -half], [half, half], [-half, -half], [-half, half]]), np.float32(c).reshape(4, 2))


class Scorer:
    def __init__(self, clip, log, tele_start, flipped, duration=120.0, fps=10.0, pad=3.0):
        self.started, S = align_log.read_log(log)
        if flipped: S = S.copy(); S[:, 1] *= -1; S[:, 2] *= -1
        self.t_rel = S[:, 0] - self.started
        self.S = S
        self.spd = np.hypot(np.gradient(S[:, 1], S[:, 0]), np.gradient(S[:, 2], S[:, 0]))
        fr = align_log.motion_frames(clip, tele_start - pad, tele_start + duration + pad, fps)
        self.ft = np.array([f[0] for f in fr])
        self.ii = [cv2.integral((f[1] > 12).astype(np.uint8)) for f in fr]   # integral images: a box sum is 4 lookups
        self.tele_start, self.fps, self.duration = tele_start, fps, duration
        self.grid = np.arange(0.5, min(duration, self.t_rel[-1]) - 0.5, 1 / fps)
        self.sp = np.interp(self.grid, self.t_rel, self.spd)
        self.xs = np.interp(self.grid, self.t_rel, S[:, 1]); self.ys = np.interp(self.grid, self.t_rel, S[:, 2])
        self.h, self.w = fr[0][1].shape

    def r(self, corners, half, delta):
        Hm = homography(corners, half)
        k = np.searchsorted(self.ft, self.tele_start + self.grid + delta - 0.5 / self.fps)
        ok = (k >= 0) & (k < len(self.ft))
        b = HALF_BOX_IN
        pts = np.stack([np.stack([self.xs + dx, self.ys + dy], 1) for dx, dy in ((-b, -b), (b, -b), (b, b), (-b, b))], 1).astype(np.float32)
        q = cv2.perspectiveTransform(pts.reshape(-1, 1, 2), Hm).reshape(-1, 4, 2) * align_log.SC
        x0 = np.clip(q[:, :, 0].min(1), 0, self.w - 1).astype(int); x1 = np.clip(q[:, :, 0].max(1), 1, self.w).astype(int)
        y0 = np.clip(q[:, :, 1].min(1), 0, self.h - 1).astype(int); y1 = np.clip(q[:, :, 1].max(1), 1, self.h).astype(int)
        e = np.full(len(self.grid), np.nan)
        for i in np.where(ok & (x1 > x0) & (y1 > y0))[0]:
            I = self.ii[k[i]]
            e[i] = (I[y1[i], x1[i]] - I[y0[i], x1[i]] - I[y1[i], x0[i]] + I[y0[i], x0[i]]) / float((x1[i] - x0[i]) * (y1[i] - y0[i]))
        m = ~np.isnan(e)
        if m.sum() < 100 or e[m].std() == 0: return -1.0
        return float(np.corrcoef(e[m], self.sp[m])[0, 1])

    def best_delta(self, corners, half, lo=-2.0, hi=3.0):
        sc = [(self.r(corners, half, d), d) for d in np.arange(lo, hi + 1e-9, 0.1)]
        return max(sc)[1], max(sc)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--event", required=True); ap.add_argument("--label", required=True)
    ap.add_argument("--corners", required=True, help='"x,y x,y x,y x,y": near_red near_blue far_red far_blue, rough')
    ap.add_argument("--write", action="store_true"); ap.add_argument("--field", help="with --write: store as this field's reference")
    ap.add_argument("--hives", help='with --write: "x0,y0,x1,y1 x0,y1,x1,y1": red and blue hive boxes in this match\'s picture')
    a = ap.parse_args()
    L = F.canon(a.label)
    ev_dir = os.path.join(ROOT, "robot-logs", "events", a.event); vdir = os.path.join(ev_dir, "video")
    cfg = json.load(open(os.path.join(vdir, "video_config.json")))
    half = cfg["field_half_in"]
    clip = F.clip_for(L, cfg["clip_dir"]); logs = F.logs_for(ev_dir, L)
    ov = os.path.join(vdir, f"{L}_overlay.csv")
    if not os.path.exists(ov):
        subprocess.run([sys.executable, "-I", os.path.join(HERE, "overlay_ocr.py"), "read", clip, "--glyphs", os.path.join(HERE, "glyphs_fim2026"),
                        "--start", "120", "--end", "340", "--fps", "10", "--out", ov], check=True)
    auto_start, tele_start = F.periods(list(csv.DictReader(open(ov))))
    fm = F.fim_match(json.load(open(os.path.join(vdir, "fim_scores.json"))), L)
    alliance = next(t["alliance"] for t in fm["teams"] if t["teamNumber"] == 8533).lower()
    flipped = (F.log_alliance(logs["teleop"]) or alliance) != alliance
    c0 = np.array([[float(v) for v in p.split(",")] for p in a.corners.split()]).reshape(-1)
    sc = Scorer(clip, logs["teleop"], tele_start, flipped)
    d0, r0 = sc.best_delta(c0, half)
    print(f"{L}: rough corners: r {r0:.3f} at clock offset {d0:+.1f} s")

    def cost(c, d):
        pen = np.maximum(0, np.abs(c - c0) - MAX_MOVE_PX).sum() * 0.01
        return -sc.r(c, half, d) + pen

    c, d = c0.copy(), d0
    for it in range(3):
        res = minimize(cost, c, args=(d,), method="Nelder-Mead",
                       options={"initial_simplex": np.vstack([c] + [c + np.eye(8)[i] * (40 if it == 0 else 15) for i in range(8)]),
                                "xatol": 1.0, "fatol": 1e-4, "maxfev": 1500})
        c = res.x
        d, r = sc.best_delta(c, half)
        print(f"  pass {it + 1}: r {r:.3f} at clock offset {d:+.1f} s; corners moved {np.abs(c - c0).reshape(4, 2).max(1).round().tolist()} px")
    names = ("near_red", "near_blue", "far_red", "far_blue")
    corners = {n: [round(float(c[2 * i]), 1), round(float(c[2 * i + 1]), 1)] for i, n in enumerate(names)}
    print("  corners:", corners)

    # check picture: the grid, and the logged track around a frame's own moment
    os.makedirs(os.path.join(vdir, "views"), exist_ok=True)
    Hm = homography(c, half)
    P = lambda x, y: tuple(int(v) for v in cv2.perspectiveTransform(np.float32([[[x, y]]]), Hm)[0, 0])
    tiles = []
    for tt in (15.0, 45.0, 75.0, 105.0):
        r_ = subprocess.run([align_log.FF, "-v", "error", "-ss", f"{tele_start + tt + d:.2f}", "-i", clip, "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"], capture_output=True)
        im = np.frombuffer(r_.stdout, np.uint8).reshape(1080, 1920, 3).copy()
        for v in np.arange(-half, half + 1, half / 3):
            cv2.line(im, P(v, -half), P(v, half), (0, 255, 255), 1); cv2.line(im, P(-half, v), P(half, v), (0, 255, 255), 1)
        x, y = np.interp(tt, sc.t_rel, sc.S[:, 1]), np.interp(tt, sc.t_rel, sc.S[:, 2])
        box = np.int32([P(x + dx, y + dy) for dx, dy in ((-9, -9), (9, -9), (9, 9), (-9, 9))])
        cv2.polylines(im, [box], True, (255, 0, 255), 3)
        cv2.putText(im, f"{L} teleop {tt:.0f} s: logged robot footprint", (20, 300), 0, 0.9, (255, 0, 255), 2)
        tiles.append(cv2.resize(im[260:], (960, 410)))
    out = os.path.join(vdir, "views", f"{L}_fromlog.jpg")
    cv2.imwrite(out, np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])]))
    print("  check picture:", out)

    if a.write:
        hives = None
        if a.hives:
            hv = [[int(float(v)) for v in p.split(",")] for p in a.hives.split()]
            hives = {"red_hive": hv[0], "blue_hive": hv[1]}
        view = cfg.setdefault("views", {}).setdefault(L, {})
        view.update({"field_corners_px": corners, "from": f"robot log fit (r {r:.2f}) from rough marks"})
        if hives: view["hive_rois"] = hives
        if a.field:
            f = cfg.setdefault("fields", {}).setdefault(str(a.field), {})
            f.update({"reference_label": L, "reference_clip_s": round(tele_start + 45.0, 1), "field_corners_px": corners})
            if hives: f["hive_rois"] = hives
            view["field"] = str(a.field)
        json.dump(cfg, open(os.path.join(vdir, "video_config.json"), "w"), indent=2)
        print("  written to video_config.json")


if __name__ == "__main__":
    main()
