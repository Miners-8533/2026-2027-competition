"""Read the FiM BIOBUZZ broadcast overlay (clock, scores, tip counters) from a match clip.

    # once per overlay design: collect glyph clusters, look at the sheet, write the labels file
    python overlay_ocr.py collect CLIP.mp4 --start 130 --end 340 --out glyphs/      # glyphs/sheet.png + clusters.npz
    python overlay_ocr.py label glyphs/ "8,3,0,..."                                  # one label per cluster, in sheet order ('x' = not a digit)
    # every match
    python overlay_ocr.py read CLIP.mp4 --glyphs glyphs/ --start 130 --end 340 --fps 10 --out overlay.csv

Frames are piped from ffmpeg (grayscale top strip). Each region is binarized and split into glyphs (connected
components); a glyph is normalized (height 36, aspect kept, centred in 36x36) and matched to the nearest labelled
template of its font family. The overlay is rendered, so the templates are exact and the match is unambiguous.
"""
import argparse, csv, os, subprocess, sys
import numpy as np
import cv2

FF = os.path.join(os.environ["LOCALAPPDATA"], r"Microsoft\WinGet\Packages\yt-dlp.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-N-125875-g5d4d3bdc61-win64-gpl\bin\ffmpeg.exe")
W, H = 1920, 260
REGIONS = {   # name: ((x0, y0, x1, y1) in 1920x1080, font family, dark text on light?)
    "clock":      ((868, 118, 1056, 212), "clock", True),
    "red_score":  ((645, 148, 852, 252), "score", False),
    "blue_score": ((1068, 148, 1280, 252), "score", False),
    "red_tip_a":  ((120, 150, 188, 202), "tip", True),    # auto layout
    "blue_tip_a": ((1710, 150, 1778, 202), "tip", True),
    "red_tip_t":  ((230, 150, 298, 202), "tip", True),    # teleop layout (LEAVE/PARK boxes shift it)
    "blue_tip_t": ((1600, 150, 1668, 202), "tip", True),
    # playoffs: no "/4" "/7" RP target next to the tip count, so the boxes sit elsewhere
    "red_tip_pa":  ((142, 150, 216, 202), "tip", True),
    "blue_tip_pa": ((1752, 150, 1828, 202), "tip", True),
    "red_tip_pt":  ((260, 150, 336, 202), "tip", True),
    "blue_tip_pt": ((1634, 150, 1708, 202), "tip", True),
}
TIP_COLS = {"red_tips": ("red_tip_a", "red_tip_t", "red_tip_pa", "red_tip_pt"),
            "blue_tips": ("blue_tip_a", "blue_tip_t", "blue_tip_pa", "blue_tip_pt")}
S = 36
CLUSTER_MAX = 0.10
MATCH_MAX = 0.18


def norm(g):
    h, w = g.shape
    nw = max(1, min(S, round(w * S / h)))
    r = cv2.resize(g, (nw, S), interpolation=cv2.INTER_AREA)
    out = np.zeros((S, S), np.float32)
    x = (S - nw) // 2
    out[:, x:x + nw] = r / 255.0
    return out


def glyphs(gray, name):
    (x0, y0, x1, y1), fam, dark = REGIONS[name]
    roi = gray[y0:y1, x0:x1]
    b = ((roi < 110) if dark else (roi > 200)).astype(np.uint8) * 255
    n, _, st, _ = cv2.connectedComponentsWithStats(b, 8)
    hmin = 0.45 * (y1 - y0)
    comps = [st[i] for i in range(1, n) if st[i][3] >= hmin and st[i][4] > 40
             and st[i][1] > 0 and st[i][1] + st[i][3] < (y1 - y0) and st[i][0] > 0 and st[i][0] + st[i][2] < (x1 - x0)]
    comps.sort(key=lambda s: s[0])
    return fam, [norm(b[c[1]:c[1] + c[3], c[0]:c[0] + c[2]]) for c in comps]


def frames(clip, start, end, fps):
    dur = [] if end is None else ["-t", f"{end - start:.2f}"]
    cmd = [FF, "-v", "error", "-ss", f"{start:.2f}", "-i", clip, *dur,
           "-vf", f"fps={fps},crop={W}:{H}:0:0,format=gray", "-f", "rawvideo", "-"]
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    n = 0
    while True:
        buf = p.stdout.read(W * H)
        if len(buf) < W * H: break
        yield start + n / fps, np.frombuffer(buf, np.uint8).reshape(H, W)
        n += 1
    p.wait()


def collect(a):
    reps = []   # (fam, template, count)
    for _, g in (fr for clip in a.clip.split("|") for fr in frames(clip, a.start, a.end, a.fps)):
        for name in REGIONS:
            fam, gl = glyphs(g, name)
            for v in gl:
                for i, (f, t, c) in enumerate(reps):
                    if f == fam and np.abs(v - t).mean() < CLUSTER_MAX:
                        reps[i] = (f, t, c + 1); break
                else:
                    reps.append((fam, v, 1))
    reps = [r for r in reps if r[2] >= a.min_count]
    os.makedirs(a.out, exist_ok=True)
    np.savez(os.path.join(a.out, "clusters.npz"), fam=np.array([r[0] for r in reps]), tpl=np.stack([r[1] for r in reps]))
    cols = 12
    rows = (len(reps) + cols - 1) // cols
    sheet = np.full((rows * 70, cols * 60, 3), 255, np.uint8)
    for i, (fam, t, c) in enumerate(reps):
        y, x = (i // cols) * 70, (i % cols) * 60
        sheet[y + 4:y + 4 + S, x + 12:x + 12 + S] = cv2.cvtColor((255 - t * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        cv2.putText(sheet, f"{i}{fam[0]}", (x + 4, y + 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 200), 1)
    cv2.imwrite(os.path.join(a.out, "sheet.png"), cv2.resize(sheet, None, fx=1.5, fy=1.5))
    print(f"{len(reps)} clusters (min count {a.min_count}) -> {a.out}/sheet.png; label them in order with the 'label' command")


def label(a):
    d = np.load(os.path.join(a.dir, "clusters.npz"))
    labs = [x.strip() for x in a.labels.split(",")]
    if len(labs) != len(d["fam"]):
        sys.exit(f"{len(d['fam'])} clusters, {len(labs)} labels")
    keep = [i for i, l in enumerate(labs) if l.isdigit()]
    np.savez(os.path.join(a.dir, "templates.npz"), fam=d["fam"][keep], tpl=d["tpl"][keep], lab=np.array([labs[i] for i in keep]))
    print(f"{len(keep)} templates saved")


def read(a):
    d = np.load(os.path.join(a.glyphs, "templates.npz"))
    T = {}
    for f, t, l in zip(d["fam"], d["tpl"], d["lab"]):
        T.setdefault(str(f), []).append((str(l), t))
    rows, worst = [], 0.0
    for t, g in frames(a.clip, a.start, a.end, a.fps):
        row = {"t": round(t, 2)}
        for name in REGIONS:
            fam, gl = glyphs(g, name)
            s = ""
            for v in gl:
                dist, lab = min((float(np.abs(v - tp).mean()), l) for l, tp in T[fam])
                if dist > MATCH_MAX:
                    s = ""; break
                worst = max(worst, dist); s += lab
            if fam == "clock":
                s = f"{s[0]}:{s[1:]}" if len(s) == 3 else ""
            row[name] = s
        for col, srcs in TIP_COLS.items():   # one tip count per alliance, whichever layout box is showing it
            row[col] = next((row[c] for c in srcs if row[c]), "")
        rows.append(row)
    with open(a.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t", "clock", "red_score", "blue_score", *TIP_COLS, *[r for r in REGIONS if r not in ("clock", "red_score", "blue_score")]])
        w.writeheader(); w.writerows(rows)
    print(f"{len(rows)} frames, worst accepted glyph distance {worst:.3f} -> {a.out}")


def main():
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest="cmd", required=True)
    for c in ("collect", "read"):
        p = sp.add_parser(c); p.add_argument("clip"); p.add_argument("--start", type=float, default=0)
        p.add_argument("--end", type=float); p.add_argument("--fps", type=float, default=10 if c == "read" else 2)
        p.add_argument("--out", required=True)
    sp.choices["collect"].add_argument("--min-count", type=int, default=3)
    sp.choices["read"].add_argument("--glyphs", required=True)
    p = sp.add_parser("label"); p.add_argument("dir"); p.add_argument("labels")
    a = ap.parse_args()
    {"collect": collect, "label": label, "read": read}[a.cmd](a)


if __name__ == "__main__":
    main()
