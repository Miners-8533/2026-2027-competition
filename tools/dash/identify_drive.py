"""tools/dash/identify_drive.py (2026-11-06, Phase 1 of the dash planner rework, design/dash-planner-grid.md): run `python tools/dash/identify_drive.py` (all TeleOp driving rows) or `... dash` (dash rows only); writes drive_model_all.npy / drive_model_dash.npy next to it.
Body-level drive model fitted for multi-step prediction: windows of W log rows, initial state from the measured velocity, error = predicted pose vs logged pose at each later row.
Per axis (forward, strafe, turn): target = K * (volt/12) * sign(c) * max(0, |c| - p0), first-order lag with tau_up / tau_down; c is the command after the wheel-power normalization."""
import glob, math, sys
import numpy as np
from scipy.optimize import least_squares

import os
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "robot-logs")
SP = os.path.join(os.path.dirname(os.path.abspath(__file__)), "")
W = 8   # rows per window (7 steps, about 0.7 s)
DASH_ONLY = len(sys.argv) > 1 and sys.argv[1] == "dash"

wins = []
for f in sorted(glob.glob(os.path.join(ROOT, "2026-11-0[56]", "*teleop.log"))):
    ev, S = [], []
    for l in open(f, encoding="utf-8", errors="replace"):
        p = l.rstrip("\n").split(",")
        if l.startswith("E,"):
            try:
                ev.append((float(p[1]), l.split(",", 2)[2]))
            except Exception:
                pass
        elif l.startswith("S,") and len(p) > 14:
            try:
                S.append((float(p[1]), float(p[5]), float(p[6]), math.radians(float(p[7])), float(p[10]), float(p[11]), float(p[12]), float(p[4]), p[13]))
            except Exception:
                pass
    if not S:
        continue
    mode = [s[8] for s in S]
    S = np.array([s[:8] for s in S])
    keep = np.ones(len(S), bool)
    if DASH_ONLY:
        keep[:] = False
        cur = None
        for tt, m in ev:
            if m.startswith("HIVE DASH start") or m.startswith("PARK DASH start"):
                cur = tt
            elif cur is not None and (m.startswith("HIVE DASH arrived") or m.startswith("PARK DASH arrived") or ("cancelled" in m and "DASH" in m)):
                keep |= (S[:, 0] >= cur - 0.05) & (S[:, 0] <= tt + 0.3)
                cur = None
    for i in range(1, len(S) - W):
        if not keep[i]:
            continue
        w = S[i:i + W]
        d = np.diff(w[:, 0])
        if d.min() < 0.03 or d.max() > 0.25:
            continue
        if any(mode[i + k] == "backup" for k in range(W)):
            continue
        if np.max(np.abs(w[:, 4:7])) < 0.12:
            continue
        wins.append(w)
        if i % 2 == 0:
            pass
A = np.array(wins)
A[:, :, 3] = np.unwrap(A[:, :, 3], axis=1)
print("windows:", A.shape, "(dash rows only)" if DASH_ONLY else "(all TeleOp driving rows)")
t = A[:, :, 0]; X = A[:, :, 1]; Y = A[:, :, 2]; H = A[:, :, 3]
cf = A[:, :, 4]; cl = A[:, :, 5]; ct = A[:, :, 6]; volt = np.clip(A[:, :, 7], 6.0, 13.5)
dt = np.diff(t, axis=1)
n = len(A)
lf, lb, rb, rf = cf - cl - ct, cf + cl - ct, cf - cl + ct, cf + cl + ct
m = np.maximum(1.0, np.maximum.reduce([np.abs(lf), np.abs(lb), np.abs(rb), np.abs(rf)]))
nf, nl, nt = cf / m, cl / m, ct / m
CMD = np.stack([nf, nl, nt], axis=0)   # [3, n, W]
# initial body velocity from the first step
dx0, dy0, dh0 = X[:, 1] - X[:, 0], Y[:, 1] - Y[:, 0], H[:, 1] - H[:, 0]
hm0 = 0.5 * (H[:, 1] + H[:, 0])
v0 = np.stack([(dx0 * np.cos(hm0) + dy0 * np.sin(hm0)) / dt[:, 0], (-dx0 * np.sin(hm0) + dy0 * np.cos(hm0)) / dt[:, 0], dh0 / dt[:, 0]], axis=0)


def simulate(th):
    K = th[0:3]; p0 = th[3:6]; tu = th[6:9]; td = th[9:12]
    v = v0.copy()
    px = X[:, 0].copy(); py = Y[:, 0].copy(); ph = H[:, 0].copy()
    out = np.zeros((3, n, W))
    out[0, :, 0] = px; out[1, :, 0] = py; out[2, :, 0] = ph
    for i in range(1, W):
        d = dt[:, i - 1]
        c = CMD[:, :, i - 1]
        tgt = K[:, None] * (volt[:, i - 1] / 12.0)[None] * np.sign(c) * np.maximum(0.0, np.abs(c) - p0[:, None])
        tau = np.where(np.abs(tgt) > np.abs(v), tu[:, None], td[:, None])
        vn = v + (tgt - v) * (1 - np.exp(-d[None] / tau))
        vm = 0.5 * (v + vn)
        hmid = ph + 0.5 * vm[2] * d
        px = px + (vm[0] * np.cos(hmid) - vm[1] * np.sin(hmid)) * d
        py = py + (vm[0] * np.sin(hmid) + vm[1] * np.cos(hmid)) * d
        ph = ph + vm[2] * d
        v = vn
        out[0, :, i] = px; out[1, :, i] = py; out[2, :, i] = ph
    return out


def resid(th):
    o = simulate(th)
    r = np.stack([o[0] - X, o[1] - Y, (o[2] - H) * 12.0], axis=0)[:, :, 1:]
    return r.ravel()


th0 = np.array([74, 33, 6.0, 0.02, 0.02, 0.02, 0.2, 0.15, 0.15, 0.11, 0.1, 0.1])
lo = np.array([30, 10, 1.0, 0, 0, 0, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03]); hi = np.array([250, 250, 20, 0.4, 0.4, 0.4, 2, 2, 2, 2, 2, 2])
base = np.stack([X - X[:, :1], Y - Y[:, :1], (H - H[:, :1]) * 12], axis=0)[:, :, 1:].ravel()
print("do-nothing rms (pose error within a window): %.2f in" % np.sqrt(np.mean(base ** 2)))
fit = least_squares(resid, th0, bounds=(lo, hi), x_scale=[50, 30, 3, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2, 0.1, 0.1, 0.1], max_nfev=80)
r = resid(fit.x)
print("model rms (pose error within a window): %.2f in" % np.sqrt(np.mean(r ** 2)))
nm = ["forward", "strafe", "turn"]
print("%-8s %8s %6s %8s %8s" % ("axis", "gain", "dead", "tau_up", "tau_dn"))
for k in range(3):
    print("%-8s %8.2f %6.3f %8.3f %8.3f" % (nm[k], fit.x[k], fit.x[3 + k], fit.x[6 + k], fit.x[9 + k]))
o = simulate(fit.x)
for T in (2, 4, 7):
    e = np.hypot(o[0, :, T] - X[:, T], o[1, :, T] - Y[:, T]); hh = np.degrees(np.abs(o[2, :, T] - H[:, T]))
    print("after %d steps (%.2f s): position median %.2f in, 90th %.2f; heading median %.1f deg, 90th %.1f" % (T, np.median(t[:, T] - t[:, 0]), np.median(e), np.percentile(e, 90), np.median(hh), np.percentile(hh, 90)))
np.save(SP + ("drive_model_dash.npy" if DASH_ONLY else "drive_model_all.npy"), fit.x)
