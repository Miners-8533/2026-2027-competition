"""Fits the WHEEL drive model to Drive Characterization logs (design/dash-planner-grid.md, "Drive characterization").

    python -I tools/dash/fit_drivetest.py [drivetest.log ...]      (default: robot-logs/latest/drivetest.log)

Writes tools/dash/drive_model_wheel.npy (the parameters, in the order of NAMES) and prints the fit, the error per test against the tuned plant of
2026-11-06 (DashSim --plant tuned: the dash fit per axis, field-frame momentum, grip 150 in/s^2, turn boost 1.5, 12.5 V) and the held-out tests.

THE MODEL (per wheel, so braking at zero power and coasting at part power are different, which the per-axis model could not have):
  the wheel speeds u (in/s along each wheel's drive direction) follow from the body velocity (forward vf, left vl in/s, turn w rad/s, robot frame):
      u_lf = vf - vl/s - kw w,  u_lb = vf + vl/s - kw w,  u_rb = vf - vl/s + kw w,  u_rf = vf + vl/s + kw w
  (s: the strafe ratio, kw: in of wheel travel per radian of turn). The wheel powers p (the logged ones, after MecanumDrive's scaling) are split
  into the body command (forward, strafe, turn), each axis loses its own DEAD ZONE (d_f, d_t; the strafe's d_l only near rest, fading out by
  V_FADE in/s: the rollers' static friction, the robot does not creep sideways at 0.16 while it creeps forward at 0.12), and the wheel powers
  p' from that command set each wheel's target speed g_i (volts / 12) sign(p') F(|p'|), F a CURVE through (0, 0), (0.3, F3), (0.6, F6),
  (0.15, F15), ..., (1, F10) (speed per power is flat to 0.6 and falls above it; g_i the wheels' share, mean 1); and the wheel
      BRAKES at zero power:                     du/dt = -u / tau_b - a_b sign(u)   (stops at zero; a_b per side, a_bL and a_bR: the
                                                                                    robot yaws while it brakes)
      COASTS when driven slower than it runs:   du/dt = -a_c sign(u) - (u - target) / tau_c   (power and speed the same sign, |target| < |u|;
                                                                                    measured: about 37 in/s^2 at 0.2 power from 54 down to 30 in/s)
      DRIVES otherwise:                         du/dt = (target - u) / tau_u
  the wheel accelerations give the body's (the same kinematics backward); the translation accel is at most A_up while it speeds the robot up
  (the motors' current; the battery sags to 11 V at a start) and A_down while it slows it (the tires: about 210 in/s^2 down to 20 in/s when
  braking from 80), less YAW_GRIP |turn accel| kw (the turn uses the same tires; 0.3 from the dash replay); the momentum keeps its direction in
  the field while the robot turns.
The error is the predicted body velocity against the measured one (Pinpoint) at every row of 0.45 s windows started from the measured velocity
(turn rate weighted 10 in per rad).
"""
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from drivetest_report import load  # noqa: E402

NAMES = ["g_lf", "g_lb", "g_rb", "F15", "F3", "F6", "F85", "F10", "d_f", "d_l", "d_t", "tau_u", "tau_b", "a_bL", "a_bR", "a_c", "tau_c", "s", "kw", "A_up", "A_down", "sa", "K_t"]
X0 = [1.0, 1.0, 1.0, 15, 30, 59, 79, 88, 0.07, 0.15, 0.09, 0.17, 0.12, 133, 100, 37, 5.0, 0.77, 10.5, 148, 184, 0.8, 8.8]
LO = [0.8, 0.8, 0.8, 5, 10, 30, 50, 55, 0.0, 0.0, 0.0, 0.02, 0.03, 0.0, 0.0, 0.0, 0.1, 0.4, 5.0, 60, 60, 0.2, 3.0]
HI = [1.2, 1.2, 1.2, 30, 50, 90, 110, 120, 0.2, 0.4, 0.2, 0.6, 3.0, 600, 600, 200, 20.0, 1.0, 16.0, 600, 600, 2.0, 15.0]
KNOTS = [0.0, 0.15, 0.3, 0.6, 0.85, 1.0]
V_FADE = float(os.environ.get("FIT_V_FADE", "1e9"))   # in/s: the strafe dead zone fades out by this FORWARD speed (1e9: always on; the strafe steps are 0.15 short while strafing)
# held at measurements unless FIT_FIX says otherwise: the coast of test 10 (the only clean one), and the forward dead zone and speed curve from the
# steady ends of the forward steps (speed x 12 / vbat: 0.12 -> 4.9, 0.16 -> 9.0, 0.3 -> 23.4, 0.6 -> 53.5, 0.7 -> 62.3, 0.85 -> 72.6, 1.0 -> 84.3 in/s;
# the fit itself, ruled by the transients, put the dead zone at 0.006 and 0.12 at 8.9 in/s)
# and the turn line from the spins' steady rates (x 12 / vbat: 0.3 -> 104, 0.5 -> 198, 0.7 -> 305, 1.0 -> 465 deg/s: 516 deg/s = 9.0 rad/s per unit above 0.098)
FIX_DEFAULT = "a_c=37,tau_c=5,d_f=0.072,F15=15.3,F3=30.6,F6=59.8,F85=78.1,F10=89.5,d_t=0.098,K_t=9.0"
W = 30        # rows per window (about 0.45 s at 15 ms)
STRIDE = 3
HOLD_OUT = {33, 34}
SUB = 0.005   # integration step (s)
W_TURN = 10.0
YAW_GRIP = float(os.environ.get("FIT_YAW_GRIP", "0.3"))
LAG = int(os.environ.get("FIT_LAG", "0"))   # rows of delay between a logged command and the motors (checked 2026-11-07: see the design notes)


def windows(paths):
    out = []
    for p in paths:
        rows, _ = load(p)
        t = np.array([r["t"] for r in rows])
        h = np.radians(np.array([r["hdg_deg"] for r in rows]))
        vx = np.array([r["vx"] for r in rows])
        vy = np.array([r["vy"] for r in rows])
        c, s = np.cos(h), np.sin(h)
        vf, vl = vx * c + vy * s, -vx * s + vy * c
        w = np.radians(np.array([r["w_dps"] for r in rows]))
        P = np.array([[r["p_lf"], r["p_lb"], r["p_rb"], r["p_rf"]] for r in rows])
        vb = np.array([r["vbat"] for r in rows])
        test = np.array([int(r["test"]) for r in rows])
        phase = [r["phase"] for r in rows]
        for i in range(max(LAG, 0), len(rows) - W, STRIDE):
            j = slice(i, i + W)
            jp = slice(i - LAG, i + W - LAG)   # the commands LAG rows earlier (a delay between the command and the motors)
            dt = np.diff(t[j])
            if dt.min() <= 0 or dt.max() > 0.05:
                continue
            if np.min(vb[j]) < 9.0:
                continue
            # something must happen: power on a wheel or the robot moving
            if np.max(np.abs(P[j])) < 0.05 and np.max(np.hypot(vf[j], vl[j])) < 3.0:
                continue
            if phase[i] in ("PAUSED", "DONE", "SETUP"):
                continue
            out.append(dict(t=t[j] - t[i], vf=vf[j], vl=vl[j], w=w[j], P=P[jp], vb=vb[jp], test=test[i], phase=phase[i]))
    return out


def stack(ws):
    t = np.stack([w["t"] for w in ws])
    V = np.stack([np.stack([w["vf"], w["vl"], w["w"]], axis=1) for w in ws])   # N x W x 3
    P = np.stack([w["P"] for w in ws])                                          # N x W x 4
    vb = np.stack([w["vb"] for w in ws])
    return t, V, P, vb


def unpack(th):
    g = np.array([th[0], th[1], th[2], 4.0 - th[0] - th[1] - th[2]])
    F = [0.0, th[3], th[4], th[5], th[6], th[7]]
    return g, F, th[8:]


def sim_wheel(th, t, V, P, vb):
    g, F, rest = unpack(th)
    d_f, d_l, d_t, tau_u, tau_b, a_bL, a_bR, a_c, tau_c, s, kw, A_up, A_down, sa, K_t = rest
    a_b = np.array([a_bL, a_bL, a_bR, a_bR])[None, :]
    N, Wn = t.shape
    v = V[:, 0, :].copy()
    out = np.zeros_like(V)
    out[:, 0, :] = v
    for k in range(1, Wn):
        dt = t[:, k] - t[:, k - 1]
        n = max(1, int(math.ceil(dt.max() / SUB)))
        h = dt / n
        p = P[:, k - 1, :]
        brake = np.abs(p) < 0.005
        # the body command, each axis less its dead zone (the strafe's only near rest)
        cf = p.mean(axis=1)
        cl = (-p[:, 0] + p[:, 1] - p[:, 2] + p[:, 3]) / 4.0
        ct = (-p[:, 0] - p[:, 1] + p[:, 2] + p[:, 3]) / 4.0
        cf = np.sign(cf) * np.maximum(0.0, np.abs(cf) - d_f)
        ct = np.sign(ct) * np.maximum(0.0, np.abs(ct) - d_t)
        for _ in range(n):
            vf, vl, w = v[:, 0], v[:, 1], v[:, 2]
            dl = d_l * np.maximum(0.0, 1.0 - np.abs(vf) / V_FADE)   # fades with the FORWARD speed only: a pure strafe keeps it
            cl2 = np.sign(cl) * np.maximum(0.0, np.abs(cl) - dl)
            # the translation through the forward speed curve, the turn linear (K_t rad/s per unit at 12 V: the spins are linear to full power, 2026-11-07)
            q = np.stack([cf - cl2, cf + cl2, cf - cl2, cf + cl2], axis=1)
            rot = np.stack([-ct, -ct, ct, ct], axis=1) * K_t * kw
            tgt = g[None, :] * (vb[:, k - 1, None] / 12.0) * (np.sign(q) * np.interp(np.abs(q), KNOTS, F) + rot)
            u = np.stack([vf - vl / s - kw * w, vf + vl / s - kw * w, vf - vl / s + kw * w, vf + vl / s + kw * w], axis=1)
            coast = (p * u > 0) & (np.abs(tgt) < np.abs(u)) & ~brake
            du = np.where(brake, -u / tau_b - a_b * np.sign(u), np.where(coast, -a_c * np.sign(u) - (u - tgt) / tau_c, (tgt - u) / tau_u))
            # a coasting wheel does not drop below its target
            du = np.where(coast & (np.abs(du * h[:, None]) > np.abs(u - tgt)), -(u - tgt) / h[:, None], du)
            # a braking wheel stops at zero
            du = np.where(brake & (np.abs(du * h[:, None]) > np.abs(u)), -u / h[:, None], du)
            af = du.mean(axis=1)
            # sa: the sideways response (the rollers) is slower than the forward one; it does not change the strafe's steady speed
            al = sa * s * (-du[:, 0] + du[:, 1] - du[:, 2] + du[:, 3]) / 4.0
            aw = (-du[:, 0] - du[:, 1] + du[:, 2] + du[:, 3]) / (4.0 * kw)
            mag = np.hypot(af, al)
            lim = np.where(af * vf + al * vl >= 0.0, A_up, A_down)
            # the turn uses the same tires: the translation gets what is left after YAW_GRIP |aw| kw (0.3 chosen on the dash replay, 2026-11-07)
            lim = np.maximum(0.3 * lim, lim - YAW_GRIP * np.abs(aw) * kw)
            sc = np.where(mag > lim, lim / np.maximum(mag, 1e-9), 1.0)
            af, al = af * sc, al * sc
            v = np.stack([vf + (af + w * vl) * h, vl + (al - w * vf) * h, w + aw * h], axis=1)
        out[:, k, :] = v
    return out


# the tuned plant (DashSim --plant tuned) for comparison
DASH = [99.09112149473049, 52.32274777761792, 6.646016054636474, 0.11350099123039413, 0.022569029578922054, 0.06683631950982398,
        0.29624991310327675, 0.2501358421085961, 0.20178995973904792, 0.21469726830227168, 0.2176978050149036, 0.10626490959126848]


def sim_tuned(t, V, P, vb):
    K, p0, tu, td = np.array(DASH[0:3]), np.array(DASH[3:6]), np.array(DASH[6:9]), np.array(DASH[9:12])
    N, Wn = t.shape
    v = V[:, 0, :].copy()
    out = np.zeros_like(V)
    out[:, 0, :] = v
    for k in range(1, Wn):
        dt = t[:, k] - t[:, k - 1]
        n = max(1, int(math.ceil(dt.max() / SUB)))
        h = dt / n
        p = P[:, k - 1, :]
        # back from the wheel powers to the scaled body command
        c = np.stack([p.mean(axis=1), (-p[:, 0] + p[:, 1] - p[:, 2] + p[:, 3]) / 4.0, (-p[:, 0] - p[:, 1] + p[:, 2] + p[:, 3]) / 4.0], axis=1)
        tgt = K[None, :] * (12.5 / 12.0) * np.sign(c) * np.maximum(0.0, np.abs(c) - p0[None, :])
        for _ in range(n):
            vf, vl, w = v[:, 0], v[:, 1], v[:, 2]
            tg = tgt.copy()
            tg[:, 2] *= 1.0 + 1.5 * np.hypot(vf, vl) / 60.0
            tau = np.where(np.abs(tg) > np.abs(v), tu[None, :], td[None, :])
            a = (tg - v) / tau
            mag = np.hypot(a[:, 0], a[:, 1])
            sc = np.where(mag > 150.0, 150.0 / np.maximum(mag, 1e-9), 1.0)
            v = np.stack([vf + (a[:, 0] * sc + w * vl) * h, vl + (a[:, 1] * sc - w * vf) * h, w + a[:, 2] * h], axis=1)
        out[:, k, :] = v
    return out


def resid(pred, V):
    e = pred[:, 1:, :] - V[:, 1:, :]
    e[:, :, 2] *= W_TURN
    return e


def main():
    paths = [a for a in sys.argv[1:]] or [os.path.join("robot-logs", "latest", "drivetest.log")]
    ws = windows(paths)
    train = [w for w in ws if w["test"] not in HOLD_OUT]
    hold = [w for w in ws if w["test"] in HOLD_OUT]
    print(f"{len(ws)} windows of {W} rows ({len(train)} to fit, {len(hold)} held out: tests {sorted(HOLD_OUT)})")
    T = stack(train)
    sub = np.arange(0, len(train), 2)   # every other window for the fit itself (speed)
    Ts = tuple(a[sub] for a in T)

    def f(th):
        return resid(sim_wheel(th, *Ts), Ts[1]).ravel()

    lo, hi, x0 = list(LO), list(HI), list(X0)
    # FIT_FIX="a_c=37,tau_c=5": hold parameters at measured values (the coast of test 10, 2026-11-07)
    for item in filter(None, os.environ.get("FIT_FIX", FIX_DEFAULT).split(",")):
        k, v = item.split("=")
        i = NAMES.index(k)
        x0[i], lo[i], hi[i] = float(v), float(v) - 1e-6, float(v) + 1e-6
    r = least_squares(f, x0, bounds=(lo, hi), x_scale="jac", diff_step=1e-3, max_nfev=200, verbose=0)
    th = r.x
    print("fit:", ", ".join(f"{n} {v:.4g}" for n, v in zip(NAMES, th)), f"(cost {r.cost:.4g}, {r.nfev} evaluations)")
    np.save(os.path.join(HERE, "drive_model_wheel.npy"), th)

    def rms(pred, V):
        e = pred[:, 1:, :] - V[:, 1:, :]
        return np.sqrt((e[:, :, 0] ** 2).mean()), np.sqrt((e[:, :, 1] ** 2).mean()), math.degrees(np.sqrt((e[:, :, 2] ** 2).mean()))

    for label, group in (("fitted tests", train), ("held-out tests", hold)):
        if not group:
            continue
        G = stack(group)
        a, b = rms(sim_wheel(th, *G), G[1]), rms(sim_tuned(*G), G[1])
        print(f"{label}: velocity error rms, wheel model: forward {a[0]:.1f} in/s, left {a[1]:.1f} in/s, turn {a[2]:.1f} deg/s;  tuned plant: {b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}")
    print("\nper test (rms of the translation error, in/s, and turn, deg/s): wheel model | tuned plant")
    tests = sorted({w["test"] for w in ws})
    for k in tests:
        g = [w for w in ws if w["test"] == k]
        G = stack(g)
        a, b = rms(sim_wheel(th, *G), G[1]), rms(sim_tuned(*G), G[1])
        print(f"  test {k:2d} ({len(g):3d} windows){' held out' if k in HOLD_OUT else '         '}: {math.hypot(a[0], a[1]):5.1f} {a[2]:6.1f} | {math.hypot(b[0], b[1]):5.1f} {b[2]:6.1f}")
    # what the model says in words
    g, F, rest = unpack(th)
    d_f, d_l, d_t, tau_u, tau_b, a_bL, a_bR, a_c, tau_c, s_, kw, A_up, A_down, sa, K_t = rest
    vs = 12.9 / 12.0
    def fwd(pw):
        return vs * np.interp(max(0.0, pw - d_f), KNOTS, F)
    print("\nat 12.9 V, forward: " + ", ".join(f"{pw} -> {fwd(pw):.1f}" for pw in (0.12, 0.16, 0.3, 0.6, 0.85, 1.0)) + " in/s;"
          f" strafe 0.4 / 0.7 / 0.85 -> " + " / ".join(f"{s_ * vs * np.interp(max(0, pw - d_l), KNOTS, F):.1f}" for pw in (0.4, 0.7, 0.85)) + " in/s;"
          f" spin 0.3 / 0.5 / 0.7 / 1.0 -> " + " / ".join(f"{math.degrees(vs * K_t * max(0, pw - d_t)):.0f}" for pw in (0.3, 0.5, 0.7, 1.0)) + " deg/s;"
          f" wheel shares {', '.join(f'{x:.3f}' for x in g)}")
    print(f"dead zones forward {d_f:.3f}, strafe {d_l:.3f} (at rest), turn {d_t:.3f}; braking: wheels {77 / tau_b + a_bL:.0f} / {77 / tau_b + a_bR:.0f} in/s^2 from 77 in/s,"
          f" the robot at most {A_down:.0f}; coasting {a_c:.0f} + 1/{tau_c:.2f} s; speeding up {tau_u:.2f} s, at most {A_up:.0f} in/s^2; sideways response x {sa:.2f}")

if __name__ == "__main__":
    main()
