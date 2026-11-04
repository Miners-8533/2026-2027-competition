"""Offline simulation of the flywheel speed controllers (bang-bang + feed-forward against feed-forward + P), on a plant fitted loosely to the 2026-11-03 logs.

    python tools/flywheel/ffp_sim.py                 # compare at the default plant guess
    python tools/flywheel/ffp_sim.py --accel 800 --delay 0.1

The plant (a model, not a measurement): speed v (ticks/s) obeys dv/dt = ACCEL * (P - P_hold(v)) with P the motor power applied DELAY seconds ago and
P_hold(v) = 4.3e-4 v - 0.14 (the average power of the bang-bang at steady speeds 1295 to 1406, 2026-11-03 logs; 0.44 at 1350). The hub reports the speed in steps of
20 ticks/s, the loop runs every 25 ms, and a pollen leaving the wheel takes SALVO_DIP ticks/s of speed away in one step. ACCEL and DELAY are guesses; the script prints the
bang-bang's ripple so the guess can be tuned until it matches the logs (idle ripple: standard deviation 1.2 percent, 95th percentile +3 percent).
"""
import argparse
import random
import statistics

LOOP = 0.025
QUANT = 20.0


def p_hold(v):
    return 4.3e-4 * v - 0.14


def simulate(controller, accel, delay, target=1350.0, seconds=30.0, dips=(), seed=1):
    rng = random.Random(seed)
    v = target
    hist = [0.44] * max(1, int(delay / LOOP) + 1)
    out = []
    t = 0.0
    state = {"integral": 0.0, "filt": [target] * 5}
    dip_i = 0
    while t < seconds:
        raw = round(v / QUANT) * QUANT                      # the hub's step
        state["filt"] = state["filt"][1:] + [raw]
        power = controller(raw, sum(state["filt"]) / 5.0, target, state)
        hist.append(power)
        applied = hist[-1 - int(delay / LOOP)] if len(hist) > int(delay / LOOP) else hist[0]
        # the plant, integrated in 1 ms steps over the loop
        for _ in range(25):
            v += accel * (applied - p_hold(v)) * 0.001
        if dip_i < len(dips) and t >= dips[dip_i][0]:
            v -= dips[dip_i][1]
            dip_i += 1
        v += rng.gauss(0.0, 1.0)                            # a little noise
        out.append((t, v, power))
        t += LOOP
    return out


def bang_bang(raw, filt, target, st, ff=0.35):
    return ff if raw > target else 1.0


def ffp(raw, filt, target, st, kp=0.005, ki=0.002, use_filtered=True, base=0.44, slope=4.3e-4):
    v = filt if use_filtered else raw
    if v < 0.92 * target:
        st["integral"] = 0.0
        return 1.0
    ff = base + slope * (target - 1350.0)
    e = target - v
    st["integral"] = max(-0.15 / max(ki, 1e-9), min(0.15 / max(ki, 1e-9), st["integral"] + e * LOOP))
    return max(0.0, min(1.0, ff + kp * e + ki * st["integral"]))


def stats(trace, target, skip=5.0):
    e = [(v - target) / target * 100.0 for t, v, p in trace if t >= skip]
    e.sort()
    return statistics.pstdev(e), e[int(0.05 * len(e))], e[int(0.95 * len(e))], max(e), min(e)


def after_dip(trace, target, t0):
    seg = [v for t, v, p in trace if t0 <= t <= t0 + 1.0]
    low = min(seg)
    back = next((t - t0 for t, v, p in trace if t > t0 and abs(v - target) <= 0.015 * target and t > t0 + 0.05), float("nan"))
    return (low - target) / target * 100.0, back


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--accel", type=float, default=600.0, help="ticks/s^2 per unit of power (a guess)")
    ap.add_argument("--delay", type=float, default=0.075, help="seconds from the command to the motor")
    ap.add_argument("--dip", type=float, default=40.0, help="ticks/s a pollen takes away")
    args = ap.parse_args()
    dips = [(10.0 + 0.33 * k, args.dip) for k in range(4)]
    print("plant: ACCEL %.0f ticks/s^2 per power, DELAY %.3f s, hub step %.0f ticks/s, loop %.0f ms; target 1350" % (args.accel, args.delay, QUANT, LOOP * 1000))
    print("%-34s %10s %8s %8s %8s %8s | pollen dips: lowest, back within 1.5%%" % ("controller", "std %", "p5", "p95", "max", "min"))
    cases = [("bang-bang, FF 0.35 (now)", bang_bang)]
    for kp in (0.003, 0.005, 0.008, 0.012):
        cases.append(("FF+P kP %.3f kI 0.002" % kp, lambda r, f, t, s, kp=kp: ffp(r, f, t, s, kp=kp)))
    cases.append(("FF+P kP 0.008, raw speed", lambda r, f, t, s: ffp(r, f, t, s, kp=0.008, use_filtered=False)))
    for name, c in cases:
        idle = simulate(c, args.accel, args.delay, seconds=20.0)
        sd, p5, p95, mx, mn = stats(idle, 1350.0)
        sal = simulate(c, args.accel, args.delay, seconds=14.0, dips=dips)
        low, back = after_dip(sal, 1350.0, 10.0)
        print("%-34s %10.2f %+8.1f %+8.1f %+8.1f %+8.1f | %+.1f%%, %.2f s" % (name, sd, p5, p95, mx, mn, low, back))


if __name__ == "__main__":
    main()
