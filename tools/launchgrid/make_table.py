"""Builds TeamCode/.../drive/LaunchGridData.java (the 2-D launch grid: flywheel speed and aim residual per launching spot) from the Launch Grid Tuner CSVs.

    python tools/launchgrid/make_table.py            # writes the generated source and prints the leave-one-out check

Input: the CSVs the Launch Grid Tuner runs were tabulated into (tools/launchgrid/data/*_BLUE_REAR_*.csv, made by log_to_csv.py from the runs' GRID POINT log lines; read in name order and a
later file REPLACES an earlier one's row for the same spot number, so a retest corrects the first run; spot numbers 101 and up are extra spots and are added as they are). The
spot is where the robot ACTUALLY stood (pose_x, pose_y, pose_heading_deg at the moment the operator accepted it), not the lattice point it was sent to.

For each spot the table holds: x, y, the flywheel speed (ticks/s), the aim RESIDUAL (deg: the heading flown minus the bearing from the robot to the cell's center; it
contains the launcher offset and whatever the aim needed besides, so the runtime aim is bearing + residual and does not depend on TargetLock's constants) and the status
(1 = a shot works, 0 = no shot possible). The triangulation (Delaunay) is stored too, so the robot code needs no triangulation.

Spots dropped as not evidence: a GOOD spot where no salvo was fired (unverified) and a NO SHOT spot where no salvo was fired out in the wall column (the robot was stopped
short of it there, not tested). A NO SHOT spot at the hive side where no salvo was fired stays (the operator judged it).

ESTIMATES (2026-11-13): the launching spots the Launch Grid Tuner would visit (tools/launchgrid/data/lattice_<CELL>.csv, from DumpLattice.java) that nobody measured (the spots in the
wall columns the robot could not reach or that were not tested, the corners) get an ESTIMATE from the nearest measured good spots: a distance-weighted local plane through the 6
nearest (value = a + b dx + c dy, Gaussian weights of 16 in), clamped to the range of those neighbors plus a margin. They are marked estimated in the data, so the grid covers the
launching region up to the walls and the robot never falls back to the formula.

SEED for the other blue hive cell (BLUE_AUDIENCE): the blue hive is symmetric about X = 0, so the spots are mirrored (x -> -x) and the same BALL PATH is kept: the flown launch
line (the launcher sits TargetLock.LAUNCHER_OFFSET_LEFT_IN to the left of the pose point) crosses the opening face at some y; the mirrored robot aims its launcher line at that
point of the mirrored face. The speed (a function of distance) is kept. A guess to seed the cell until it is measured; marked seeded in the data.
"""
import csv
import math
import os
import sys

import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator
from scipy.spatial import Delaunay

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
DATA_DIR = os.path.join(ROOT, "tools", "launchgrid", "data")
OUT = os.path.join(ROOT, "TeamCode", "src", "main", "java", "org", "firstinspires", "ftc", "teamcode", "drive", "LaunchGridData.java")

LAUNCHER_OFFSET_LEFT_IN = -2.5   # TargetLock.LAUNCHER_OFFSET_LEFT_IN
CELLS = {"BLUE_REAR": (-21.5, 12.75), "BLUE_AUDIENCE": (21.5, 12.75)}


def norm_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def load():
    import glob
    by_spot = {}
    files = sorted(glob.glob(os.path.join(DATA_DIR, "*_BLUE_REAR_*.csv")))
    for path in files:
        with open(path, newline="") as fh:
            for r in csv.DictReader(fh):
                by_spot[int(r["spot"])] = r          # a later file replaces an earlier one's row
    print("data files: " + ", ".join(os.path.basename(f) for f in files))
    spots = []
    dropped = []
    if True:
        for n in sorted(by_spot):
            r = by_spot[n]
            x, y, h = float(r["pose_x"]), float(r["pose_y"]), float(r["pose_heading_deg"])
            salvos = int(r["salvos_fired"])
            good = r["status"] == "GOOD"
            if good and salvos == 0:
                dropped.append((n, "GOOD but no salvo was fired (unverified)"))
                continue
            if not good and salvos == 0 and x < -50.0:
                dropped.append((n, "NO SHOT with no salvo fired in the wall column (the robot was stopped short, not tested)"))
                continue
            spots.append(dict(n=n, x=x, y=y, heading=h, good=good, speed=int(r["speed_ticks_s"]) if good else None))
    return spots, dropped


def residual(x, y, heading_deg, cell):
    bearing = math.degrees(math.atan2(cell[1] - y, cell[0] - x))
    return norm_deg(heading_deg - bearing)


def seed_mirror(spots, src_cell, dst_cell):
    """The same spots mirrored about X = 0, aiming the launcher line at the mirrored crossing of the opening face."""
    out = []
    for s in spots:
        mx = -s["x"]
        if not s["good"]:
            out.append(dict(n=s["n"], x=mx, y=s["y"], good=False, speed=None, resid=None))
            continue
        h = math.radians(s["heading"])
        ox = s["x"] + LAUNCHER_OFFSET_LEFT_IN * (-math.sin(h))
        oy = s["y"] + LAUNCHER_OFFSET_LEFT_IN * math.cos(h)
        face_y = oy + (src_cell[0] - ox) * math.tan(h)          # where the flown launch line crosses the opening face
        tx, ty = dst_cell[0], face_y                              # the mirrored crossing point
        dx, dy = tx - mx, ty - s["y"]
        dist = math.hypot(dx, dy)
        par = math.asin(max(-0.9, min(0.9, LAUNCHER_OFFSET_LEFT_IN / dist)))
        h2 = math.degrees(math.atan2(dy, dx) - par)
        out.append(dict(n=s["n"], x=mx, y=s["y"], good=True, speed=s["speed"], resid=residual(mx, s["y"], h2, dst_cell)))
    return out


def local_plane(pts, vals_list, at, k=6, sigma=16.0, margins=None):
    """Weighted local linear fit through the k nearest points, evaluated at `at`; each value list is clamped to the range of the neighbors plus its margin."""
    d = np.hypot(pts[:, 0] - at[0], pts[:, 1] - at[1])
    idx = np.argsort(d)[:k]
    w = np.exp(-(d[idx] / sigma) ** 2) + 1e-3
    A = np.column_stack([np.ones(len(idx)), pts[idx, 0] - at[0], pts[idx, 1] - at[1]])
    W = np.sqrt(w)[:, None]
    out = []
    for j, vals in enumerate(vals_list):
        coef, *_ = np.linalg.lstsq(A * W, np.asarray(vals)[idx] * W[:, 0], rcond=None)
        v = float(coef[0])
        m = (margins or [0.0] * len(vals_list))[j]
        v = min(max(v, float(np.min(np.asarray(vals)[idx])) - m), float(np.max(np.asarray(vals)[idx])) + m)
        out.append(v)
    return out


SPEED_MARGIN, RESID_MARGIN = 30.0, 3.0


def estimate_missing(spots, lattice_csv, near_in=8.0):
    """Estimated spots for the lattice spots with no recorded spot (good or no shot) within near_in."""
    good = [s for s in spots if s["good"]]
    pts = np.array([(s["x"], s["y"]) for s in good])
    sp = [s["speed"] for s in good]
    rs = [s["resid"] for s in good]
    known = np.array([(s["x"], s["y"]) for s in spots])
    out = []
    with open(lattice_csv, newline="") as fh:
        for r in csv.DictReader(fh):
            x, y = float(r["x"]), float(r["y"])
            if np.min(np.hypot(known[:, 0] - x, known[:, 1] - y)) <= near_in:
                continue
            speed, resid = local_plane(pts, [sp, rs], (x, y), margins=[SPEED_MARGIN, RESID_MARGIN])
            out.append(dict(n=0, x=x, y=y, good=True, speed=int(round(speed)), resid=resid, est=True))
    return out


def extrapolation_test(spots):
    """How good is the estimator at the EDGE: leave out each good spot on the outline of the good spots, estimate it from the rest, report the errors."""
    good = [s for s in spots if s["good"]]
    pts = np.array([(s["x"], s["y"]) for s in good])
    hull = set(Delaunay(pts).convex_hull.ravel().tolist())
    es, er = [], []
    for i in sorted(hull):
        keep = [j for j in range(len(good)) if j != i]
        sp, rs = local_plane(pts[keep], [[good[j]["speed"] for j in keep], [good[j]["resid"] for j in keep]], pts[i], margins=[SPEED_MARGIN, RESID_MARGIN])
        es.append(sp - good[i]["speed"])
        er.append(rs - good[i]["resid"])
    return np.array(es), np.array(er)


def prepare(spots, cell):
    for s in spots:
        if "resid" not in s:
            s["resid"] = residual(s["x"], s["y"], s["heading"], cell) if s["good"] else None
    return spots


def loo(spots):
    """Leave one GOOD spot out, predict it from the other GOOD spots (linear on the triangles, nearest outside), report the errors."""
    good = [s for s in spots if s["good"]]
    pts = np.array([(s["x"], s["y"]) for s in good])
    sp = np.array([s["speed"] for s in good], float)
    rs = np.array([s["resid"] for s in good], float)
    es, er, inside = [], [], 0
    for i in range(len(good)):
        keep = [j for j in range(len(good)) if j != i]
        lin_s = LinearNDInterpolator(pts[keep], sp[keep])
        lin_r = LinearNDInterpolator(pts[keep], rs[keep])
        near_s = NearestNDInterpolator(pts[keep], sp[keep])
        near_r = NearestNDInterpolator(pts[keep], rs[keep])
        ps, pr = float(lin_s(pts[i])[0]), float(lin_r(pts[i])[0])
        if np.isnan(ps):
            ps, pr = float(np.ravel(near_s(pts[i]))[0]), float(np.ravel(near_r(pts[i]))[0])
        else:
            inside += 1
        es.append(ps - sp[i])
        er.append(pr - rs[i])
    return np.array(es), np.array(er), inside, len(good)


def java_table(name, spots, seeded):
    lines = ["    /** %s: x, y (in), flywheel speed (ticks/s), aim residual (deg), status (1 a shot works, 0 no shot), estimated (1 = estimated from the nearest measured spots, not measured). %s */" % (name, "SEEDED by mirroring BLUE_REAR (a guess)" if seeded else "measured 2026-11-12"),
             "    public static final double[][] %s = {" % name]
    for s in spots:
        if s["good"]:
            lines.append("            {%.1f, %.1f, %d, %.2f, 1, %d},  // %s" % (s["x"], s["y"], s["speed"], s["resid"], 1 if s.get("est") else 0, "ESTIMATED" if s.get("est") else "spot %d" % s["n"]))
        else:
            lines.append("            {%.1f, %.1f, 0, 0.00, 0, 0},  // spot %d no shot" % (s["x"], s["y"], s["n"]))
    lines.append("    };")
    pts = np.array([(s["x"], s["y"]) for s in spots])
    tri = Delaunay(pts)
    lines.append("    /** The Delaunay triangles of %s (indexes into the table above). */" % name)
    lines.append("    public static final int[][] %s_TRI = {" % name)
    for t in tri.simplices:
        lines.append("            {%d, %d, %d}," % tuple(int(i) for i in t))
    lines.append("    };")
    return "\n".join(lines)


def main():
    spots, dropped = load()
    print("blue rear: %d spots used (%d GOOD, %d NO SHOT); dropped: %s" % (len(spots), sum(s["good"] for s in spots), sum(not s["good"] for s in spots),
                                                                             "; ".join("spot %d (%s)" % d for d in dropped) or "none"))
    spots = prepare(spots, CELLS["BLUE_REAR"])
    es, er, inside, n = loo(spots)
    print("leave-one-out over the %d GOOD spots (%d predicted inside the triangles of the rest, the others from the nearest spot):" % (n, inside))
    print("  speed error  mean abs %.1f ticks/s, rms %.1f, worst %.0f" % (np.mean(np.abs(es)), np.sqrt(np.mean(es ** 2)), np.max(np.abs(es))))
    print("  aim residual error  mean abs %.2f deg, rms %.2f, worst %.1f" % (np.mean(np.abs(er)), np.sqrt(np.mean(er ** 2)), np.max(np.abs(er))))
    good = [s for s in spots if s["good"]]
    print("  (the residuals themselves range %.1f to %.1f deg; speeds %d to %d)" % (min(s["resid"] for s in good), max(s["resid"] for s in good), min(s["speed"] for s in good), max(s["speed"] for s in good)))
    es_, er_ = extrapolation_test(spots)
    print("extrapolation test (each good spot on the outline left out and estimated from the rest, %d spots): speed error mean abs %.1f ticks/s, worst %.0f; aim residual mean abs %.2f deg, worst %.1f" % (
        len(es_), np.mean(np.abs(es_)), np.max(np.abs(es_)), np.mean(np.abs(er_)), np.max(np.abs(er_))))
    data_dir = os.path.join(ROOT, "tools", "launchgrid", "data")
    est_rear = estimate_missing(spots, os.path.join(data_dir, "lattice_BLUE_REAR.csv"))
    print("blue rear: %d missing lattice spots estimated: %s" % (len(est_rear), "; ".join("(%.0f, %.0f) speed %d residual %+.1f" % (e["x"], e["y"], e["speed"], e["resid"]) for e in est_rear)))
    aud = seed_mirror(spots, CELLS["BLUE_REAR"], CELLS["BLUE_AUDIENCE"])
    est_aud = estimate_missing(aud, os.path.join(data_dir, "lattice_BLUE_AUDIENCE.csv"))
    print("blue audience (seeded): %d missing lattice spots estimated" % len(est_aud))
    spots = spots + est_rear
    aud = aud + est_aud
    ag = [s for s in aud if s["good"]]
    print("blue audience seed (mirrored): %d spots, residuals %.1f to %.1f deg" % (len(aud), min(s["resid"] for s in ag), max(s["resid"] for s in ag)))

    src = ["package org.firstinspires.ftc.teamcode.drive;", "",
           "/**",
           " * GENERATED by tools/launchgrid/make_table.py from tools/launchgrid/data/*.csv (the Launch Grid Tuner run of 2026-11-12). Do not edit by hand: run the script again.",
           " * Read by {@link LaunchGridMap}.",
           " */",
           "public final class LaunchGridData {", "",
           "    private LaunchGridData() {}", "",
           java_table("BLUE_REAR", spots, False), "",
           java_table("BLUE_AUDIENCE", aud, True), "", "}", ""]
    with open(OUT, "w", newline="\n", encoding="utf-8") as fh:
        fh.write("\n".join(src))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
