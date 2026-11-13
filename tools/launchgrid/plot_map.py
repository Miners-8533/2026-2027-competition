"""Pictures of the 2-D launch map: how far the grid moves the speed and the aim away from today's method (the launch map's speed for the distance and the formula aim), as 3-D
surfaces and top-down maps, with the measured and estimated spots on them. Shows how continuous the adjustment is.

    java -cp build/dumpmap DumpMap BLUE_REAR robot-logs/launch-grid/map_BLUE_REAR.csv 1.0     # first: evaluate the map on a fine grid (see DumpMap.java)
    python tools/launchgrid/plot_map.py BLUE_REAR                                             # writes robot-logs/launch-grid/map_BLUE_REAR.png

"Today's method" here is LaunchMap.TABLE (the speed for the distance to the cell) and TargetLock's formula (the aim with the launcher offset, plus the far-cell extra for the audience cell).
"""
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import TwoSlopeNorm
from matplotlib.lines import Line2D

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
OUT_DIR = os.path.join(ROOT, "robot-logs", "launch-grid")
DATA = os.path.join(ROOT, "tools", "launchgrid", "data", "2026-11-12_BLUE_REAR_193153.csv")
CELLS = {"BLUE_REAR": (-21.5, 12.75, 0.0), "BLUE_AUDIENCE": (21.5, 12.75, -3.0)}   # x, y, TargetLock's far-cell extra (in, + left)
LAUNCHER_OFFSET_LEFT_IN = -2.5
MAP_TABLE = [(28.6, 1325), (32.0, 1350), (35.8, 1350), (42.3, 1350), (46.7, 1375), (62.0, 1450), (82.2, 1650)]   # LaunchMap.TABLE at 2026-11-13


def old_speed(d):
    xs = [p[0] for p in MAP_TABLE]
    ys = [p[1] for p in MAP_TABLE]
    return float(np.interp(d, xs, ys))


def old_resid_deg(d, extra):
    """TargetLock.aimHeadingRad minus the bearing: minus the parallax of the launcher's sideways offset."""
    return -math.degrees(math.asin(max(-0.9, min(0.9, (LAUNCHER_OFFSET_LEFT_IN + extra) / d))))


def main():
    cell = sys.argv[1] if len(sys.argv) > 1 else "BLUE_REAR"
    cx, cy, extra = CELLS[cell]
    rows = list(csv.DictReader(open(os.path.join(OUT_DIR, "map_%s.csv" % cell))))
    xs = sorted({float(r["x"]) for r in rows})
    ys = sorted({float(r["y"]) for r in rows})
    ix = {v: i for i, v in enumerate(xs)}
    iy = {v: i for i, v in enumerate(ys)}
    nx, ny = len(xs), len(ys)
    speed = np.full((ny, nx), np.nan)
    resid = np.full((ny, nx), np.nan)
    zone = np.empty((ny, nx), dtype=object)
    for r in rows:
        i, j = iy[float(r["y"])], ix[float(r["x"])]
        speed[i, j] = float(r["speed"])
        resid[i, j] = float(r["residual_deg"])
        zone[i, j] = r["zone"]
    X, Y = np.meshgrid(xs, ys)
    D = np.hypot(X - cx, Y - cy)
    sp_adj = speed - np.vectorize(old_speed)(D)
    an_adj = resid - np.vectorize(lambda d: old_resid_deg(max(d, 5.0), extra))(D)
    noshot = (zone == "NO_SHOT")

    # the open side of the cell and the region the data is about (the rear half for the rear cell): x from the wall to the face
    sign = -1 if cx < 0 else 1
    side = (X - cx) * sign > 8.0
    show = side & (np.abs(Y) <= 64) & (np.abs(X) <= 66)

    # the steepness of the adjustment over the usable (shot-possible) region
    gy_s, gx_s = np.gradient(sp_adj, ys, xs)
    gy_a, gx_a = np.gradient(an_adj, ys, xs)
    grad_s = np.hypot(gx_s, gy_s)
    grad_a = np.hypot(gx_a, gy_a)
    use = show & ~noshot
    # the measured spots
    spots = []
    if cell == "BLUE_REAR":
        for r in csv.DictReader(open(DATA)):
            x, y, h = float(r["pose_x"]), float(r["pose_y"]), float(r["pose_heading_deg"])
            d = math.hypot(cx - x, cy - y)
            good = r["status"] == "GOOD" and int(r["salvos_fired"]) > 0
            noshot_spot = r["status"] == "NO_SHOT"
            if r["status"] == "GOOD" and int(r["salvos_fired"]) == 0:
                continue
            if noshot_spot and int(r["salvos_fired"]) == 0 and x < -50:
                continue
            bearing = math.degrees(math.atan2(cy - y, cx - x))
            res = (h - bearing + 180.0) % 360.0 - 180.0
            spots.append(dict(x=x, y=y, good=good, sa=(int(r["speed_ticks_s"]) - old_speed(d)) if good else None, aa=(res - old_resid_deg(d, extra)) if good else None))

    fig = plt.figure(figsize=(19, 12.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0], hspace=0.12, wspace=0.10, left=0.04, right=0.97, top=0.92, bottom=0.07)
    norm_s = TwoSlopeNorm(vmin=min(-60, np.nanmin(sp_adj[use])), vcenter=0, vmax=max(60, np.nanmax(sp_adj[use])))
    norm_a = TwoSlopeNorm(vmin=min(-4, np.nanmin(an_adj[use])), vcenter=0, vmax=max(4, np.nanmax(an_adj[use])))

    def surface(ax, Z, norm, label, key):
        Zm = np.where(use, Z, np.nan)
        s = ax.plot_surface(X, Y, Zm, cmap="coolwarm", norm=norm, rstride=1, cstride=1, linewidth=0, antialiased=True, alpha=0.95)
        if spots:
            gx = [p["x"] for p in spots if p[key] is not None]
            gy = [p["y"] for p in spots if p[key] is not None]
            gz = [p[key] for p in spots if p[key] is not None]
            ax.scatter(gx, gy, gz, color="k", s=22, depthshade=False)
        ax.set_xlabel("field X (in)")
        ax.set_ylabel("field Y (in)")
        ax.set_zlabel(label)
        ax.view_init(elev=28, azim=-62 if cx < 0 else -118)
        ax.set_xlim(-66 if cx < 0 else 8, -8 if cx < 0 else 66)
        ax.set_ylim(-64, 64)
        return s

    ax1 = fig.add_subplot(gs[0, 0], projection="3d")
    surface(ax1, sp_adj, norm_s, "speed vs today's map (ticks/s)", "sa")
    ax1.set_title("Flywheel SPEED adjustment: the grid minus the launch map's speed for the distance", fontsize=10)
    ax2 = fig.add_subplot(gs[0, 1], projection="3d")
    surface(ax2, an_adj, norm_a, "aim vs today's formula (deg)", "aa")
    ax2.set_title("AIM adjustment: the grid's heading minus the formula's (+ = further left)", fontsize=10)

    def heat(ax, Z, norm, label, key, levels):
        """Top-down map with field Y across and field X up (the cell is at the top, the wall at the bottom)."""
        Zm = np.where(show, Z, np.nan)
        im = ax.pcolormesh(Y, X, Zm, cmap="coolwarm", norm=norm, shading="auto")
        cs = ax.contour(Y, X, np.where(use, Z, np.nan), levels=levels, colors="k", linewidths=0.6, alpha=0.7)
        ax.clabel(cs, fontsize=7, fmt="%.0f")
        ax.pcolormesh(Y, X, np.where(show & noshot, 1.0, np.nan), cmap="Greys", vmin=0, vmax=2.5, shading="auto", alpha=0.55)
        for p in spots:
            if p["good"]:
                ax.plot(p["y"], p["x"], "o", ms=6, color="k", mfc="white", zorder=5)
            else:
                ax.plot(p["y"], p["x"], "X", ms=8, color="#d62728", mec="k", mew=0.5, zorder=5)
        ax.plot(cy, cx, "*", ms=16, color="#d4a000", mec="k", zorder=6)
        ax.plot([cy - 11, cy + 11], [cx, cx], color="#d4a000", lw=3, zorder=6)
        ax.set_aspect("equal")
        ax.set_ylim(-68 if cx < 0 else 6, -6 if cx < 0 else 68)
        ax.set_xlim(-66, 66)
        ax.set_xlabel("field Y (in): + toward the Blue Wall")
        ax.set_ylabel("field X (in)")
        cb = plt.colorbar(im, ax=ax, fraction=0.03, pad=0.02)
        cb.set_label(label)

    ax3 = fig.add_subplot(gs[1, 0])
    heat(ax3, sp_adj, norm_s, "speed vs today's map (ticks/s)", "sa", np.arange(-120, 241, 20))
    ax3.set_title("SPEED adjustment, top-down (contours every 20 ticks/s; gray = no-shot zone)\nabsolute speed here: %.0f to %.0f ticks/s" % (np.nanmin(np.where(use, speed, np.nan)), np.nanmax(np.where(use, speed, np.nan))), fontsize=10)
    ax4 = fig.add_subplot(gs[1, 1])
    heat(ax4, an_adj, norm_a, "aim vs today's formula (deg)", "aa", np.arange(-10, 25, 2))
    ax4.set_title("AIM adjustment, top-down (contours every 2 deg; gray = no-shot zone)\nabsolute aim residual here: %.1f to %.1f deg" % (np.nanmin(np.where(use, resid, np.nan)), np.nanmax(np.where(use, resid, np.nan))), fontsize=10)

    leg = [Line2D([], [], marker="o", color="w", mfc="white", mec="k", ms=7, label="measured good spot (its own adjustment; black dots on the surfaces)"),
           Line2D([], [], marker="X", color="w", mfc="#d62728", mec="k", ms=8, label="measured NO SHOT spot"),
           Line2D([], [], marker="*", color="w", mfc="#d4a000", mec="k", ms=12, label="the cell"),
           Line2D([], [], marker="s", color="w", mfc="#888", ms=10, label="no-shot zone (still gives a speed and an aim; the screen warns)")]
    fig.legend(handles=leg, loc="lower center", ncol=4, fontsize=9, frameon=False, bbox_to_anchor=(0.5, 0.0))
    fig.suptitle("2-D launch map, %s%s: the adjustment of speed and aim away from today's method. Steepest change over the usable area: %.1f ticks/s per inch, %.2f deg per inch (mean %.1f ticks/s, %.2f deg per inch)" % (
        cell, " (SEEDED: mirrored from the measured cell)" if cell == "BLUE_AUDIENCE" else "", np.nanmax(grad_s[use]), np.nanmax(grad_a[use]), np.nanmean(grad_s[use]), np.nanmean(grad_a[use])), fontsize=11)
    out = os.path.join(OUT_DIR, "map_%s.png" % cell)
    fig.savefig(out, dpi=95)
    print(out)
    print("steepest speed adjustment change %.1f ticks/s per in, aim %.2f deg per in; adjustment ranges: speed %.0f to %.0f, aim %.1f to %.1f deg" % (
        np.nanmax(grad_s[use]), np.nanmax(grad_a[use]), np.nanmin(sp_adj[use]), np.nanmax(sp_adj[use]), np.nanmin(an_adj[use]), np.nanmax(an_adj[use])))


if __name__ == "__main__":
    main()
