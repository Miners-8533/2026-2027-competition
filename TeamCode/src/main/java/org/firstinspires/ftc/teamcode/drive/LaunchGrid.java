package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.subsystems.HiveTarget;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * The grid of launching spots for ONE hive cell, for the Launch Grid Tuner (2026-11-10). Pure geometry: no hardware, it compiles on a PC
 * (tools/hive/LaunchGridCheck.java prints and checks the grids).
 *
 * A CELL opens toward one wall only (the audience cell toward the audience wall, the rear cell toward the rear wall), so a shot has to come from the
 * open side: a lattice point is kept only when
 *  - it is on the open side of the cell's opening face (x beyond the face, signed by the side),
 *  - it is at least minDistanceIn from the cell (TargetLock.MIN_DISTANCE_IN, 12 in: closer than that the bearing is meaningless),
 *  - the shot is within angleLimitDeg of the opening's normal (a shot nearly parallel to the face cannot enter it),
 *  - the robot, at its AIM HEADING there (the launcher pointed at the cell), does not overlap a foot bar, an A-frame leg or a flower stand (HiveRoute's CAD
 *    keep-outs, the robot's rectangle grown by the planner's margin), and fits inside the walls.
 * There is no maximum distance: the field and the open side bound the grid. A point that fails ONLY on a wall (turned to face the cell the robot's corner
 * would cross it) is slid toward the field center until it fits, when that is no more than MAX_SLIDE_IN, so the wall area (where the start and most launching
 * happen) is still covered.
 *
 * The lattice is centered on the field: the points are multiples of pitchIn from the center out to the last one that is inside 64 in (a 12 in pitch gives
 * -60 to 60).
 */
public final class LaunchGrid {

    /** The aim heading, radians, that points the launcher at the cell from (x, y). */
    public interface Aim {
        double headingRad(double x, double y);
    }

    public enum Verdict { VALID, WRONG_SIDE, TOO_CLOSE, COLLISION, WALL, ANGLE }

    public static final double MAX_SLIDE_IN = 8.0;
    /** The lattice runs out to this distance from the field center (a robot's center cannot be farther than the walls less its back). */
    public static final double LATTICE_EDGE_IN = 64.0;

    /** One launching spot. */
    public static final class Point {
        /** Where the robot stands (the lattice point, or the spot it was slid to off a wall). */
        public final double x, y;
        public final double latticeX, latticeY;
        /** The aim heading there before any trim, radians. */
        public final double headingRad;
        public final double distanceIn;
        public final boolean slid;

        Point(double x, double y, double latticeX, double latticeY, double headingRad, double distanceIn, boolean slid) {
            this.x = x;
            this.y = y;
            this.latticeX = latticeX;
            this.latticeY = latticeY;
            this.headingRad = headingRad;
            this.distanceIn = distanceIn;
            this.slid = slid;
        }
    }

    private LaunchGrid() {}

    /** The numbers in a list like "8-10,22,25-27" (null when blank or nothing in it is a valid number); problems go in note. */
    public static java.util.Set<Integer> parseSpotList(String text, int max, StringBuilder note) {
        if (text == null || text.trim().isEmpty()) return null;
        java.util.Set<Integer> out = new java.util.TreeSet<>();
        for (String tok : text.split(",")) {
            String t = tok.trim();
            if (t.isEmpty()) continue;
            try {
                int a, b;
                if (t.contains("-")) {
                    String[] ab = t.split("-");
                    a = Integer.parseInt(ab[0].trim());
                    b = Integer.parseInt(ab[1].trim());
                } else {
                    a = b = Integer.parseInt(t);
                }
                for (int k = Math.min(a, b); k <= Math.max(a, b); k++) {
                    if (k >= 1 && k <= max) out.add(k);
                    else note.append("spot ").append(k).append(" is not in the grid (1 to ").append(max).append("); ");
                }
            } catch (RuntimeException e) {
                note.append("cannot read \"").append(t).append("\" in RETEST_SPOTS; ");
            }
        }
        if (out.isEmpty()) {
            note.append("RETEST_SPOTS has no valid spot: visiting all; ");
            return null;
        }
        return out;
    }

    /** A spot at a chosen position (the Launch Grid Tuner's EXTRA_SPOTS): the aim heading there and the distance to the cell; the caller checks it with {@link #check} first. */
    public static Point spotAt(double x, double y, HiveTarget cell, Aim aim) {
        return new Point(x, y, x, y, aim.headingRad(x, y), Math.hypot(x - cell.x, y - cell.y), false);
    }

    /** The side the cell opens toward: +1 audience (x), -1 rear. */
    public static int openSide(HiveTarget cell) {
        return cell.x > 0 ? 1 : -1;
    }

    /** The largest distance, inches, any corner of the robot's rectangle at this pose is beyond the wall (0 when it fits). */
    static double wallOver(double x, double y, double heading, HiveRoute.Params p, double halfIn) {
        double[] c = HiveRoute.poseRect(x, y, heading, p, 0.0).corners();
        double over = 0.0;
        for (double v : c) over = Math.max(over, Math.abs(v) - halfIn);
        return over;
    }

    /** What a spot is, ignoring the wall (WALL is reported when only the wall fails). */
    public static Verdict check(double x, double y, HiveTarget cell, double angleLimitDeg, double minDistanceIn, double halfIn, Aim aim, HiveRoute.Params p) {
        int side = openSide(cell);
        double dx = x - cell.x, dy = y - cell.y;
        double along = dx * side;
        double dist = Math.hypot(dx, dy);
        if (along <= 0.0) return Verdict.WRONG_SIDE;
        if (dist < minDistanceIn) return Verdict.TOO_CLOSE;
        double heading = aim.headingRad(x, y);
        HiveRoute.Rect me = HiveRoute.poseRect(x, y, heading, p, p.marginIn);
        for (HiveRoute.Rect r : HiveRoute.keepOuts(p, false)) if (HiveRoute.overlap(me, r)) return Verdict.COLLISION;
        for (HiveRoute.Rect r : HiveRoute.flowerKeepOuts()) if (HiveRoute.overlap(me, r)) return Verdict.COLLISION;
        double off = Math.toDegrees(Math.acos(Math.max(-1.0, Math.min(1.0, along / dist))));
        if (off > angleLimitDeg) return Verdict.ANGLE;
        if (wallOver(x, y, heading, p, halfIn) > 1e-9) return Verdict.WALL;
        return Verdict.VALID;
    }

    /**
     * Whether the robot can drive a straight line from one pose to the next, turning steadily on the way, without its rectangle (grown by the planner's margin)
     * touching a foot bar, an A-frame leg or a flower stand, or crossing a wall. The router (HiveRoute.plan) keeps the planner's whole far-corner circle clear of a flower stand,
     * which refuses a spot beside a stand that the robot, aimed at the cell, fits at; this is the check for the short straight hop the Launch Grid Tuner then drives instead.
     */
    public static boolean straightHopClear(double ax, double ay, double ah, double bx, double by, double bh, double halfIn, HiveRoute.Params p) {
        double len = Math.hypot(bx - ax, by - ay);
        double turn = Math.atan2(Math.sin(bh - ah), Math.cos(bh - ah));
        int n = Math.max(1, (int) Math.ceil(len / 0.5));
        List<HiveRoute.Rect> fixed = new ArrayList<>(HiveRoute.keepOuts(p, false));
        fixed.addAll(HiveRoute.flowerKeepOuts());
        for (int i = 0; i <= n; i++) {
            double t = (double) i / n;
            double x = ax + (bx - ax) * t, y = ay + (by - ay) * t, h = ah + turn * t;
            HiveRoute.Rect me = HiveRoute.poseRect(x, y, h, p, p.marginIn);
            for (HiveRoute.Rect r : fixed) if (HiveRoute.overlap(me, r)) return false;
            if (wallOver(x, y, h, p, halfIn) > 0.5) return false;
        }
        return true;
    }

    /** All the spots for a cell, in no particular order (see {@link #order}). */
    public static List<Point> generate(HiveTarget cell, double pitchIn, double angleLimitDeg, double minDistanceIn, double halfIn, Aim aim, HiveRoute.Params p) {
        List<Point> out = new ArrayList<>();
        double lim = pitchIn * Math.floor(LATTICE_EDGE_IN / pitchIn);
        for (double x = -lim; x <= lim + 1e-9; x += pitchIn) {
            for (double y = -lim; y <= lim + 1e-9; y += pitchIn) {
                Verdict v = check(x, y, cell, angleLimitDeg, minDistanceIn, halfIn, aim, p);
                if (v == Verdict.VALID) {
                    out.add(new Point(x, y, x, y, aim.headingRad(x, y), Math.hypot(x - cell.x, y - cell.y), false));
                } else if (v == Verdict.WALL) {
                    Point q = slideIn(x, y, cell, angleLimitDeg, minDistanceIn, halfIn, aim, p);
                    if (q != null) out.add(q);
                }
            }
        }
        return out;
    }

    /** Slides a spot that fails only on a wall toward the field center until it fits; null when it still fails or would move more than MAX_SLIDE_IN. */
    private static Point slideIn(double px, double py, HiveTarget cell, double angleLimitDeg, double minDistanceIn, double halfIn, Aim aim, HiveRoute.Params p) {
        double x = px, y = py;
        boolean fits = false;
        for (int i = 0; i < 40 && !fits; i++) {
            double h = aim.headingRad(x, y);
            double[] c = HiveRoute.poseRect(x, y, h, p, 0.0).corners();
            double ox = 0.0, oy = 0.0;
            for (int k = 0; k < c.length; k += 2) {
                ox = Math.max(ox, Math.abs(c[k]) - halfIn);
                oy = Math.max(oy, Math.abs(c[k + 1]) - halfIn);
            }
            if (ox <= 1e-9 && oy <= 1e-9) {
                fits = true;
                break;
            }
            if (ox > 0.0) x -= Math.copySign(ox + 0.05, x);
            if (oy > 0.0) y -= Math.copySign(oy + 0.05, y);
        }
        if (!fits || Math.hypot(x - px, y - py) > MAX_SLIDE_IN) return null;
        if (check(x, y, cell, angleLimitDeg, minDistanceIn, halfIn, aim, p) != Verdict.VALID) return null;
        return new Point(x, y, px, py, aim.headingRad(x, y), Math.hypot(x - cell.x, y - cell.y), true);
    }

    /**
     * Puts the spots in visiting order: a serpentine over the columns (x), the column nearest the start first, and inside each column the direction that
     * starts nearest the previous spot, so each hop is short and the robot sweeps the field instead of crossing it.
     */
    public static void order(List<Point> pts, double startX, double startY) {
        List<Double> cols = new ArrayList<>();
        for (Point p : pts) {
            boolean seen = false;
            for (double c : cols) if (Math.abs(c - p.latticeX) < 1e-6) seen = true;
            if (!seen) cols.add(p.latticeX);
        }
        Collections.sort(cols, Comparator.comparingDouble(c -> Math.abs(c - startX)));
        List<Point> out = new ArrayList<>();
        double lastY = startY;
        for (double c : cols) {
            List<Point> col = new ArrayList<>();
            for (Point p : pts) if (Math.abs(p.latticeX - c) < 1e-6) col.add(p);
            Collections.sort(col, Comparator.comparingDouble(p -> p.y));
            if (!col.isEmpty() && Math.abs(col.get(col.size() - 1).y - lastY) < Math.abs(col.get(0).y - lastY)) Collections.reverse(col);
            out.addAll(col);
            if (!col.isEmpty()) lastY = col.get(col.size() - 1).y;
        }
        pts.clear();
        pts.addAll(out);
    }
}
