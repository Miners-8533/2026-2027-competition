package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Route planning around the HIVE's supports for Hive Dash (design/hive-dash.md). Pure geometry: no hardware and no SDK classes, so tools/hive/HiveRouteCheck.java
 * runs it on a PC. Field frame: origin at the field center, +X toward the audience wall, +Y toward the Blue Wall, inches, radians.
 *
 * KEEP-OUTS (from the field CAD, parts am-5878 foot bar and am-5876 A-frame leg; CAD X = our Y, CAD Z = our X; symmetric in sign):
 *  - two foot bars, x -19.47 to +19.47, |y| 22.75 to 24.73, 2.15 in tall (the chassis cannot cross them);
 *  - four A-frame legs that lean inward both ways: for a robot of height H the leg is the thick line from its foot (|x| 18.56, |y| 23.81 at 0.53 in) to its point at height H
 *    (|x| = 18.56 - 0.445 (H - 0.53), |y| = 23.81 - 0.273 (H - 0.53); at 18 in: 10.78, 19.04), 1.14 in thick.
 *  - when under-hive routes are off, a virtual box between the bars (x +-19.47, |y| up to 22.75) so no route crosses the hive; a robot that STARTS inside it may leave.
 *
 * ROUTE: a straight line when it is clear, otherwise the cheapest chain (at most MAX_VIA points) through the given points (the helper corners and the lane gates), over legs
 * that are clear. A leg is the robot's real rectangle swept along it, ALIGNED to the leg (front or rear leading), when the leg is at least alignMinLegIn long; shorter legs
 * are not aligned, so they are swept with the far-corner circle. The cost is the length (an unaligned leg counts length / unalignedSpeedRatio, because it is driven slower)
 * plus turnPenaltyInPerDeg for each degree turned between aligned legs (a leg can be driven front or rear first, so the turn is folded to 0 to 90 degrees).
 */
public final class HiveRoute {

    public static final double BAR_HALF_LENGTH_IN = 19.47;
    public static final double BAR_INNER_Y_IN = 22.75;
    public static final double BAR_OUTER_Y_IN = 24.73;
    public static final double LEG_FOOT_X_IN = 18.56;
    public static final double LEG_FOOT_Y_IN = 23.81;
    public static final double LEG_FOOT_HEIGHT_IN = 0.53;
    public static final double LEG_DX_PER_IN = 0.445;
    public static final double LEG_DY_PER_IN = 0.273;
    public static final double LEG_HALF_THICK_IN = 0.57;

    public static final int MAX_VIA = 3;

    private HiveRoute() {}

    /** Everything the planner needs besides the poses. */
    public static final class Params {
        public double robotHeightIn = 18.0;
        public double marginIn = 1.5;
        public double depthIn = 17.48, widthIn = 13.70, fromBackIn = 7.598;
        public double alignMinLegIn = 40.0;
        public double turnPenaltyInPerDeg = 0.1;
        /** Speed of a leg that is not driven aligned (short legs, so they strafe or go diagonal) as a fraction of an aligned one: measured 0.72 to 0.75 from the TeleOp logs of 2026-11-01 to 11-03. */
        public double unalignedSpeedRatio = 0.75;
        public boolean underHive = true;
        /** Route points (helper corners and lane gates), field frame, every sign already expanded. */
        public final List<double[]> points = new ArrayList<>();

        public double frontIn() {
            return depthIn - fromBackIn;
        }

        /** Origin to the farthest corner. */
        public double radiusIn() {
            return Math.hypot(Math.max(frontIn(), fromBackIn), widthIn / 2.0);
        }
    }

    /** An oriented rectangle: center, half sizes along its own x and y axes, and the angle of its x axis. */
    public static final class Rect {
        public final double cx, cy, hx, hy, ang;
        public final boolean lane;   // the virtual box between the bars

        Rect(double cx, double cy, double hx, double hy, double ang, boolean lane) {
            this.cx = cx;
            this.cy = cy;
            this.hx = hx;
            this.hy = hy;
            this.ang = ang;
            this.lane = lane;
        }

        /** Corners (x0, y0, x1, y1, ...): (+,+), (-,+), (-,-), (+,-) in the rectangle's own axes. */
        public double[] corners() {
            double ux = Math.cos(ang), uy = Math.sin(ang), vx = -uy, vy = ux;
            return new double[]{
                    cx + ux * hx + vx * hy, cy + uy * hx + vy * hy,
                    cx - ux * hx + vx * hy, cy - uy * hx + vy * hy,
                    cx - ux * hx - vx * hy, cy - uy * hx - vy * hy,
                    cx + ux * hx - vx * hy, cy + uy * hx - vy * hy};
        }

        /** Distance from a point to the rectangle (0 inside). */
        public double distanceTo(double px, double py) {
            double ux = Math.cos(ang), uy = Math.sin(ang);
            double lx = (px - cx) * ux + (py - cy) * uy;
            double ly = -(px - cx) * uy + (py - cy) * ux;
            return Math.hypot(Math.max(Math.abs(lx) - hx, 0.0), Math.max(Math.abs(ly) - hy, 0.0));
        }

        boolean contains(double px, double py) {
            return distanceTo(px, py) == 0.0;
        }
    }

    /** A planned route, or a refusal. */
    public static final class Plan {
        public final boolean ok;
        public final String refusal;
        /** start, via points, goal: field frame x, y. */
        public final double[][] waypoints;
        /** Per leg: true when the robot drives it aligned (front or rear leading). */
        public final boolean[] aligned;
        public final double lengthIn;
        public final String description;

        Plan(boolean ok, String refusal, double[][] waypoints, boolean[] aligned, double lengthIn, String description) {
            this.ok = ok;
            this.refusal = refusal;
            this.waypoints = waypoints;
            this.aligned = aligned;
            this.lengthIn = lengthIn;
            this.description = description;
        }

        static Plan refused(String why) {
            return new Plan(false, why, new double[0][], new boolean[0], 0.0, "refused: " + why);
        }
    }

    // ---- keep-outs ----

    /** The supports for a robot of the params' height; with lane = true also the virtual box between the bars. */
    public static List<Rect> keepOuts(Params p, boolean lane) {
        List<Rect> out = new ArrayList<>();
        double barCenterY = (BAR_INNER_Y_IN + BAR_OUTER_Y_IN) / 2.0, barHalfY = (BAR_OUTER_Y_IN - BAR_INNER_Y_IN) / 2.0;
        double up = Math.max(0.0, p.robotHeightIn - LEG_FOOT_HEIGHT_IN);
        double topX = LEG_FOOT_X_IN - LEG_DX_PER_IN * up, topY = LEG_FOOT_Y_IN - LEG_DY_PER_IN * up;
        for (int sy = -1; sy <= 1; sy += 2) {
            out.add(new Rect(0.0, sy * barCenterY, BAR_HALF_LENGTH_IN, barHalfY, 0.0, false));
            for (int sx = -1; sx <= 1; sx += 2) {
                double x0 = sx * LEG_FOOT_X_IN, y0 = sy * LEG_FOOT_Y_IN, x1 = sx * topX, y1 = sy * topY;
                double len = Math.hypot(x1 - x0, y1 - y0);
                if (len < 1e-6) continue;
                out.add(new Rect((x0 + x1) / 2.0, (y0 + y1) / 2.0, len / 2.0 + LEG_HALF_THICK_IN, LEG_HALF_THICK_IN, Math.atan2(y1 - y0, x1 - x0), false));
            }
        }
        if (lane) out.add(new Rect(0.0, 0.0, BAR_HALF_LENGTH_IN, BAR_INNER_Y_IN, 0.0, true));
        return out;
    }

    /** The robot's rectangle at a pose, grown by grow on every side. */
    public static Rect poseRect(double x, double y, double heading, Params p, double grow) {
        double mid = (p.frontIn() - p.fromBackIn) / 2.0;
        return new Rect(x + Math.cos(heading) * mid, y + Math.sin(heading) * mid, p.depthIn / 2.0 + grow, p.widthIn / 2.0 + grow, heading, false);
    }

    /** The rectangle swept by a leg from A to B: ext past each end along the leg, halfWidth either side. */
    static Rect swept(double ax, double ay, double bx, double by, double ext, double halfWidth) {
        return swept(ax, ay, bx, by, ext, ext, halfWidth);
    }

    /** As above with a different extension behind A (extA) and past B (extB). */
    static Rect swept(double ax, double ay, double bx, double by, double extA, double extB, double halfWidth) {
        double len = Math.hypot(bx - ax, by - ay);
        double dir = Math.atan2(by - ay, bx - ax);
        double mid = (len + extB - extA) / 2.0;   // the middle of the extended rectangle, measured from A along the leg
        return new Rect(ax + Math.cos(dir) * mid, ay + Math.sin(dir) * mid, (len + extA + extB) / 2.0, halfWidth, dir, false);
    }

    /** Separating-axis test for two oriented rectangles (touching counts as overlapping). */
    public static boolean overlap(Rect a, Rect b) {
        double[] ca = a.corners(), cb = b.corners();
        double[][] axes = {{Math.cos(a.ang), Math.sin(a.ang)}, {-Math.sin(a.ang), Math.cos(a.ang)},
                {Math.cos(b.ang), Math.sin(b.ang)}, {-Math.sin(b.ang), Math.cos(b.ang)}};
        for (double[] ax : axes) {
            double minA = Double.MAX_VALUE, maxA = -Double.MAX_VALUE, minB = Double.MAX_VALUE, maxB = -Double.MAX_VALUE;
            for (int i = 0; i < 8; i += 2) {
                double pa = ca[i] * ax[0] + ca[i + 1] * ax[1], pb = cb[i] * ax[0] + cb[i + 1] * ax[1];
                minA = Math.min(minA, pa);
                maxA = Math.max(maxA, pa);
                minB = Math.min(minB, pb);
                maxB = Math.max(maxB, pb);
            }
            if (maxA < minB || maxB < minA) return false;
        }
        return true;
    }

    // ---- planning ----

    /** True when the robot's footprint (no margin) overlaps a real support at this pose. */
    public static boolean touchingSupport(double x, double y, double heading, Params p) {
        Rect me = poseRect(x, y, heading, p, 0.0);
        for (Rect r : keepOuts(p, false)) if (overlap(me, r)) return true;
        return false;
    }

    private static boolean legClear(double ax, double ay, double bx, double by, Params p, List<Rect> rects, boolean exemptLane, boolean fromStart, boolean[] alignedOut) {
        double len = Math.hypot(bx - ax, by - ay);
        boolean aligned = len >= p.alignMinLegIn;
        if (alignedOut != null) alignedOut[0] = aligned;
        double ext = (aligned ? Math.max(p.frontIn(), p.fromBackIn) : p.radiusIn()) + p.marginIn;
        double halfWidth = (aligned ? p.widthIn / 2.0 : p.radiusIn()) + p.marginIn;
        // behind the START of the route the robot is only moving away, and its footprint there was already checked at the start (the robot is not touching a support). A short
        // leg's box is a square of half-width radius + margin, whose corner reached about 19 in from the origin and caught a support bar 14 in away (2026-11-05: a Park Dash 22 in from
        // the corner drove 126 in via a helper point). So a short leg from the start is boxed from the start point onward, and the robot's own circle at the start (radius, no margin: it may
        // turn there) is checked on its own.
        boolean shortFromStart = fromStart && !aligned;
        Rect sw = swept(ax, ay, bx, by, shortFromStart ? 0.0 : ext, ext, halfWidth);
        for (Rect r : rects) {
            if (r.lane && exemptLane) continue;
            if (shortFromStart && r.distanceTo(ax, ay) <= p.radiusIn()) return false;
            if (overlap(sw, r)) return false;
        }
        return true;
    }

    private static double foldDeg(double a, double b) {
        double d = Math.abs(Math.toDegrees(normalize(a - b)));
        return Math.min(d, 180.0 - d);
    }

    private static double normalize(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    /** Plans a route from the robot's pose to a goal. */
    public static Plan plan(double sx, double sy, double sHeading, double gx, double gy, Params p) {
        if (touchingSupport(sx, sy, sHeading, p)) return Plan.refused("the robot is touching a hive support: drive clear of it by hand");
        List<Rect> rects = keepOuts(p, !p.underHive);
        boolean startInLane = false;
        if (!p.underHive) for (Rect r : rects) if (r.lane && r.contains(sx, sy)) startInLane = true;

        // nodes: 0 start, 1 goal, 2.. route points that are not inside a keep-out (the far-corner circle plus the margin)
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{sx, sy});
        nodes.add(new double[]{gx, gy});
        for (double[] pt : p.points) {
            boolean ok = true;
            for (Rect r : rects) if (r.distanceTo(pt[0], pt[1]) <= p.radiusIn() + p.marginIn) ok = false;
            if (ok) nodes.add(pt);
        }
        int n = nodes.size();
        boolean[][] clear = new boolean[n][n];
        boolean[][] aligned = new boolean[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                boolean[] al = new boolean[1];
                boolean c = legClear(nodes.get(i)[0], nodes.get(i)[1], nodes.get(j)[0], nodes.get(j)[1], p, rects, startInLane && i == 0, i == 0, al);
                clear[i][j] = clear[j][i] = c;
                aligned[i][j] = aligned[j][i] = al[0];
            }
        }

        List<int[]> best = new ArrayList<>();
        double[] bestCost = {Double.MAX_VALUE};
        search(new int[MAX_VIA + 2], 1, 0, nodes, clear, aligned, sHeading, p, best, bestCost, new boolean[n]);
        if (best.isEmpty()) return Plan.refused("no clear route to the goal (a hive support is in the way)");

        int[] path = best.get(0);
        double[][] wp = new double[path.length][];
        boolean[] al = new boolean[path.length - 1];
        double length = 0.0;
        StringBuilder via = new StringBuilder();
        for (int k = 0; k < path.length; k++) {
            wp[k] = nodes.get(path[k]);
            if (k > 0) {
                al[k - 1] = aligned[path[k - 1]][path[k]];
                length += Math.hypot(wp[k][0] - wp[k - 1][0], wp[k][1] - wp[k - 1][1]);
            }
            if (k > 0 && k < path.length - 1) via.append(String.format(Locale.US, " (%.0f, %.0f)", wp[k][0], wp[k][1]));
        }
        String desc = (path.length == 2 ? "direct" : "via" + via) + String.format(Locale.US, ", %.0f in", length);
        return new Plan(true, null, wp, al, length, desc);
    }

    /** Depth-first over the chains of via points (distinct, at most MAX_VIA), keeping the cheapest complete path. */
    private static void search(int[] path, int len, int depth, List<double[]> nodes, boolean[][] clear, boolean[][] aligned, double sHeading, Params p,
                               List<int[]> best, double[] bestCost, boolean[] used) {
        // try to finish: last node -> goal (node 1)
        int last = path[len - 1];
        if (clear[last][1]) {
            int[] full = new int[len + 1];
            System.arraycopy(path, 0, full, 0, len);
            full[len] = 1;
            double cost = cost(full, nodes, aligned, sHeading, p);
            if (cost < bestCost[0]) {
                bestCost[0] = cost;
                best.clear();
                best.add(full);
            }
        }
        if (depth >= MAX_VIA) return;
        for (int j = 2; j < nodes.size(); j++) {
            if (used[j] || !clear[last][j]) continue;
            used[j] = true;
            path[len] = j;
            search(path, len + 1, depth + 1, nodes, clear, aligned, sHeading, p, best, bestCost, used);
            used[j] = false;
        }
    }

    private static double cost(int[] path, List<double[]> nodes, boolean[][] aligned, double sHeading, Params p) {
        double total = 0.0, prevDir = Double.NaN;
        for (int k = 1; k < path.length; k++) {
            double[] a = nodes.get(path[k - 1]), b = nodes.get(path[k]);
            double legLen = Math.hypot(b[0] - a[0], b[1] - a[1]);
            if (!aligned[path[k - 1]][path[k]]) {
                total += legLen / Math.max(0.1, p.unalignedSpeedRatio);
                prevDir = Double.NaN;
                continue;
            }
            total += legLen;
            double dir = Math.atan2(b[1] - a[1], b[0] - a[0]);
            total += p.turnPenaltyInPerDeg * foldDeg(Double.isNaN(prevDir) ? (k == 1 ? sHeading : dir) : prevDir, dir);
            prevDir = dir;
        }
        return total;
    }
}
