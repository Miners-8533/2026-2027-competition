package org.firstinspires.ftc.teamcode.navigation;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.math.Vec;
import org.firstinspires.ftc.teamcode.robot.Footprint;

import java.util.ArrayList;
import java.util.List;

/**
 * The route planner (specs/dashes.md section 1): a list of waypoints from the robot's position to a goal that keeps the
 * robot clear of the hive supports and the flower stands. Pure logic; planned once, when a dash starts.
 */
@Config
public final class RoutePlanner {
    private RoutePlanner() { }

    public static double ROBOT_HEIGHT_IN = 18.0;
    public static double MARGIN_IN = 1.5;
    /** 3 in is the largest flower clearance that still leaves every launch spot reachable. */
    public static double FLOWER_CLEARANCE_IN = 3.0;
    public static double LANE_HALF_WIDTH_IN = 12.35;
    public static double ALIGN_LENGTH_IN = 40.0;
    public static double UNALIGNED_SPEED_RATIO = 0.75;
    public static double TURN_PENALTY_IN_PER_DEG = 0.1;
    public static boolean UNDER_HIVE_ALLOWED = true;
    public static int MAX_VIA = 3;
    /** SPEC GAP: "the walls are only checked from a short distance along a leg that starts at the robot"; the distance is not given. */
    public static double LANE_WALL_SKIP_IN = 12.0;

    enum Kind { SUPPORT, FLOWER, LANE_WALL }

    /** One obstacle: a convex polygon. */
    public static final class Obstacle {
        public final double[][] poly;
        final Kind kind;
        final Vec center;

        Obstacle(double[][] poly, Kind kind) {
            this.poly = poly;
            this.kind = kind;
            double sx = 0, sy = 0;
            for (double[] p : poly) {
                sx += p[0];
                sy += p[1];
            }
            this.center = new Vec(sx / poly.length, sy / poly.length);
        }
    }

    /** The hive's two foot bars and four legs, for a robot of the planner's height. */
    public static List<Obstacle> supports() {
        List<Obstacle> out = new ArrayList<>();
        for (int sy = -1; sy <= 1; sy += 2) {
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, Math.min(sy * Field.BAR_Y_NEAR, sy * Field.BAR_Y_FAR),
                    Field.BAR_HALF_X, Math.max(sy * Field.BAR_Y_NEAR, sy * Field.BAR_Y_FAR)), Kind.SUPPORT));
            for (int sx = -1; sx <= 1; sx += 2) {
                double[] top = Field.legAtHeight(ROBOT_HEIGHT_IN);
                out.add(new Obstacle(Geom.band(sx * Field.LEG_FOOT_X, sy * Field.LEG_FOOT_Y, sx * top[0], sy * top[1],
                        Field.LEG_THICKNESS / 2), Kind.SUPPORT));
            }
        }
        return out;
    }

    /** Everything a route must clear: the supports, the flower stands grown by a clearance, and the virtual lane walls. */
    public static List<Obstacle> obstacles() {
        List<Obstacle> out = supports();
        for (int i = 0; i < 4; i++) {
            double[] r = Field.flowerRect(i);
            double c = FLOWER_CLEARANCE_IN;
            out.add(new Obstacle(Geom.rect(r[0] - c, r[1] - c, r[2] + c, r[3] + c), Kind.FLOWER));
        }
        if (UNDER_HIVE_ALLOWED) {
            // Walls that keep a route under the hive centered on the lane's middle line.
            double w = LANE_HALF_WIDTH_IN;
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, w, Field.BAR_HALF_X, w + 0.1), Kind.LANE_WALL));
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, -w - 0.1, Field.BAR_HALF_X, -w), Kind.LANE_WALL));
        } else {
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, -Field.BAR_Y_NEAR, Field.BAR_HALF_X, Field.BAR_Y_NEAR), Kind.LANE_WALL));
        }
        return out;
    }

    /** Whether the robot's own footprint already touches a support. A dash is refused in that case. */
    public static boolean touchesSupport(Pose pose, Footprint fp) {
        double[][] body = fp.corners(pose.x, pose.y, pose.h);
        for (Obstacle o : supports()) {
            if (Geom.polyPoly(body, o.poly) <= 0) return true;
        }
        return false;
    }

    /**
     * Whether a straight leg is acceptable: the area the robot sweeps along it, grown by the margin, touches no obstacle.
     *
     * @param first        the leg starts where the robot is now
     * @param toLaunchSpot the leg ends at a launch spot, which is judged with the far-corner circle only
     */
    static boolean legOk(Vec a, Vec b, boolean first, boolean toLaunchSpot, Footprint fp, List<Obstacle> obstacles) {
        double len = a.dist(b);
        if (len < 1e-6) return true;
        Vec u = b.minus(a).unit();
        boolean aligned = len >= ALIGN_LENGTH_IN;
        double corner = fp.farCorner();
        for (Obstacle o : obstacles) {
            double[][] poly = o.poly;
            if (first) {
                // A robot that starts within reach of a flower and moves away from it ignores that flower (it has just gathered there).
                if (o.kind == Kind.FLOWER && Geom.polyPoint(poly, a.x, a.y) < corner + MARGIN_IN && o.center.minus(a).dot(u) < 0) continue;
                // The robot's own circle at the start, without margin. The lane walls and the flowers are not solid there.
                if (o.kind == Kind.SUPPORT && Geom.polyPoint(o.poly, a.x, a.y) < corner) return false;
                // Behind the start the robot is only moving away. A robot inside the lane may always leave it.
                double skip = o.kind == Kind.LANE_WALL ? LANE_WALL_SKIP_IN : 0;
                poly = Geom.clip(poly, a.x + u.x * skip, a.y + u.y * skip, u.x, u.y);
                if (poly.length == 0) continue;
            }
            // The sweep. Near a launch spot it is checked without the margin (the audience spot is close to a flower stand).
            double[][] far = poly;
            if (toLaunchSpot) {
                double back = corner + MARGIN_IN;
                far = Geom.clip(poly, b.x - u.x * back, b.y - u.y * back, -u.x, -u.y);
                if (!sweepClear(poly, a, b, aligned, fp, 0)) return false;
            }
            if (far.length > 0 && !sweepClear(far, a, b, aligned, fp, MARGIN_IN)) return false;
            // Where the heading changes, the robot is a circle of its far-corner radius.
            if (Geom.polyPoint(poly, b.x, b.y) < corner + (toLaunchSpot ? 0 : MARGIN_IN)) return false;
        }
        return true;
    }

    private static boolean sweepClear(double[][] poly, Vec a, Vec b, boolean aligned, Footprint fp, double margin) {
        if (aligned) {
            // A long leg is driven aligned, front or rear leading: the robot's real rectangle moved along the leg.
            return Geom.polyPoly(Geom.band(a.x, a.y, b.x, b.y, fp.halfWidth), poly) >= margin;
        }
        // A short leg is not aligned: the far-corner radius.
        return Geom.polySeg(poly, a.x, a.y, b.x, b.y) >= fp.farCorner() + margin;
    }

    private static double turnFolded(Vec a, Vec b, Vec c) {
        double d = Math.toDegrees(c.minus(b).angle() - b.minus(a).angle());
        d = Math.abs(((d + 540) % 360) - 180);      // 0 to 180
        return d > 90 ? 180 - d : d;                // a leg can be driven front or rear first
    }

    private static double cost(Vec[] pts) {
        double c = 0;
        for (int i = 0; i + 1 < pts.length; i++) {
            double len = pts[i].dist(pts[i + 1]);
            boolean aligned = len >= ALIGN_LENGTH_IN;
            c += aligned ? len : len / UNALIGNED_SPEED_RATIO;
            if (aligned && i + 2 < pts.length && pts[i + 1].dist(pts[i + 2]) >= ALIGN_LENGTH_IN) {
                c += TURN_PENALTY_IN_PER_DEG * turnFolded(pts[i], pts[i + 1], pts[i + 2]);
            }
        }
        return c;
    }

    private static Plan make(Vec[] pts) {
        boolean[] aligned = new boolean[pts.length - 1];
        for (int i = 0; i < aligned.length; i++) aligned[i] = pts[i].dist(pts[i + 1]) >= ALIGN_LENGTH_IN;
        return new Plan(pts, aligned, cost(pts));
    }

    /** Plans from the robot's position to a goal. */
    public static Plan plan(Vec start, Vec goal, boolean goalIsLaunchSpot, Footprint fp) {
        List<Obstacle> obs = obstacles();
        if (legOk(start, goal, true, goalIsLaunchSpot, fp, obs)) return make(new Vec[] {start, goal});

        Vec[] via = Field.routePoints();
        int n = via.length;
        boolean[] fromStart = new boolean[n], toGoal = new boolean[n];
        boolean[][] between = new boolean[n][n];
        for (int i = 0; i < n; i++) {
            fromStart[i] = legOk(start, via[i], true, false, fp, obs);
            toGoal[i] = legOk(via[i], goal, false, goalIsLaunchSpot, fp, obs);
            for (int j = 0; j < n; j++) between[i][j] = i != j && legOk(via[i], via[j], false, false, fp, obs);
        }
        Vec[] best = null;
        double bestCost = Double.MAX_VALUE;
        int[] chain = new int[Math.max(1, MAX_VIA)];
        // Chains of at most MAX_VIA route points over acceptable legs; take the cheapest.
        List<int[]> stack = new ArrayList<>();
        for (int i = 0; i < n; i++) if (fromStart[i]) stack.add(new int[] {i});
        while (!stack.isEmpty()) {
            int[] c = stack.remove(stack.size() - 1);
            int last = c[c.length - 1];
            if (toGoal[last]) {
                Vec[] pts = new Vec[c.length + 2];
                pts[0] = start;
                for (int k = 0; k < c.length; k++) pts[k + 1] = via[c[k]];
                pts[pts.length - 1] = goal;
                double cst = cost(pts);
                if (cst < bestCost) {
                    bestCost = cst;
                    best = pts;
                }
            }
            if (c.length >= chain.length) continue;
            for (int j = 0; j < n; j++) {
                if (!between[last][j]) continue;
                boolean used = false;
                for (int k : c) used |= k == j;
                if (used) continue;
                int[] next = new int[c.length + 1];
                System.arraycopy(c, 0, next, 0, c.length);
                next[c.length] = j;
                stack.add(next);
            }
        }
        if (best == null) return new Plan("no clear route from " + start + " to " + goal);
        return make(best);
    }
}
