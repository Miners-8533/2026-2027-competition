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
    /** A leg from the robot checks the virtual lane walls only from this far along the leg onward. */
    public static double LANE_WALL_SKIP_IN = 10.0;
    /** Two shapes closer than this touch. */
    private static final double TOUCH_EPS = 1e-9;

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
            // Walls that keep a route under the hive centered on the lane's middle line: each fills the space from
            // the lane half width out to the inner edge of a bar, along the bars' length.
            double w = LANE_HALF_WIDTH_IN;
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, w, Field.BAR_HALF_X, Field.BAR_Y_NEAR), Kind.LANE_WALL));
            out.add(new Obstacle(Geom.rect(-Field.BAR_HALF_X, -Field.BAR_Y_NEAR, Field.BAR_HALF_X, -w), Kind.LANE_WALL));
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
     * Whether the robot's footprint at a pose stays at least a margin from every hive support and every flower stand
     * (grown by the planner's clearance). The virtual lane walls are not obstacles for a standing robot.
     * Used by the launch grid's spot generator (specs/aiming.md section 4.4).
     */
    public static boolean bodyClear(Pose pose, Footprint fp, double margin) {
        double[][] body = fp.corners(pose.x, pose.y, pose.h);
        for (Obstacle o : obstacles()) {
            if (o.kind != Kind.LANE_WALL && Geom.polyPoly(body, o.poly) < margin) return false;
        }
        return true;
    }

    /**
     * Whether the robot's rectangle at a pose, grown by a margin, touches no hive support and no flower stand. The stands
     * are taken as they stand, not grown by the planner's clearance. Used for the Launch Grid Tuner's short straight hop
     * (specs/support-opmodes.md section 2.2).
     */
    public static boolean standsClear(Pose pose, Footprint fp, double margin) {
        double[][] body = fp.corners(pose.x, pose.y, pose.h);
        for (Obstacle o : supports()) {
            if (Geom.polyPoly(body, o.poly) <= margin) return false;
        }
        for (int i = 0; i < 4; i++) {
            double[] r = Field.flowerRect(i);
            if (Geom.polyPoly(body, Geom.rect(r[0], r[1], r[2], r[3])) <= margin) return false;
        }
        return true;
    }

    /** The rectangle laid along a leg from a to b: it starts "behind" before a, ends "ahead" past b, and reaches halfWidth to each side. */
    private static double[][] swept(Vec a, Vec u, double len, double behind, double ahead, double halfWidth) {
        return Geom.band(a.x - u.x * behind, a.y - u.y * behind, a.x + u.x * (len + ahead), a.y + u.y * (len + ahead), halfWidth);
    }

    private static boolean touches(double[][] p, double[][] q) { return Geom.polyPoly(p, q) <= TOUCH_EPS; }

    private static boolean circleTouches(double[][] poly, Vec c, double radius) { return Geom.polyPoint(poly, c.x, c.y) <= radius; }

    /** Inside the box between the two bars: a robot there may always leave. */
    static boolean betweenBars(Vec p) {
        return Math.abs(p.x) <= Field.BAR_HALF_X && Math.abs(p.y) <= Field.BAR_Y_NEAR;
    }

    /**
     * Whether a straight leg is acceptable (specs/dashes.md section 1.2): the area the robot sweeps along it touches
     * no obstacle. The swept area is a rectangle laid along the leg.
     *
     *   long leg (at least the align length; driven aligned, front or rear leading):
     *       half width = half the robot's width + the margin; it reaches past each end by the longer of
     *       origin-to-front and origin-to-back + the margin;
     *   short leg (not aligned): half width and reach = the far-corner radius + the margin.
     *
     * @param fromRobot the leg starts where the robot is now
     * @param toGoal    the leg ends at the goal
     */
    public static boolean legOk(Vec a, Vec b, boolean fromRobot, boolean toGoal, Footprint fp, List<Obstacle> obstacles) {
        double len = a.dist(b);
        if (len < 1e-6) return true;
        Vec u = b.minus(a).unit();
        boolean isLong = len >= ALIGN_LENGTH_IN;
        double corner = fp.farCorner();
        double halfWidth = (isLong ? fp.halfWidth : corner) + MARGIN_IN;
        double reach = (isLong ? Math.max(fp.front, fp.back) : corner) + MARGIN_IN;
        // A short leg from the robot is boxed from the start point onward, with nothing behind it: behind the start
        // the robot is only moving away. A long leg from the robot keeps its reach behind the start.
        boolean boxedFromStart = fromRobot && !isLong;
        double behind = boxedFromStart ? 0 : reach;
        boolean startBetweenBars = fromRobot && betweenBars(a);

        for (Obstacle o : obstacles) {
            switch (o.kind) {
                case SUPPORT:
                    if (touches(swept(a, u, len, behind, reach, halfWidth), o.poly)) return false;
                    // The robot's own circle at the start (no margin), checked on its own.
                    if (boxedFromStart && circleTouches(o.poly, a, corner)) return false;
                    break;
                case FLOWER: {
                    // A flower just gathered from is ignored on a leg from the robot: the stand is within the far-corner
                    // radius plus the margin of the start, and the leg ends farther from it than it starts.
                    // ("The stand" is taken as the planner's obstacle, the stand grown by its clearance; with the bare
                    // stand a robot 13.5 to 15 in from it could not leave, its own circle touching the grown stand.)
                    double fromStart = Geom.polyPoint(o.poly, a.x, a.y);
                    if (fromRobot && fromStart <= corner + MARGIN_IN && Geom.polyPoint(o.poly, b.x, b.y) > fromStart) break;
                    // A leg that ends at the goal: the rectangle ends at the goal and does not reach past it, and the
                    // robot at the goal is the far-corner circle with no margin.
                    if (touches(swept(a, u, len, behind, toGoal ? 0 : reach, halfWidth), o.poly)) return false;
                    if (toGoal && circleTouches(o.poly, b, corner)) return false;
                    if (boxedFromStart && circleTouches(o.poly, a, corner)) return false;
                    break;
                }
                case LANE_WALL:
                    if (fromRobot) {
                        // A robot that starts between the bars may always leave: the whole leg ignores the walls.
                        if (startBetweenBars) break;
                        // Any other leg from the robot checks the walls only from a distance along the leg onward, so a
                        // robot at the lane's mouth can leave or turn away, while one heading into the lane must be centered.
                        if (len <= LANE_WALL_SKIP_IN) break;
                        Vec from = new Vec(a.x + u.x * LANE_WALL_SKIP_IN, a.y + u.y * LANE_WALL_SKIP_IN);
                        if (touches(swept(from, u, len - LANE_WALL_SKIP_IN, 0, reach, halfWidth), o.poly)) return false;
                    } else if (touches(swept(a, u, len, behind, reach, halfWidth), o.poly)) {
                        return false;
                    }
                    break;
            }
        }
        return true;
    }

    /** The turn between two directions of travel, in degrees, folded to 0 to 90: a leg can be driven front or rear first. */
    private static double turnFolded(double fromRad, double toRad) {
        double d = Math.toDegrees(toRad - fromRad);
        d = Math.abs(((d % 360 + 540) % 360) - 180);    // 0 to 180
        return d > 90 ? 180 - d : d;
    }

    /**
     * The cost of a route (specs/dashes.md section 1.3): the sum over its legs, in order from the robot.
     *
     *   short leg (not aligned):            its length divided by the unaligned speed ratio; no turn is charged and
     *                                       the direction of travel is forgotten;
     *   long leg, the first of the route:   its length + the turn penalty for each degree between the robot's heading
     *                                       when the route is planned and the leg;
     *   long leg after another long leg:    its length + the turn penalty for each degree between the two legs;
     *   long leg after a short leg:         its length; no turn is charged.
     *
     * The last row is the page's known limit: a short hop to a nearby route point can cost less than turning onto the
     * straight line. It is kept, as the page says.
     */
    public static double cost(Vec[] pts, double headingRad) {
        double c = 0;
        boolean haveDirection = true;       // the robot's own heading, before the first leg
        double direction = headingRad;
        for (int i = 0; i + 1 < pts.length; i++) {
            double len = pts[i].dist(pts[i + 1]);
            if (len < ALIGN_LENGTH_IN) {
                c += len / UNALIGNED_SPEED_RATIO;
                haveDirection = false;
                continue;
            }
            double leg = pts[i + 1].minus(pts[i]).angle();
            c += len;
            if (haveDirection) c += TURN_PENALTY_IN_PER_DEG * turnFolded(direction, leg);
            haveDirection = true;
            direction = leg;
        }
        return c;
    }

    private static Plan make(Vec[] pts, double headingRad) {
        boolean[] aligned = new boolean[pts.length - 1];
        for (int i = 0; i < aligned.length; i++) aligned[i] = pts[i].dist(pts[i + 1]) >= ALIGN_LENGTH_IN;
        return new Plan(pts, aligned, cost(pts, headingRad));
    }

    /** Whether a route point may be used: not within the far-corner radius plus the margin of an obstacle. */
    public static boolean routePointUsable(Vec p, Footprint fp, List<Obstacle> obstacles) {
        for (Obstacle o : obstacles) {
            if (circleTouches(o.poly, p, fp.farCorner() + MARGIN_IN)) return false;
        }
        return true;
    }

    /**
     * Plans from the robot's pose to a goal (specs/dashes.md section 1.3). The candidates are the straight line, when it
     * is acceptable, and every chain of at most MAX_VIA different route points over acceptable legs; the cheapest is the
     * route. The straight line is not preferred: it is tried first, so it wins a tie and nothing more.
     */
    public static Plan plan(Pose robot, Vec goal, Footprint fp) {
        Vec start = robot.vec();
        List<Obstacle> obs = obstacles();
        Vec[] best = null;
        double bestCost = Double.MAX_VALUE;
        if (legOk(start, goal, true, true, fp, obs)) {
            best = new Vec[] {start, goal};
            bestCost = cost(best, robot.h);
        }

        Vec[] via = Field.routePoints();
        int n = via.length;
        boolean[] usable = new boolean[n], fromStart = new boolean[n], toGoal = new boolean[n];
        boolean[][] between = new boolean[n][n];
        for (int i = 0; i < n; i++) usable[i] = routePointUsable(via[i], fp, obs);
        for (int i = 0; i < n; i++) {
            if (!usable[i]) continue;
            fromStart[i] = legOk(start, via[i], true, false, fp, obs);
            toGoal[i] = legOk(via[i], goal, false, true, fp, obs);

            for (int j = 0; j < n; j++) between[i][j] = i != j && usable[j] && legOk(via[i], via[j], false, false, fp, obs);
        }
        int[] chain = new int[Math.max(1, MAX_VIA)];
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
                double cst = cost(pts, robot.h);
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
        return make(best, robot.h);
    }
}
