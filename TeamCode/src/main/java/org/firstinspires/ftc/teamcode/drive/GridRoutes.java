package org.firstinspires.ftc.teamcode.drive;

import org.firstinspires.ftc.teamcode.math.Rotation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The precomputed dash routes (design/dash-planner-grid.md, "The grid planner"): for each goal a shortest-time TREE over the states of a grid, generated offline
 * (tools/dash/RouteGen.java) with the robot's footprint, a tracking allowance and corner speed limits from the drive model. Pure Java, no SDK: the robot and the
 * PC simulation use the same lookup.
 *
 * A STATE is a grid node and a travel direction: "the robot at this node, leaving along this direction". Nodes are every PITCH_IN inches from -HALF_SPAN_IN to
 * +HALF_SPAN_IN in x and y (blue frame: red is turned 180 deg about the field center); the 16 directions are the steps (1,0), (2,1), (1,1), (1,2), (0,1), ... in
 * grid units, and direction FINAL means "straight from this node to the goal". For each state the table holds the next state (the tree), the time to the goal and
 * the most speed allowed when the edge ends (so the robot can still slow for the corners and the stop ahead).
 *
 * A dash: {@link #pick} chooses the start state near the robot that is cheapest to reach and to finish from (the user's design: drive to one grid point first,
 * then follow the tree), walks the tree, and merges straight runs into legs. The routes were checked offline; the only test here is the short hop to the first
 * node (the robot's footprint at its present heading along it, against the supports and the flowers), and nodes farther than PICK_RADIUS_IN are tried only when
 * no nearer one serves (a robot against a wall is about 9.4 in from the nearest node).
 */
public final class GridRoutes {

    public static final double PITCH_IN = 6.0;
    public static final double HALF_SPAN_IN = 66.0;
    public static final int N = (int) Math.round(2.0 * HALF_SPAN_IN / PITCH_IN) + 1;   // nodes per axis
    public static final int[][] DIRS = {{1, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 1}, {-1, 2}, {-1, 1}, {-2, 1}, {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2}, {0, -1}, {1, -2}, {1, -1}, {2, -1}};
    public static final int NDIR = DIRS.length;
    /** The extra direction index: from the node straight to the goal. */
    public static final int FINAL = NDIR;
    public static final int STATES_PER_NODE = NDIR + 1;
    public static final int NONE = -1;

    /** The start node is chosen among the nodes this close to the robot, or, when none of them serves, as far as PICK_RADIUS_MAX_IN. */
    public static double PICK_RADIUS_IN = 9.0, PICK_RADIUS_MAX_IN = 18.0;
    /** The hop to the start node is costed at this speed, and a body turn at this many seconds per radian. */
    public static double HOP_SPEED_IN_PER_S = 30.0, TURN_SEC_PER_RAD = 0.2;
    /** The turn at the start node is checked with the footprint grown by this (the robot overshoots the node a little while it turns). */
    public static double START_TURN_MARGIN_IN = 1.5;

    // ---- the motion model the routes are generated and driven with (the tuned simulation plant of 2026-11-06; regenerate the tables after a refit) ----
    /** Cruise speed (in/s, about 0.85 power aligned), the braking the speed profile plans with (in/s^2), and the most sideways push of the tires (in/s^2). */
    public static double V_MAX = 66.0, A_DEC = 120.0, A_GRIP = 150.0;   // A_DEC below what the robot can do (the follower must start braking in time)
    /** The tracking allowance (in): the clearance a route keeps beyond the margin, E_MAX away from the goal, falling to E_GOAL at it over E_TAPER. */
    public static double E_MAX = 3.0, E_GOAL = 0.5, E_TAPER = 24.0;   // (1.5 at the goal did not remove the plain-fit plants' audience flower grazes and slowed the Park Dash 0.3 to 0.6 s)
    /** Corners closer than this along the route (in) are one turn for the corner speed (three small turns in 22 in swung a robot 8.8 in wide; 18 in left two corners 27 in apart swinging 5.5 in). */
    public static double CORNER_CLUSTER_IN = 30.0;
    /** The corner speeds are calibrated to keep the swing within this share of E_MAX (a margin for corners that follow each other). */
    public static double CORNER_SAFETY = 0.75;

    public static double allowance(double x, double y, double gx, double gy) {
        double d = Math.hypot(x - gx, y - gy);
        return E_GOAL + (E_MAX - E_GOAL) * Math.min(1.0, d / E_TAPER);
    }

    /**
     * The CORNER SPEED TABLE: the most speed (in/s) into a turn of the travel direction by CORNER_DEG that keeps GridFollower's swing within E_MAX. CALIBRATED by
     * driving GridFollower through single corners in the simulation (tools/dash/DashSim calibrate, also run by the generation): regenerate after a drive refit. The
     * values are the generation's ({@link GridTables}, written by tools/dash/run-sim.cmd gentables with the tables, so the two always agree).
     */
    public static double[] CORNER_DEG = {0, 10, 20, 30, 45, 60, 75, 90, 120, 150, 180};
    public static double[] CORNER_SPEED = GridTables.CORNER_SPEED.clone();

    /** The corner speed for a turn of theta (rad) and an allowance e (in): the table, scaled by sqrt(e / E_MAX) (the swing grows with the square of the speed). */
    public static double cornerSpeed(double theta, double e) {
        double deg = Math.min(180.0, Math.toDegrees(Math.abs(theta)));
        double v = CORNER_SPEED[CORNER_SPEED.length - 1];
        for (int i = 0; i + 1 < CORNER_DEG.length; i++) {
            if (deg <= CORNER_DEG[i + 1]) {
                double f = (deg - CORNER_DEG[i]) / (CORNER_DEG[i + 1] - CORNER_DEG[i]);
                v = CORNER_SPEED[i] + f * (CORNER_SPEED[i + 1] - CORNER_SPEED[i]);
                break;
            }
        }
        return Math.min(V_MAX, v * Math.sqrt(Math.max(0.05, e) / E_MAX));
    }

    private GridRoutes() {}

    // ---- the generated tables (GridTables) ----

    /** The grid's shape, as the generated tables record it: tables of another shape are not used. */
    public static String geometry() {
        return String.format(Locale.US, "pitch %.1f in, half span %.1f in, %d directions", PITCH_IN, HALF_SPAN_IN, NDIR);
    }

    /** The motion values the routes are driven with, as the generated tables record theirs: a difference is reported (regenerate), the tables are still used. */
    public static String model() {
        return String.format(Locale.US, "V_MAX %.1f, A_DEC %.1f, A_GRIP %.1f, E_MAX %.2f, E_GOAL %.2f, E_TAPER %.1f, CORNER_CLUSTER_IN %.1f, CORNER_SAFETY %.2f",
                V_MAX, A_DEC, A_GRIP, E_MAX, E_GOAL, E_TAPER, CORNER_CLUSTER_IN, CORNER_SAFETY);
    }

    private static Table[] tables;
    private static String tablesProblem;

    /** The four blue tables (decoded once from GridTables), or null when they cannot be used ({@link #tablesProblem} says why). */
    public static synchronized Table[] tables() {
        if (tables == null && tablesProblem == null) {
            try {
                if (!GridTables.GEOMETRY.equals(geometry())) {
                    tablesProblem = "the tables were generated for another grid (" + GridTables.GEOMETRY + "): regenerate them";
                } else {
                    Table[] t = new Table[GridTables.NAMES.length];
                    for (int i = 0; i < t.length; i++) {
                        t[i] = decode(GridTables.NAMES[i], GridTables.GOAL_X[i], GridTables.GOAL_Y[i], GridTables.AIM[i], GridTables.NEXT[i], GridTables.COST[i], GridTables.SPEED[i]);
                    }
                    tables = t;
                }
            } catch (RuntimeException e) {
                tablesProblem = "the tables did not decode: " + e;
            }
        }
        return tables;
    }

    public static synchronized String tablesProblem() {
        tables();
        return tablesProblem;
    }

    /** Puts other tables in place of the generated ones (the PC simulation, with tables built in memory). */
    public static synchronized void useTables(Table[] t) {
        tables = t;
        tablesProblem = t == null ? "no tables" : null;
    }

    /** The table whose goal (blue frame) is within 1 in of (gx, gy), or null (no table, or the goal was moved on the Dashboard: regenerate). */
    public static Table tableFor(double gx, double gy) {
        Table[] ts = tables();
        if (ts == null) return null;
        for (Table t : ts) if (Math.hypot(t.goalX - gx, t.goalY - gy) <= 1.0) return t;
        return null;
    }

    /**
     * The packed form of a table, as GridTables holds it: per state, the next state as 2 bytes (0xFFFF: none), the time to the goal in milliseconds as 2 bytes
     * (0xFFFF: cannot reach it) and the end speed in half in/s as 1 byte (rounded down), each array base64. The PC side (RouteGen) packs; this unpacks.
     */
    public static Table decode(String name, double gx, double gy, double aim, String next64, String cost64, String speed64) {
        byte[] nb = unbase64(next64), cb = unbase64(cost64), sb = unbase64(speed64);
        int states = N * N * STATES_PER_NODE;
        if (nb.length != 2 * states || cb.length != 2 * states || sb.length != states) {
            throw new IllegalArgumentException(name + ": " + nb.length + " / " + cb.length + " / " + sb.length + " bytes for " + states + " states");
        }
        int[] next = new int[states];
        float[] cost = new float[states], speed = new float[states];
        for (int s = 0; s < states; s++) {
            int n = ((nb[2 * s] & 0xFF) << 8) | (nb[2 * s + 1] & 0xFF);
            int c = ((cb[2 * s] & 0xFF) << 8) | (cb[2 * s + 1] & 0xFF);
            next[s] = n == 0xFFFF ? NONE : n;
            cost[s] = c == 0xFFFF ? Float.POSITIVE_INFINITY : c / 1000f;
            speed[s] = (sb[s] & 0xFF) / 2f;
        }
        return new Table(name, gx, gy, aim, next, cost, speed);
    }

    private static final String B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    /** Base64 (java.util.Base64 needs Android 8; the Control Hub runs 7). */
    static byte[] unbase64(String s) {
        int[] map = new int[128];
        java.util.Arrays.fill(map, -1);
        for (int i = 0; i < B64.length(); i++) map[B64.charAt(i)] = i;
        int len = s.length();
        while (len > 0 && s.charAt(len - 1) == '=') len--;
        byte[] out = new byte[len * 3 / 4];
        int buf = 0, bits = 0, o = 0;
        for (int i = 0; i < len; i++) {
            char ch = s.charAt(i);
            int v = ch < 128 ? map[ch] : -1;
            if (v < 0) throw new IllegalArgumentException("not base64 at " + i);
            buf = (buf << 6) | v;
            bits += 6;
            if (bits >= 8) {
                bits -= 8;
                out[o++] = (byte) (buf >> bits);
                buf &= (1 << bits) - 1;
            }
        }
        return out;
    }

    public static int node(int i, int j) {
        return i * N + j;
    }

    public static double nodeX(int node) {
        return -HALF_SPAN_IN + PITCH_IN * (node / N);
    }

    public static double nodeY(int node) {
        return -HALF_SPAN_IN + PITCH_IN * (node % N);
    }

    public static int state(int node, int dir) {
        return node * STATES_PER_NODE + dir;
    }

    /** The node a direction step leads to, or NONE off the grid. */
    public static int stepNode(int node, int dir) {
        int i = node / N + DIRS[dir][0], j = node % N + DIRS[dir][1];
        return i < 0 || j < 0 || i >= N || j >= N ? NONE : node(i, j);
    }

    public static double dirAngle(int dir) {
        return Math.atan2(DIRS[dir][1], DIRS[dir][0]);
    }

    /** One goal's tree (blue frame). */
    public static final class Table {
        public final String name;
        public final double goalX, goalY;
        /** The heading to aim at the goal (a hive cell), or NaN (a park corner: the arrival heading is kept). */
        public final double aimHeading;
        /** Per state: the next state (NONE at a FINAL state, and for a state that cannot reach the goal), the time to the goal (s, +inf when it cannot), the most speed (in/s) when the state's edge ends. */
        public final int[] next;
        public final float[] cost, endSpeed;

        public Table(String name, double goalX, double goalY, double aimHeading, int[] next, float[] cost, float[] endSpeed) {
            this.name = name;
            this.goalX = goalX;
            this.goalY = goalY;
            this.aimHeading = aimHeading;
            this.next = next;
            this.cost = cost;
            this.endSpeed = endSpeed;
        }

        public boolean reachable(int state) {
            return !Float.isInfinite(cost[state]);
        }
    }

    /** A route in the BLUE frame: waypoints (the robot, the start node, the corners, the goal), with the most speed allowed on reaching each. */
    public static final class Route {
        public final double[] xs, ys, speeds;
        public final int startState;
        public final double predictedSec;

        Route(double[] xs, double[] ys, double[] speeds, int startState, double predictedSec) {
            this.xs = xs;
            this.ys = ys;
            this.speeds = speeds;
            this.startState = startState;
            this.predictedSec = predictedSec;
        }

        public int legs() {
            return xs.length - 1;
        }
    }

    /**
     * The route from the robot's pose (blue frame) along t's tree, or null when no node near the robot can reach the goal. The start state minimizes the hop to the
     * node (distance at HOP_SPEED, plus the body turn to the state's direction) plus the state's time to the goal.
     */
    public static Route pick(Table t, double x, double y, double heading) {
        Route r = pick(t, x, y, heading, PICK_RADIUS_IN);
        return r != null || PICK_RADIUS_MAX_IN <= PICK_RADIUS_IN ? r : pick(t, x, y, heading, PICK_RADIUS_MAX_IN);
    }

    private static Route pick(Table t, double x, double y, double heading, double radius) {
        // every candidate start state with its score; then the cheapest whose start turn is clear (scores first: the turn test is the costly part)
        List<double[]> cand = new ArrayList<>();
        int ci = (int) Math.round((x + HALF_SPAN_IN) / PITCH_IN), cj = (int) Math.round((y + HALF_SPAN_IN) / PITCH_IN);
        int r = (int) Math.ceil(radius / PITCH_IN);
        for (int i = ci - r; i <= ci + r; i++) {
            for (int j = cj - r; j <= cj + r; j++) {
                if (i < 0 || j < 0 || i >= N || j >= N) continue;
                int n = node(i, j);
                double hop = Math.hypot(nodeX(n) - x, nodeY(n) - y);
                if (hop > radius) continue;
                boolean any = false;
                for (int d = 0; d < STATES_PER_NODE && !any; d++) any = t.reachable(state(n, d));
                if (!any || !hopClear(x, y, heading, nodeX(n), nodeY(n))) continue;
                for (int d = 0; d < STATES_PER_NODE; d++) {
                    int s = state(n, d);
                    if (!t.reachable(s)) continue;
                    double dirAng = d == FINAL ? Math.atan2(t.goalY - nodeY(n), t.goalX - nodeX(n)) : dirAngle(d);
                    double turn = axisTurn(heading, dirAng);
                    cand.add(new double[]{hop / HOP_SPEED_IN_PER_S + turn * TURN_SEC_PER_RAD + t.cost[s], s, dirAng});
                }
            }
        }
        cand.sort((a, b) -> Double.compare(a[0], b[0]));
        for (double[] c : cand) {
            int s = (int) c[1], n = s / STATES_PER_NODE;
            if (!startTurnClear(nodeX(n), nodeY(n), heading, c[2])) continue;
            return walk(t, s, x, y, c[0]);
        }
        return null;
    }

    /**
     * The turn at the start node from the robot's present heading onto the first leg (front or rear first, the nearer) touches no support and no flower (the robot's
     * footprint, no margin, every 7.5 deg): the tree checks the turns between its own legs, not this one (two Park Dash starts beside a bar grazed it in the simulation).
     */
    static boolean startTurnClear(double nx, double ny, double heading, double dirAng) {
        hopClear(nx, ny, heading, nx, ny);   // loads the obstacles
        double d = Rotation2d.normalize(dirAng - heading);
        if (d > Math.PI / 2) d -= Math.PI;
        if (d < -Math.PI / 2) d += Math.PI;
        int steps = (int) Math.ceil(Math.abs(d) / Math.toRadians(7.5));
        for (int k = 0; k <= steps; k++) {
            HiveRoute.Rect me = HiveRoute.poseRect(nx, ny, heading + d * k / Math.max(1, steps), footprintParams, START_TURN_MARGIN_IN);
            for (HiveRoute.Rect o : hopObstacles) if (HiveRoute.overlap(me, o)) return false;
        }
        return true;
    }

    private static HiveRoute.Params footprintParams;
    private static java.util.List<HiveRoute.Rect> hopObstacles;

    /** The robot's footprint (ROBOT_1's, no margin) at the present heading, every 2 in along the hop, touches no support and no flower. */
    static boolean hopClear(double x, double y, double heading, double nx, double ny) {
        if (footprintParams == null) {
            footprintParams = new HiveRoute.Params();
            hopObstacles = HiveRoute.keepOuts(footprintParams, false);
            hopObstacles.addAll(HiveRoute.flowerKeepOuts(0.0));
        }
        double len = Math.hypot(nx - x, ny - y);
        int steps = Math.max(1, (int) Math.ceil(len / 2.0));
        HiveRoute.Rect start = HiveRoute.poseRect(x, y, heading, footprintParams, 0.0);
        for (HiveRoute.Rect o : hopObstacles) {
            if (HiveRoute.overlap(start, o)) continue;   // already touching it (a robot that has just gathered at a flower): it is leaving, not hitting
            for (int k = 1; k <= steps; k++) {
                double px = x + (nx - x) * k / steps, py = y + (ny - y) * k / steps;
                if (HiveRoute.overlap(HiveRoute.poseRect(px, py, heading, footprintParams, 0.0), o)) return false;
            }
        }
        return true;
    }

    /** The smallest turn (rad, 0 to pi/2) from the heading to either end of the axis along direction ang (a leg may be driven front or rear first). */
    public static double axisTurn(double heading, double ang) {
        double d = Math.abs(Rotation2d.normalize(ang - heading));
        return Math.min(d, Math.PI - d);
    }

    /**
     * The waypoints from the robot through the tree: the robot, the start node, a waypoint at every node where the direction changes (with the most speed allowed
     * there: the end speed of the state before it), and the goal (speed 0). Null when the chain does not end at a FINAL state.
     */
    static Route walk(Table t, int start, double x, double y, double score) {
        List<Integer> chain = new ArrayList<>();
        int s = start;
        while (s != NONE && chain.size() < STATES_PER_NODE * N * N) {
            chain.add(s);
            if (s % STATES_PER_NODE == FINAL) break;
            s = t.next[s];
        }
        if (chain.isEmpty() || chain.get(chain.size() - 1) % STATES_PER_NODE != FINAL) return null;
        List<double[]> pts = new ArrayList<>();
        pts.add(new double[]{x, y, Double.MAX_VALUE});
        int n0 = chain.get(0) / STATES_PER_NODE;
        pts.add(new double[]{nodeX(n0), nodeY(n0), Double.MAX_VALUE});
        for (int i = 0; i + 1 < chain.size(); i++) {
            int a = chain.get(i), b = chain.get(i + 1);
            if (a % STATES_PER_NODE != b % STATES_PER_NODE) {
                int m = b / STATES_PER_NODE;
                pts.add(new double[]{nodeX(m), nodeY(m), t.endSpeed[a]});
            }
        }
        pts.add(new double[]{t.goalX, t.goalY, 0.0});
        // a hop of (almost) nothing: the robot is on the start node
        if (Math.hypot(pts.get(1)[0] - x, pts.get(1)[1] - y) < 0.5) pts.remove(1);
        double[] xs = new double[pts.size()], ys = new double[pts.size()], sp = new double[pts.size()];
        for (int i = 0; i < pts.size(); i++) {
            xs[i] = pts.get(i)[0];
            ys[i] = pts.get(i)[1];
            sp[i] = pts.get(i)[2];
        }
        limitSpeeds(xs, ys, sp, t.goalX, t.goalY);
        return new Route(xs, ys, sp, start, score);
    }
    /**
     * The route's own speed limits on top of the table's: at each corner the corner speed for the whole turn made within CORNER_CLUSTER_IN of it (a run of small
     * corners is one turn), then a backward pass so the robot can brake (A_DEC) for every limit ahead. The robot's own point (index 0) has no limit.
     */
    public static void limitSpeeds(double[] xs, double[] ys, double[] sp, double gx, double gy) {
        int n = xs.length;
        double[] s = new double[n], turn = new double[n];
        for (int i = 1; i < n; i++) s[i] = s[i - 1] + Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
        for (int i = 1; i + 1 < n; i++) {
            double a = Math.atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]), b = Math.atan2(ys[i + 1] - ys[i], xs[i + 1] - xs[i]);
            turn[i] = Math.abs(Rotation2d.normalize(b - a));
        }
        for (int i = 1; i + 1 < n; i++) {
            double sum = 0.0;
            for (int j = 1; j + 1 < n; j++) if (Math.abs(s[j] - s[i]) <= CORNER_CLUSTER_IN) sum += turn[j];
            sp[i] = Math.min(sp[i], cornerSpeed(Math.min(Math.PI, sum), allowance(xs[i], ys[i], gx, gy)));
        }
        sp[n - 1] = Math.min(sp[n - 1], 0.0);
        for (int i = n - 2; i >= 1; i--) sp[i] = Math.min(sp[i], Math.sqrt(sp[i + 1] * sp[i + 1] + 2.0 * A_DEC * (s[i + 1] - s[i])));
    }
}
