package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.math.Vec;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Grid routes (specs/dashes.md section 5): precomputed, time-optimal routes that know about momentum.
 *
 * This class is the reading of the generated tables (section 5.1) and the checks that decide whether they may be
 * used. The tables themselves are generated data (GridTables); none exist on this branch yet, so every goal
 * answers "not usable, and why", and the dash logs the reason and uses the route planner.
 *
 * NOT BUILT: picking a start (5.2), the follower (5.3) and the off-route guard (5.4). Until they are, a dash uses
 * the route planner even with usable tables, and says so in the log.
 *
 * The lattice: nodes every 6 in from -66 to 66 on both axes, each with 16 travel directions plus a final state.
 *   node (i, j), i and j from 0 to 22, is at x = -66 + 6 i, y = -66 + 6 j, and has the number 23 i + j;
 *   a state's number is 17 x node + direction: 8,993 states per goal;
 *   directions 0 to 15 are lattice steps; direction 16 is the final state: straight from the node to the goal.
 * A state is read here as one leg: from its node, one lattice step in its direction (or, for the final state, to the
 * goal). Its next state starts at the node where that leg ends.
 */
public final class GridRoutes {
    private GridRoutes() { }

    public static final double PITCH_IN = 6.0, HALF_SPAN_IN = 66.0;
    public static final int DIRECTIONS = 16;
    public static final int NODES_PER_AXIS = (int) Math.round(2 * HALF_SPAN_IN / PITCH_IN) + 1;    // 23
    public static final int NODES = NODES_PER_AXIS * NODES_PER_AXIS;                               // 529
    public static final int STATES_PER_NODE = DIRECTIONS + 1;                                      // 17
    public static final int STATES = NODES * STATES_PER_NODE;                                      // 8,993
    /** The final state: straight from the node to the goal. */
    public static final int FINAL_DIRECTION = DIRECTIONS;
    public static final double FINAL_MAX_IN = 30.0;
    /** The lattice step of each direction, in nodes. */
    public static final int[][] STEP = {
            {1, 0}, {2, 1}, {1, 1}, {1, 2}, {0, 1}, {-1, 2}, {-1, 1}, {-2, 1},
            {-1, 0}, {-2, -1}, {-1, -1}, {-1, -2}, {0, -1}, {1, -2}, {1, -1}, {2, -1}};
    /** A goal has a table when a table's goal is within this of it. Farther than that, the goal has moved. */
    public static final double GOAL_MOVED_IN = 1.0;
    /** The corner angles, in degrees, that the corner speed table has an entry for. */
    public static final double[] CORNER_DEG = {0, 10, 20, 30, 45, 60, 75, 90, 120, 150, 180};
    /**
     * The model text of this code: the motion values the tables must have been built with (top speed, planning
     * deceleration, grip, the tracking allowance on the route and at the goal and the distance over which it tapers,
     * the corner cluster length, the corner safety factor). A table whose model text differs is reported in the log
     * and still used.
     */
    public static String MODEL = "V_MAX 66.0, A_DEC 120.0, A_GRIP 150.0, E_MAX 3.00, E_GOAL 0.50, E_TAPER 24.0, CORNER_CLUSTER_IN 30.0, CORNER_SAFETY 0.75";

    /** The geometry text of this code, as the generated file writes it. */
    public static String geometry() {
        return String.format(Locale.US, "pitch %.1f in, half span %.1f in, %d directions", PITCH_IN, HALF_SPAN_IN, DIRECTIONS);
    }

    // ---- The numbering of the states ----

    public static int node(int i, int j) { return NODES_PER_AXIS * i + j; }
    public static int state(int node, int direction) { return STATES_PER_NODE * node + direction; }
    public static int nodeOf(int state) { return state / STATES_PER_NODE; }
    public static int directionOf(int state) { return state % STATES_PER_NODE; }
    public static int nodeI(int node) { return node / NODES_PER_AXIS; }
    public static int nodeJ(int node) { return node % NODES_PER_AXIS; }
    public static Vec nodePoint(int node) { return new Vec(-HALF_SPAN_IN + PITCH_IN * nodeI(node), -HALF_SPAN_IN + PITCH_IN * nodeJ(node)); }

    /** The node a lattice step leads to, or -1 when it leaves the lattice (or the direction is the final state). */
    public static int stepFrom(int node, int direction) {
        if (direction < 0 || direction >= DIRECTIONS) return -1;
        int i = nodeI(node) + STEP[direction][0], j = nodeJ(node) + STEP[direction][1];
        return i < 0 || j < 0 || i >= NODES_PER_AXIS || j >= NODES_PER_AXIS ? -1 : node(i, j);
    }

    // ---- One goal's tree ----

    public static final class Table {
        public final String name;
        /** The goal, blue frame, and the heading at the goal in radians (a cell's aim; NaN for a park corner). */
        public final double x, y, headingRad;
        private final byte[] next, timeMs, endSpeed;

        Table(String name, double x, double y, double headingRad, byte[] next, byte[] timeMs, byte[] endSpeed) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.headingRad = headingRad;
            this.next = next;
            this.timeMs = timeMs;
            this.endSpeed = endSpeed;
        }

        private static int u16(byte[] b, int i) { return ((b[2 * i] & 0xFF) << 8) | (b[2 * i + 1] & 0xFF); }    // high byte first

        /** The next state, or -1 for none. */
        public int next(int state) {
            int v = u16(next, state);
            return v == 0xFFFF ? -1 : v;
        }

        /** The time to the goal in seconds, or NaN when the goal cannot be reached from this state. */
        public double seconds(int state) {
            int v = u16(timeMs, state);
            return v == 0xFFFF ? Double.NaN : v / 1000.0;
        }

        public boolean reachable(int state) { return u16(timeMs, state) != 0xFFFF; }

        /** The speed to carry at the end of the state's leg, in/s (stored in half inches per second, rounded down). */
        public double endSpeed(int state) { return (endSpeed[state] & 0xFF) * 0.5; }

        public Vec goal() { return new Vec(x, y); }

        /**
         * The route from a state to the goal: the start node, the node at the end of every lattice step, then the goal.
         * Null when the tree does not lead to the goal from here, or is not consistent with the numbering.
         */
        public List<Vec> trace(int state) {
            List<Vec> pts = new ArrayList<>();
            if (state < 0 || state >= STATES || !reachable(state)) return null;
            pts.add(nodePoint(nodeOf(state)));
            for (int guard = 0; guard <= STATES; guard++) {
                int node = nodeOf(state), dir = directionOf(state);
                if (dir == FINAL_DIRECTION) {
                    pts.add(goal());
                    return pts;
                }
                int to = stepFrom(node, dir), nx = next(state);
                if (to < 0 || nx < 0 || nx >= STATES || nodeOf(nx) != to) return null;
                pts.add(nodePoint(to));
                state = nx;
            }
            return null;    // a loop
        }
    }

    // ---- Reading the generated file, and the checks that decide whether it may be used ----

    private static boolean loaded;
    private static String problem;          // why no table at all can be used; null when the file is good
    private static String modelNote;        // a note for the log about the model text; null for none
    private static Table[] tables = new Table[0];
    private static double[][] cornerSpeed = new double[0][];

    /** Reads GridTables once. */
    private static synchronized void load() {
        if (loaded) return;
        loaded = true;
        problem = read(GridTables.GEOMETRY, GridTables.MODEL, GridTables.CORNER_SPEED, GridTables.NAMES, GridTables.GOAL_X, GridTables.GOAL_Y,
                GridTables.AIM, GridTables.NEXT, GridTables.COST, GridTables.SPEED);
    }

    /** For a PC check: read tables from somewhere other than the generated file. Returns the problem, or null. */
    public static synchronized String setData(String geometryText, String modelText, double[] cornerSpeeds, String[] goal, double[] goalX, double[] goalY,
                                              double[] aim, String[] next, String[] timeMs, String[] endSpeed) {
        loaded = true;
        problem = read(geometryText, modelText, cornerSpeeds, goal, goalX, goalY, aim, next, timeMs, endSpeed);
        return problem;
    }

    private static String read(String geometryText, String modelText, double[] cornerSpeeds, String[] goal, double[] goalX, double[] goalY,
                               double[] aim, String[] next, String[] timeMs, String[] endSpeed) {
        tables = new Table[0];
        cornerSpeed = new double[0][];
        modelNote = null;
        if (geometryText == null || geometryText.isEmpty() || goal == null || goal.length == 0) {
            return "no grid route tables in this build (they are generated by the dash simulator; see GridTables)";
        }
        // A table whose geometry text differs from the code's is not used.
        if (!geometryText.equals(geometry())) {
            return "the tables were built on another lattice (\"" + geometryText + "\"; this code has \"" + geometry() + "\")";
        }
        if (cornerSpeeds == null || cornerSpeeds.length != CORNER_DEG.length) {
            return "the corner speed table does not have an entry for each of the " + CORNER_DEG.length + " corner angles";
        }
        double[][] corner = new double[CORNER_DEG.length][];
        for (int k = 0; k < corner.length; k++) corner[k] = new double[] {CORNER_DEG[k], cornerSpeeds[k]};
        if (goalX == null || goalY == null || aim == null || goalX.length != goal.length || goalY.length != goal.length || aim.length != goal.length
                || next.length != goal.length || timeMs.length != goal.length || endSpeed.length != goal.length) {
            return "the tables do not hold the same number of goals in every part";
        }
        Table[] out = new Table[goal.length];
        for (int g = 0; g < goal.length; g++) {
            byte[] n = base64(next[g]), t = base64(timeMs[g]), s = base64(endSpeed[g]);
            if (n == null || t == null || s == null) return "the table for " + goal[g] + " is not valid Base64";
            if (n.length != 2 * STATES || t.length != 2 * STATES || s.length != STATES) {
                return String.format(Locale.US, "the table for %s has %d / %d / %d states (next / time / speed); this code's lattice has %d",
                        goal[g], n.length / 2, t.length / 2, s.length, STATES);
            }
            out[g] = new Table(goal[g], goalX[g], goalY[g], aim[g], n, t, s);
        }
        tables = out;
        cornerSpeed = corner;
        // A model text that differs is reported in the log and the tables are still used.
        String m = modelText == null ? "" : modelText;
        if (!MODEL.equals(m)) modelNote = "the tables' model differs from this code's (tables: \"" + m + "\"; code: \"" + MODEL + "\"); the tables are still used";
        return null;
    }

    private static Table nearest(Vec blueGoal) {
        Table best = null;
        double bestD = Double.MAX_VALUE;
        for (Table t : tables) {
            double d = Math.hypot(t.x - blueGoal.x, t.y - blueGoal.y);
            if (d < bestD) {
                bestD = d;
                best = t;
            }
        }
        return best;
    }

    /** Null when a table can be used for this blue-frame goal; otherwise the reason it cannot, for the dash's log. */
    public static String unusable(Vec blueGoal) {
        load();
        if (problem != null) return problem;
        Table t = nearest(blueGoal);
        double d = Math.hypot(t.x - blueGoal.x, t.y - blueGoal.y);
        if (d > GOAL_MOVED_IN) {
            // A launch spot or park corner changed on the Dashboard after the tables were made.
            return String.format(Locale.US, "the goal has moved: it is at (%.2f, %.2f) and the nearest table, %s, was built for (%.2f, %.2f)",
                    blueGoal.x, blueGoal.y, t.name, t.x, t.y);
        }
        return null;
    }

    /** The table for a blue-frame goal, or null when none can be used (unusable says why). */
    public static Table table(Vec blueGoal) {
        return unusable(blueGoal) == null ? nearest(blueGoal) : null;
    }

    /** A note about the tables' model text for the log, or null. */
    public static String modelNote() {
        load();
        return problem == null ? modelNote : null;
    }

    /**
     * The fastest speed, in/s, at which a turn of an angle can be taken: the corner speed table, interpolated between
     * two entries and held beyond its ends. NaN when there are no tables.
     */
    public static double cornerSpeed(double angleDeg) {
        load();
        double[][] c = cornerSpeed;
        if (c.length == 0) return Double.NaN;
        double a = Math.abs(angleDeg);
        if (a <= c[0][0]) return c[0][1];
        for (int k = 1; k < c.length; k++) {
            if (a <= c[k][0]) return c[k - 1][1] + (c[k][1] - c[k - 1][1]) * (a - c[k - 1][0]) / (c[k][0] - c[k - 1][0]);
        }
        return c[c.length - 1][1];
    }

    /** Standard Base64 (padding optional, white space ignored). Null when the text is not Base64. java.util.Base64 needs a newer Android than the hub's. */
    static byte[] base64(String text) {
        if (text == null) return null;
        byte[] out = new byte[text.length() * 3 / 4 + 3];
        int n = 0, acc = 0, bits = 0;
        for (int k = 0; k < text.length(); k++) {
            char ch = text.charAt(k);
            int v;
            if (ch >= 'A' && ch <= 'Z') v = ch - 'A';
            else if (ch >= 'a' && ch <= 'z') v = ch - 'a' + 26;
            else if (ch >= '0' && ch <= '9') v = ch - '0' + 52;
            else if (ch == '+') v = 62;
            else if (ch == '/') v = 63;
            else if (ch == '=' || ch == ' ' || ch == '\n' || ch == '\r' || ch == '\t') continue;
            else return null;
            acc = (acc << 6) | v;
            bits += 6;
            if (bits >= 8) {
                bits -= 8;
                out[n++] = (byte) ((acc >> bits) & 0xFF);
            }
        }
        byte[] r = new byte[n];
        System.arraycopy(out, 0, r, 0, n);
        return r;
    }
}
