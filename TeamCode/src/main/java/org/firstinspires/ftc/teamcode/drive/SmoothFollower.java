package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.math.Rotation2d;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * PROTOTYPE (2026-11-17, simulation only, behind {@link GridDash#SMOOTH}, never run on the robot): a grid route driven as ONE SMOOTH PATH instead of leg by leg.
 *
 * What it changes against {@link GridFollower}, which lost about a second per trip to its corners, its body turns and its last leg (specs/dashes.md section 6.2):
 * - the route's points are cut down to the fewest that keep the path clear (a straight line where one is clear), so the lattice's small corners go;
 * - each corner left is an ARC, as wide as the legs and the clearance allow, driven at sqrt(A_LAT x radius) with the sideways push fed forward, not a point to brake for;
 * - the robot translates in any direction while it turns, and the heading has a PLAN of its own, one of two kinds:
 *     DIRECT  one turn from the heading it has to the heading it needs at the goal (a cell's aim; a park corner keeps its heading when that fits there);
 *     CRUISE  a turn to a cruise heading, that heading kept in the middle, and a turn to the goal heading at the end. The cruise headings tried are the one the
 *             robot has (so it only turns at the end) and the four that put the front, the rear or a side along the route's longest leg (a mecanum drive
 *             is slowest on a diagonal, and narrowest front or rear first);
 *     FOLLOW  the same, with the cruise heading turning with the path: the front, the rear or a side kept along it leg after leg (what the trees were checked
 *             for, so it fits where the others do not: beside the hive, around its corners);
 *   every plan that is clear is timed and the quickest is driven;
 * - the speed is one profile over the whole path (the direction the wheels are driven in, the arcs, then the stop at STOP_DECEL), with no cap on the last leg.
 *
 * CLEARANCE is checked here, when the dash starts, because the trees were checked for a robot aligned with its legs and this one is not always: the robot's real
 * footprint at every heading it may have, every CHECK_STEP_IN along the path, with RouteGen's rules (MARGIN_IN plus the tracking allowance, the supports and flower
 * stands grown and the walls moved in by GridRoutes.DRIFT_IN, the footprint at the goal against them as they are) and the hop's rule near the start (within
 * HOP_ZONE_IN the robot keeps the clearance it has, it may not come nearer). Which headings it may have is decided by where it is:
 *   HOLD    nearer than holdIn to the start: the heading it started with;
 *   TURN    from there to turn1End: every heading between that and the next (the goal heading, or the cruise heading);
 *   CRUISE  the cruise heading, within CRUISE_TOL_DEG (a FOLLOW plan on an arc: between the two legs' directions);
 *   TURN 2  within turn2Start of the goal: every heading between the cruise heading and the goal heading;
 *   FINAL   within endIn of the goal (a direct plan: or past turn1End): the goal heading (within HEADING_TOL_DEG, outside the lock radius).
 * The follower keeps its side of that: it starts each turn where its zone starts, and while a turn is not done it slows so as to stop before the zone ends.
 * {@link #build} returns null when no plan is clear: the dash then drives the route with GridFollower as before.
 *
 * Pure logic, field frame (the supports and flower stands are the same turned 180 deg, so red needs nothing).
 */
@Config
public final class SmoothFollower {

    // ---- the path ----
    /** The path is sampled this often (in) for the follower, and its clearance is checked this often (in). */
    public static double SAMPLE_IN = 1.0, CHECK_STEP_IN = 2.0;
    /** The sideways acceleration an arc is driven with (in/s^2; the wheel model's limit is about 146), the smallest arc radius used (in) and the sharpest corner rounded (deg). */
    public static double A_LAT = 100.0, RADIUS_MIN_IN = 1.5, MAX_CORNER_DEG = 150.0;
    /** On a FOLLOW plan the body turns with the path: an arc is driven no faster than this turn rate allows (rad/s), and the turn is fed forward (rad/s per unit of power). */
    public static double ARC_TURN_RATE = 3.0, TURN_FF_GAIN = 9.0;
    /** The braking the profile plans with for an arc ahead and for the stop at the goal (in/s^2; the robot brakes at 175 to 210 at zero power). */
    public static double A_DEC = 130.0, STOP_DECEL = 120.0;
    /** For the predicted time (choosing between plans, and the log): the speeding up (in/s^2), the body's turn rate (rad/s) and the time a turn takes to start and settle (s). */
    public static double A_ACC_PREDICT = 110.0, TURN_RATE_PREDICT = 3.0, TURN_SETTLE_SEC = 0.15;
    /** RouteGen's margin, and its growth of the footprint at the goal (half the margin plus GridRoutes.E_GOAL). */
    public static double MARGIN_IN = 1.5, GOAL_GROW_IN = 1.25;
    /**
     * The TRACKING ALLOWANCE of a smooth path (in): the clearance kept beyond the margin and the drift margin for the robot being off its path, away from the goal
     * (it falls to GridRoutes.E_GOAL at the goal over GridRoutes.E_TAPER, as the trees' does). The trees keep GridRoutes.E_MAX (3.0), sized for GridFollower's
     * swing at a corner; this follower's own figure is the "followed within" line of its log.
     */
    public static double TRACK_IN = 3.0;

    /** The allowance at (x, y) for a goal at (gx, gy). */
    static double allowance(double x, double y, double gx, double gy) {
        double d = Math.hypot(x - gx, y - gy);
        return GridRoutes.E_GOAL + (Math.max(GridRoutes.E_GOAL, TRACK_IN) - GridRoutes.E_GOAL) * Math.min(1.0, d / GridRoutes.E_TAPER);
    }
    /** Within this of the start the robot keeps the clearance it has (GridRoutes.hopClear's rule; the longest hop is 18 in). */
    public static double HOP_ZONE_IN = 18.0;
    /** A turn counts as done within this of its heading; the zone after it is checked that much either side. For the goal heading, and for the cruise heading. */
    public static double HEADING_TOL_DEG = 10.0, CRUISE_TOL_DEG = 5.0;
    /** A turn's zone is this long: a base plus this much per radian of turn (in). Shorter costs time (the robot waits for the turn), not safety. */
    public static double TURN_BASE_IN = 8.0, TURN_IN_PER_RAD = 16.0;
    /** A turn's headings are also checked this far into the zone after it (a robot that stops at the boundary to finish its turn overshoots a little). */
    public static double ZONE_SLOP_IN = 3.0;
    /** A cruise plan is tried on a path at least this long (in). */
    public static double CRUISE_MIN_IN = 36.0;
    /** Where the first turn may start (in from the start) and how long the FINAL zone before the goal may be (in), tried in this order. */
    private static final double[] HOLD_TRY_IN = {0.0, 6.0, 12.0, 18.0, 24.0}, END_TRY_IN = {12.0, 24.0, 36.0};
    /** A turn's zone may be shorter than TURN_IN_PER_RAD makes it (the robot then goes slowly until it has turned): these shares of it, tried in this order. */
    private static final double[] ZONE_TRY = {1.0, 0.5, 0.2};

    // ---- the follower ----
    public static double MAX_POWER = 0.85;
    /** Speed to power: in/s per unit forward, and the strafe's share of it (the drive test of 2026-11-07: about 0.80). The dead zones are GridFollower's KS values. */
    public static double FF_GAIN = 95.0, STRAFE_RATIO = 0.80;
    /** Feedback on the speed (in/s of command per in/s of error). */
    public static double SPEED_KP = 1.0;
    /** Sideways correction, in speed: in/s per inch off the path, per in/s of sideways speed, and its limit (in/s). */
    public static double CROSS_KP = 5.5, CROSS_KD = 1.1, CROSS_MAX = 37.0;
    /** On an arc the command leads the turn by this many seconds of sideways acceleration (the drive's lag: the wheel model's rise time is 0.17 s). */
    public static double CENTRIPETAL_SEC = 0.17;
    /** The arc's sideways push is read this many seconds ahead on the path: the drive answers late, so the push must start before the arc does and end before it ends. */
    public static double ARC_LEAD_SEC = 0.0;   // 0.08 with CROSS_KP 9 and CENTRIPETAL_SEC 0.25 were tried on 2026-11-17: the most off the path fell from 3.0 to about 2.4 in; not adopted (no safety gate run with them)
    /** The profile is read this many seconds ahead (the loop and the Pinpoint are late by about one loop). */
    public static double LOOKAHEAD_SEC = 0.05;
    /** Inside this of the goal the robot drives straight at the goal point (in). */
    public static double PURSUIT_IN = 10.0;
    /** The target lock takes the heading this close to a cell's launch spot (in): later than GridFollower's 20, the robot arrives already turned. */
    public static double LOCK_RADIUS_IN = 10.0;

    // ================================================================ clearance

    private static final double WALL_IN = GridRoutes.WALL_IN, SQRT2 = Math.sqrt(2.0);
    private static final HiveRoute.Params ROBOT = new HiveRoute.Params();
    private static final double MID = (ROBOT.frontIn() - ROBOT.fromBackIn) / 2.0, HALF_L = ROBOT.depthIn / 2.0, HALF_W = ROBOT.widthIn / 2.0;

    /**
     * The supports and the flower stands HELD READY: each one's center, axis, half sizes, circle and corners, as it is and grown by the drift margin, worked out
     * once (again only when GridRoutes.DRIFT_IN changes), so that a clearance check allocates nothing.
     */
    private static final class Shapes {
        final double drift;
        final int n;
        final boolean[] flower;
        final double[] cx, cy, ux, uy, hx, hy, rad, hxG, hyG, radG;
        final double[][] corners, cornersG;
        final int digest;

        Shapes(double drift) {
            this.drift = drift;
            List<HiveRoute.Rect> all = new ArrayList<>(HiveRoute.keepOuts(ROBOT, false));
            all.addAll(HiveRoute.flowerKeepOuts(0.0));
            n = all.size();
            flower = new boolean[n];
            cx = new double[n];
            cy = new double[n];
            ux = new double[n];
            uy = new double[n];
            hx = new double[n];
            hy = new double[n];
            rad = new double[n];
            hxG = new double[n];
            hyG = new double[n];
            radG = new double[n];
            corners = new double[n][];
            cornersG = new double[n][];
            int dg = 17;
            for (int i = 0; i < n; i++) {
                HiveRoute.Rect r = all.get(i);
                flower[i] = r.flower;
                cx[i] = r.cx;
                cy[i] = r.cy;
                ux[i] = Math.cos(r.ang);
                uy[i] = Math.sin(r.ang);
                hx[i] = r.hx;
                hy[i] = r.hy;
                rad[i] = Math.hypot(r.hx, r.hy);
                hxG[i] = r.hx + drift;
                hyG[i] = r.hy + drift;
                radG[i] = Math.hypot(hxG[i], hyG[i]);
                corners[i] = r.corners();
                cornersG[i] = new HiveRoute.Rect(r.cx, r.cy, hxG[i], hyG[i], r.ang, false, r.flower).corners();
                for (double v : new double[]{r.cx, r.cy, r.hx, r.hy, r.ang}) dg = dg * 31 + (int) Math.round(v * 1000.0);
            }
            digest = dg;
        }
    }

    private static volatile Shapes shapes;

    private static Shapes shapes() {
        Shapes sh = shapes;
        if (sh == null || sh.drift != GridRoutes.DRIFT_IN) {
            sh = new Shapes(GridRoutes.DRIFT_IN);
            shapes = sh;
        }
        return sh;
    }

    /** The distance (in) from a point to the nearest support or flower stand as they are: what the clearance map holds (the offline generator calls this). */
    public static double obstacleDistance(double x, double y) {
        double best = Double.MAX_VALUE;
        Shapes sh = shapes();
        for (int i = 0; i < sh.n; i++) {
            double lx = (x - sh.cx[i]) * sh.ux[i] + (y - sh.cy[i]) * sh.uy[i], ly = -(x - sh.cx[i]) * sh.uy[i] + (y - sh.cy[i]) * sh.ux[i];
            best = Math.min(best, Math.hypot(Math.max(Math.abs(lx) - sh.hx[i], 0.0), Math.max(Math.abs(ly) - sh.hy[i], 0.0)));
        }
        return best;
    }

    /**
     * HiveRoute.overlap's test (touching counts) for the footprint (corners k, center (fx, fy), axis (co, si), half sizes a and b) and a rectangle held ready,
     * with no garbage: on each rectangle's own axes its extent is its center plus or minus its half size.
     */
    private static boolean hit(double[] k, double fx, double fy, double co, double si, double a, double b,
                               double ox, double oy, double oux, double ouy, double ohx, double ohy, double[] oc) {
        double p = fx * co + fy * si, mn = Double.MAX_VALUE, mx = -Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            double v = oc[i] * co + oc[i + 1] * si;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (p + a < mn || mx < p - a) return false;
        p = -fx * si + fy * co;
        mn = Double.MAX_VALUE;
        mx = -Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            double v = -oc[i] * si + oc[i + 1] * co;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (p + b < mn || mx < p - b) return false;
        p = ox * oux + oy * ouy;
        mn = Double.MAX_VALUE;
        mx = -Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            double v = k[i] * oux + k[i + 1] * ouy;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        if (p + ohx < mn || mx < p - ohx) return false;
        p = -ox * ouy + oy * oux;
        mn = Double.MAX_VALUE;
        mx = -Double.MAX_VALUE;
        for (int i = 0; i < 8; i += 2) {
            double v = -k[i] * ouy + k[i + 1] * oux;
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        return !(p + ohy < mn || mx < p - ohy);
    }

    // ================================================================ the generated tables (SmoothTables) and when they are stale

    /** Bump when the planning logic changes in a way the values of {@link #planInputs} do not show: the plan tables are then stale. */
    public static final int PLAN_LOGIC = 1;
    /** The plan tables' starts: every START_PITCH_IN inches from -GridRoutes.HALF_SPAN_IN to +HALF_SPAN_IN on both axes (START_N per axis), at START_HEADINGS headings. */
    public static final double START_PITCH_IN = 3.0;
    public static final int START_N = (int) Math.round(2.0 * GridRoutes.HALF_SPAN_IN / START_PITCH_IN) + 1, START_HEADINGS = 16;
    /** The clearance map: a value per inch from -MAP_SPAN_IN to +MAP_SPAN_IN on both axes, in quarter inches (rounded down, at most 63.75 in). */
    public static final int MAP_SPAN_IN = 71, MAP_N = 2 * MAP_SPAN_IN + 1;
    /** False: the clearance map is not used (every check goes obstacle by obstacle); for timing its worth. */
    public static boolean USE_CLEARANCE_MAP = true;

    private static final String[] PLAN_KEYS = {"drift", "goal allowance", "allowance taper", "top speed", "start turn margin", "wall", "sample", "check step", "arc accel",
            "least radius", "sharpest corner", "braking", "stop braking", "predicted accel", "predicted turn rate", "turn settle", "margin", "goal growth", "tracking allowance",
            "hop zone", "heading tolerance", "cruise tolerance", "turn zone base", "turn zone per rad", "zone slop", "cruise least length", "arc turn rate", "power", "speed per power",
            "strafe ratio", "lock radius"};

    private static double[] planValues() {
        return new double[]{GridRoutes.DRIFT_IN, GridRoutes.E_GOAL, GridRoutes.E_TAPER, GridRoutes.V_MAX, GridRoutes.START_TURN_MARGIN_IN, WALL_IN, SAMPLE_IN, CHECK_STEP_IN, A_LAT,
                RADIUS_MIN_IN, MAX_CORNER_DEG, A_DEC, STOP_DECEL, A_ACC_PREDICT, TURN_RATE_PREDICT, TURN_SETTLE_SEC, MARGIN_IN, GOAL_GROW_IN, TRACK_IN,
                HOP_ZONE_IN, HEADING_TOL_DEG, CRUISE_TOL_DEG, TURN_BASE_IN, TURN_IN_PER_RAD, ZONE_SLOP_IN, CRUISE_MIN_IN, ARC_TURN_RATE, MAX_POWER, FF_GAIN,
                STRAFE_RATIO, LOCK_RADIUS_IN};
    }

    private static GridRoutes.Table[] digestOf;
    private static int treesDigest;

    /** A digest of the route trees in force (their goals, aims and every state): plan tables made for other trees are stale. */
    private static synchronized int treesDigest() {
        GridRoutes.Table[] ts = GridRoutes.tables();
        if (ts == digestOf) return treesDigest;
        int d = 17;
        if (ts != null) {
            for (GridRoutes.Table t : ts) {
                d = d * 31 + t.name.hashCode();
                d = d * 31 + (int) Math.round(t.goalX * 100.0);
                d = d * 31 + (int) Math.round(t.goalY * 100.0);
                d = d * 31 + (Double.isNaN(t.aimHeading) ? 7 : (int) Math.round(t.aimHeading * 10000.0));
                for (int i = 0; i < t.next.length; i++) d = d * 31 + t.next[i] + 131 * Float.floatToIntBits(t.endSpeed[i]) + 17 * Float.floatToIntBits(t.cost[i]);
            }
        }
        digestOf = ts;
        treesDigest = d;
        return d;
    }

    /** Everything the plan tables depend on, as the generated file records it ("name value; ..."): a difference makes them stale. */
    public static String planInputs() {
        StringBuilder b = new StringBuilder();
        b.append(String.format(Locale.US, "logic %d; starts %.1f in x %d headings; robot %.2f x %.2f back %.3f height %.1f; obstacles %08x; route trees %08x", PLAN_LOGIC, START_PITCH_IN,
                START_HEADINGS, ROBOT.depthIn, ROBOT.widthIn, ROBOT.fromBackIn, ROBOT.robotHeightIn, shapes().digest, treesDigest()));
        double[] v = planValues();
        for (int i = 0; i < v.length; i++) b.append(String.format(Locale.US, "; %s %.3f", PLAN_KEYS[i], v[i]));
        return b.toString();
    }

    /** Everything the clearance map depends on. */
    public static String fieldInputs() {
        return String.format(Locale.US, "span %d in; pitch 1 in; quarter inches; obstacles %08x", MAP_SPAN_IN, shapes().digest);
    }

    /** The parts of two inputs texts that differ, as "name: was X, now Y". */
    public static String differences(String was, String now) {
        String[] a = was.split("; "), b = now.split("; ");
        if (a.length != b.length) return "made by another version (" + (was.length() > 60 ? was.substring(0, 60) + "..." : was) + ")";
        StringBuilder d = new StringBuilder();
        for (int i = 0; i < a.length; i++) {
            if (a[i].equals(b[i])) continue;
            int sa = a[i].lastIndexOf(' '), sb = b[i].lastIndexOf(' ');
            d.append(d.length() == 0 ? "" : ", ").append(b[i], 0, Math.max(0, sb)).append(": was ").append(a[i].substring(sa + 1)).append(", now ").append(b[i].substring(sb + 1));
        }
        return d.toString();
    }

    private static boolean checked = false;
    private static long checkedFor;
    private static String plansProblem, mapProblem;
    private static short[][] plans;
    private static String[] planNames;
    private static byte[] map;

    private static long liveHash() {
        long h = 17 + System.identityHashCode(GridRoutes.tables());
        for (double v : planValues()) h = h * 31 + Double.doubleToLongBits(v);
        return h;
    }

    /**
     * Null when the generated plan tables fit the values in force, else why they are stale and what to run. Checked again whenever one of the values changes
     * (the Dashboard), so a stale table is never used: the dash then drives leg by leg.
     */
    public static synchronized String tablesProblem() {
        long h = liveHash();
        if (checked && h == checkedFor) return plansProblem;
        checked = true;
        checkedFor = h;
        String now = planInputs();
        plansProblem = null;
        if (!now.equals(SmoothTables.PLAN_INPUTS)) {
            plansProblem = "the smooth plan tables are STALE (" + differences(SmoothTables.PLAN_INPUTS, now) + "): regenerate with tools\\dash\\run-sim.cmd gentables";
            plans = null;
        } else if (plans == null) {
            planNames = SmoothTables.NAMES;
            plans = new short[SmoothTables.PLANS.length][];
            for (int t = 0; t < plans.length; t++) {
                // a table is several texts (a Java text constant holds at most 65535 bytes), each a whole number of entries
                int total = 0;
                byte[][] raw = new byte[SmoothTables.PLANS[t].length][];
                for (int c = 0; c < raw.length; c++) {
                    raw[c] = GridRoutes.unbase64(SmoothTables.PLANS[t][c]);
                    total += raw[c].length / 2;
                }
                plans[t] = new short[total];
                int at = 0;
                for (byte[] r : raw) for (int i = 0; i + 1 < r.length; i += 2) plans[t][at++] = (short) (((r[i] & 0xFF) << 8) | (r[i + 1] & 0xFF));
            }
        }
        String fnow = fieldInputs();
        mapProblem = fnow.equals(SmoothTables.FIELD_INPUTS) ? null
                : "the clearance map is STALE (" + differences(SmoothTables.FIELD_INPUTS, fnow) + "): regenerate with tools\\dash\\run-sim.cmd gentables";
        map = mapProblem == null ? GridRoutes.unbase64(SmoothTables.FIELD) : null;
        if (map != null && map.length != MAP_N * MAP_N) {
            mapProblem = "the clearance map has " + map.length + " values, not " + MAP_N * MAP_N + ": regenerate with tools\\dash\\run-sim.cmd gentables";
            map = null;
        }
        return plansProblem;
    }

    /** Null when the clearance map fits the supports and flower stands in the code; a stale map is not used (the checks then go obstacle by obstacle, slower, the same answers). */
    public static synchronized String mapProblem() {
        tablesProblem();
        return mapProblem;
    }

    /** The plan codes in force for a goal's table (one per grid node and start heading), or null: for the offline check that compares them with a fresh generation. */
    public static synchronized short[] tablePlans(String name) {
        if (tablesProblem() != null || plans == null) return null;
        for (int t = 0; t < planNames.length; t++) if (planNames[t].equals(name)) return plans[t];
        return null;
    }

    /** The clearance map in force, or null. */
    public static synchronized byte[] tableMap() {
        tablesProblem();
        return map;
    }

    /** The obstacles, the start's allowances and the heading zones of one plan. */
    private static final class Zones {
        final Shapes sh = shapes();
        final byte[] clear = USE_CLEARANCE_MAP ? tableMap() : null;
        /** The footprint in hand: its corners. */
        final double[] k = new double[8];
        final double x0, y0, h0, gx, gy;
        final double[] hopGrow, hopRad;
        final double wallX0, wallY0;
        boolean startOk = true;
        // one plan
        double hf, hc, tol, ctol, off;
        boolean cruise, follow, turning;
        double holdIn, turn1End, turn2Start, endIn, delta1, delta2, zone = 1.0;
        /** What the last check that failed found (for the log): in words, or null with the check's pose and what it was too near. */
        String fail = "";
        double failX, failY, failH;
        String failWhat = "";
        int failAt = -1;

        Zones(double x0, double y0, double h0, double gx, double gy) {
            this.x0 = x0;
            this.y0 = y0;
            this.h0 = h0;
            this.gx = gx;
            this.gy = gy;
            double d = sh.drift;
            // near the start: per obstacle the clearance the robot has, to the half inch, at most what a hop and its start turn keep; the walls no further out than it is
            double full = d + GridRoutes.START_TURN_MARGIN_IN;   // the hop's drift margin and the start turn's margin, as GridRoutes checks them
            hopGrow = new double[sh.n];
            hopRad = new double[sh.n];
            double co = Math.cos(h0), si = Math.sin(h0), cx = x0 + co * MID, cy = y0 + si * MID;
            for (int i = 0; i < sh.n; i++) {
                double q = full;
                while (q >= 0.0) {
                    corners(cx, cy, co, si, HALF_L + q, HALF_W + q);
                    if (!hit(k, cx, cy, co, si, HALF_L + q, HALF_W + q, sh.cx[i], sh.cy[i], sh.ux[i], sh.uy[i], sh.hx[i], sh.hy[i], sh.corners[i])) break;
                    q -= 0.5;
                }
                if (q < 0.0) startOk = false;   // the pose overlaps it: the move-clear leg's case, not this follower's
                hopGrow[i] = Math.max(0.0, q);
                hopRad[i] = Math.hypot(HALF_L + hopGrow[i], HALF_W + hopGrow[i]);
            }
            corners(cx, cy, co, si, HALF_L, HALF_W);
            double mx = 0.0, my = 0.0;
            for (int i = 0; i < 8; i += 2) {
                mx = Math.max(mx, Math.abs(k[i]));
                my = Math.max(my, Math.abs(k[i + 1]));
            }
            wallX0 = Math.max(WALL_IN - d, mx + 1e-6);
            wallY0 = Math.max(WALL_IN - d, my + 1e-6);
        }

        /** The footprint's corners into k: center (cx, cy), axis (co, si), half sizes a (along) and b (across). */
        void corners(double cx, double cy, double co, double si, double a, double b) {
            k[0] = cx + co * a - si * b;
            k[1] = cy + si * a + co * b;
            k[2] = cx - co * a - si * b;
            k[3] = cy - si * a + co * b;
            k[4] = cx - co * a + si * b;
            k[5] = cy - si * a - co * b;
            k[6] = cx + co * a + si * b;
            k[7] = cy + si * a - co * b;
        }

        /** A DIRECT plan: one turn to hf, from holdIn out, done by endIn before the goal. */
        void direct(double hf, double holdIn, double endIn, double zone) {
            cruise = false;
            follow = false;
            this.zone = zone;
            this.hf = hf;
            tol = Math.toRadians(HEADING_TOL_DEG);
            ctol = tol;
            delta1 = Rotation2d.normalize(hf - h0);
            delta2 = 0.0;
            turning = Math.abs(delta1) > tol;
            this.holdIn = holdIn;
            turn1End = holdIn + TURN_BASE_IN + zone * TURN_IN_PER_RAD * Math.abs(delta1);
            turn2Start = 0.0;
            // a start nearer the goal than the FINAL zone still needs somewhere to turn
            this.endIn = Math.max(0.0, Math.min(endIn, Math.hypot(gx - x0, gy - y0) - 4.0));
        }

        /** A CRUISE plan: a turn to the heading hc from holdIn out, hc kept, and a turn to hf near the goal, done by endIn before it. */
        void cruise(double hf, double hc, double holdIn, double endIn, double zone) {
            cruise = true;
            follow = false;
            this.zone = zone;
            this.hf = hf;
            this.hc = hc;
            tol = Math.toRadians(HEADING_TOL_DEG);
            ctol = Math.toRadians(CRUISE_TOL_DEG);
            this.holdIn = holdIn;
            this.endIn = endIn;
            delta1 = Rotation2d.normalize(hc - h0);
            delta2 = Rotation2d.normalize(hf - hc);
            turning = Math.abs(delta1) > 1e-6;
            turn1End = holdIn + TURN_BASE_IN + zone * TURN_IN_PER_RAD * Math.abs(delta1);
            turn2Start = endIn + TURN_BASE_IN + zone * TURN_IN_PER_RAD * Math.abs(delta2);
        }

        /** A FOLLOW plan: as a cruise plan, the cruise heading being the path's direction plus off (firstDir where the path starts, lastDir where it ends). */
        void follow(double hf, double off, double firstDir, double lastDir, double holdIn, double endIn, double zone) {
            cruise(hf, firstDir + off, holdIn, endIn, zone);
            follow = true;
            this.off = off;
            legs(firstDir, lastDir);
        }

        /** A FOLLOW plan's two turns for the first and the last leg's directions (they are known only once the path's points are chosen). */
        void legs(double firstDir, double lastDir) {
            if (!follow) return;
            delta1 = Rotation2d.normalize(firstDir + off - h0);
            delta2 = Rotation2d.normalize(hf - lastDir - off);
            turning = Math.abs(delta1) > 1e-6;
            turn1End = holdIn + TURN_BASE_IN + zone * TURN_IN_PER_RAD * Math.abs(delta1);
            turn2Start = endIn + TURN_BASE_IN + zone * TURN_IN_PER_RAD * Math.abs(delta2);
        }

        /** There is somewhere to turn: the first turn starts outside the zone that must be entered already turned (else the robot would wait there for a turn it has not begun). */
        boolean roomToTurn() {
            double dg0 = Math.hypot(gx - x0, gy - y0);
            boolean ok = cruise ? (turning ? dg0 > turn2Start + holdIn + 3.0 : dg0 - endIn >= 3.0) : !turning || endIn == 0.0 || dg0 - endIn - holdIn >= 3.0;
            if (!ok) fail = String.format(Locale.US, "no room to turn: the goal is %.0f in away, the turn starts %.0f in out and must be done %.0f in before the goal", dg0, holdIn, cruise ? turn2Start : endIn);
            return ok;
        }

        /** The footprint at a heading, grown by g, is clear at (x, y), d0 from the start. */
        boolean fits(double x, double y, double h, double g, double d0) {
            // most checks are far from everything: the clearance map and the circles first, the rectangles only when those meet
            double co = Math.cos(h), si = Math.sin(h), cx = x + co * MID, cy = y + si * MID, reach = Math.max(Math.abs(cx), Math.abs(cy));
            if (d0 < HOP_ZONE_IN) {
                if (reach + Math.hypot(HALF_L, HALF_W) > Math.min(wallX0, wallY0)) {
                    corners(cx, cy, co, si, HALF_L, HALF_W);
                    for (int i = 0; i < 8; i += 2) if (Math.abs(k[i]) > wallX0 || Math.abs(k[i + 1]) > wallY0) return failed(x, y, h, "a wall (nearer than at the start)");
                }
                for (int i = 0; i < sh.n; i++) {
                    double ex = cx - sh.cx[i], ey = cy - sh.cy[i], r = hopRad[i] + sh.rad[i];
                    if (ex * ex + ey * ey > r * r) continue;
                    double a = HALF_L + hopGrow[i], b = HALF_W + hopGrow[i];
                    corners(cx, cy, co, si, a, b);
                    if (hit(k, cx, cy, co, si, a, b, sh.cx[i], sh.cy[i], sh.ux[i], sh.uy[i], sh.hx[i], sh.hy[i], sh.corners[i])) return failed(x, y, h, i, " (nearer than at the start)");
                }
                return true;
            }
            double a = HALF_L + g, b = HALF_W + g, rad = Math.hypot(a, b), lim = WALL_IN - sh.drift;
            boolean have = false;
            if (reach + rad > lim) {
                corners(cx, cy, co, si, a, b);
                have = true;
                for (int i = 0; i < 8; i++) if (Math.abs(k[i]) > lim) return failed(x, y, h, "a wall");
            }
            if (clear != null) {
                // the map: nothing is nearer the footprint's center than its circle plus the grown obstacles' reach (a cell is up to 0.71 in from the point)
                int mi = (int) Math.round(cx) + MAP_SPAN_IN, mj = (int) Math.round(cy) + MAP_SPAN_IN;
                if (mi >= 0 && mj >= 0 && mi < MAP_N && mj < MAP_N && (clear[mi * MAP_N + mj] & 0xFF) / 4.0 - 0.75 > rad + sh.drift * SQRT2) return true;
            }
            for (int i = 0; i < sh.n; i++) {
                double ex = cx - sh.cx[i], ey = cy - sh.cy[i], r = rad + sh.radG[i];
                if (ex * ex + ey * ey > r * r) continue;
                if (!have) {
                    corners(cx, cy, co, si, a, b);
                    have = true;
                }
                if (hit(k, cx, cy, co, si, a, b, sh.cx[i], sh.cy[i], sh.ux[i], sh.uy[i], sh.hxG[i], sh.hyG[i], sh.cornersG[i])) return failed(x, y, h, i, "");
            }
            return true;
        }

        /** Every heading from ha to hb (the short way), and pad beyond each end. */
        boolean range(double x, double y, double ha, double hb, double pad, double g, double d0) {
            double d = Rotation2d.normalize(hb - ha), sg = d < 0.0 ? -1.0 : 1.0;
            double from = ha - sg * pad, span = Math.abs(d) + 2.0 * pad;
            int steps = Math.max(1, (int) Math.ceil(span / Math.toRadians(10.0)));
            if (span < 1e-9) return fits(x, y, ha, g, d0);
            for (int i = 0; i <= steps; i++) if (!fits(x, y, from + sg * span * i / steps, g, d0)) return false;
            return true;
        }

        boolean failed(double x, double y, double h, String what) {
            failX = x;
            failY = y;
            failH = h;
            failWhat = what;
            failAt = -1;
            fail = null;
            return false;
        }

        /** As above for obstacle i (its name is put together only when the text is asked for). */
        boolean failed(double x, double y, double h, int i, String more) {
            failed(x, y, h, more);
            failAt = i;
            return false;
        }

        /** The last failed check in words (built when asked for: it is not free, and most failures are of tries that are not reported). */
        String why() {
            if (fail != null) return fail;
            double x = failX, y = failY, h = failH;
            String what = failAt < 0 ? failWhat : String.format(Locale.US, "the %s at (%.0f, %.0f)%s", sh.flower[failAt] ? "flower stand" : "support", sh.cx[failAt], sh.cy[failAt], failWhat);
            double d0 = Math.hypot(x - x0, y - y0), dg = Math.hypot(x - gx, y - gy);
            fail = String.format(Locale.US, "at (%.0f, %.0f) turned to %.0f deg, %.0f in from the start and %.0f in from the goal: too near %s", x, y,
                    Math.toDegrees(Rotation2d.normalize(h)), d0, dg, what);
            return fail;
        }

        /**
         * A point of the path: clear at every heading the robot may have there. tA, tB: the path's direction there (two on an arc: its legs').
         * started: the first turn has begun on the path so far (updated).
         */
        boolean poseOk(double x, double y, double tA, double tB, boolean[] started) {
            double d0 = Math.hypot(x - x0, y - y0), dg = Math.hypot(x - gx, y - gy);
            double g = MARGIN_IN + allowance(x, y, gx, gy);
            if (!cruise && !turning) return fits(x, y, h0, g, d0) && (hf == h0 || fits(x, y, hf, g, d0));
            if (!started[0] && (d0 >= holdIn || !turning)) started[0] = true;
            if (!started[0]) return fits(x, y, h0, g, d0);
            boolean fin = dg <= endIn - ZONE_SLOP_IN || (!cruise && d0 >= turn1End + ZONE_SLOP_IN);
            if (fin) return range(x, y, hf, hf, dg <= LOCK_RADIUS_IN ? 0.0 : tol, g, d0);
            if (!cruise) return range(x, y, h0, hf, 0.0, g, d0) && fits(x, y, hf + Math.signum(delta1) * tol, g, d0);
            double cA = follow ? tA + off : hc, cB = follow ? tB + off : hc;
            if (!range(x, y, cA, cB, ctol, g, d0)) return false;
            if (turning && d0 < turn1End + ZONE_SLOP_IN && !(range(x, y, h0, cA, 0.0, g, d0) && (cB == cA || range(x, y, h0, cB, 0.0, g, d0)))) return false;
            return dg > turn2Start + ZONE_SLOP_IN || (range(x, y, cA, hf, 0.0, g, d0) && (cB == cA || range(x, y, cB, hf, 0.0, g, d0)));
        }

        boolean goalFits(double h) {
            double co = Math.cos(h), si = Math.sin(h), cx = gx + co * MID, cy = gy + si * MID, a = HALF_L + GOAL_GROW_IN, b = HALF_W + GOAL_GROW_IN;
            corners(cx, cy, co, si, a, b);
            for (int i = 0; i < 8; i++) if (Math.abs(k[i]) > WALL_IN) return failed(gx, gy, h, "a wall (at the goal)");
            for (int i = 0; i < sh.n; i++) {
                if (hit(k, cx, cy, co, si, a, b, sh.cx[i], sh.cy[i], sh.ux[i], sh.uy[i], sh.hx[i], sh.hy[i], sh.corners[i])) return failed(gx, gy, h, i, " (at the goal)");
            }
            return true;
        }

        /** The goal itself: the footprint there against the obstacles and the walls as they are (RouteGen's rule). */
        boolean goalOk() {
            if (cruise || !turning || endIn > 0.0) return goalFits(hf);
            double d = Rotation2d.normalize(hf - h0);
            int steps = Math.max(1, (int) Math.ceil(Math.abs(d) / Math.toRadians(10.0)));
            for (int i = 0; i <= steps; i++) if (!goalFits(h0 + d * i / steps)) return false;
            return true;
        }

        boolean segOk(double ax, double ay, double bx, double by, boolean[] started, boolean endsAtGoal) {
            double len = Math.hypot(bx - ax, by - ay), dir = Math.atan2(by - ay, bx - ax);
            int steps = Math.max(1, (int) Math.ceil(len / CHECK_STEP_IN));
            for (int k = 1; k <= steps; k++) {
                if (k == steps && endsAtGoal) return goalOk();
                if (!poseOk(ax + (bx - ax) * k / steps, ay + (by - ay) * k / steps, dir, dir, started)) return false;
            }
            return true;
        }
    }

    // ================================================================ building the path

    /** Either end of the axis along ang, the nearer to ref. */
    private static double nearEnd(double ang, double ref) {
        return Math.abs(Rotation2d.normalize(ang - ref)) <= Math.abs(Rotation2d.normalize(ang + Math.PI - ref)) ? ang : ang + Math.PI;
    }

    /** Why the last {@link #build} that returned null found no smooth path (for the log). */
    public static String lastWhy = "";

    private static final int DIRECT = 1, CRUISE = 2, FOLLOW = 3;

    /** A plan as a number (0: none): its kind, which goal heading, which cruise heading or leading side, and where its turns start and end. */
    private static int code(int kind, int fin, int q, int hi, int zi, int ei) {
        return kind | fin << 2 | q << 5 | hi << 8 | zi << 11 | ei << 13;
    }

    /** The goal headings a park corner may be reached with, by number: the one the robot has, either end of the last leg, along the wall, square to it. */
    private static double[] finals(double[] xs, double[] ys, double h0) {
        int n = xs.length;
        double last = Math.atan2(ys[n - 1] - ys[n - 2], xs[n - 1] - xs[n - 2]);
        return new double[]{h0, nearEnd(last, h0), nearEnd(last, h0) + Math.PI, nearEnd(0.0, h0), nearEnd(Math.PI / 2.0, h0)};
    }

    private static double routeLength(double[] xs, double[] ys) {
        double len = 0.0;
        for (int i = 1; i < xs.length; i++) len += Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
        return len;
    }

    /**
     * Puts one plan into z: kind, goal heading hf, cruise heading or leading side q, and the numbers of its hold, zone and end among those tried. False when
     * there is no such plan from this start (a cruise heading that is the goal's or the robot's own, a path too short to cruise on).
     */
    private static boolean setPlan(Zones z, int kind, double hf, int q, int hi, int zi, int ei, double[] xs, double[] ys, double h0) {
        int n = xs.length;
        if (hi >= HOLD_TRY_IN.length || zi >= ZONE_TRY.length || ei >= END_TRY_IN.length) return false;
        if (kind == DIRECT) {
            boolean turning = Math.abs(Rotation2d.normalize(hf - h0)) > Math.toRadians(HEADING_TOL_DEG);
            z.direct(hf, turning ? HOLD_TRY_IN[hi] : 0.0, turning ? END_TRY_IN[ei] : 0.0, turning ? ZONE_TRY[zi] : 1.0);
            return true;
        }
        if (routeLength(xs, ys) < CRUISE_MIN_IN) return false;
        if (kind == CRUISE) {
            // the heading the robot has, or the front, the left side, the rear or the right side along the longest leg
            double mainDir = 0.0, longest = -1.0;
            for (int i = 1; i < n; i++) {
                double l = Math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
                if (l > longest) {
                    longest = l;
                    mainDir = Math.atan2(ys[i] - ys[i - 1], xs[i] - xs[i - 1]);
                }
            }
            if (q > 4) return false;
            double hc = q == 0 ? h0 : mainDir + (q == 1 ? 0.0 : q == 2 ? -Math.PI / 2.0 : q == 3 ? Math.PI : Math.PI / 2.0);
            if (Math.abs(Rotation2d.normalize(hc - hf)) <= Math.toRadians(HEADING_TOL_DEG)) return false;   // the direct plan
            if (q > 0 && Math.abs(Rotation2d.normalize(hc - h0)) <= Math.toRadians(CRUISE_TOL_DEG)) return false;   // the first candidate
            z.cruise(hf, hc, q > 0 ? HOLD_TRY_IN[hi] : 0.0, END_TRY_IN[ei], ZONE_TRY[zi]);
            return true;
        }
        if (kind != FOLLOW || q > 3) return false;
        z.follow(hf, -q * Math.PI / 2.0, Math.atan2(ys[1] - ys[0], xs[1] - xs[0]), Math.atan2(ys[n - 1] - ys[n - 2], xs[n - 1] - xs[n - 2]), HOLD_TRY_IN[hi], END_TRY_IN[ei], ZONE_TRY[zi]);
        return true;
    }

    /** The first plan of a kind that is clear, trying its holds, zones and ends in order (nh, nz, ne of them), or null. */
    private static SmoothFollower firstClear(Zones z, int kind, int fin, double hf, int q, int nh, int nz, int ne, double[] xs, double[] ys, double h0, boolean aim, double goalTolIn) {
        for (int hi = 0; hi < nh; hi++) {
            for (int zi = 0; zi < nz; zi++) {
                for (int ei = 0; ei < ne; ei++) {
                    if (!setPlan(z, kind, hf, q, hi, zi, ei, xs, ys, h0)) return null;
                    z.fail = "";
                    SmoothFollower f = tryBuild(z, xs, ys, aim, goalTolIn);
                    if (f != null) {
                        f.code = code(kind, fin, q, hi, zi, ei);
                        return f;
                    }
                }
            }
        }
        return null;
    }

    /**
     * THE SEARCH (offline: the table generator and the PC studies; it tries a few hundred plans and is too slow for the robot's loop): the quickest clear smooth
     * path for a grid route (field frame: the robot, the route's points, the goal), or null. aimHeading: the heading at the goal (a cell), or NaN (a park corner:
     * the heading the robot has when the footprint fits at the corner with it, else the nearest that does).
     */
    public static SmoothFollower search(double[] xs, double[] ys, double aimHeading, double goalTolIn, double h0) {
        int n = xs.length;
        lastWhy = "the route has no legs";
        if (n < 2) return null;
        Zones z = new Zones(xs[0], ys[0], h0, xs[n - 1], ys[n - 1]);
        lastWhy = "the pose overlaps a support or a flower stand";
        if (!z.startOk) return null;
        boolean aim = !Double.isNaN(aimHeading);
        double[] c = aim ? new double[]{aimHeading} : finals(xs, ys, h0);
        Integer[] order = new Integer[c.length];
        for (int i = 0; i < c.length; i++) order[i] = i;
        final double[] cc = c;
        java.util.Arrays.sort(order, (a, b) -> Double.compare(Math.abs(Rotation2d.normalize(cc[a] - h0)), Math.abs(Rotation2d.normalize(cc[b] - h0))));
        boolean any = false;
        for (int fin : order) {
            double hf = c[fin];
            if (!aim && !z.goalFits(hf)) continue;
            any = true;
            boolean turning = Math.abs(Rotation2d.normalize(hf - h0)) > Math.toRadians(HEADING_TOL_DEG);
            SmoothFollower best = firstClear(z, DIRECT, fin, hf, 0, turning ? HOLD_TRY_IN.length : 1, turning ? ZONE_TRY.length : 1, turning ? END_TRY_IN.length : 1, xs, ys, h0, aim, goalTolIn);
            for (int q = 0; q <= 4; q++) {
                SmoothFollower f = firstClear(z, CRUISE, fin, hf, q, q > 0 ? HOLD_TRY_IN.length : 1, ZONE_TRY.length, END_TRY_IN.length, xs, ys, h0, aim, goalTolIn);
                if (f != null && (best == null || f.predictedSec < best.predictedSec)) best = f;
            }
            for (int q = 0; q <= 3; q++) {
                SmoothFollower f = firstClear(z, FOLLOW, fin, hf, q, HOLD_TRY_IN.length, ZONE_TRY.length, END_TRY_IN.length, xs, ys, h0, aim, goalTolIn);
                if (f != null && (best == null || f.predictedSec < best.predictedSec)) best = f;
            }
            if (best != null) return best;   // a park corner: the nearest heading that fits and has a plan
        }
        lastWhy = any ? "no plan is clear; the last one tried failed " + z.why() : "no heading fits at the goal";
        return null;
    }

    /** One plan, from its code, for this route and start: the path, or null when it is not clear from here. */
    private static SmoothFollower fromCode(Zones z, int code, double[] xs, double[] ys, double aimHeading, double goalTolIn, double h0) {
        int kind = code & 3, fin = code >> 2 & 7, q = code >> 5 & 7, hi = code >> 8 & 7, zi = code >> 11 & 3, ei = code >> 13 & 3;
        boolean aim = !Double.isNaN(aimHeading);
        double hf = aimHeading;
        if (!aim) {
            double[] c = finals(xs, ys, h0);
            if (fin >= c.length || !z.goalFits(c[fin])) return null;
            hf = c[fin];
        }
        if (kind == 0 || !setPlan(z, kind, hf, q, hi, zi, ei, xs, ys, h0)) return null;
        z.fail = "";
        SmoothFollower f = tryBuild(z, xs, ys, aim, goalTolIn);
        if (f != null) f.code = code;
        return f;
    }

    /** The plans of the starts next to the robot's are tried too, this many in all at most (each is one check of one path). */
    public static int MAX_TABLE_TRIES = 10;
    /** When the table's plan is not clear from the exact pose: the same plan with a later hold, a shorter turn zone or a longer final zone (steps of hold, zone, end). */
    private static final int[][] VARIANTS = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 0}, {0, 1, 1}, {2, 0, 0}};

    /**
     * THE ROBOT'S WAY to a smooth path: the plan the generated table holds for the grid node and start heading nearest the robot (tableName: the goal's table;
     * bx, by, bh: the robot's pose in the tables' blue frame), checked here for the robot's exact pose; when it is not clear from there, the plans of the
     * neighbouring starts. Null when none is, when the table has no smooth path from here, or when the tables are stale ({@link #lastWhy} says which).
     */
    public static SmoothFollower build(String tableName, double bx, double by, double bh, double[] xs, double[] ys, double aimHeading, double goalTolIn, double h0) {
        int n = xs.length;
        lastWhy = "the route has no legs";
        if (n < 2) return null;
        lastWhy = tablesProblem();
        if (lastWhy != null) return null;
        short[] table = tablePlans(tableName);
        lastWhy = "there is no plan table for " + tableName + ": regenerate with tools\\dash\\run-sim.cmd gentables";
        if (table == null) return null;
        Zones z = new Zones(xs[0], ys[0], h0, xs[n - 1], ys[n - 1]);
        lastWhy = "the pose overlaps a support or a flower stand";
        if (!z.startOk) return null;
        int N = START_N, i0 = (int) Math.round((bx + GridRoutes.HALF_SPAN_IN) / START_PITCH_IN), j0 = (int) Math.round((by + GridRoutes.HALF_SPAN_IN) / START_PITCH_IN);
        if (table.length != N * N * START_HEADINGS) {
            lastWhy = "the plan table for " + tableName + " has " + table.length + " starts, not " + N * N * START_HEADINGS + ": regenerate with tools\\dash\\run-sim.cmd gentables";
            return null;
        }
        double bin = 2.0 * Math.PI / START_HEADINGS, kf = bh / bin;
        int k0 = (int) Math.round(kf), side = kf >= k0 ? 1 : -1;
        int[][] near = {{0, 0, 0}, {0, 0, side}, {0, 0, -side}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}};
        int tried = 0;
        int[] seen = new int[near.length + VARIANTS.length];
        for (int[] d : near) {
            int i = Math.max(0, Math.min(N - 1, i0 + d[0])), j = Math.max(0, Math.min(N - 1, j0 + d[1])), k = ((k0 + d[2]) % START_HEADINGS + START_HEADINGS) % START_HEADINGS;
            int code = table[(i * N + j) * START_HEADINGS + k] & 0xFFFF;
            boolean again = code == 0;
            for (int t = 0; t < tried; t++) if (seen[t] == code) again = true;
            if (again) continue;
            seen[tried++] = code;
            SmoothFollower f = fromCode(z, code, xs, ys, aimHeading, goalTolIn, h0);
            if (f != null) return f;
            if (tried == 1) {
                // the nearest start's plan first in its near variants: the exact pose mostly needs the turn a little later or shorter
                int hi = code >> 8 & 7, zi = code >> 11 & 3, ei = code >> 13 & 3;
                for (int[] v : VARIANTS) {
                    if (tried >= MAX_TABLE_TRIES) break;
                    if (hi + v[0] > 7 || zi + v[1] > 3 || ei + v[2] > 3) continue;
                    tried++;
                    f = fromCode(z, code(code & 3, code >> 2 & 7, code >> 5 & 7, hi + v[0], zi + v[1], ei + v[2]), xs, ys, aimHeading, goalTolIn, h0);
                    if (f != null) return f;
                }
            }
            if (tried >= MAX_TABLE_TRIES) break;
        }
        lastWhy = tried == 0 ? "the plan table has no smooth path from here" : "the table's plan" + (tried > 1 ? "s (" + tried + ") are" : " is") + " not clear from this exact pose: " + z.why();
        return null;
    }

    private static SmoothFollower tryBuild(Zones z, double[] xs, double[] ys, boolean aim, double goalTolIn) {
        int n = xs.length;
        // 1. the fewest of the route's points that keep the path clear
        boolean[] st = {false};
        double lastDir = Math.atan2(ys[n - 1] - ys[n - 2], xs[n - 1] - xs[n - 2]);
        List<double[]> v = new ArrayList<>();
        v.add(new double[]{xs[0], ys[0]});
        int i = 0;
        while (i < n - 1) {
            boolean keep = st[0], ok = false;
            int j = n - 1;
            for (; j > i; j--) {
                st[0] = keep;
                if (i == 0) {
                    // the first leg kept sets a follow plan's first turn (the route's own first leg is only the hop to its first point)
                    double dir = Math.atan2(ys[j] - ys[0], xs[j] - xs[0]);
                    z.legs(dir, j == n - 1 ? dir : lastDir);
                    st[0] = false;
                    if (!z.poseOk(xs[0], ys[0], dir, dir, st)) continue;
                }
                if (z.segOk(xs[i], ys[i], xs[j], ys[j], st, j == n - 1)) {
                    ok = true;
                    break;
                }
            }
            if (!ok) return null;
            v.add(new double[]{xs[j], ys[j]});
            i = j;
        }
        int kept = v.size() - 1;
        z.legs(Math.atan2(v.get(1)[1] - v.get(0)[1], v.get(1)[0] - v.get(0)[0]), Math.atan2(v.get(kept)[1] - v.get(kept - 1)[1], v.get(kept)[0] - v.get(kept - 1)[0]));
        if (!z.roomToTurn()) return null;
        // 2. an arc at every corner: as wide as wanted for full speed, as the legs allow, and as stays clear
        int m = v.size() - 1;
        double[] len = new double[m + 1], dx = new double[m + 1], dy = new double[m + 1];   // leg k: v[k-1] -> v[k]
        for (int k = 1; k <= m; k++) {
            len[k] = Math.hypot(v.get(k)[0] - v.get(k - 1)[0], v.get(k)[1] - v.get(k - 1)[1]);
            if (len[k] < 1e-6) return null;
            dx[k] = (v.get(k)[0] - v.get(k - 1)[0]) / len[k];
            dy[k] = (v.get(k)[1] - v.get(k - 1)[1]) / len[k];
        }
        double[] phi = new double[m + 1], radius = new double[m + 1];
        double want = GridRoutes.V_MAX * GridRoutes.V_MAX / A_LAT;
        for (int k = 1; k < m; k++) {
            phi[k] = Math.atan2(dx[k] * dy[k + 1] - dy[k] * dx[k + 1], dx[k] * dx[k + 1] + dy[k] * dy[k + 1]);
            if (Math.abs(phi[k]) < Math.toRadians(0.5)) {
                phi[k] = 0.0;
                continue;
            }
            if (Math.abs(phi[k]) > Math.toRadians(MAX_CORNER_DEG)) {
                z.fail = String.format(Locale.US, "a corner of %.0f deg at (%.0f, %.0f)", Math.toDegrees(phi[k]), v.get(k)[0], v.get(k)[1]);
                return null;
            }
            double in = k == 1 ? len[k] - 0.5 : len[k] / 2.0, out = k + 1 == m ? Math.max(len[k + 1] - 4.0, len[k + 1] / 2.0) : len[k + 1] / 2.0;
            radius[k] = Math.min(want, Math.max(0.0, Math.min(in, out)) / Math.tan(Math.abs(phi[k]) / 2.0));
            if (radius[k] < RADIUS_MIN_IN) {
                z.fail = String.format(Locale.US, "no room for an arc at (%.0f, %.0f): a turn of %.0f deg between legs of %.0f and %.0f in", v.get(k)[0], v.get(k)[1],
                        Math.toDegrees(phi[k]), len[k], len[k + 1]);
                return null;
            }
        }
        for (int attempt = 0; attempt < 24; attempt++) {
            SmoothFollower f = new SmoothFollower(z, v, len, dx, dy, phi, radius, aim, goalTolIn);
            int bad = f.verify(z);
            if (bad < 0) return f;
            int k = f.cornerOf[bad];
            if (k <= 0) return null;   // a straight part is not clear on the finer samples
            radius[k] *= 0.6;
            if (radius[k] < RADIUS_MIN_IN) return null;
        }
        return null;
    }

    // ================================================================ one path

    private final double x0, y0, h0, hf, hc, off, gx, gy, tol, ctol, goalTolIn, holdIn, turn1End, turn2Start, endIn, delta1, delta2;
    private final boolean aim, turning, cruise, follow;
    private final double[][] vertices;
    private final int samples;          // the last sample's index
    private final double step, length;
    private final double[] px, py, tx, ty, kappa, vlim, dirA, dirB;
    /** Per sample: the corner whose arc it is on, or 0 (a straight part). */
    private final int[] cornerOf;
    private final int arcs;
    private final double predictedSec, slowestArc;

    private final GridFollower.Output out = new GridFollower.Output();
    private int idx = 0;
    /** The plan's code (see {@link #code()}). */
    private int code = 0;
    private boolean started = false, cruiseDone = false, turnDone = false, arrived = false, sawSpeed = false;
    private double offRouteIn = 0.0, mostOffIn = 0.0;
    private String mostOffWhere = "at the start";

    /** The most speed (in/s) in a direction rho from the robot's front, at MAX_POWER: each wheel's power is forward plus or minus strafe, so a diagonal is the slowest. */
    private static double speedAt(double rho) {
        return Math.min(GridRoutes.V_MAX, MAX_POWER * FF_GAIN / (Math.abs(Math.cos(rho)) + Math.abs(Math.sin(rho)) / STRAFE_RATIO));
    }

    private SmoothFollower(Zones z, List<double[]> v, double[] len, double[] dx, double[] dy, double[] phi, double[] radius, boolean aim, double goalTolIn) {
        this.x0 = z.x0;
        this.y0 = z.y0;
        this.h0 = z.h0;
        this.hf = z.hf;
        this.hc = z.hc;
        this.off = z.off;
        this.follow = z.follow;
        this.gx = z.gx;
        this.gy = z.gy;
        this.tol = z.tol;
        this.ctol = z.ctol;
        this.turning = z.turning;
        this.cruise = z.cruise;
        this.holdIn = z.holdIn;
        this.turn1End = z.turn1End;
        this.turn2Start = z.turn2Start;
        this.endIn = z.endIn;
        this.delta1 = z.delta1;
        this.delta2 = z.delta2;
        this.aim = aim;
        this.goalTolIn = goalTolIn;
        int m = v.size() - 1;
        vertices = v.toArray(new double[0][]);
        // the pieces in order: per leg a straight part, per corner an arc (tangent length t either side of the corner point)
        double[] t = new double[m + 1];
        int na = 0;
        double slow = Double.MAX_VALUE;
        for (int k = 1; k < m; k++) {
            t[k] = phi[k] == 0.0 ? 0.0 : radius[k] * Math.tan(Math.abs(phi[k]) / 2.0);
            if (phi[k] != 0.0) {
                na++;
                slow = Math.min(slow, arcSpeed(radius[k]));
            }
        }
        arcs = na;
        slowestArc = na == 0 ? Double.NaN : Math.min(slow, GridRoutes.V_MAX);
        double total = 0.0;
        double[] lineLen = new double[m + 1], arcLen = new double[m + 1];
        for (int k = 1; k <= m; k++) {
            lineLen[k] = Math.max(0.0, len[k] - t[k - 1] - t[k]);
            arcLen[k] = k < m && phi[k] != 0.0 ? radius[k] * Math.abs(phi[k]) : 0.0;
            total += lineLen[k] + arcLen[k];
        }
        length = total;
        samples = Math.max(1, (int) Math.ceil(total / SAMPLE_IN));
        step = total / samples;
        px = new double[samples + 1];
        py = new double[samples + 1];
        tx = new double[samples + 1];
        ty = new double[samples + 1];
        kappa = new double[samples + 1];
        vlim = new double[samples + 1];
        dirA = new double[samples + 1];
        dirB = new double[samples + 1];
        cornerOf = new int[samples + 1];
        int k = 1;
        boolean onArc = false;
        double before = 0.0;   // the path length before the piece in hand
        for (int i = 0; i <= samples; i++) {
            double s = Math.min(total, i * step);
            while (true) {
                double pieceLen = onArc ? arcLen[k] : lineLen[k];
                if (s <= before + pieceLen + 1e-9 || (k == m && !onArc)) break;
                before += pieceLen;
                if (onArc) {
                    onArc = false;
                    k++;
                } else {
                    onArc = true;
                }
            }
            double u = s - before;
            double[] a = v.get(k - 1), b = v.get(k);
            if (!onArc) {
                double sx = a[0] + dx[k] * t[k - 1], sy = a[1] + dy[k] * t[k - 1];
                px[i] = sx + dx[k] * u;
                py[i] = sy + dy[k] * u;
                tx[i] = dx[k];
                ty[i] = dy[k];
                dirA[i] = dirB[i] = Math.atan2(dy[k], dx[k]);
            } else {
                double sgn = Math.signum(phi[k]), r = radius[k];
                double ax = b[0] - dx[k] * t[k], ay = b[1] - dy[k] * t[k];          // where the arc starts
                double cx = ax - dy[k] * r * sgn, cy = ay + dx[k] * r * sgn;       // its center, on the inside of the turn
                double th = sgn * u / r, co = Math.cos(th), si = Math.sin(th);
                px[i] = cx + (ax - cx) * co - (ay - cy) * si;
                py[i] = cy + (ax - cx) * si + (ay - cy) * co;
                tx[i] = dx[k] * co - dy[k] * si;
                ty[i] = dx[k] * si + dy[k] * co;
                kappa[i] = sgn / r;
                cornerOf[i] = k;
                dirA[i] = Math.atan2(dy[k], dx[k]);
                dirB[i] = Math.atan2(dy[k + 1], dx[k + 1]);
            }
        }
        // the speed profile: the direction the wheels are driven in, the arcs, the stop, and the braking for each from before it
        for (int i = 0; i <= samples; i++) {
            boolean late = !cruise || Math.hypot(px[i] - gx, py[i] - gy) <= turn2Start;
            double rho = late ? Math.atan2(ty[i], tx[i]) - hf : follow ? off : Math.atan2(ty[i], tx[i]) - hc;
            double lim = speedAt(rho);
            if (kappa[i] != 0.0) lim = Math.min(lim, arcSpeed(1.0 / Math.abs(kappa[i])));
            vlim[i] = Math.min(lim, Math.sqrt(2.0 * STOP_DECEL * Math.max(0.0, total - i * step)));
        }
        for (int i = samples - 1; i >= 0; i--) vlim[i] = Math.min(vlim[i], Math.sqrt(vlim[i + 1] * vlim[i + 1] + 2.0 * A_DEC * step));
        // the predicted time: the profile from rest, plus what each turn makes the robot wait when its zone is passed sooner than the turn takes
        double[] at = new double[samples + 1];
        double vel = 0.0;
        for (int i = 1; i <= samples; i++) {
            double next = Math.min(vlim[i], Math.sqrt(vel * vel + 2.0 * A_ACC_PREDICT * step));
            at[i] = at[i - 1] + step / Math.max(1.0, 0.5 * (vel + next));
            vel = next;
        }
        double sec = at[samples];
        if (turning) {
            double zoneEnd = cruise ? turn1End : Math.min(turn1End, total - endIn);
            sec += Math.max(0.0, Math.abs(delta1) / TURN_RATE_PREDICT + TURN_SETTLE_SEC - (timeAt(at, zoneEnd) - timeAt(at, holdIn)));
        }
        if (cruise && Math.abs(delta2) > tol) {
            sec += Math.max(0.0, Math.abs(delta2) / TURN_RATE_PREDICT + TURN_SETTLE_SEC - (timeAt(at, total - endIn) - timeAt(at, total - turn2Start)));
        }
        predictedSec = sec;
    }

    private double arcSpeed(double radius) {
        double v = Math.sqrt(A_LAT * radius);
        return follow ? Math.min(v, ARC_TURN_RATE * radius) : v;
    }

    private double timeAt(double[] at, double s) {
        double f = clamp(s / step, 0.0, samples);
        int i = Math.min(samples - 1, (int) f);
        return at[i] + (at[i + 1] - at[i]) * (f - i);
    }

    /** The first sample that is not clear, or -1. */
    private int verify(Zones z) {
        boolean[] st = {false};
        int every = Math.max(1, (int) Math.floor(CHECK_STEP_IN / step));
        for (int i = 0; i < samples; i += every) if (!z.poseOk(px[i], py[i], dirA[i], dirB[i], st)) return i;
        return z.goalOk() ? -1 : samples;
    }

    // ================================================================ following

    public boolean arrived() {
        return arrived;
    }

    /** The plan as a number: what the plan tables hold. */
    public int code() {
        return code;
    }

    /** The time the path is predicted to take from rest (s): the speed profile, plus the waits for the turns. */
    public double predictedSec() {
        return predictedSec;
    }

    public double offRouteIn() {
        return offRouteIn;
    }

    /** The most the robot has been off the path so far (in, by the pose it believes): what the tracking allowance must cover. */
    public double mostOffIn() {
        return mostOffIn;
    }

    /** Where on the path that was. */
    public String mostOffWhere() {
        return mostOffWhere;
    }

    /** The path's corner points (the robot, the points kept, the goal), field frame. */
    public double[][] vertices() {
        return vertices;
    }

    public String describe() {
        StringBuilder b = new StringBuilder(vertices.length == 2 ? "direct" : "via");
        for (int i = 1; i + 1 < vertices.length; i++) b.append(String.format(Locale.US, " (%.0f, %.0f)", vertices[i][0], vertices[i][1]));
        b.append(String.format(Locale.US, ", %.0f in, SMOOTH path, %.2f s predicted, %d arc%s", length, predictedSec, arcs, arcs == 1 ? "" : "s"));
        if (arcs > 0) b.append(String.format(Locale.US, " (slowest %.0f in/s)", slowestArc));
        double hfDeg = Math.toDegrees(Rotation2d.normalize(hf));
        if (follow) {
            String[] lead = {"front", "left side", "rear", "right side"};
            b.append(String.format(Locale.US, ", %s along the path (a turn of %.0f deg from %.0f in out), then %.0f deg to %.0f deg over the last %.0f in, done %.0f in before the goal",
                    lead[(int) Math.round(-off / (Math.PI / 2.0)) & 3], Math.toDegrees(delta1), holdIn, Math.toDegrees(delta2), hfDeg, turn2Start, endIn));
        } else if (cruise) {
            b.append(String.format(Locale.US, ", cruises at %.0f deg (a turn of %.0f deg from %.0f in out), then %.0f deg to %.0f deg over the last %.0f in, done %.0f in before the goal",
                    Math.toDegrees(Rotation2d.normalize(hc)), Math.toDegrees(delta1), holdIn, Math.toDegrees(delta2), hfDeg, turn2Start, endIn));
        } else if (turning) {
            b.append(String.format(Locale.US, ", turns %.0f deg to %.0f deg from %.0f in out, done %.0f in before the goal or %.0f in out", Math.toDegrees(delta1), hfDeg, holdIn, endIn, turn1End));
        } else {
            b.append(String.format(Locale.US, ", heading %.0f deg kept", hfDeg));
        }
        return b.toString();
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double profile(double s) {
        double f = clamp(s / step, 0.0, samples);
        int i = Math.min(samples - 1, (int) f);
        return vlim[i] + (vlim[i + 1] - vlim[i]) * (f - i);
    }

    /** One loop: the pose and field velocity (in, rad, in/s, rad/s). The output is GridFollower's (one leg). */
    public GridFollower.Output update(double x, double y, double h, double vx, double vy, double w) {
        out.translation = true;
        out.turnActive = false;
        out.lockWanted = false;
        out.arrived = false;
        out.forward = out.left = out.turn = 0.0;
        out.leg = 1;
        double goalDist = Math.hypot(x - gx, y - gy), speed = Math.hypot(vx, vy);
        if (arrived) {
            out.lockWanted = aim;
            out.arrived = true;
            return out;
        }
        boolean stalledAtWall = !aim && GridRoutes.DRIFT_IN > 0.0 && goalDist <= goalTolIn + GridRoutes.DRIFT_IN && speed <= GridFollower.PARK_STALL_SPEED && sawSpeed;
        if (speed > 2.0 * GridFollower.PARK_STALL_SPEED) sawSpeed = true;
        if ((goalDist <= goalTolIn && speed <= GridFollower.ARRIVE_MAX_SPEED) || stalledAtWall) {
            arrived = true;
            out.arrived = true;
            out.lockWanted = aim;
            return out;
        }
        // where on the path: the nearest sample at or a little after the last one
        int lo = Math.max(0, idx - 3), hi = Math.min(samples, idx + 30);
        double best = Double.MAX_VALUE;
        for (int i = lo; i <= hi; i++) {
            double d = (x - px[i]) * (x - px[i]) + (y - py[i]) * (y - py[i]);
            if (d < best) {
                best = d;
                idx = i;
            }
        }
        double ux = tx[idx], uy = ty[idx], nx = -uy, ny = ux;
        double along = (x - px[idx]) * ux + (y - py[idx]) * uy, cross = (x - px[idx]) * nx + (y - py[idx]) * ny;
        offRouteIn = (idx == 0 && along < 0.0) || (idx == samples && along > 0.0) ? Math.sqrt(best) : Math.abs(cross);
        if (offRouteIn > mostOffIn) {
            mostOffIn = offRouteIn;
            mostOffWhere = String.format(Locale.US, "%.0f in along its %.0f in, %s, at %.0f in/s", idx * step, length, kappa[idx] != 0.0 ? "on an arc" : "on a straight part", speed);
        }
        double s = idx * step + along, remaining = length - s;
        // the heading plan
        double d0 = Math.hypot(x - x0, y - y0);
        if (!started && (d0 >= holdIn || !turning)) started = true;
        boolean ending = !cruise || goalDist <= turn2Start;
        double cruiseHeading = follow ? Math.atan2(uy, ux) + off : hc;
        if (cruise && started && !cruiseDone && Math.abs(Rotation2d.normalize(cruiseHeading - h)) <= ctol) cruiseDone = true;
        if (ending && started && !turnDone && Math.abs(Rotation2d.normalize(hf - h)) <= tol) turnDone = true;
        // inside the lock radius the heading is the target lock's, which aims from where the robot is, not from the goal: waiting for the goal heading there
        // never ends (a new route picked 10 in from the spot stopped 6 in short of it for good, simulation 2026-11-17)
        if (aim && goalDist <= LOCK_RADIUS_IN) turnDone = true;
        double headingTarget = !started ? h0 : ending ? hf : cruiseHeading;
        // the speed
        boolean pursuit = remaining <= PURSUIT_IN || idx >= samples;
        double dirX = ux, dirY = uy;
        if (pursuit) {
            double d = Math.max(goalDist, 1e-6);
            dirX = (gx - x) / d;
            dirY = (gy - y) / d;
        }
        double vAlong = vx * dirX + vy * dirY;
        double ahead = Math.max(0.0, vAlong) * LOOKAHEAD_SEC;
        double vTarget = pursuit ? Math.min(GridRoutes.V_MAX, Math.sqrt(2.0 * STOP_DECEL * Math.max(0.0, goalDist - ahead))) : profile(s + ahead);
        // a turn that is not done: no further than where its zone ends, the next zone was checked for the next heading only
        double room = Double.MAX_VALUE;
        if (!turnDone && (cruise || turning)) room = goalDist - endIn;
        if (!cruise && turning && !turnDone) room = Math.min(room, turn1End - d0);
        if (cruise && turning && !cruiseDone && !ending) room = Math.min(room, turn1End - d0);
        if (room < 1e6) vTarget = Math.min(vTarget, Math.sqrt(2.0 * A_DEC * Math.max(0.0, room - ahead)));
        boolean brake = vAlong > vTarget + GridFollower.BRAKE_OVER_IN_PER_S || vTarget <= 0.5;
        double cmdX = 0.0, cmdY = 0.0;
        if (pursuit) {
            if (!brake) {
                // straight at the goal: the whole velocity is steered, so what the robot carries sideways is taken out and it does not circle the spot
                cmdX = dirX * vTarget + SPEED_KP * (dirX * vTarget - vx);
                cmdY = dirY * vTarget + SPEED_KP * (dirY * vTarget - vy);
            }
        } else {
            double cmdAlong = brake ? 0.0 : Math.max(0.0, vTarget + SPEED_KP * (vTarget - vAlong));
            double vCross = vx * nx + vy * ny;
            double side = clamp(-CROSS_KP * cross - CROSS_KD * vCross, -CROSS_MAX, CROSS_MAX) + CENTRIPETAL_SEC * vAlong * vAlong * kappa[ARC_LEAD_SEC <= 0.0 ? idx : (int) clamp(Math.round((s + Math.max(0.0, vAlong) * ARC_LEAD_SEC) / step), 0, samples)];
            cmdX = dirX * cmdAlong + nx * side;
            cmdY = dirY * cmdAlong + ny * side;
        }
        // into the robot's axes and into power (the strafe needs more for the same speed)
        double fwd = (cmdX * Math.cos(h) + cmdY * Math.sin(h)) / FF_GAIN;
        double left = (-cmdX * Math.sin(h) + cmdY * Math.cos(h)) / (FF_GAIN * STRAFE_RATIO);
        double sum = Math.abs(fwd) + Math.abs(left);
        if (sum > MAX_POWER) {
            fwd *= MAX_POWER / sum;
            left *= MAX_POWER / sum;
        }
        if (aim && goalDist <= LOCK_RADIUS_IN) {
            out.lockWanted = true;
            double scale = Math.max(GridFollower.AIM_ALIGN_FLOOR, Math.cos(Rotation2d.normalize(hf - h)));
            fwd *= scale;
            left *= scale;
        } else {
            double err = Rotation2d.normalize(headingTarget - h);
            // a follow plan's body turns with the path on an arc
            double rate = follow && started && !ending ? vAlong * kappa[idx] : 0.0;
            double c = GridFollower.HEADING_KP * err - GridFollower.HEADING_KD * (w - rate) + rate / TURN_FF_GAIN;
            out.turn = clamp(GridFollower.withKs(c, GridFollower.KS_TURN), -GridFollower.TURN_MAX, GridFollower.TURN_MAX);
            out.turnActive = true;
        }
        if (fwd != 0.0 || left != 0.0) {
            fwd = GridFollower.withKs(fwd, GridFollower.KS_FORWARD);
            left = GridFollower.withKs(left, GridFollower.KS_STRAFE);
        }
        out.forward = fwd;
        out.left = left;
        return out;
    }
}
