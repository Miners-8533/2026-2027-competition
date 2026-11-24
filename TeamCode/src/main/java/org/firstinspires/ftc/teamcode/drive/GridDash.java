package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * One dash on the GRID planner (specs/dashes.md), shared by {@link HiveDash} and {@link ParkDash} when their GRID_ROUTES switch is on: the route from the
 * precomputed tree ({@link GridRoutes}, tables generated on a PC into {@link GridTables}), driven by {@link GridFollower}, with the OFF-ROUTE GUARD: when the robot is
 * more than OFF_ROUTE_IN from the leg it is driving (a push, a slip, a swing the tables did not allow for), the dash brakes, and once stopped picks a new route from where
 * the robot is, at most MAX_REPICKS times; after that, or when no route is found, it gives up (the dash cancels and the driver has the sticks).
 *
 * Works in the field frame; the tables are blue, so a red dash turns the pose 180 deg about the field center into them and the route back. Pure logic, no hardware.
 */
@Config
public final class GridDash {

    /** The off-route guard: more than this from the leg being driven (in) brakes the dash; 0 turns the guard off. */
    public static double OFF_ROUTE_IN = 6.0;
    /** Braked down to this speed (in/s), or after STOP_TIMEOUT_SEC, the dash picks a new route; at most MAX_REPICKS times per dash. */
    public static double REPICK_SPEED_IN_PER_S = 6.0, STOP_TIMEOUT_SEC = 1.0;
    public static int MAX_REPICKS = 2;
    /**
     * True drives the grid route as one smooth path ({@link SmoothFollower}: the fewest points, arcs at the corners, one turn to the goal heading, one speed
     * profile) when such a path is clear, and leg by leg with GridFollower when it is not. ON since 2026-11-18 for both dashes (the team's decision), in simulation
     * only: NOT YET RUN ON THE ROBOT. False drives every grid route leg by leg.
     */
    public static boolean SMOOTH = true;
    /** The smooth path is taken when its predicted time is no more than this (s) over the leg-by-leg route's. */
    public static double SMOOTH_MAX_EXTRA_SEC = 0.3;
    /**
     * False (the robot): the smooth path is the one the generated plan table names for the start nearest the robot (SmoothTables, made offline), checked at the
     * robot's pose. True (PC studies only): the full search for the quickest plan on the spot, which the generator runs offline; it is far too slow for the loop.
     */
    public static boolean SMOOTH_SEARCH = false;

    public static final class Output {
        public boolean translation, turnActive, lockWanted, arrived;
        public double forward, left, turn;
        /** 1-based leg being driven, and the number of legs. */
        public int leg, legs;
        /** Not null: the dash gives up (why). */
        public String failed;

        void reset() {
            translation = turnActive = lockWanted = arrived = false;
            forward = left = turn = 0.0;
            failed = null;
        }
    }

    /** The blue table for a goal given in the field frame, or null (then {@link #whyNoTable} says why). */
    public static GridRoutes.Table tableFor(double gx, double gy, boolean blue) {
        double s = blue ? 1.0 : -1.0;
        return GridRoutes.tableFor(s * gx, s * gy);
    }

    public static String whyNoTable(double gx, double gy, boolean blue) {
        String p = GridRoutes.tablesProblem();
        if (p != null) return p;
        return String.format(Locale.US, "no table has the goal (%.1f, %.1f)%s: was it moved on the Dashboard? regenerate the tables", gx, gy, blue ? "" : " (red)");
    }

    /** Decodes the tables and warms the pick (the first one loads the obstacles); returns the problem with the tables, or null. Call at init. */
    public static String warm() {
        GridRoutes.Table[] ts = GridRoutes.tables();
        if (ts == null) return GridRoutes.tablesProblem();
        if (ts.length > 0) GridRoutes.pick(ts[0], -30.0, -30.0, 0.0);
        String grid = gridProblem(), plans = SmoothFollower.tablesProblem(), map = SmoothFollower.mapProblem();
        if (plans == null && ts.length > 0) {
            // one lookup and check now, so the first press does not pay for unpacking the plan tables
            GridRoutes.Route r = GridRoutes.pick(ts[0], -30.0, -30.0, 0.0);
            if (r != null) SmoothFollower.build(ts[0].name, -30.0, -30.0, 0.0, r.xs, r.ys, ts[0].aimHeading, 2.5, 0.0);
        }
        StringBuilder b = new StringBuilder();
        for (String p : new String[]{grid, plans, map}) if (p != null) b.append(b.length() == 0 ? "" : "; ").append(p);
        return b.length() == 0 ? null : b.toString();
    }

    /** The route trees were generated with the motion values in force, or what differs. */
    private static String gridProblem() {
        return GridTables.MODEL.equals(GridRoutes.model()) ? null
                : "the tables were generated with other motion values (" + GridTables.MODEL + "; now " + GridRoutes.model() + "): they are used, but regenerate them";
    }

    private final String prefix;
    private final GridRoutes.Table table;
    private final boolean blue;
    private final double goalTolIn;
    private final Output out = new Output();
    private final ElapsedTime stopClock = new ElapsedTime();
    private GridRoutes.Route route;
    private GridFollower follower;
    private SmoothFollower smooth;   // the smooth path of this route, or null (then the follower drives it)
    private boolean smoothReported = false;
    /** The time the smooth path's planning took for this route (ms): on the robot it must stay a small part of a loop. */
    private double planMs = 0.0;
    private double[][] waypoints;
    private boolean stopping = false;
    private int repicks = 0, lastLeg = 0;

    /** prefix: the log line's start ("HIVE DASH" or "PARK DASH"). */
    public GridDash(String prefix, GridRoutes.Table table, boolean blue, double goalTolIn) {
        this.prefix = prefix;
        this.table = table;
        this.blue = blue;
        this.goalTolIn = goalTolIn;
    }

    /** Picks the route from the robot's pose (field frame). Null when there is one, otherwise why not. */
    public String start(double x, double y, double h) {
        double s = blue ? 1.0 : -1.0;
        GridRoutes.Route r = GridRoutes.pickOrMoveClear(table, s * x, s * y, blue ? h : h + Math.PI);
        if (r == null) return "no grid route from here, and no straight move of up to " + (int) GridRoutes.MOVE_CLEAR_MAX_IN + " in leads clear to one";
        route = r;
        smoothReported = false;
        boolean slower = false;
        double[][] f = GridFollower.toField(r, blue);
        double aim = Double.isNaN(table.aimHeading) ? Double.NaN : blue ? table.aimHeading : table.aimHeading + Math.PI;
        long t0 = System.nanoTime();
        smooth = !SMOOTH || r.moveClearLegs > 0 ? null : SMOOTH_SEARCH ? SmoothFollower.search(f[0], f[1], aim, goalTolIn, h)
                : SmoothFollower.build(table.name, s * x, s * y, blue ? h : h + Math.PI, f[0], f[1], aim, goalTolIn, h);
        planMs = (System.nanoTime() - t0) / 1e6;
        if (smooth != null && smooth.predictedSec() > r.predictedSec + SMOOTH_MAX_EXTRA_SEC) {
            // both times are predictions from rest and both run short; the smooth path's turns can cost more than the legs' corners
            RunLog.event(String.format(Locale.US, "%s smooth path not taken (%.2f s predicted against %.2f s leg by leg): %s", prefix, smooth.predictedSec(), r.predictedSec, smooth.describe()));
            smooth = null;
            slower = true;
        }
        if (smooth != null) {
            follower = null;
            waypoints = smooth.vertices();
        } else {
            if (SMOOTH && !slower) RunLog.event(prefix + " no smooth path (" + (r.moveClearLegs > 0 ? "the route moves clear first" : SmoothFollower.lastWhy)
                    + String.format(Locale.US, "; planned in %.1f ms): leg by leg", planMs));
            follower = new GridFollower(f[0], f[1], f[2], aim, goalTolIn, h, r.moveClearLegs);
            waypoints = new double[f[0].length][];
            for (int i = 0; i < waypoints.length; i++) waypoints[i] = new double[]{f[0][i], f[1][i]};
        }
        stopping = false;
        lastLeg = 1;
        return null;
    }

    /** The route's waypoints in the field frame (the robot, the start point, the corners, the goal). */
    public double[][] waypoints() {
        return waypoints;
    }

    /** The route in HiveRoute's words ("via (x, y) (x, y), 123 in"), with the grid's predicted time. */
    public String describe() {
        if (smooth != null) return smooth.describe() + String.format(Locale.US, ", planned in %.1f ms", planMs);
        StringBuilder b = new StringBuilder();
        if (route.moveClearLegs > 0) {
            b.append(String.format(Locale.US, "MOVES CLEAR %.0f in first (heading held), then ",
                    Math.hypot(waypoints[1][0] - waypoints[0][0], waypoints[1][1] - waypoints[0][1])));
        }
        b.append(waypoints.length == 2 ? "direct" : "via");
        double len = 0.0;
        for (int i = 1; i < waypoints.length; i++) {
            len += Math.hypot(waypoints[i][0] - waypoints[i - 1][0], waypoints[i][1] - waypoints[i - 1][1]);
            if (i + 1 < waypoints.length) b.append(String.format(Locale.US, " (%.0f, %.0f)", waypoints[i][0], waypoints[i][1]));
        }
        b.append(String.format(Locale.US, ", %.0f in, GRID route, %.2f s predicted, corner speeds", len, route.predictedSec));
        for (int i = 1; i + 1 < route.speeds.length; i++) b.append(String.format(Locale.US, " %.0f", Math.min(route.speeds[i], GridRoutes.V_MAX)));
        return b.toString();
    }

    /** One loop: the pose and the field velocity (in, rad, in/s, rad/s). */
    public Output update(double x, double y, double h, double vx, double vy, double w) {
        out.reset();
        out.legs = smooth != null ? 1 : follower.legs();
        if (stopping) {
            out.translation = true;   // no power: the motors brake
            out.turnActive = true;    // and no turn (the stick does not take over)
            out.leg = lastLeg;
            double speed = Math.hypot(vx, vy);
            if (speed > REPICK_SPEED_IN_PER_S && stopClock.seconds() < STOP_TIMEOUT_SEC) return out;
            if (repicks >= MAX_REPICKS) {
                out.failed = String.format(Locale.US, "off the route again after %d new routes", repicks);
                return out;
            }
            repicks++;
            String why = start(x, y, h);
            if (why != null) {
                out.failed = "off the route, and " + why;
                return out;
            }
            RunLog.event(String.format(Locale.US, "%s new route %d from (%.1f, %.1f, %.1f deg): %s", prefix, repicks, x, y, Math.toDegrees(h), describe()));
            out.legs = smooth != null ? 1 : follower.legs();
        }
        GridFollower.Output f = smooth != null ? smooth.update(x, y, h, vx, vy, w) : follower.update(x, y, h, vx, vy, w);
        double off = smooth != null ? smooth.offRouteIn() : follower.offRouteIn();
        if (f.arrived && smooth != null && !smoothReported) {
            smoothReported = true;
            RunLog.event(String.format(Locale.US, "%s smooth path followed within %.1f in (the tracking allowance is %.1f in); the most was %s", prefix, smooth.mostOffIn(), SmoothFollower.TRACK_IN, smooth.mostOffWhere()));
        }
        out.leg = f.leg;
        if (f.leg != lastLeg && !f.arrived) {
            double[] p = waypoints[f.leg - 1];
            RunLog.event(String.format(Locale.US, "%s via point (%.0f, %.0f) reached at (%.1f, %.1f), leg %d of %d", prefix, p[0], p[1], x, y, f.leg, out.legs));
            lastLeg = f.leg;
        }
        if (!f.arrived && OFF_ROUTE_IN > 0.0 && off > OFF_ROUTE_IN) {
            stopping = true;
            stopClock.reset();
            RunLog.event(String.format(Locale.US, "%s OFF ROUTE: %.1f in from leg %d of %d at (%.1f, %.1f, %.1f deg), %.1f in/s: braking", prefix, off, f.leg, out.legs,
                    x, y, Math.toDegrees(h), Math.hypot(vx, vy)));
            out.translation = true;
            out.turnActive = true;
            return out;
        }
        out.translation = f.translation;
        out.forward = f.forward;
        out.left = f.left;
        out.turnActive = f.turnActive;
        out.turn = f.turn;
        out.lockWanted = f.lockWanted;
        out.arrived = f.arrived;
        return out;
    }
}
