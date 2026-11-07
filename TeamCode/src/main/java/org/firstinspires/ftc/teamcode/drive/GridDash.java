package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.RunLog;

import java.util.Locale;

/**
 * One dash on the GRID planner (design/dash-planner-grid.md), shared by {@link HiveDash} and {@link ParkDash} when their GRID_ROUTES switch is on: the route from the
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
        GridRoutes.Route r = GridRoutes.pick(table, s * x, s * y, blue ? h : h + Math.PI);
        if (r == null) return "no grid route from here (no grid point near the robot reaches the goal)";
        route = r;
        double[][] f = GridFollower.toField(r, blue);
        double aim = Double.isNaN(table.aimHeading) ? Double.NaN : blue ? table.aimHeading : table.aimHeading + Math.PI;
        follower = new GridFollower(f[0], f[1], f[2], aim, goalTolIn, h);
        waypoints = new double[f[0].length][];
        for (int i = 0; i < waypoints.length; i++) waypoints[i] = new double[]{f[0][i], f[1][i]};
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
        StringBuilder b = new StringBuilder(waypoints.length == 2 ? "direct" : "via");
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
        out.legs = follower.legs();
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
            out.legs = follower.legs();
        }
        GridFollower.Output f = follower.update(x, y, h, vx, vy, w);
        out.leg = f.leg;
        if (f.leg != lastLeg && !f.arrived) {
            double[] p = waypoints[f.leg - 1];
            RunLog.event(String.format(Locale.US, "%s via point (%.0f, %.0f) reached at (%.1f, %.1f), leg %d of %d", prefix, p[0], p[1], x, y, f.leg, out.legs));
            lastLeg = f.leg;
        }
        if (!f.arrived && OFF_ROUTE_IN > 0.0 && follower.offRouteIn() > OFF_ROUTE_IN) {
            stopping = true;
            stopClock.reset();
            RunLog.event(String.format(Locale.US, "%s OFF ROUTE: %.1f in from leg %d of %d at (%.1f, %.1f, %.1f deg), %.1f in/s: braking", prefix, follower.offRouteIn(), f.leg, out.legs,
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
